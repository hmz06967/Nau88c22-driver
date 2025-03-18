#include "codec.h"

/* power / setting*/

/*
    R1 Power management, enable control for PLL (default = disabled)
    
    R6 Master/slave mode, clock scaling, clock selection
    R7 Sample rate indication (scales DSP coefficients and timing – does NOT affect actual sample rate
    R8 MUX control and division factor for PLL output on GPIO1
    
    R36 PLL Prescaler, Integer portion of PLL frequency multiplier
    R37 Highest order bits of 24-bit fraction of PLL frequency multiplier
    R38 Middle order bits of 24-bit fraction of PLL frequency multiplier
    R39 Lowest order bits of 24-bit fraction of PLL frequency multiplier

    clock_control_2
        SMPLR:
            000 = 48kHz
            001 = 32kHz
            010 = 24kHz
            011 = 16kHz
            100 = 12kHz
            101 = 8kHz
            110 = reserved
            111 = reserved
        
        SCLKEN: Slow timer clock enable, 0

    clock_control_1

        MCLKSEL:
            000 = divide by 1
            001 = divide by 1.5
            010 = divide by 2
            011 = divide by 3
            100 = divide by 4
            101 = divide by 6
            110 = divide by 8
            111 = divide by 12
        
        BCLKSEL:
            Scaling of output frequency at BCLK pin#8 when chip is in master mode
            000 = divide by 1
            001 = divide by 2
            010 = divide by 4
            011 = divide by 8
            100 = divide by 16
            101 = divide by 32
            110 = reserved
            111 = reserved

        CLKIOEN: 
            0 = FS and BCLK are inputs
            1 = FS and BCLK are driven as outputs by internally generated clocks
    }

*/

uint8_t codec_init(alc_control_t *alc){//mclk freq, freq sample: 48khz
    uint8_t status = NAU_OK;

    if(nau8822_init(&alc->div_id) != NAU_OK){
        return NAU_ERROR_ID;
    }
    
    uint8_t pllon = (alc->pll.freq_in!=0);
    status = setup_clock_pll(&alc->pll);
    status &= settup_adc(alc->adc_invert[0], alc->adc_invert[1]);
    status &= setup_dac(alc->dac_automt, alc->dac_invert[0], alc->dac_invert[1]);
    return status;
}

uint32_t setup_clock_pll(struct nau8822_pll *pll){//1, 48khz, 12000khz
    //R6[8]
    uint32_t IMCLK = 256 * pll->fs;

    if(pll->fs==96 || pll->fs==192){
        clock_pll(0);
        khz192_sampling(1,(pll->fs==192),1, 1);//adc bias, 49mhz pll 
        gac_clock_control1(0, 0, 0, 0);
    }else{
        khz192_sampling(0,1,0,0);//adc bias, 49mhz pll 
        clock_pll(pll->pllon);//pll, aç kapat 0: mclk->fpll, 1:mclk->pll->fpll
    /*
    [PLL Prescaler]{
        
        f1pll = (R36[4] ? mclk/2 : mclk;
        f2 = R(f1pll) / 2; //90, 100MHZ arası olması daha iyi
        fpll = f2/2
            = R(f1pll) / 4;
        ****************************
        FPLL = R6[8] ? fpll : mclk; 
        PLL49MOUT =  R72[2] && Master Clock Select R6[8], 192kHzSampling
        N = R6[7,6,5] //MCLKSEL
        IMCLK = PLL49MOUT ? f2 : (FPLL * 1/N);
        
        R = f2/f1 = xy.abcdefgh
        N = xy
        K = (224)*(0.abcdefgh)
        ****************************

        DAI: FS = IMCLK (f/256), sanırım 256 kanal başı örnekleme hızı.
        DAI: BCLK = R6[4,3,2] f/n
        DAI  = R6[0] M/S select

        //PLL to GPIO1,  R8[5,4] = f/N, ENABLE = R8[2,1,0]
        M/S = R6[0]
        BCLK SCALER = R6[4,3,2]
    }

    //IMCLK = desired Master Clock = (256)*(desired codec sample rate)
       master_clock = (sample_rate)khz * 256
       12  = 
       2048, 3072, 4096, 8192, 12288, 
       24576, 49152


    R72, 192kHZ ampling
        ADCB_OVER 0: 8-48khz, 1: 96, 192;


    */
        float f2_n[8] = {1.0,1.5,2.0,3.0,4.0,6.0,8.0,12.0};
        float f2,R;
        uint8_t i=0, md= 0,pd = 4;
        uint32_t f1 = pll->freq_in;
        
        while(i<8){
            f2 = pd*IMCLK*f2_n[i];
            if(f2>=90000.0 && f2<=100000.0){
                break;
            }
            i++;
            if(i==8 && pd==4){
                i=0;
                pd=2;
            } 
        }

        pll->freq_out = f2;
        gac_clock_control1(pll->pllon, i, 0, 0);
        R = f2/f1;
        pll->pll_frac = R;

        uint8_t pll_n = R;
        uint32_t pll_kn = (R - pll_n)*BIT_2_24;
        uint16_t pll_k3 = pll_kn & 0x1ff;
        uint16_t pll_k2 = ((pll_kn >> 9) & 0x1ff);
        uint16_t pll_k1 = ((pll_kn >> 18) & 0x1ff);

        pll_nn(0, pll_n); //pll, tam sayı
        pll_k_1(pll_k1); //high
        pll_k_2(pll_k2); //middle
        pll_k_3(pll_k3); //low, 24 bit pll bölücü

        gac_clock_control2(0, 0, 0); //io interface pll out 
    }

    return IMCLK;
}

uint8_t settup_adc(uint8_t adc_linv, uint8_t adc_rinv){
    //ADC rate 128
    return gac_adc_control(0,0,0, 1, adc_rinv, adc_linv);//hpf,  128x rate, lr ch 0 normal polarity
}

uint8_t setup_dac(uint8_t auto_mute, uint8_t adc_linv, uint8_t adc_rinv){
    dac_limiter_1(0,0,0);//dc en
    dac_limiter_2(0,0);
    dac_dither(0,0);
    gac_dac_control(0, 1, auto_mute, adc_rinv, adc_linv);//128x sample
}

/* buffer */

/*
    # (api) miscellaneous

    input control
    0: not connect

    micbias
        Normal Mode Low Noise Mode
        00 = 0.9x 00 = 0.85x
        01 = 0.65x 01 = 0.60x
        10 = 0.75x 10 = 0.70x
        11 = 0.50x 11 = 0.50x


*/

uint8_t alc_control(alc_control_t *alc){
    misc_input_control(alc->micbias, alc->pga);
    alc_enhancements1(0,0,0, alc->pga.ch_pga[0]);//pdf page 91
    alc_enhancements2(0,alc->pga.ch_pga[1]);
    return control_and_status(alc->dac_automt,0,0);//dac automute
}

uint8_t input_gain(alc_control_t *alc){
    misc_left_input_pga_gain(alc->vol.input_pga_vol);
    return misc_right_input_pga_gain(alc->vol.input_pga_vol);
}

/* buffer */
/* mixer */ 
uint8_t adc_mix_boost(alc_control_t *alc){
    misc_left_adc_boost(alc->abst);
    return misc_right_adc_boost(alc->abst);
}

/*
    LAUXMXGAIN
        Gain value between LAUXIN auxiliary input and input to LMAIN left output mixer
        000 = -15dB (default)
        001 = -12dB
        010 = -9.0dB
        011 = -6.0dB
        100 = -3.0dB
        101 = 0.0dB
        110 = +3.0dB
        111 = +6.0dB
    LAUXLMX
        LAUXIN input to LMAIN left output mixer path control
        0 = LAUXIN not connected to LMAIN left output mixer (default)
        1 = LAUXIN connected to LMAIN left output mixer
    LBYPMXGAIN
        Gain value for bypass from LADC Mix/Boost output to LMAIN left output mixer.
        000 = -15dB (default)
        001 = -12dB
        010 = -9.0dB
        011 = -6.0dB
        100 = -3.0dB
        101 = 0.0dB
        110 = +3.0dB
        111 = +6.0dB
    LBYPLMX
        Left bypass path control from LADC Mix/Boost output to LMAIN left output mixer
        0 = path not connected
        1 = bypass path connected
    LDACLMX
        Left DAC output to LMIX left output mixer path control
        0 = path disconnected

*/
uint8_t main_mixer(alc_control_t *alc){
	misc_left_mixer(alc->mix.aux_gain[0], alc->mix.aux_en[0], alc->mix.adc_mx_gain[0],
        alc->mix.adc_mx_en[0], alc->mix.dac_mx_en[0]);
    return misc_right_mixer(alc->mix.aux_gain[1], alc->mix.aux_en[1], alc->mix.adc_mx_gain[1], 
        alc->mix.adc_mx_en[1], alc->mix.dac_mx_en[1]);
}

uint8_t spk_submix(alc_control_t *alc){
    return misc_right_speaker_submix(
        alc->mix.rspk_mix[0],
        alc->mix.rspk_mix[1],
        alc->mix.rspk_mix[2],
        alc->mix.rspk_mix[3]
    );
}

uint8_t aux_submix(alc_control_t *alc){
	misc_aux1_mixer(alc->mix.aux1_mix[0],alc->mix.aux1_mix[1],alc->mix.aux1_mix[2],alc->mix.aux1_mix[3],
        alc->mix.aux1_mix[4], alc->mix.aux1_mix[5], alc->mix.aux1_mix[6]);
    return misc_aux2_mixer(alc->mix.aux2_mix[0],alc->mix.aux2_mix[1],alc->mix.aux2_mix[2],alc->mix.aux2_mix[3],alc->mix.aux2_mix[4]);
}

uint8_t output_control(alc_control_t *alc){//pdf 86
    return misc_output_control(
        alc->mix.out_mix[0],
        alc->mix.out_mix[1],
        alc->mix.out_mix[2],
        alc->mix.out_mix[3],
        alc->mix.out_mix[4],
        alc->mix.out_mix[5],
        alc->mix.out_mix[6]
    );
}

/* mixer */ 

/* filter */
/*
    alc_en
        00 = right and left ALCs disabled
        01 = only right channel ALC enabled
        10 = only left channel ALC enabled
        11 = both right and left channel ALCs enabled
    mx_gain
        111 = +35.25dB (default)
        110 = +29.25dB
        101 = +23.25dB
        100 = +17.25dB
        011 = +11.25dB
        010 = +5.25dB
        001 = -0.75dB
        000 = -6.75dB

    mg_gain
        000 = -12dB (default)
        001 = -6.0dB
        010 = 0.0dB
        011 = +6.0dB
        100 = +12dB
        101 = +18dB
        110 = +24dB
        111 = +30dB

    ht
        Hold time before ALC automated gain increase
        0000 = 0.00ms (default)
        0001 = 2.00ms
        0010 = 4.00ms
        - time value doubles with each bit value increment –
        1001 = 512ms
        1010 through 1111 = 1000ms 

    sl
        1111 = -1.5dB below full scale (FS)
        1110 = -1.5dB FS (same value as 1111)
        1101 = -3.0dB FS
        1100 = -4.5dB FS
        1011 = -6.0dB FS (default)
        - target level varies 1.5dB per binary step throughout control range –
        0001 = -21.0dB FS
        0000 = -22.5dB FS (lowest possible target signal level)
    
    m
      0 = normal ALC operation
      1 = Limiter Mode operation
  
    dcy
        PGA'nın 0,75dB'lik kazanç artışı için kazanç değişikliği adımı başına ALC bozulma süresi süresi
        kazanmak. Toplam yanıt süresi, gerekli toplam adım sayısı ile tahmin edilebilir.
        sinyaldeki belirli bir büyüklük değişikliğini telafi eder. Örneğin, 6dB'lik bir azalma
        sinyalde telafi etmek için sekiz ALC adımı gerekir.
        Her mod için adım boyutu şu şekilde verilir:

        Normal Mode  Limiter Mode
        0000 = 500us 0000 = 125us
        0001 = 1.0ms 0001 = 250us
        0010 = 2.0ms (default) 0010 = 500us (default)
        ------- time value doubles with each binary bit value --------
        1000 = 128ms 1000 = 32ms
        1001 = 256ms 1001 = 64ms
        1010 through 1111 = 512ms 1010 through 1111 = 128ms
    
    atk
        PGA'nın 0,75dB'lik kazanç düşüşü için kazanç değişikliği adımı başına ALC saldırı süresi süresi
        kazanmak. Toplam yanıt süresi, gerekli toplam adım sayısı ile tahmin edilebilir.
        sinyaldeki belirli bir büyüklük değişikliğini telafi eder. Örneğin, 6dB'lik bir artış
        sinyalde telafi etmek için sekiz ALC adımı gerekir.
        Step size for each mode is given by:
        Normal Mode Limiter Mode
        0000 = 125us 0000 = 31us
        0001 = 250us 0001 = 62us
        0010 = 500us (default) 0010 = 124us (default)
        ------- time value doubles with each binary bit value --------
        1000 = 26.5ms 1000 = 7.95ms
        1001 = 53.0ms 1001 = 15.9ms
        1010 through 1111 = 128ms 1010 through 1111 = 31.7ms

    alc noise gate
        ALC noise gate threshold level
        000 = -39dB (default)
        001 = -45dB
        010 = -51dB
        011 = -57dB
        100 = -63dB
        101 = -69dB
        110 = -75dB
        111 = -81dB

    ALC_CONTROL
        ALCEN:
            00 = right and left ALCs disabled
            01 = only right channel ALC enabled
            10 = only left channel ALC enabled
            11 = both right and left channel ALCs enabled
        
        ALCMXGAIN:
            Set maximum gain limit for PGA volume setting changes under ALC control
            111 = +35.25dB (default)
            110 = +29.25dB
            101 = +23.25dB
            100 = +17.25dB
            011 = +11.25dB
            010 = +5.25dB
            001 = -0.75dB
            000 = -6.75dB
        
        ALCMNGAIN:
            000 = -12dB (default)
            001 = -6.0dB
            010 = 0.0dB
            011 = +6.0dB
            100 = +12dB
            101 = +18dB
            110 = +24dB
            111 = +30dB

        ALCHT:
            Hold time before ALC automated gain increase
            0000 = 0.00ms (default)
            0001 = 2.00ms
            0010 = 4.00ms
            - time value doubles with each bit value increment –
            1001 = 512ms
            1010 through 1111 = 1000ms

        ALCSL:
            ALC target level at ADC output
            1111 = -1.5dB below full scale (FS)
            1110 = -1.5dB FS (same value as 1111)
            1101 = -3.0dB FS
            1100 = -4.5dB FS
            1011 = -6.0dB FS (default)
            - target level varies 1.5dB per binary step throughout control range –
            0001 = -21.0dB FS
            0000 = -22.5dB FS (lowest possible target signal level)

        ALCM:
            ALC mode control setting
            0 = normal ALC operation
            1 = Limiter Mode operation
        ALCDCY:
            ALC decay time duration per step of gain change for gain increase of 0.75dB of PGA
            gain. Total response time can be estimated by the total number of steps necessary to
            compensate for a given magnitude change in the signal. For example, a 6dB decrease
            in the signal would require eight ALC steps to compensate.
            Step size for each mode is given by:
            Normal Mode Limiter Mode
            0000 = 500us 0000 = 125us
            0001 = 1.0ms 0001 = 250us
            0010 = 2.0ms (default) 0010 = 500us (default)
            ------- time value doubles with each binary bit value --------
            1000 = 128ms 1000 = 32ms
            1001 = 256ms 1001 = 64ms
            1010 through 1111 = 512ms 1010 through 1111 = 128ms
        ALCATK:
            ALC attack time duration per step of gain change for gain decrease of 0.75dB of PGA
            gain. Total response time can be estimated by the total number of steps necessary to
            compensate for a given magnitude change in the signal. For example, a 6dB increase
            in the signal would require eight ALC steps to compensate.
            Step size for each mode is given by:
            Normal Mode Limiter Mode
            0000 = 125us 0000 = 31us
            0001 = 250us 0001 = 62us
            0010 = 500us (default) 0010 = 124us (default)
            ------- time value doubles with each binary bit value --------
            1000 = 26.5ms 1000 = 7.95ms
            1001 = 53.0ms 1001 = 15.9ms
            1010 through 1111 = 128ms 1010 through 1111 = 31.7ms
        
        ALCNEN
            ALC noise gate function control bit
            0 = disabled
            1 = enabled
        ALCNTH
            ALC noise gate threshold level
            000 = -39dB (default)
            001 = -45dB
            010 = -51dB
            011 = -57dB
            100 = -63dB
            101 = -69dB
            110 = -75dB
            111 = -81dB
*/


uint8_t alc_control_filter(alc_control_t *alc){
    alc_control_1(alc->alctrl.en, alc->alctrl.mx_gain, alc->alctrl.mn_gain);
    alc_control_2(alc->alctrl.ht, alc->alctrl.sl_level);
    alc_control_3(alc->alctrl.mode, alc->alctrl.dcy_time, alc->alctrl.atk_time);
    return noise_gate(alc->alctrl.ngate_en, alc->alctrl.ngate_th);
}

uint8_t three_d_efect(alc_control_t *alc){
    misc_3d_control(alc->filter.threeD_th);
}

uint8_t hpf_filter(alc_control_t *alc){
    reg_write(NAU8822_REG_ADC_CONTROL, 8, 1, alc->filter.hpf_en);
    reg_write(NAU8822_REG_ADC_CONTROL, 7, 1, alc->filter.hpf_am);
    return reg_write(NAU8822_REG_ADC_CONTROL, 4, 3, alc->filter.hpf);
}

uint8_t notch_filter(alc_control_t *alc){
    notch_filter_1(alc->filter.notch_cu[0], alc->filter.hpf_en, alc->filter.notch_ca[0]);
    notch_filter_2(alc->filter.notch_cu[1], alc->filter.notch_ca[1]);
    notch_filter_3(alc->filter.notch_cu[2], alc->filter.notch_ca[2]);
    return notch_filter_4(alc->filter.notch_cu[3], alc->filter.notch_ca[3]);
}

uint8_t band_eq(alc_control_t *alc){
    eq1_low_cutoff(&alc->eq[0]);
    eq2_peak_1(&alc->eq[1]);
    eq2_peak_2(&alc->eq[2]);
    eq2_peak_3(&alc->eq[3]);
    return eq5_high_cutoff(&alc->eq[4]);
    //read_eq_array(alc->eq);
}

uint8_t channel_volume(alc_control_t *alc, enum vol_mode mode){
    uint8_t status = NAU_OK;
    switch (mode) {
        case VOL_DAC:     
            status = gac_left_dac_volume(alc->vol.dac_vol);
            status &= gac_right_dac_volume(alc->vol.dac_vol);
        break;
        case VOL_ADC: 
            status = gac_left_adc_volume(alc->vol.adc_vol);
            status &= gac_right_adc_volume(alc->vol.adc_vol);
        break;
        case VOL_HP: 
            status = misc_lhp_volume(alc->vol.hp_vol);
            status &= misc_rhp_volume(alc->vol.hp_vol);
        break;
        case VOL_SPK: 
            status = misc_lspkout_volume(alc->vol.spk_vol);
            status &= misc_rspkout_volume(alc->vol.spk_vol);
        break;
    }

    return status;
}

/* filter */ 
