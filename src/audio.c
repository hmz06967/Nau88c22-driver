#include "audio.h"

alc_control_t *ALC;

uint8_t audio_run_cmd(uint8_t *data){
    /*
        burası değiştirilebilir spi sadece write modundayken 
        okuma yapamazsa write komutlarının success olup olmadığı doğrulanabilr
        böylece device id koduna gerek kalmaz
    */
    /*if(device_id() != NAU_DEV_ID){
        return AUDIO_ERROR;
    }*/
    enum audio_com cmd = data[0];
    switch (cmd){
        case MIC_INTR: {
            //MICBIASM: mode select
            if(ALC->micbias != data[1]){
                reg_write(NAU8822_REG_POWER_MANAGEMENT_4, 4, 1, data[1]);
                ALC->micbias = data[1];
            };

            ALC->dac_automt = data[2];
            struct alc_control_pga pga;
            uint8_t s = sizeof(uint8_t)*12;
                memcpy(&pga, data+3, s);
            struct block_volume_set ipavol;
            uint8_t s2 = sizeof(uint8_t)*6;
                memcpy(&ipavol, data+s, s2);
            struct adc_boost abst;
            uint8_t s3 = sizeof(uint8_t)*6;
                memcpy(&abst, data+s2, s3);
            
            ALC->abst = abst;
            ALC->pga = pga;
            ALC->vol.input_pga_vol = ipavol;
            return microfone_interface();
        } break;
        case INP_MIXER: {
            memcpy(ALC->mix.aux_en,      data+1,   sizeof(uint8_t)*2);
            memcpy(ALC->mix.aux_gain,    data+3, sizeof(uint8_t)*2);
            memcpy(ALC->mix.adc_mx_en,   data+5, sizeof(uint8_t)*2);
            memcpy(ALC->mix.adc_mx_gain, data+7, sizeof(uint8_t)*2);
            memcpy(ALC->mix.dac_mx_en,   data+9, sizeof(uint8_t)*2);
            return input_mixer();
        } break;
        case OUTP_MIXER: {
            memcpy(ALC->mix.rspk_mix, data, sizeof(uint8_t)*4);
            memcpy(ALC->mix.aux1_mix, data+4, sizeof(uint8_t)*7);
            memcpy(ALC->mix.aux2_mix, data+11, sizeof(uint8_t)*5);
            return output_mixer();
        } break;
        case AUD_VOL: {
            //data = {mode, upbit, gain1, gain2, mute1, mute2, zc1, zc2};
            uint8_t mode = data[1];
            struct block_volume_set bvol;
            bvol.upbit = data[2];
            for(uint8_t i=0;i<2;i++){
                bvol.ch_gain[i] = data[i+3];
                bvol.ch_mute[i] = data[i+5];
                bvol.ch_zc[i] = data[i+7];
            }
            //assign
            if(mode == VOL_DAC)     
                ALC->vol.dac_vol = bvol;
            else if(mode == VOL_ADC)
                ALC->vol.adc_vol = bvol;
            else if(mode == VOL_HP)
                ALC->vol.hp_vol = bvol;
            else if(mode == VOL_SPK) 
                ALC->vol.spk_vol = bvol;
            return audio_volume(mode);
        } break;
        case OUTP_DRV:{
            memcpy(ALC->mix.out_mix, data, sizeof(uint8_t)*7);
            return output_driver();
        } break;
        case AUD_FILTER:{
            ALC->alctrl. en = data[1];       //Automatic Level Control function control bits 0: rf, 1:r, 2:l, 3: both rl
            ALC->alctrl. mx_gain = data[2];  //Set minimum gain value limit for PGA volume setting changes under ALC control
            ALC->alctrl. mn_gain = data[3];  //7-0, = 111 = +35.25dB (default), -6.75dB
            ALC->alctrl. ht = data[4];       //Hold time before ALC automated gain increase
            ALC->alctrl. sl_level = data[5]; //ALC target level at ADC output pg 82
            ALC->alctrl. mode = data[6];     //Limiter Mode operation
            ALC->alctrl. dcy_time = data[7]; //ALC decay time duration  page 83
            ALC->alctrl. atk_time = data[8]; //Attack time refers to how quickly a system responds to an
            ALC->alctrl. ngate_en = data[9]; //ALC noise gate en
            ALC->alctrl. ngate_th = data[10]; //ALC noise gate threshold level
            memcpy(ALC->filter.threeD_th, data+10, sizeof(uint16_t));
            ALC->filter.hpf_en = data[13];
            ALC->filter.hpf_am = data[14];
            ALC->filter.hpf = data[15];
            ALC->filter.notch_en = data[16];
            memcpy(ALC->filter.notch_cu, data+16, sizeof(uint8_t)*4);
            memcpy(ALC->filter.notch_ca, data+20, sizeof(uint8_t)*4);
            mau88c22_EQ eq;
            uint8_t s = 0;
            for(uint8_t i=0;i<5;i++){
                s = (i)+21;
                eq.eq_bw = data[s];
                eq.eq_em = data[s+4];
                eq.eq_cf = data[s+8];
                eq.eq_gc = data[s+12];
            }
            return audio_filter();
        } break;
    }
}

uint8_t audio_init(alc_control_t *ALC_M){
    /*
        write:
            ALC->pll->fs: enum alc_freq, 96-192-> pllon: disable
            ALC->pll->pllon: 1: enable
            ALC->pll.freq_in: MCKL, x Mhz
            ALC->adc_invert: 0:dis
            ALC->dac_invert: 0: dis
            ALC->dac_automt: 0, 1: enable
        read:
            ALC.alc->div_id: 0x01A
            ALC->pll.pll_frac
            ALC->pll.freq_out

        not_use:
            //Sample Rate in kHz (FS) 7(SMPLR) = 101 or 100 R7(SMPLR) = 011 or 010 R7(SMPLR) = 001 or 000
    */
    ALC = ALC_M;
    uint8_t int_status = codec_init(ALC);//MCLK 12mhz, 48khz
    if(int_status == AUDIO_OK){
        /**
            device_revision();
            device_id();
        dac_dither( mod_dit,  analog_dit);
        //alc_enhancements1( tblsel,  pksel,  ngsel,  gainL); //tblsel,  pksel,  ngsel
        //alc_enhancements2( pklimena,  gainR); // pklimena
        //khz192_sampling( adcb_over,  pll49mout,  dac_osr32,  adc_osr32);
        misc_controls(uint16_t data);
        tie_off_overrides(uint16_t data);
        power_tie_off_ctrl(uint16_t data);
        p2p_detector_read(uint16_t data);
        peak_detector_read(uint16_t data);
        control_and_status( amutctrl,  hvdet,  nsgate);
        output_tie_off_control(uint16_t data); 

        gac_compandig
            Companding, azaltılmış veri biti ile sinyal-gürültü oranlarını optimize etmek için dijital iletişim sistemlerinde kullanılır.
            
         * 
         * **/
    }
    return AUDIO_OK;
}

uint8_t microfone_interface(){
    /*
        *****************
        ALC->micbias,
            PM4: MICBIASM
            Normal Mode(default)   Low Noise Mode
                00 = 0.9x       00 = 0.85x
                01 = 0.65x      01 = 0.60x
                10 = 0.75x      10 = 0.70x
                11 = 0.50x      11 = 0.50x
        ALC->dac_automt: (0-1: mute),
        ALC->pga
            uint8_t mic_n_pga[2];//1: negative connect: (1)
            uint8_t mic_p_pga[2];//1: positive connect: (1)
            uint8_t lin_pga[2];//1 lin connect: (1)

            uint8_t ch_pga[2];//(0-3f), Real time readout alc channel PGA
            uint8_t pga_gain[2];//(0-63), -12db-35.25db
            uint8_t ch_pga_mute[2];//(0-1:mute)
        *****************
        ALC->vol.input_pga_vol
            uint8_t upbit;//volume update bit
            uint8_t ch_zc[2];//zero cross detection 1: on
            uint8_t ch_gain[2];//0-3f, -12db, +35.25db
            uint8_t ch_mute[2];
        *****************    
        ALC->abst
            uint8_t ch_bst_en[2];//0: no gain, 1: +20db
            uint8_t ch_bst_gain[2];//0: disconnect,  1-7; -12db, +6db
            uint8_t aux_bst_gain[2];//0: disconnect,  1-7; -12db, +6db
        *****************
    */
    alc_control(ALC);
    input_gain(ALC);
    adc_mix_boost(ALC);
    return AUDIO_OK;
}

uint8_t input_mixer(){
    /*
        uint8_t aux_en[2];// (0, 1:enable)
        uint8_t aux_gain[2];//Gain value between AUXIN  (0-7) -15db, +6db
        uint8_t adc_mx_en[2];//xAUXIN input to xMAIN x output mixer path control   (0, 1:enable)
        uint8_t adc_mx_gain[2];//(0-7) -15db, +6db
        uint8_t dac_mx_en[2];//xDAC output to xMIX x output mixer path control
    */
    main_mixer(ALC);
    return AUDIO_OK;
}

uint8_t output_mixer(){
    /*
        uint8_t rspk_mix[4];
            RMIXMUT: Mutes the RMIX speaker signal gain stage output in the right speaker submixer
                0 = gain stage output enabled
                1 = gain stage output muted
            RSUBBYP: Right speaker submixer bypass control
                0 = right speaker amplifier directly connected to RMIX speaker signal gain stage
                1 = right speaker amplifier connected to submixer output (inverts RMIX for BTL)
            RAUXRSUBG: RAUXIN to Right Speaker Submixer input gain control
                0 = -15dB (default)(-3db)
                7 = +6.0dB
            RAUXSMUT: RAUXIN to Right Speaker Submixer mute control: 0: muted
        
        uint8_t aux1_mix[7];
            (1: connected)
            AUXOUT1MT: (1: mute) AUXOUT1: output mute control (1: mute)
            AUX1HALF: 6dB attenuation enable (0: normal: 1: 6.0dB)
            LMIXAUX1: Left LMAIN MIXER output to AUX1 MIXER input path control
            LDACAUX1: Left DAC output to AUX1 MIXER input path control
            RADCAUX1: Right RADC Mix/Boost output RINMIX path control to AUX1 MIXER input
            RMIXAUX1: Right RMIX output to AUX1 MIXER input path control
            RDACAUX1: Right DAC output to AUX1 MIXER input path control

        uint8_t aux2_mix[5];
            AUXOUT2MT: AUXOUT2 output mute control: 1 mute
            AUX1MIX>2: AUX1 Mixer output to AUX2 MIXER input path control : 1
            LADCAUX2: Left LADC Mix/Boost output LINMIX path control to AUX2 MIXER input: 1
            LMIXAUX2: Left LMAIN MIXER output to AUX2 MIXER input path control: 1
            LDACAUX2: Left DAC output to AUX2 MIXER input path control: 1

    */
    spk_submix(ALC);
    aux_submix(ALC);
    return AUDIO_OK;
}

uint8_t audio_volume(enum vol_mode mode){
    /*
        alc->vol.(mode)_vol:
            uint8_t upbit;//volume update bit
            uint8_t ch_zc[2];//zero cross detection 1: on
            uint8_t ch_gain[2];//0-3f, -12db, +35.25db
            uint8_t ch_mute[2];

        channel_volume(ALC, VOL_DAC);//for dac 
        channel_volume(ALC, VOL_ADC);//for dac 
        channel_volume(ALC, VOL_HP);//for dac 
        channel_volume(ALC, VOL_SPK);//for dac 
    */

    return channel_volume(ALC, mode);
}

uint8_t output_driver(){
    /*
        ALC.mix.out_mix[7]

        ldacrmx: Left DAC output to RMIX (0-1: connect),   
        rdaclmx: Right DAC output to LMIX left,   
        aux1bst: AUXOUT1 gain boost control (0: -1db, 1: 1.5db),
            aux2bst,  
            spkbst,  
        tsen: Thermal shutdown enable (1: enable),  
        aout1mp: Output resistance control option (0: 1k, 1: 30k)

    */
    return output_control(ALC);
}

uint8_t audio_filter(){
    /*
    ALC->alctrl: 
        alc_control_filter: 
            uint8_t en;       //Automatic Level Control function control bits 0: rf, 1:r, 2:l, 3: both rl
            uint8_t mx_gain;  //Set minimum gain value limit for PGA volume setting changes under ALC control
                Range: -6.75dB to +35.25dB @ 6dB
            uint8_t mn_gain;  //7-0, = 111 = +35.25dB (default), -6.75dB
                Range: -6.75dB to +35.25dB @ 6dB
            uint8_t ht;       //Hold time before ALC automated gain increase
                Range: 0ms to 1024ms at 1010 and: 0.75dB steps
            uint8_t sl_level; //ALC target level at ADC output pg 82
                Range: -22.5dB to -1.5dBFS @ 1.5dB
            uint8_t mode;     //Limiter Mode operation
            uint8_t dcy_time; //ALC decay time duration  page 83
                ALCM = 0 - Range: 500μs to 512ms
                ALCM = 1 - Range: 125μs to 128ms
            uint8_t atk_time; //Attack time refers to how quickly a system responds to an
                ALCM=0 - Range: 125μs to 128ms
                ALCM=1 - Range: 31μs to 32ms  increasing volume level that is greater than some defined threshold
            uint8_t ngate_en; //ALC noise gate en
                1: enable
            uint8_t ngate_th; //ALC noise gate threshold level
                0 = -39dB (default)
                7 = -81dB

    alc->filter
        three_d_efect:
            uint16_t alc->filter.threeD_th:
                0  = 0.0% effect (disabled, default)
                15 = 100% effect (maximum effect)

        hpf_filter:
            uint8_t hpf_en;//high pas filter en
            uint8_t hpf_am;//High pass filter mode selection 0: normal
            uint8_t hpf;//changed sample rate. see page 30; 0-7 #Cut-off Frequencies

        notch_filter: //page 81
            uint8_t notch_en;//(band_pass)^-1 en page 31
            uint8_t notch_cu[4];//Update bit feature for simultaneous change of all notch filter parameters
            uint8_t notch_ca[4];//Notch filter A0 coefficient most significant bits. See text and table for details.
                Notch filter A0 coefficient most significant bits. See text and table for details.
                Notch filter A0 coefficient least significant bits. See text and table for details
                Notch filter A1 coefficient most significant bits. See text and table for details
                Notch filter A1 coefficient least significant bits. See text and table for details
    
    band_eq:  //PAGE 34
        Table 8: Equalizer Center/Cutoff Frequencies
        Table 9: Equalizer Gains: (0-24), +12db, -12db
        alc->eq
            eq1_low_cutoff(&alc->eq[0]); 
            	eq->eq_em);//0: adc stream on, 1: dac stream on (def: dac)
                eq->eq_cf);//0:80hz, 1:105hz, 2:135hz, 3:175hz
                eq->eq_gc);//0:+12db, 24: -12db
            eq2_peak_1(&alc->eq[1]);
                eq1->eq_bw);//0: narrow, 1: wide band band2
                eq1->eq_cf);//0: 230hz, 1: 300hz, 2: 385hz, 3:500hz
                eq1->eq_gc);//0:+12db, 24: -12db
            eq2_peak_2(&alc->eq[2]);
                eq2->eq_bw);//0: narrow, 1: wide band band2
                eq2->eq_cf);//0: 650hz, 1: 850hz, 2: 1.1khz, 3:1.4khz
                eq2->eq_gc);//0:+12db, 24: -12db
            eq2_peak_3(&alc->eq[3]);
                 eq3->eq_bw);//0: narrow, 1: wide band band2
                 eq3->eq_cf);//0: 1.8khz, 1: 2.4khz, 2: 3.2khz, 3:4.1khz
                 eq3->eq_gc);//0:+12db, 24: -12db
            return eq5_high_cutoff(&alc->eq[4]);
                eq5->eq_cf);//0: 5.3khz, 1: 6.9khz, 2: 9khz, 3:11.7khz
                eq5->eq_cf);//0:+12db, 24: -12db  
    */

    alc_control_filter(ALC);
    three_d_efect(ALC);
    hpf_filter(ALC);
    notch_filter(ALC);
    band_eq(ALC);
    return AUDIO_OK;
}
