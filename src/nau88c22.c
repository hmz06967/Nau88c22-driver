#include "nau88c22.h"

/*edit hw interface*/

uint8_t write_bus(reg, data){
	return NAU_ERROR_WRITE;
}

uint8_t read_bus(reg, data){
	return NAU_ERROR_READ;
}

void delay(uint32_t ms){

}

/*******************/

uint8_t nau8822_init(uint8_t *devid){

	if(device_id() == 0x01A){
		return power_on();
	}

	*devid = 0x01A;
	return NAU_ERROR_ID;
}

uint16_t change_reg_data(uint16_t data,  uint8_t bit, uint8_t bs, uint16_t value){
	uint8_t mask = (bs<<bit);
	data &= ~mask;//delete old value
	data |= ((value<<bit) & mask);//write new value*/
	return data;
}
uint8_t reg_write(uint8_t reg, uint8_t bit, uint8_t bs, uint16_t value){
	uint16_t data = 0;
	if(reg_read(reg, &data) != NAU_OK){
		printf("Error NAU88C22 Codec write reg: 0x%X\r\n", reg);
		return NAU_ERROR;
	}
	data = change_reg_data(data, bit, bs, value);
	return write_bus(reg, data);
}
uint8_t reg_read(uint8_t reg, uint16_t *data){
	//read
}

/* power */
uint8_t power_on(){
	uint8_t st = NAU_OK;
	write_bus(NAU8822_REG_RESET, 0xff); //reset
	delay(1);

	//pwr
	uint16_t pm1 = change_reg_data(REG_FULL_VALUE, 2, 1, 0); //IOBUFEN: 0 tie-off buffer used in non-boost mode (-1.0x gain)
	st = write_bus(NAU8822_REG_POWER_MANAGEMENT_1, pm1);
	
	uint16_t pm2 = change_reg_data(REG_FULL_VALUE, 6, 1, 0);//SLEEP: 0, normal
	st &= write_bus(NAU8822_REG_POWER_MANAGEMENT_2, pm2);
	st &= write_bus(NAU8822_REG_POWER_MANAGEMENT_3, 0x18F);
	st &= write_bus(NAU8822_REG_POWER_MANAGEMENT_4, 0);
	return st; 
}
uint8_t clock_pll(uint8_t pllen){
	return reg_write(NAU8822_REG_POWER_MANAGEMENT_1, 5, 1, (pllen & 0x1));//power registerinin  5. bitini pllon ile değiştirir
}
/* audio */
uint8_t gac_audio_interface (){
	uint16_t data = 0x70;//0,0,32bit,standart_i auid_2s,lphs,stereo
	return write_bus(NAU8822_REG_AUDIO_INTERFACE, data);
}
uint8_t gac_companding(uint8_t cmb8, uint8_t daccm, uint8_t addccm, uint8_t addap){
 	uint8_t reg = NAU8822_REG_COMPANDING_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 5, 1, cmb8);
	data = change_reg_data(data, 3, 2, daccm);
	data = change_reg_data(data, 1, 2, addccm);//ADDCM, 0:off, 1:, 2: u-law, 3:a-law
	data = change_reg_data(data, 0, 1, addap);//ADDAP
	return write_bus(reg, data);
}
uint8_t gac_clock_control1(uint8_t clkm, uint8_t mclksel, uint8_t bclksel, uint8_t clkioen){
 	uint8_t reg = NAU8822_REG_CLOCKING;
	uint16_t data = 0x140;

	data = change_reg_data(data, 8, 1, clkm);//0: master clock, 1: internal plls osilator
	data = change_reg_data(data, 5, 3, mclksel);//0, 7 divide 1,1.5,2,3,4,6,8,12
	data = change_reg_data(data, 2, 3, bclksel);//0, 5 divide 1,2,4,8,16,32
	data = change_reg_data(data, 0, 1, clkioen);//fs: 0: bclk input, 1: bclk are internal clk
	return write_bus(reg, data);
}
uint8_t gac_clock_control2(uint8_t spi4en, uint8_t smplr, uint8_t sclken){
 	uint8_t reg = NAU8822_REG_ADDITIONAL_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, spi4en);//0: master clock, 1: internal plls osilator
	data = change_reg_data(data, 1, 3, smplr);//0, 5 divide 1,2,4,8,16,32
	data = change_reg_data(data, 0, 1, sclken);//fs: 0: bclk input, 1: bclk are internal clk
	return write_bus(reg, data); 
}
uint8_t gac_gpio(uint8_t gpio1pll, uint8_t gpio1pl, uint8_t gpio1sel){
 	uint8_t reg = NAU8822_REG_GPIO_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 5, 2, gpio1pll);//4 wire spi en
	data = change_reg_data(data, 4, 1, gpio1pl);//0-5, 48khz, 32,24,16,12,8
	data = change_reg_data(data, 0, 3, gpio1sel);//1:enable
	return write_bus(reg, data);
}
uint8_t gac_jack_detect(uint8_t en, uint8_t den, uint8_t dio){
 	uint8_t reg = NAU8822_REG_JACK_DETECT_CONTROL_1;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 2, en);
	data = change_reg_data(data, 6, 1, den); //Jack detection feature enable 0: disable
	data = change_reg_data(data, 0, 4, dio);
	return write_bus(reg, data);
}
uint8_t gac_dac_control(uint8_t sfmt, uint8_t dacos,uint8_t automt,uint8_t rdacpl, uint8_t ldacpl){
	uint8_t reg = NAU8822_REG_DAC_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 6, 1, sfmt);//SOFTMT: 1, Softmute enable
	data = change_reg_data(data, 3, 1, dacos);//DACOS: 1,  128x oversampling
	data = change_reg_data(data, 2, 1, automt);//AUTOMT: 1,  automute enable
	data = change_reg_data(data, 1, 1, rdacpl);//RDACPL: 0,  normal polarity
	data = change_reg_data(data, 0, 1, ldacpl);//lDACPL: 0,  normal polarity
	return write_bus(reg, data);
}
uint8_t gac_left_dac_volume(struct block_volume_set vol){
	/*DAC left digital volume control (0dB default attenuation value). Expressed as an
	attenuation value in 0.5dB steps as follows:
	0000 0000 = digital mute condition
	0000 0001 = -127.0dB (highly attenuated)
	0000 0010 = -126.5dB attenuation
	- all intermediate 0.5 step values through maximum –
	1111 1110 = -0.5dB attenuation
	1111 1111 = 0.0dB attenuation (no attenuation)*/
	uint8_t reg = NAU8822_REG_LEFT_DAC_DIGITAL_VOLUME;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, vol.upbit);//LDACVU: 1, DAC volume update bit feature
	data = change_reg_data(data, 0, 8, vol.ch_gain[0]);//LDACGAIN: 0xff,  0.0dB attenuation (no attenuation
	return write_bus(reg, data);
}
uint8_t gac_right_dac_volume(struct block_volume_set vol){//upbit active volume
	uint8_t reg = NAU8822_REG_RIGHT_DAC_DIGITAL_VOLUME;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, vol.upbit);//LDACVU: 1, DAC volume update bit feature
	data = change_reg_data(data, 0, 8, vol.ch_gain[1]);//LDACGAIN: 0xff,  0.0dB attenuation (no attenuation
	return write_bus(reg, data);
}
uint8_t gac_jack_detect_2(uint8_t jckd0en1, uint8_t jckd0en0){
 	uint8_t reg = NAU8822_REG_JACK_DETECT_CONTROL_2;
	uint16_t data = 0;

	data = change_reg_data(data, 7, 4, jckd0en1);//
	data = change_reg_data(data, 0, 4, jckd0en0);//
	return write_bus(reg, data);
}
uint8_t gac_adc_control(uint8_t hpfen, uint8_t hpfam, uint8_t  hpf, uint8_t adcos, uint8_t rdacpl, uint8_t ldacpl){
	uint8_t reg = NAU8822_REG_ADC_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, hpfen);//HPFEN: 1, High pass filter enable
	data = change_reg_data(data, 7, 1, hpfam);//DACOS: 1,  128x oversampling
	data = change_reg_data(data, 4, 3, hpf);//AUTOMT: 1,  
	data = change_reg_data(data, 3, 1, adcos);//RDACPL: 0,  normal polarity
	data = change_reg_data(data, 1, 1, rdacpl);//RDACPL: 0,  normal polarity
	data = change_reg_data(data, 0, 1, ldacpl);//RDACPL: 0,  normal polarity
	return write_bus(reg, data);
}
uint8_t gac_left_adc_volume(struct block_volume_set vol){
	uint8_t reg = NAU8822_REG_LEFT_ADC_DIGITAL_VOLUME;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, vol.upbit);//LADCVU: 1, DAC volume update bit feature
	data = change_reg_data(data, 0, 8, vol.ch_gain[0]);//LADCGAIN: 0xff,  0.0dB attenuation (no attenuation
	return write_bus(reg, data);
}
uint8_t gac_right_adc_volume(struct block_volume_set vol){
	uint8_t reg = NAU8822_REG_RIGHT_ADC_DIGITAL_VOLUME;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, vol.upbit);//
	data = change_reg_data(data, 0, 8, vol.ch_gain[1]);//
	return write_bus(reg, data);
}

//EQ
uint8_t eq1_low_cutoff(mau88c22_EQ *eq){
 	uint8_t reg = NAU8822_REG_EQ1;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, eq->eq_em);//0: adc stream on, 1: dac stream on (def: dac)
	data = change_reg_data(data, 5, 2, eq->eq_cf);//0:80hz, 1:105hz, 2:135hz, 3:175hz
	data = change_reg_data(data, 0, 5, eq->eq_gc);//0:+12db, 24: -12db
	return write_bus(reg, data);
}
uint8_t eq2_peak_1(mau88c22_EQ *eq1){
 	uint8_t reg = NAU8822_REG_EQ2;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, eq1->eq_bw);//0: narrow, 1: wide band band2
	data = change_reg_data(data, 5, 2, eq1->eq_cf);//0: 230hz, 1: 300hz, 2: 385hz, 3:500hz
	data = change_reg_data(data, 0, 5, eq1->eq_gc);//0:+12db, 24: -12db
	return write_bus(reg, data);
}
uint8_t eq3_peak_2(mau88c22_EQ *eq2){
 	uint8_t reg = NAU8822_REG_EQ3;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, eq2->eq_bw);//0: narrow, 1: wide band band2
	data = change_reg_data(data, 5, 2, eq2->eq_cf);//0: 650hz, 1: 850hz, 2: 1.1khz, 3:1.4khz
	data = change_reg_data(data, 0, 5, eq2->eq_gc);//0:+12db, 24: -12db
	return write_bus(reg, data);
}
uint8_t eq4_peak3(mau88c22_EQ *eq3){
 	uint8_t reg = NAU8822_REG_EQ4;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, eq3->eq_bw);//0: narrow, 1: wide band band2
	data = change_reg_data(data, 5, 2, eq3->eq_cf);//0: 1.8khz, 1: 2.4khz, 2: 3.2khz, 3:4.1khz
	data = change_reg_data(data, 0, 5, eq3->eq_gc);//0:+12db, 24: -12db
	return write_bus(reg, data);
}
uint8_t eq5_high_cutoff(mau88c22_EQ *eq5){
 	uint8_t reg = NAU8822_REG_EQ5;
	uint16_t data = 0;

	data = change_reg_data(data, 5, 2, eq5->eq_cf);//0: 5.3khz, 1: 6.9khz, 2: 9khz, 3:11.7khz
	data = change_reg_data(data, 0, 5, eq5->eq_cf);//0:+12db, 24: -12db
	return write_bus(reg, data);
}
uint8_t read_eq_array(mau88c22_EQ eq[5]){
	uint8_t data = 0, st= NAU_OK;
	read_bus(NAU8822_REG_EQ1, &data);
	eq[0].eq_em = (data & (1<<8));
	eq[0].eq_cf = (data & (2<<5));
	eq[0].eq_gc = (data & (5));
	for (size_t i = 0; i < 4; i++){
		st = read_bus(NAU8822_REG_EQ1 + i, &data);
		eq[i+1].eq_bw= (data & (1<<8));
		eq[i+1].eq_cf = (data & (2<<5));
		eq[i+1].eq_gc = (data & (5));
	}
	return st;
}
/**************************************/

//dac_limiter
uint8_t dac_limiter_1(uint8_t daclimen, uint8_t daclimdcy,uint8_t daclimatk){
 	uint8_t reg = NAU8822_REG_DAC_LIMITER_1;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, daclimen);//enable
	data = change_reg_data(data, 4, 4, daclimdcy);//def 0x3: 4.36ms DAC limiter decay time
	data = change_reg_data(data, 0, 4, daclimatk);//def 0x2: 272us DAC limiter attack time.
	return write_bus(reg, data);
}
uint8_t dac_limiter_2(uint8_t daclimthl, uint8_t daclimbst){
 	uint8_t reg = NAU8822_REG_DAC_LIMITER_2;
	uint16_t data = 0;

	data = change_reg_data(data, 4, 3, daclimthl);//dac limiter threshold 0:-1db, 5: -6db
	data = change_reg_data(data, 0, 4, daclimbst);//def, 0: 0db, 12: +12db DAC limiter maximum automatic gain boost in limiter mode
	return write_bus(reg, data);
}

//notch_filter
uint8_t notch_filter_1(uint8_t nfcu, uint8_t nfcen,uint8_t nfca0){
 	uint8_t reg = NAU8822_REG_NOTCH_FILTER_1;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, nfcu);//update bit
	data = change_reg_data(data, 7, 1, nfcen);//enb 1
	data = change_reg_data(data, 0, 7, nfca0);//Notch filter A0 coefficient
	return write_bus(reg, data);
}
uint8_t notch_filter_2(uint8_t nfcu2, int8_t nfcao){
 	uint8_t reg = NAU8822_REG_NOTCH_FILTER_2;
	uint16_t data = 0;

	data = change_reg_data(nfcu2, 8, 1, nfcu2);//update bit
	data = change_reg_data(data, 0, 7, nfcao);//
	return write_bus(reg, data);
}
uint8_t notch_filter_3(uint8_t nfcu3, int8_t nfca1){
 	uint8_t reg = NAU8822_REG_NOTCH_FILTER_3;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, nfcu3);//
	data = change_reg_data(data, 0, 7, nfca1);//
	return write_bus(reg, data);
}
uint8_t notch_filter_4(uint8_t nfcu4, int8_t nfca1){
 	uint8_t reg = NAU8822_REG_NOTCH_FILTER_4;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, nfcu4);//
	data = change_reg_data(data, 0, 7, nfca1);//
	return write_bus(reg, data);
}

//alc_and_noise_gate_control
uint8_t alc_control_1(uint8_t en, uint8_t xgain, uint8_t ngain){
 	uint8_t reg = NAU8822_REG_ALC_CONTROL_1;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 2, en);//
	data = change_reg_data(data, 3, 3, xgain);//
	data = change_reg_data(data, 0, 3, ngain);//
	return write_bus(reg, data);
}
uint8_t alc_control_2(uint8_t ht, uint8_t sl){
 	uint8_t reg = NAU8822_REG_ALC_CONTROL_2;
	uint16_t data = 0;

	data = change_reg_data(data, 4, 4, ht);//
	data = change_reg_data(data, 0, 4, sl);//
	return write_bus(reg, data);
}
uint8_t alc_control_3(uint8_t cm, uint8_t dcy, uint8_t atk){
 	uint8_t reg = NAU8822_REG_ALC_CONTROL_3;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, cm);//
	data = change_reg_data(data, 4, 4, dcy);//
	data = change_reg_data(data, 0, 4, atk);//
	return write_bus(reg, data);
}
uint8_t noise_gate(uint8_t en, uint8_t th){
 	uint8_t reg = NAU8822_REG_NOISE_GATE;
	uint16_t data = 0;

	data = change_reg_data(data, 3, 1, en);//1: enable
	data = change_reg_data(data, 0, 3, th);//-39db---81db
	return write_bus(reg, data);
}

//phase_locked_loop
uint8_t pll_nn(uint8_t pllmclk, uint16_t plln){
 	uint8_t reg = NAU8822_REG_PLL_N;
	uint16_t data = 0;

	data = change_reg_data(data, 4, 1, pllmclk);//
	data = change_reg_data(data, 0, 4, plln);//
	return write_bus(reg, data);
}
uint8_t pll_k_1(uint16_t pllk){
 	uint8_t reg = NAU8822_REG_PLL_K1;
	uint16_t data = 0;

	data = change_reg_data(data, 0, 6, pllk);//
	return write_bus(reg, data);
}
uint8_t pll_k_2(uint16_t pllk){
 	uint8_t reg = NAU8822_REG_PLL_K2;
	uint16_t data = 0;

	data = change_reg_data(data, 0, 9, pllk);//
	return write_bus(reg, data);
}
uint8_t pll_k_3(uint16_t pllk){
 	uint8_t reg = NAU8822_REG_PLL_K3;
	uint16_t data = 0;

	data = change_reg_data(data, 0, 9, pllk);//
	return write_bus(reg, data);
}


//miscellaneous
uint8_t misc_3d_control(uint8_t depth){
 	uint8_t reg = NAU8822_REG_3D_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 0, 4, depth);//0:0effect, 15: 100% 3d
	return write_bus(reg, data);
}
uint8_t misc_right_speaker_submix(uint8_t rmixmute, uint8_t rsubbyp, uint8_t rauxrsubg, uint8_t rauxsmut){
 	uint8_t reg = NAU8822_REG_RIGHT_SPEAKER_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 5, 1, rmixmute);//
	data = change_reg_data(data, 4, 1, rsubbyp);//
	data = change_reg_data(data, 1, 3, rauxrsubg);//
	data = change_reg_data(data, 0, 1, rauxsmut);//
	return write_bus(reg, data);
}
uint8_t misc_input_control(uint8_t micbias, struct alc_control_pga pga){
 	uint8_t reg = NAU8822_REG_INPUT_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 7, 2, micbias);//
	data = change_reg_data(data, 6, 1, pga.lin_pga[0]);//
	data = change_reg_data(data, 5, 1, pga.mic_n_pga[0]);//
	data = change_reg_data(data, 4, 1, pga.mic_p_pga[0]);//
	data = change_reg_data(data, 2, 1, pga.lin_pga[1]);//
	data = change_reg_data(data, 1, 1, pga.mic_n_pga[1]);//
	data = change_reg_data(data, 0, 1, pga.mic_p_pga[1]);//
	return write_bus(reg, data);
}
uint8_t misc_left_input_pga_gain(struct block_volume_set vol){
 	uint8_t reg = NAU8822_REG_LEFT_INP_PGA_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, vol.upbit);//
	data = change_reg_data(data, 7, 1, vol.ch_zc[0]);//
	data = change_reg_data(data, 6, 1, vol.ch_mute[0]);//
	data = change_reg_data(data, 0, 6, vol.ch_gain[0]);//
	return write_bus(reg, data);
}
uint8_t misc_right_input_pga_gain(struct block_volume_set vol){
 	uint8_t reg = NAU8822_REG_RIGHT_INP_PGA_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, vol.upbit);//
	data = change_reg_data(data, 7, 1, vol.ch_zc[1]);//
	data = change_reg_data(data, 6, 1, vol.ch_mute[1]);//
	data = change_reg_data(data, 0, 6, vol.ch_gain[1]);//
	return write_bus(reg, data);
}
uint8_t misc_left_adc_boost(struct adc_boost  abst){
 	uint8_t reg = NAU8822_REG_LEFT_ADC_BOOST_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, abst.ch_bst_en);//
	data = change_reg_data(data, 4, 3, abst.ch_bst_gain);//
	data = change_reg_data(data, 0, 3, abst.aux_bst_gain);//
	return write_bus(reg, data);
}
uint8_t misc_right_adc_boost(struct adc_boost  abst){
 	uint8_t reg = NAU8822_REG_RIGHT_ADC_BOOST_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, abst.ch_bst_en);//
	data = change_reg_data(data, 4, 3, abst.ch_bst_gain);//
	data = change_reg_data(data, 0, 3, abst.aux_bst_gain);//
	return write_bus(reg, data);
}
uint8_t misc_output_control(	
	uint8_t ldacrmx, 
	uint8_t rdaclmx,  uint8_t aux1bst,
	uint8_t aux2bst, uint8_t spkbst,
	uint8_t tsen, uint8_t aout1mp
){
 	uint8_t reg = NAU8822_REG_OUTPUT_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 6, 1, ldacrmx);//
	data = change_reg_data(data, 5, 1, rdaclmx);//
	data = change_reg_data(data, 4, 1, aux1bst);//
	data = change_reg_data(data, 3, 1, aux2bst);//
	data = change_reg_data(data, 2, 1, spkbst);//
	data = change_reg_data(data, 1, 1, tsen);//
	data = change_reg_data(data, 0, 1, aout1mp);//
	return write_bus(reg, data);
}
uint8_t misc_left_mixer(uint8_t auxmxgain, uint8_t auxlmx, uint8_t bypmxgain, uint8_t bypmx, uint8_t dacmx){
 	uint8_t reg = NAU8822_REG_LEFT_MIXER_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 6, 3, auxmxgain);//
	data = change_reg_data(data, 5, 1, auxlmx);//
	data = change_reg_data(data, 2, 3, bypmxgain);//
	data = change_reg_data(data, 1, 1, bypmx);//
	data = change_reg_data(data, 0, 1, dacmx);//
	return write_bus(reg, data);
}
uint8_t misc_right_mixer(uint8_t auxmxgain, uint8_t auxlmx, uint8_t bypmxgain, uint8_t bypmx, uint8_t dacmx){
 	uint8_t reg = NAU8822_REG_RIGHT_MIXER_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 6, 3, auxmxgain);//
	data = change_reg_data(data, 5, 1, auxlmx);//
	data = change_reg_data(data, 2, 3, bypmxgain);//
	data = change_reg_data(data, 1, 1, bypmx);//
	data = change_reg_data(data, 0, 1, dacmx);//
	return write_bus(reg, data);
}
uint8_t misc_lhp_volume(struct block_volume_set vol ){
 	uint8_t reg = NAU8822_REG_LHP_VOLUME;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, vol.upbit);//
	data = change_reg_data(data, 7, 1, vol.ch_zc[0]);//
	data = change_reg_data(data, 6, 1, vol.ch_zc[0]);//
	data = change_reg_data(data, 0, 6, vol.ch_zc[0]);//
	return write_bus(reg, data);
}
uint8_t misc_rhp_volume(struct block_volume_set vol){
 	uint8_t reg = NAU8822_REG_RHP_VOLUME;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, vol.upbit);//
	data = change_reg_data(data, 7, 1, vol.ch_zc[1]);//
	data = change_reg_data(data, 6, 1, vol.ch_zc[1]);//
	data = change_reg_data(data, 0, 6, vol.ch_zc[1]);//
	return write_bus(reg, data);
}
uint8_t misc_lspkout_volume(struct block_volume_set vol){
 	uint8_t reg = NAU8822_REG_LSPKOUT_VOLUME;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, vol.upbit);//
	data = change_reg_data(data, 7, 1, vol.ch_zc[0]);//
	data = change_reg_data(data, 6, 1, vol.ch_zc[0]);//
	data = change_reg_data(data, 0, 6, vol.ch_zc[0]);//
	return write_bus(reg, data);
}
uint8_t misc_rspkout_volume(struct block_volume_set vol){
 	uint8_t reg = NAU8822_REG_RSPKOUT_VOLUME;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, vol.upbit);//
	data = change_reg_data(data, 7, 1, vol.ch_zc[1]);//
	data = change_reg_data(data, 6, 1, vol.ch_zc[1]);//
	data = change_reg_data(data, 0, 6, vol.ch_zc[1]);//
	return write_bus(reg, data);
}
uint8_t misc_aux2_mixer(uint8_t auxoutmt, uint8_t aux1mix, uint8_t ladcaux, uint8_t lmixaux, uint8_t ldacaux){
 	uint8_t reg = NAU8822_REG_AUX2_MIXER;
	uint16_t data = 0;

	data = change_reg_data(data, 6, 1, auxoutmt);//
	data = change_reg_data(data, 3, 1, aux1mix);//
	data = change_reg_data(data, 2, 1, ladcaux);//
	data = change_reg_data(data, 1, 1, lmixaux);//
	data = change_reg_data(data, 0, 1, ldacaux);//
	return write_bus(reg, data);
}
uint8_t misc_aux1_mixer(uint8_t auxoutmt, uint8_t aux1half, uint8_t lmixaux1,  uint8_t ldacaux1, uint8_t radcaux, uint8_t rmixaux, uint8_t rdacaux){
 	uint8_t reg = NAU8822_REG_AUX1_MIXER;
	uint16_t data = 0;

	data = change_reg_data(data, 6, 1, auxoutmt);//
	data = change_reg_data(data, 5, 1, aux1half);//
	data = change_reg_data(data, 4, 1, lmixaux1);//
	data = change_reg_data(data, 3, 1, ldacaux1);//
	data = change_reg_data(data, 2, 1, radcaux);//
	data = change_reg_data(data, 1, 1, rmixaux);//
	data = change_reg_data(data, 0, 1, rdacaux);//
	return write_bus(reg, data);
}

//pcm_time_slot_and_adcout_impedance_option_control
uint8_t left_time_slot(uint16_t ltslot){
 	uint8_t reg = NAU8822_REG_LEFT_TIME_SLOT;
	uint16_t data = 0;

	data = change_reg_data(data, 0, 8, ltslot);//
	return write_bus(reg, data);
}
uint8_t misc(uint8_t misc_array[]){
 	uint8_t reg = NAU8822_REG_MISC;
	uint16_t data = 0;

	for(uint8_t i=0;i<9;i++){
		data = change_reg_data(data, i, 1, misc_array[i]);//
	}
	return write_bus(reg, data);
}
uint8_t right_time_slot(uint16_t rtslot){
 	uint8_t reg = NAU8822_REG_RIGHT_TIME_SLOT;
	uint16_t data = 0;

	data = change_reg_data(data, 0, 8, rtslot);//
	return write_bus(reg, data);
}

//silicon_revision_and_device_id
uint8_t device_revision(){
 	uint8_t reg = NAU8822_REG_DEVICE_REVISION;
	uint16_t data = 0;
	read_bus(reg, &data);
	return data & 0x7f;
}
uint8_t device_id(){
 	uint8_t reg = NAU8822_REG_DEVICE_ID;
	uint16_t data = 0;

	read_bus(reg, &data);
	return data;
}
uint8_t dac_dither(uint8_t mod_dit, uint8_t analog_dit){//dac için gürültüyü kaldırmak için rastgele noise ekle
	uint8_t reg = NAU8822_REG_DAC_DITHER;
	uint16_t data = 0;
	data = change_reg_data(data, 4, 5, mod_dit);//0x00 off, 0x11: nominal, 0x1f maks
	data = change_reg_data(data, 0, 4, analog_dit);//0x00 off, 0x04: nominal, 0xf: maks
	return write_bus(reg, data);
}
uint8_t alc_enhancements1(uint8_t tblsel, uint8_t pksel, uint8_t ngsel, uint8_t gainL){
 	uint8_t reg = NAU8822_REG_ALC_ENHANCE_1;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, tblsel);//
	data = change_reg_data(data, 7, 1, pksel);//
	data = change_reg_data(data, 6, 1, ngsel);//
	data = change_reg_data(data, 0, 6, gainL);//
	return write_bus(reg, data);
}
uint8_t alc_enhancements2(uint8_t pklimena, uint8_t gainR){
 	uint8_t reg = NAU8822_REG_ALC_ENHANCE_2;
	uint16_t data = 0;

	data = change_reg_data(data, 8, 1, pklimena);//
	data = change_reg_data(data, 0, 6, gainR);//
	return write_bus(reg, data);
}
uint8_t khz192_sampling(uint8_t adcb_over, uint8_t pll49mout, uint8_t dac_osr32, uint8_t adc_osr32){
 	uint8_t reg = NAU8822_REG_192KHZ_SAMPLING;
	uint16_t data = 0;

	data = change_reg_data(data, 4, 1, adcb_over);//
	data = change_reg_data(data, 2, 1, pll49mout);//
	data = change_reg_data(data, 1, 1, dac_osr32);//
	data = change_reg_data(data, 0, 1, adc_osr32);//
	return write_bus(reg, data);
}
uint8_t misc_controls(uint16_t data){//array
 	uint8_t reg = NAU8822_REG_MISC_CONTROL;
	return write_bus(reg, data);
}
uint8_t tie_off_overrides(uint16_t data){
 	uint8_t reg = NAU8822_REG_INPUT_TIEOFF;
	return write_bus(reg, data);
}
uint8_t power_tie_off_ctrl(uint16_t data){
 	uint8_t reg = NAU8822_REG_INPUT_TIEOFF;
	return write_bus(reg, data);
}
uint8_t p2p_detector_read(uint16_t data){
 	uint8_t reg = NAU8822_REG_INPUT_TIEOFF;
	return write_bus(reg, data);
}
uint8_t peak_detector_read(uint16_t data){
 	uint8_t reg = NAU8822_REG_INPUT_TIEOFF;
	return write_bus(reg, data);
}
uint8_t control_and_status(uint8_t amutctrl, uint8_t hvdet, uint8_t nsgate){
 	uint8_t reg = NAU8822_REG_AUTOMUTE_CONTROL;
	uint16_t data = 0;

	data = change_reg_data(data, 5, 1, amutctrl);//
	data = change_reg_data(data, 4, 1, hvdet);//
	data = change_reg_data(data, 3, 1, nsgate);//
	return write_bus(reg, data);
}
uint8_t output_tie_off_control(uint16_t data){//array
 	uint8_t reg = NAU8822_REG_MISC_CONTROL;
	return write_bus(reg, data);
}
