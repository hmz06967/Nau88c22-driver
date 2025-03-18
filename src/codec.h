#ifndef __CODEC_H__
#define __CODEC_H__

#include <stdint.h>
#include <inttypes.h>

#include "nau88c22.h"

#define BIT_2_24 16777216
#define DISABLE 0
#define ENABLE 1 

/*
	Power
		Software Reset:
		pm1: 0x1FB
		pm2: 0x1BF
		pm3: 0x18F
		pm4:

	General Audio Controls
		Audio Interface:
		Companding:
		Clock Control1:
		Clock Control2:
		GPIO:
		Jack Detect:
		Dac Control:
		Left Dac Volume:
		Right Dac Volume:
		Jack Detect 2:
		Adc Control:
		Left Adc Volume:
		Right Adc Volume:

	Equalizer
		EQ1-low cutoff:
		EQ2-peak 1:
		EQ3-peak 2:
		EQ4-peak3:
		EQ5-high cutoff:

	DAC Limiter
		DAC Limiter 1:
		DAC Limiter 2:

	Notch Filter
		Notch Filter 1:
		Notch Filter 2:
		Notch Filter 3:
		Notch Filter 4:

	ALC and Noise Gate Control
		ALC Control 1:
		ALC Control 2:
		ALC Control 3:

	Phase Locked Loop
		PLL N:
		PLL K 1:
		PLL K 2:
		PLL K 3:

	Miscellaneous
		3D control:
		Right Speaker Submix:
		Input Control:
		Left Input PGA Gain:
		Right Input PGA Gain:
		Left ADC Boost:
		Right ADC Boost:
		Output Control:
		Left Mixer:
		Right Mixer:
		LHP Volume:
		RHP Volume:
		LSPKOUT Volume:
		RSPKOUT Volume:
		AUX2 Mixer:
		AUX1 Mixer:
		
	PCM Time Slot and ADCOUT Impedance Option Control
		Left Time Slot:
		Misc:
		Right Time Slot:

	Silicon Revision and Device ID
		Device Revision #:
		Device ID:
		ALC Enhancements:
		ALC Enhancements:
		192kHz Sampling:
		Misc Controls:
		Tie-Off Overrides:
		Power/Tie-off  Ctrl:
		P2P Detector Read:
		Peak Detector Read:
		Control and Status:
		Output tie-off control:
        
        
*/

enum alc_freq{
    NAU_FS_192kHz=192,
    NAU_FS_96kHz=96,
    NAU_FS_48kHz=48,
    NAU_FS_32kHz=32,
    NAU_FS_24kHz=24,
    NAU_FS_16kHz=16,
    NAU_FS_12kHz=12,
    NAU_FS_8kHz=8
};

enum vol_mode{
    VOL_DAC,
    VOL_ADC,
    VOL_HP,
    VOL_SPK
};

struct nau8822_pll {
    uint8_t pllon;
	int pre_factor;
	int mclk_scaler;
	int pll_frac;
	int pll_int;
	int freq_in;
	int freq_out;
    enum alc_freq fs;
};

struct block_volume_ctrl
{
    struct block_volume_set adc_vol;
    struct block_volume_set dac_vol;
    struct block_volume_set spk_vol;
    struct block_volume_set hp_vol;
    struct block_volume_set input_pga_vol;
};

struct alc_control{
    uint8_t en;//Automatic Level Control function control bits 0: rf, 1:r, 2:l, 3: both rl
    uint8_t mx_gain;//Set minimum gain value limit for PGA volume setting changes under ALC control
    uint8_t mn_gain;//7-0, = 111 = +35.25dB (default), -6.75dB
    uint8_t ht;//Hold time before ALC automated gain increase
    uint8_t sl_level;//ALC target level at ADC output pg 82
    uint8_t mode;//Limiter Mode operation
    uint8_t dcy_time;//ALC decay time duration  page 83
    uint8_t atk_time;
    uint8_t ngate_en;//ALC noise gate en
    uint8_t ngate_th;//ALC noise gate threshold level
};

struct alc_filter{
    uint8_t hpf_en;//high pas filter en
    uint8_t hpf_am;//High pass filter mode selection 0: normal
    uint8_t hpf;//changed sample rate. see page 30; 0-7 #Cut-off Frequencies
    
    //page 81
    uint8_t notch_en;//(band_pass)^-1 en page 31
    uint8_t notch_cu[4];//Update bit feature for simultaneous change of all notch filter parameters
    uint8_t notch_ca[4];//Notch filter A0 coefficient most significant bits. See text and table for details.

    uint16_t threeD_th;
};

struct rspk_mix{
    uint8_t raux_en;
    uint8_t raux_gain;
    uint8_t rmix_en;
    uint8_t rmix_gain;
};

struct alc_mixer{
    uint8_t aux_en[2];// (0, 1:enable)
    uint8_t aux_gain[2];//Gain value between AUXIN  (0-7) -15db, +6db
    uint8_t adc_mx_en[2];//xAUXIN input to xMAIN x output mixer path control   (0, 1:enable)
    uint8_t adc_mx_gain[2];//(0-7) -15db, +6db
    uint8_t dac_mx_en[2];//xDAC output to xMIX x output mixer path control
    uint8_t aux1_mix[7];
    uint8_t aux2_mix[5];
    uint8_t out_mix[7];
    uint8_t rspk_mix[4];
    struct rspk_mix rspk;
};

typedef struct alc_control_t_
{
    int div_id;
    uint16_t micbias;
    mau88c22_EQ eq[5];//page 34 5 band filter
    struct alc_control_pga pga;
    struct block_volume_ctrl vol;
    struct adc_boost abst;
    uint8_t dac_automt;
    struct alc_control alctrl;
    struct alc_filter filter;
    struct alc_mixer mix;
    struct nau8822_pll pll;
    uint8_t dac_invert[2];
    uint8_t adc_invert[2];
}alc_control_t;


uint8_t codec_init(alc_control_t *alc);
//setup
uint32_t setup_clock_pll(struct nau8822_pll *pll);
uint8_t settup_adc(uint8_t adc_linv, uint8_t adc_rinv);
uint8_t setup_dac(uint8_t auto_mute, uint8_t adc_linv, uint8_t adc_rinv);

//input
uint8_t alc_control(alc_control_t *alc);
uint8_t input_gain(alc_control_t *alc);
uint8_t adc_mix_boost(alc_control_t *alc);

//mixer out
uint8_t main_mixer(alc_control_t *alc);
uint8_t spk_submix(alc_control_t *alc);
uint8_t aux_submix(alc_control_t *alc);

//filter
uint8_t alc_control_filter(alc_control_t *alc);
uint8_t three_d_efect(alc_control_t *alc);
uint8_t hpf_filter(alc_control_t *alc);
uint8_t notch_filter(alc_control_t *alc);
uint8_t band_eq(alc_control_t *alc);

//volume out control
uint8_t channel_volume(alc_control_t *alc, enum vol_mode mode);
uint8_t output_control(alc_control_t *alc);

#endif	/* __CODEC_H__ */
