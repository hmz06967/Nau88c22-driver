/* SPDX-License-Identifier: GPL-2.0 */
/*
 * nau8822.h  --  NAU8822 ALSA SoC Audio driver
 *
 * Copyright 2017 Nuvoton Technology Crop.
 *
 * Author: David Lin <ctlin0@nuvoton.com>
 * Co-author: John Hsu <kchsu0@nuvoton.com>
 * Co-author: Seven Li <wtli@nuvoton.com>
 */

#ifndef __NAU8822_H__
#define __NAU8822_H__

#include <stdint.h>
#include <inttypes.h>

/*REGISTER ADDRESS*/
#define NAU8822_REG_RESET			0x00
#define NAU8822_REG_POWER_MANAGEMENT_1		0x01
#define NAU8822_REG_POWER_MANAGEMENT_2		0x02
#define NAU8822_REG_POWER_MANAGEMENT_3		0x03
#define NAU8822_REG_AUDIO_INTERFACE		0x04
#define NAU8822_REG_COMPANDING_CONTROL		0x05
#define NAU8822_REG_CLOCKING			0x06
#define NAU8822_REG_ADDITIONAL_CONTROL		0x07
#define NAU8822_REG_GPIO_CONTROL		0x08
#define NAU8822_REG_JACK_DETECT_CONTROL_1	0x09
#define NAU8822_REG_DAC_CONTROL			0x0A
#define NAU8822_REG_LEFT_DAC_DIGITAL_VOLUME	0x0B
#define NAU8822_REG_RIGHT_DAC_DIGITAL_VOLUME	0x0C
#define NAU8822_REG_JACK_DETECT_CONTROL_2	0x0D
#define NAU8822_REG_ADC_CONTROL			0x0E
#define NAU8822_REG_LEFT_ADC_DIGITAL_VOLUME	0x0F
#define NAU8822_REG_RIGHT_ADC_DIGITAL_VOLUME	0x10
#define NAU8822_REG_EQ1				0x12
#define NAU8822_REG_EQ2				0x13
#define NAU8822_REG_EQ3				0x14
#define NAU8822_REG_EQ4				0x15
#define NAU8822_REG_EQ5				0x16
#define NAU8822_REG_DAC_LIMITER_1		0x18
#define NAU8822_REG_DAC_LIMITER_2		0x19
#define NAU8822_REG_NOTCH_FILTER_1		0x1B
#define NAU8822_REG_NOTCH_FILTER_2		0x1C
#define NAU8822_REG_NOTCH_FILTER_3		0x1D
#define NAU8822_REG_NOTCH_FILTER_4		0x1E
#define NAU8822_REG_ALC_CONTROL_1		0x20
#define NAU8822_REG_ALC_CONTROL_2		0x21
#define NAU8822_REG_ALC_CONTROL_3		0x22
#define NAU8822_REG_NOISE_GATE			0x23
#define NAU8822_REG_PLL_N			0x24
#define NAU8822_REG_PLL_K1			0x25
#define NAU8822_REG_PLL_K2			0x26
#define NAU8822_REG_PLL_K3			0x27
#define NAU8822_REG_3D_CONTROL			0x29
#define NAU8822_REG_RIGHT_SPEAKER_CONTROL	0x2B
#define NAU8822_REG_INPUT_CONTROL		0x2C
#define NAU8822_REG_LEFT_INP_PGA_CONTROL	0x2D
#define NAU8822_REG_RIGHT_INP_PGA_CONTROL	0x2E
#define NAU8822_REG_LEFT_ADC_BOOST_CONTROL	0x2F
#define NAU8822_REG_RIGHT_ADC_BOOST_CONTROL	0x30
#define NAU8822_REG_OUTPUT_CONTROL		0x31
#define NAU8822_REG_LEFT_MIXER_CONTROL		0x32
#define NAU8822_REG_RIGHT_MIXER_CONTROL		0x33
#define NAU8822_REG_LHP_VOLUME			0x34
#define NAU8822_REG_RHP_VOLUME			0x35
#define NAU8822_REG_LSPKOUT_VOLUME		0x36
#define NAU8822_REG_RSPKOUT_VOLUME		0x37
#define NAU8822_REG_AUX2_MIXER			0x38
#define NAU8822_REG_AUX1_MIXER			0x39
#define NAU8822_REG_POWER_MANAGEMENT_4		0x3A
#define NAU8822_REG_LEFT_TIME_SLOT		0x3B
#define NAU8822_REG_MISC			0x3C
#define NAU8822_REG_RIGHT_TIME_SLOT		0x3D
#define NAU8822_REG_DEVICE_REVISION		0x3E
#define NAU8822_REG_DEVICE_ID			0x3F
#define NAU8822_REG_DAC_DITHER			0x41
#define NAU8822_REG_ALC_ENHANCE_1		0x46
#define NAU8822_REG_ALC_ENHANCE_2		0x47
#define NAU8822_REG_192KHZ_SAMPLING		0x48
#define NAU8822_REG_MISC_CONTROL		0x49
#define NAU8822_REG_INPUT_TIEOFF		0x4A
#define NAU8822_REG_POWER_REDUCTION		0x4B
#define NAU8822_REG_AGC_PEAK2PEAK		0x4C
#define NAU8822_REG_AGC_PEAK_DETECT		0x4D
#define NAU8822_REG_AUTOMUTE_CONTROL		0x4E
#define NAU8822_REG_OUTPUT_TIEOFF		0x4F

#define REG_FULL_VALUE				0x1ff
#define REG_EN(READ, MASK)          (READ | MASK)
#define REG_OFF(READ, MASK)         (READ & (~MASK))
#define REG_BV(READ, MASK, VALUE)    (VALUE ? REG_EN(READ, MASK) : REG_OFF(READ, MASK))
#define NAU8822_REG_MAX_BIT         0x09

#define NAU_DEV_ID 0x01A
#define NAU_OK 0
#define NAU_ERROR 1
#define NAU_ERROR_ID 2
#define NAU_ERROR_WRITE 3
#define NAU_ERROR_READ 3

#ifdef MASK
/* NAU8822_REG_POWER_MANAGEMENT_1 (0x1) */
#define NAU8822_DCBUFEN_MASK	    (0x1 << 8)
#define NAU8822_AUX1MXEN_MASK 	    (0x1 << 7)
#define NAU8822_AUX2MXEN_MASK 	    (0x1 << 6)
#define NAU8822_PLL_MASK			(0x1 << 5)
#define NAU8822_MICBIASEN_MASK 	    (0x1 << 4)
#define NAU8822_ABIASEN_MASK		(0x1 << 3)
#define NAU8822_IOBUF_MASK			(0x1 << 2)
#define NAU8822_REFIMP_MASK			0x3
#define NAU8822_REFIMP_80K			0x1
#define NAU8822_REFIMP_300K			0x2
#define NAU8822_REFIMP_3K			0x3

/* NAU8822_REG_POWER_MANAGEMENT_2 (0x2) */
#define NAU8822_RHPEN_MASK	        (0x1 << 8)
#define NAU8822_NHPEN_MASK 	        (0x1 << 7)
#define NAU8822_SLEEP_MASK 	        (0x1 << 6)
#define NAU8822_RBSTEN_MASK			(0x1 << 5)
#define NAU8822_LBSTEN_MASK 	    (0x1 << 4)
#define NAU8822_RPGAEN_MASK		    (0x1 << 3)
#define NAU8822_LPGAEN_MASK			(0x1 << 2)
#define NAU8822_RADCEN_MASK		    (0x1 << 1)
#define NAU8822_LADCEN_MASK			(0x1)

/* NAU8822_REG_POWER_MANAGEMENT_3 (0x3) */
#define NAU8822_AUXOUT1EN_MASK	    (0x1 << 8)
#define NAU8822_AUXOUT2EN_MASK 	    (0x1 << 7)
#define NAU8822_LSPKEN_MASK 	    (0x1 << 6)
#define NAU8822_RSPKEN_MASK			(0x1 << 5)
#define NAU8822_RMIXEN_MASK		    (0x1 << 3)
#define NAU8822_LMIXEN_MASK			(0x1 << 2)
#define NAU8822_RDACEN_MASK		    (0x1 << 1)
#define NAU8822_LDACEN_MASK			(0x1)

/* NAU8822_REG_AUDIO_INTERFACE (0x4) */
#define NAU8822_AIFMT_MASK			(0x3 << 3)
#define NAU8822_WLEN_MASK			(0x3 << 5)
#define NAU8822_WLEN_20				(0x1 << 5)
#define NAU8822_WLEN_24				(0x2 << 5)
#define NAU8822_WLEN_32				(0x3 << 5)
#define NAU8822_LRP_MASK			(0x1 << 7)
#define NAU8822_BCLKP_MASK			(0x1 << 8)

/* NAU8822_REG_COMPANDING_CONTROL (0x5) */
#define NAU8822_ADDAP_SFT			0
#define NAU8822_ADCCM_SFT			1
#define NAU8822_DACCM_SFT			3

/* NAU8822_REG_CLOCKING (0x6) */
#define NAU8822_CLKIOEN_MASK			0x1
#define NAU8822_CLK_MASTER			0x1
#define NAU8822_CLK_SLAVE			0x0
#define NAU8822_MCLKSEL_SFT			5
#define NAU8822_MCLKSEL_MASK			(0x7 << 5)
#define NAU8822_BCLKSEL_SFT			2
#define NAU8822_BCLKSEL_MASK			(0x7 << 2)
#define NAU8822_BCLKDIV_1			(0x0 << 2)
#define NAU8822_BCLKDIV_2			(0x1 << 2)
#define NAU8822_BCLKDIV_4			(0x2 << 2)
#define NAU8822_BCLKDIV_8			(0x3 << 2)
#define NAU8822_BCLKDIV_16			(0x4 << 2)
#define NAU8822_CLKM_MASK			(0x1 << 8)
#define NAU8822_CLKM_MCLK			(0x0 << 8)
#define NAU8822_CLKM_PLL			(0x1 << 8)

/* GPIO */
#define NAU8822_GPIO1PLL_MASK		(0x3 << 4)
#define NAU8822_GPIO1PL_MASK		(0x1 << 3)
#define NAU8822_GPIO1SEL_MASK			(0x7)

/* DAC control*/
#define NAU8822_SOFTMT_MASK		    (0x1 << 5)
#define NAU8822_DACOS_MASK		    (0x1 << 3)
#define NAU8822_AUTOMT_MASK			(0x1<< 2)
#define NAU8822_RDACPL_MASK		    (0x1 << 1)
#define NAU8822_LDACPL_MASK		    (0x1)

/*Left DAC volume */
#define NAU8822_LDACVU_MASK		    (0x1 << 8)
#define NAU8822_LDACGAIN_MASK		(0xff)

/*Right DAC volume */
#define NAU8822_LDACVU_MASK		    (0x1 << 8)
#define NAU8822_LDACGAIN_MASK		(0xff)

/*ADC control*/
#define NAU8822_HPFEN_MASK		    (0x1 << 8)
#define NAU8822_HPFAM_MASK		    (0x1 << 7)
#define NAU8822_HPF_MASK			(0x7<< 4)
#define NAU8822_ADCOS_MASK		    (0x1 << 3)
#define NAU8822_RADCPL_MASK		    (0x1<<1)
#define NAU8822_LADCPL_MASK		    (0x1)

/*Left ADC volume */
#define NAU8822_LADCVU_MASK		    (0x1 << 8)
#define NAU8822_LADCGAIN_MASK		(0xff)

/*Right ADC volume */
#define NAU8822_RADCVU_MASK		    (0x1 << 8)
#define NAU8822_RADCGAIN_MASK		(0xff)


/* NAU8822_REG_ADDITIONAL_CONTROL (0x08) */
#define NAU8822_SMPLR_SFT			1
#define NAU8822_SMPLR_MASK			(0x7 << 1)
#define NAU8822_SMPLR_48K			(0x0 << 1)
#define NAU8822_SMPLR_32K			(0x1 << 1)
#define NAU8822_SMPLR_24K			(0x2 << 1)
#define NAU8822_SMPLR_16K			(0x3 << 1)
#define NAU8822_SMPLR_12K			(0x4 << 1)
#define NAU8822_SMPLR_8K			(0x5 << 1)

/* NAU8822_REG_EQ1 (0x12) */
#define NAU8822_EQ1GC_SFT			0
#define NAU8822_EQ1CF_SFT			5
#define NAU8822_EQM_SFT				8

/* NAU8822_REG_EQ2 (0x13) */
#define NAU8822_EQ2GC_SFT			0
#define NAU8822_EQ2CF_SFT			5
#define NAU8822_EQ2BW_SFT			8

/* NAU8822_REG_EQ3 (0x14) */
#define NAU8822_EQ3GC_SFT			0
#define NAU8822_EQ3CF_SFT			5
#define NAU8822_EQ3BW_SFT			8

/* NAU8822_REG_EQ4 (0x15) */
#define NAU8822_EQ4GC_SFT			0
#define NAU8822_EQ4CF_SFT			5
#define NAU8822_EQ4BW_SFT			8

/* NAU8822_REG_EQ5 (0x16) */
#define NAU8822_EQ5GC_SFT			0
#define NAU8822_EQ5CF_SFT			5

/*DAC limiter 1 24*/
/*DAC limiter 2 25*/
/*Notch filter 1 27*/
/*Notch filter 2 28*/
/*Notch filter 3 29*/
/*Notch filter 4 30*/

/* NAU8822_REG_ALC_CONTROL_1 (0x20) */
#define NAU8822_ALCMINGAIN_SFT			0
#define NAU8822_ALCMXGAIN_SFT			3
#define NAU8822_ALCEN_SFT			7

/* NAU8822_REG_ALC_CONTROL_2 (0x21) */
#define NAU8822_ALCSL_SFT			0
#define NAU8822_ALCHT_SFT			4

/* NAU8822_REG_ALC_CONTROL_3 (0x22) */
#define NAU8822_ALCATK_SFT			0
#define NAU8822_ALCDCY_SFT			4
#define NAU8822_ALCM_SFT			8

/*Noise gate 35*/

/* NAU8822_REG_PLL_N (0x24) */
#define NAU8822_PLLMCLK_DIV2			(0x1 << 4)
#define NAU8822_PLLN_MASK			0xF

#define NAU8822_PLLK1_SFT			18
#define NAU8822_PLLK1_MASK			0x3F

/* NAU8822_REG_PLL_K2 (0x26) */
#define NAU8822_PLLK2_SFT			9
#define NAU8822_PLLK2_MASK			0x1FF

/* NAU8822_REG_PLL_K3 (0x27) */
#define NAU8822_PLLK3_MASK			0x1FF

/*3D control*/
#define NAU8822_3DDEPTH_MASK			(0x0F)

/* NAU8822_REG_RIGHT_SPEAKER_CONTROL (0x2B) */
#define NAU8822_RMIXMUT				0x20
#define NAU8822_RSUBBYP				0x10
#define NAU8822_RAUXRSUBG_SFT			1
#define NAU8822_RAUXRSUBG_MASK			0x0E
#define NAU8822_RAUXSMUT			0x01

/*Input control 44*/
#define NAU8822_MICBIASV_MASK			(0x3 << 7)
#define NAU8822_RLINRPGA_MASK			(0x1 << 6)
#define NAU8822_RMICNRPGA_MASK			(0x1 << 5)
#define NAU8822_RMICPRPGA_MASK			(0x1 << 4)
#define NAU8822_LLINLPGA_MASK			(0x1 << 2)
#define NAU8822_LMICNLPGA_MASK			(0x1 << 1)
#define NAU8822_MLMICPLPGA_MASK			(0x1)

/*Left input PGA gain 45 */
#define NAU8822_LPGAU_MASK			(0x1 << 8)
#define NAU8822_LPGAZC_MASK			(0x1 << 7)
#define NAU8822_LPGAMT_MASK			(0x1 << 6)
#define NAU8822_LPGAGAIN_MASK		(0x3f)

/*Right input PGA gain 46 */
#define NAU8822_RPGAU_MASK			(0x1 << 8)
#define NAU8822_RPGAZC_MASK			(0x1 << 7)
#define NAU8822_RPGAMT_MASK		    (0x1 << 6)
#define NAU8822_RPGAGAIN_MASK		(0x3f)

/*Left ADC boost  47*/
#define NAU8822_LPGABST_MASK			(0x1 << 8)
#define NAU8822_LPGABSTGAIN_MASK		(0x7 << 5)
#define NAU8822_LAUXBSTGAIN_MASK		(0x7)

/*Right ADC boost 48*/
#define NAU8822_RPGABST_MASK			(0x1 << 8)
#define NAU8822_RPGABSTGAIN_MASK		(0x7 << 5)
#define NAU8822_RAUXBSTGAIN_MASK		(0x7)

/*Output control 49*/
#define NAU8822_LDACRMX_MASK			(0x1 << 6)
#define NAU8822_RDACLMX_MASK		    (0x1 << 5)
#define NAU8822_AUX1BST_MASK			(0x1 << 4)
#define NAU8822_AUX2BST_MASK		    (0x1 << 3)
#define NAU8822_SPKBST_MASK			    (0x1 << 2)
#define NAU8822_TSEN_MASK		        (0x1 << 1)
#define NAU8822_AOUTIMP_MASK			(0x1)

/*Left mixer 50*/
/*Right mixer 51*/
/*LHP volume 52*/
/*RHP volume 53*/
/*LSPKOUT volume 54*/
/*RSPKOUT volume 55*/
/*AUX2 MIXER 56*/
/*AUX1 MIXER 57*/
/*Power Management 4 58*/
/*Misc 60*/
/*Right time slot 61*/
/*Device Revision Number 62*/
/*Device ID 63*/
/* DAC Dither 65*/
/*ALC Enhancement 1 70*/
/*ALC Enhancement 2 71*/
/*192kHz Sampling 72*/
/*Misc Controls 73*/
/*Input Tie-Off Direct Manual Control  74*/
/*Reduction and Output Tie-Off Direct Manual Control  75*/
/*AGC Peak-to-Peak Readout 76*/
/*AGC Peak Detector Readout 77*/
/*Automute Control and Status Readout 78*/
/*Output Tie-Off  Controls 79*/
#endif

typedef struct nau88c22_EQ_{
	uint8_t eq_em;
	uint8_t eq_bw;
	uint8_t eq_cf;
	uint8_t eq_gc;
} mau88c22_EQ;

struct block_volume_set
{
    uint8_t upbit;//volume update bit
    uint8_t ch_zc[2];//zero cross detection 1: on
    uint8_t ch_gain[2];//0-3f, -12db, +35.25db
	uint8_t ch_mute[2];
};

struct alc_control_pga
{
    uint8_t mic_n_pga[2];//1: negative connect
    uint8_t mic_p_pga[2];//1: positive connect
    uint8_t lin_pga[2];//1 lin connect

	uint8_t ch_pga[2];//0-3f, Real time readout alc channel PGA
	uint8_t pga_gain[2];//0-63, -12db-35.25db
    uint8_t ch_pga_mute[2];
};

struct adc_boost{
	uint8_t ch_bst_en[2];//0: no gain, 1: +20db
	uint8_t ch_bst_gain[2];//0: disconnect,  1-7; -12db, +6db
	uint8_t aux_bst_gain[2];//0: disconnect,  1-7; -12db, +6db
};

uint8_t write_bus(reg, data);
uint8_t read_bus(reg, data);
void delay(uint32_t ms);

uint8_t nau8822_init();

uint16_t change_reg_data(uint16_t data,  uint8_t bit, uint8_t bs, uint16_t value);
uint8_t reg_write(uint8_t reg, uint8_t bit, uint8_t bs, uint16_t value);
uint8_t reg_read(uint8_t reg, uint16_t *data);

/* power */
uint8_t power_on();
uint8_t clock_pll(uint8_t pllen);

/* audio */
uint8_t gac_audio_interface ();
uint8_t gac_companding(uint8_t cmb8, uint8_t daccm, uint8_t addccm, uint8_t addap);
uint8_t gac_clock_control1(uint8_t clkm, uint8_t mclksel, uint8_t bclksel, uint8_t clkioen);
uint8_t gac_clock_control2(uint8_t spi4en, uint8_t smplr, uint8_t sclken);
uint8_t gac_gpio(uint8_t spi4en, uint8_t smplr, uint8_t sclken);
uint8_t gac_jack_detect(uint8_t en, uint8_t den, uint8_t dio);
uint8_t gac_dac_control(uint8_t sfmt, uint8_t dacos,uint8_t automt,uint8_t rdacpl, uint8_t ldacpl);
uint8_t gac_left_dac_volume(struct block_volume_set vol);
uint8_t gac_right_dac_volume(struct block_volume_set vol);
uint8_t gac_jack_detect_2(uint8_t jckd0en1, uint8_t jckd0en0);
uint8_t gac_adc_control(uint8_t hpfen, uint8_t hpfam, uint8_t  hpf, uint8_t adcos, uint8_t radcpl, uint8_t ldacpl);
uint8_t gac_left_adc_volume(struct block_volume_set vol);
uint8_t gac_right_adc_volume(struct block_volume_set vol);

//EQ
uint8_t eq1_low_cutoff(mau88c22_EQ *eq);
uint8_t eq2_peak_1(mau88c22_EQ *eq1);
uint8_t eq3_peak_2(mau88c22_EQ *eq2);
uint8_t eq4_peak3(mau88c22_EQ *eq3);
uint8_t eq5_high_cutoff(mau88c22_EQ *eq5);
uint8_t read_eq_array(mau88c22_EQ eq[5]);

/**************************************/

//dac_limiter
uint8_t dac_limiter_1(uint8_t daclimen, uint8_t daclimdcy,uint8_t daclimatk);
uint8_t dac_limiter_2(uint8_t daclimthl, uint8_t daclimbst);

//notch_filter
uint8_t notch_filter_1(uint8_t nfcu, uint8_t nfcen,uint8_t nfca0);
uint8_t notch_filter_2(uint8_t nfcu2, int8_t nfcao);
uint8_t notch_filter_3(uint8_t nfcu3, int8_t nfca1);
uint8_t notch_filter_4(uint8_t nfcu4, int8_t nfca1);

//alc_and_noise_gate_control
uint8_t alc_control_1(uint8_t en, uint8_t xgain, uint8_t ngain);
uint8_t alc_control_2(uint8_t ht, uint8_t sl);
uint8_t alc_control_3(uint8_t cm, uint8_t dcy, uint8_t atk);
uint8_t noise_gate(uint8_t en, uint8_t th);

//phase_locked_loop
uint8_t pll_nn(uint8_t pllmclk, uint16_t plln);
uint8_t pll_k_1(uint16_t pllk);
uint8_t pll_k_2(uint16_t pllk);
uint8_t pll_k_3(uint16_t pllk);


//miscellaneous
uint8_t misc_3d_control(uint8_t depth);
uint8_t misc_right_speaker_submix(uint8_t rmixmute, uint8_t rsubbyp, uint8_t rauxrsubg, uint8_t rauxsmut);
uint8_t misc_input_control(uint8_t micbias, struct alc_control_pga pga);
uint8_t misc_left_input_pga_gain(struct block_volume_set vol);
uint8_t misc_right_input_pga_gain(struct block_volume_set vol);
uint8_t misc_left_adc_boost(struct adc_boost  abst);
uint8_t misc_right_adc_boost(struct adc_boost  abst);
uint8_t misc_output_control(	
	uint8_t ldacrmx, 
	uint8_t rdaclmx,  uint8_t aux1bst,
	uint8_t aux2bst, uint8_t spkbst,
	uint8_t tsen, uint8_t aout1mp
);
uint8_t misc_left_mixer(uint8_t auxmxgain, uint8_t auxlmx, uint8_t bypmxgain, uint8_t bypmx, uint8_t dacmx);
uint8_t misc_right_mixer(uint8_t auxmxgain, uint8_t auxlmx, uint8_t bypmxgain, uint8_t bypmx, uint8_t dacmx);
uint8_t misc_lhp_volume(struct block_volume_set vol);
uint8_t misc_rhp_volume(struct block_volume_set vol);
uint8_t misc_lspkout_volume(struct block_volume_set vol);
uint8_t misc_rspkout_volume(struct block_volume_set vol);
uint8_t misc_aux2_mixer(uint8_t auxoutmt, uint8_t aux1mix, uint8_t ladcaux, uint8_t lmixaux, uint8_t ldacaux);
uint8_t misc_aux1_mixer(uint8_t auxoutmt, uint8_t aux1half, uint8_t lmixaux1,  uint8_t ldacaux1, uint8_t radcaux, uint8_t rmixaux, uint8_t rdacaux);
//pcm_time_slot_and_adcout_impedance_option_control
uint8_t left_time_slot(uint16_t ltslot);
uint8_t misc(uint8_t misc_array[]);
uint8_t right_time_slot(uint16_t rtslot);

//silicon_revision_and_device_id
uint8_t device_revision();
uint8_t device_id();
uint8_t dac_dither(uint8_t mod_dit, uint8_t analog_dit);
uint8_t alc_enhancements1(uint8_t tblsel, uint8_t pksel, uint8_t ngsel, uint8_t gainL);
uint8_t alc_enhancements2(uint8_t pklimena, uint8_t gainR);
uint8_t khz192_sampling(uint8_t adcb_over, uint8_t pll49mout, uint8_t dac_osr32, uint8_t adc_osr32);
uint8_t misc_controls(uint16_t data);
uint8_t tie_off_overrides(uint16_t data);
uint8_t power_tie_off_ctrl(uint16_t data);
uint8_t p2p_detector_read(uint16_t data);
uint8_t peak_detector_read(uint16_t data);
uint8_t control_and_status(uint8_t amutctrl, uint8_t hvdet, uint8_t nsgate);
uint8_t output_tie_off_control(uint16_t data);


#endif	/* __NAU8822_H__ */