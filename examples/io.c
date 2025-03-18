/*
 * io.c
 *
 *  Created on: Jul 22, 2023
 *      Author: hmz_o
 */

#include "io.h"
#include "audio.h"
#include "i2cx.h"
#include <uart.h>

#include "usbd_audio_if.h"
#define BUFFER_SIZE 4

alc_control_t n_ALC;


extern TIM_HandleTypeDef htim2;
extern SPI_HandleTypeDef hspi1;
extern I2S_HandleTypeDef hi2s2;


extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t spi1_tx_buffer[2];
uint8_t spi1_rx_buffer[40];

uint8_t audio_data[50];

io_data_t io_data;

//ı2s
uint16_t adc_data[BUFFER_SIZE];
uint16_t dac_data[BUFFER_SIZE];

extern uint16_t *usb_out_data;

uint8_t codec_rdy = 0;

static volatile uint16_t *inBufPtr = &adc_data[0];
static volatile uint16_t *outBufPtr = &dac_data[0];

void led_on(enum LED ledio){
	if(ledio==BLUE)LED_BLUE_GPIO_Port->BSRR = LED_BLUE_Pin;
	else if(ledio==RED)LED_RED_GPIO_Port->BSRR = LED_RED_Pin;
}

void led_off(enum LED ledio){
	if(ledio==BLUE)LED_BLUE_GPIO_Port->BSRR = LED_BLUE_Pin << 16;
	else if(ledio==RED)LED_RED_GPIO_Port->BSRR = LED_RED_Pin << 16;
}

void default_codec(){


	for(uint8_t i=0;i<2;i++){
		n_ALC.vol.hp_vol.ch_gain[i] = 255;
		n_ALC.vol.hp_vol.ch_mute[i] = 0;
		n_ALC.vol.hp_vol.ch_zc[i] = 1;
		n_ALC.vol.hp_vol.upbit = 1;

		n_ALC.vol.dac_vol.ch_gain[i] = 255;
		n_ALC.vol.dac_vol.ch_mute[i] = 0;
		n_ALC.vol.dac_vol.ch_zc[i] = 1;
		n_ALC.vol.dac_vol.upbit = 1;

		n_ALC.vol.adc_vol.ch_gain[i] =  220;//75
		n_ALC.vol.adc_vol.ch_mute[i] = 0;
		n_ALC.vol.adc_vol.ch_zc[i] = 1;
		n_ALC.vol.adc_vol.upbit = 1;

	}

	delay();
	audio_volume(VOL_HP);
	delay();
	audio_volume(VOL_DAC);
	delay();
	audio_volume(VOL_ADC);

	n_ALC.dac_automt = 1;
	n_ALC.micbias = 0;

	for(uint8_t i=0;i<2;i++){
		n_ALC.vol.input_pga_vol.ch_gain[i] = 1;
		n_ALC.vol.input_pga_vol.ch_mute[i] = 0;
		n_ALC.vol.input_pga_vol.ch_zc[i] = 1;
		n_ALC.vol.input_pga_vol.upbit = 1;

		n_ALC.pga.mic_n_pga[i] = 1;//1: negative connect: (2)
		n_ALC.pga.mic_p_pga[i] = 1;//1: positive connect: (2)
		n_ALC.pga.lin_pga[i] = 0;//1 lin connect: (1)

		n_ALC.pga.ch_pga_mute[i] = 0;
		n_ALC.pga.pga_gain[i] = 1;
		n_ALC.pga.ch_pga[i] = 1;

		n_ALC.abst.aux_bst_gain[i] = 0;//mix boost gain

		n_ALC.abst.ch_bst_en[i] = 1;
		n_ALC.abst.ch_bst_gain[i] = 1;


	}

	delay();
	microfone_interface();

	for(uint8_t i=0;i<2;i++){
		n_ALC.mix.aux_en[i] = 0;
		n_ALC.mix.aux_gain[i] = 0;
		n_ALC.mix.adc_mx_gain[i] = 0;
		n_ALC.mix.adc_mx_en[i] = 0;//xAUXIN input to xMAIN x output mixer path control   (0, 1:enable)

		n_ALC.mix.dac_mx_en[i] = 1;
	}

	delay();
	input_mixer();

	/*
	 *
	 *  ldacrmx: Left DAC output to RMIX (0-1: connect),
        rdaclmx: Right DAC output to LMIX left,
            aux1bst: AUXOUT1 gain boost control (0: -1db, 1: 1.5db),
            aux2bst,
            spkbst,
        tsen: Thermal shutdown enable (1: enable),
        aout1mp: Output resistance control option (0: 1k, 1: 30k)
        */

	n_ALC.mix.out_mix[0] = 0;
	n_ALC.mix.out_mix[1] = 0;

	n_ALC.mix.out_mix[2] = 1;//+1.5db aux1
	n_ALC.mix.out_mix[3] = 1;//+1.5db aux2
	n_ALC.mix.out_mix[4] = 1;//+1.5db spk

	n_ALC.mix.out_mix[6] = 1;

	delay();
	output_driver();

	/*
	 *         uint8_t aux1_mix[7];
            (1: connected)
            AUXOUT1MT: (1: mute) AUXOUT1: output mute control (1: mute)
            AUX1HALF: 6dB attenuation enable (0: normal: 1: -6.0dB)

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
	 *
	 * */
	//mute ctrl
	n_ALC.mix.aux1_mix[0] = 0;//AUXOUT1MT
	n_ALC.mix.aux2_mix[0] = 0;//AUXOUT2MT
	n_ALC.mix.aux1_mix[1] = 0;//AUX1HALF

	//direct aux out

	//mix (direct mix)
	n_ALC.mix.aux1_mix[5] = 1;//RMIXAUX1
	n_ALC.mix.aux2_mix[3] = 1;//LMIXAUX2
	n_ALC.mix.aux1_mix[2] = 0;//LMIXAUX1

	//inmix to aux (direct inmix)
	n_ALC.mix.aux1_mix[4] = 0;//RADCAUX1
	n_ALC.mix.aux2_mix[2] = 0;//LADCAUX2

	//dac to xaux (direct dac)
	n_ALC.mix.aux1_mix[6] = 0;//RDACAUX1
	n_ALC.mix.aux2_mix[4] = 0;//LDACAUX2
	n_ALC.mix.aux1_mix[3] = 0;//LDACAUX1

	//compare out aux
	n_ALC.mix.aux2_mix[1] = 0;//AUX1MIX

	delay();
	output_mixer();

	//filter demo
	//3d
    //n_ALC.filter.threeD_th = 0x00;
	//three_d_efect(&n_ALC);
	//n_ALC.filter.hpf_en = 1;
	//n_ALC.filter.hpf_am = 1;
	//n_ALC.filter.hpf = 5;
	//hpf_filter(&n_ALC);

}

void io_init(){

	led_on(BLUE);

	_printf("SoundBoard V1.1\r\n");
	_printf("Usb ok.\r\n");

	_printf("Setup amp..\r\n");
	//for amp
	HAL_GPIO_WritePin(AMP_SD_GPIO_Port, AMP_SD_Pin, GPIO_PIN_SET);//anfi shut down
	HAL_GPIO_WritePin(A_MUTE_GPIO_Port, A_MUTE_Pin, GPIO_PIN_RESET);//anfi mute
	HAL_GPIO_WritePin(A_FADE_GPIO_Port, A_FADE_Pin, GPIO_PIN_RESET);//anfi fade

	_printf("Setup codec..\r\n");

	//for codec
	HAL_GPIO_WritePin(C_MODE_GPIO_Port, C_MODE_Pin, GPIO_PIN_RESET);//mode = 0 i2c, 1: 3 wire spi mode select
	i2cx_init(I2C_SDA_GPIO_Port, I2C_SDA_Pin, I2C_SCLK_GPIO_Port, I2C_SCLK_Pin);//A1:SDA, B13:SCL
	HAL_Delay(10);

	if(audio_init(&n_ALC)!=AUDIO_OK){//codec init
		led_on(RED);
		while(1);
	}

	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);//mclk 12mhz output
	//delay(5);

	//HAL_I2S_Transmit_DMA(&hi2s2, dac_data, BUFFER_SIZE);
	_printf("Started transmit audio.\r\n");

	default_codec();

	memset(dac_data, 0, sizeof(dac_data));
	memset(adc_data, 0, sizeof(dac_data));


	HAL_I2SEx_TransmitReceive_DMA(&hi2s2, dac_data, adc_data, BUFFER_SIZE);
	//HAL_I2S_Transmit_DMA(&hi2s2, usb_out_data, BUFFER_SIZE);
	//HAL_I2S_Receive_DMA(&hi2s2, adc_data, BUFFER_SIZE);
	//HAL_I2S_Transmit_DMA(&hi2s2, dac_data, BUFFER_SIZE);

	//led_off(BLUE);
}

void adc_io_run(){
	//if  adc be rdy, restart after of finished io_dma
	if(HAL_GPIO_ReadPin(ADC_RDY_GPIO_Port, ADC_RDY_Pin) == ADC_RDY && io_data.dma_done){
		spi1_tx_buffer[0] = 0x2a;
		//RESTART adc DMA
		HAL_SPI_TransmitReceive_DMA(&hspi1, spi1_tx_buffer, spi1_rx_buffer, 1);
	}else{
		//son adc verilerini işle
		//n_ALC.vol.adc_vol.ch_gain
	}

}

void codec_i2s_run(){
	//audio_run_cmd(audio_data);
	static float left_in, right_in;

	if(codec_rdy){
		for(uint8_t n=0;n<(BUFFER_SIZE/2); n += 2){
			left_in = 1.0 * inBufPtr[n];
			right_in = 1.0 * inBufPtr[n+1];

			//left_in = 1.0 * usb_out_data[n];
			//right_in = 1.0 * usb_out_data[n+1];

			outBufPtr[n] = ((uint16_t)left_in);
			outBufPtr[n+1] = ((uint16_t)right_in);
		}

		codec_rdy = 0;

	}
}

void io_run(){


	adc_io_run();
	codec_i2s_run();

    /*switch (audio_status.frequency) {
      case 44100:
          BSP_LED_Off(LED_RED);
          BSP_LED_Off(LED_GREEN);
          BSP_LED_On(LED_BLUE);
          break;
      case 48000:
          BSP_LED_Off(LED_RED);
          BSP_LED_On(LED_GREEN);
          BSP_LED_Off(LED_BLUE);
          break;
      case 96000:
          BSP_LED_On(LED_RED);
          BSP_LED_Off(LED_GREEN);
          BSP_LED_Off(LED_BLUE);
          break;
      default:
          BSP_LED_Off(LED_RED);
          BSP_LED_Off(LED_GREEN);
          BSP_LED_Off(LED_BLUE);
          break;
    }*/



}


void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
//void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s){

	inBufPtr = &adc_data[0];
	outBufPtr = &dac_data[0];

	codec_rdy = 1;
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){
//void HAL_I2S_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){

	inBufPtr = &adc_data[BUFFER_SIZE/2];
	outBufPtr = &dac_data[BUFFER_SIZE/2];

	codec_rdy = 1;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hsai){

	 outBufPtr = &dac_data[0];
}
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hsai){
	outBufPtr = &dac_data[BUFFER_SIZE / 2];
}
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hsai){
	inBufPtr = &adc_data[0];
	codec_rdy = 1;
}
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hsai){
	inBufPtr = &adc_data[BUFFER_SIZE / 2];
	codec_rdy = 1;
}
