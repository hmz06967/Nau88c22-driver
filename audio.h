#ifndef __AUDIO_H__
#define __AUDIO_H__

#define AUDIO_OK 0
#define AUDIO_ERROR 1

#include <string.h>
#include "codec.h"

enum audio_com{
    MIC_INTR,
    INP_MIXER, 
    OUTP_MIXER, 
    AUD_VOL, 
    OUTP_DRV, 
    AUD_FILTER
};

uint8_t audio_init(alc_control_t *ALC_M);
uint8_t microfone_interface();
uint8_t input_mixer();
uint8_t output_mixer();
uint8_t audio_volume(enum vol_mode mode);
uint8_t output_driver();
uint8_t audio_filter();


#endif