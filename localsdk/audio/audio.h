#ifndef _LOCALSDK_AUDIO_H_
#define _LOCALSDK_AUDIO_H_

#include <stdbool.h>

bool audio_is_enabled(int channel);
bool audio_init(void);
bool audio_free(void);

/* AO device/channel accessors for speaker.c */
int audio_get_ao_dev(void);
int audio_get_ao_chn(void);

#endif
