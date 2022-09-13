#pragma once

#ifdef AUDIO_ENABLE
// Songs defined in quantum/audio/song_list.h.
#    define STARTUP_SONG SONG(NO_SOUND)
#    ifndef MACOS_SONG
#        define MACOS_SONG SONG(GUITAR_SOUND)
#    endif
#    ifndef WINDOWS_SONG
#        define WINDOWS_SONG SONG(VIOLIN_SOUND)
#    endif
#endif

#define MIDI_BASIC

#define ENCODER_RESOLUTION 4

/*
  Set any config.h overrides for your specific keymap here.
  See config.h options at https://docs.qmk.fm/#/config_options?id=the-configh-file
*/
#define ORYX_CONFIGURATOR
#define USB_SUSPEND_WAKEUP_DELAY 0
#define IGNORE_MOD_TAP_INTERRUPT
#define FIRMWARE_VERSION u8"njLQD/G7m3j"
#define RAW_USAGE_PAGE 0xFF60
#define RAW_USAGE_ID 0x61
#define RGB_MATRIX_STARTUP_SPD 60
