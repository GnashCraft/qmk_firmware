#include QMK_KEYBOARD_H
#ifdef AUDIO_ENABLE
#include "muse.h"
#endif
#include "eeprom.h"
#include "keymap_german.h"
#include "keymap_nordic.h"
#include "keymap_french.h"
#include "keymap_spanish.h"
#include "keymap_hungarian.h"
#include "keymap_swedish.h"
#include "keymap_br_abnt2.h"
#include "keymap_canadian_multilingual.h"
#include "keymap_german_ch.h"
#include "keymap_jp.h"
#include "keymap_korean.h"
#include "keymap_bepo.h"
#include "keymap_italian.h"
#include "keymap_slovenian.h"
#include "keymap_lithuanian_azerty.h"
#include "keymap_danish.h"
#include "keymap_norwegian.h"
#include "keymap_portuguese.h"
#include "keymap_contributions.h"
#include "keymap_czech.h"
#include "keymap_romanian.h"
#include "keymap_russian.h"
#include "keymap_uk.h"
#include "keymap_estonian.h"
#include "keymap_belgian.h"
#include "keymap_us_international.h"
#include "keymap_croatian.h"
#include "keymap_turkish_q.h"
#include "keymap_slovak.h"

#define KC_MAC_UNDO LGUI(KC_Z)
#define KC_MAC_CUT LGUI(KC_X)
#define KC_MAC_COPY LGUI(KC_C)
#define KC_MAC_PASTE LGUI(KC_V)
#define KC_PC_UNDO LCTL(KC_Z)
#define KC_PC_CUT LCTL(KC_X)
#define KC_PC_COPY LCTL(KC_C)
#define KC_PC_PASTE LCTL(KC_V)
#define ES_LESS_MAC KC_GRAVE
#define ES_GRTR_MAC LSFT(KC_GRAVE)
#define ES_BSLS_MAC ALGR(KC_6)
#define NO_PIPE_ALT KC_GRAVE
#define NO_BSLS_ALT KC_EQUAL
#define LSA_T(kc) MT(MOD_LSFT | MOD_LALT, kc)
#define BP_NDSH_MAC ALGR(KC_8)
#define SE_SECT_MAC ALGR(KC_6)

enum planck_keycodes {
  RGB_SLD = EZ_SAFE_RANGE,
  OS_SWITCH_MACOS,
  OS_SWITCH_WINDOWS,
  OS_SWITCH_LINUX,
  QUICK_SWITCH,
  WIN_MAN,
  TAB_LEFT,
  TAB_RIGHT,
};

enum tap_dance_codes {
  DANCE_Q = 0,
  DANCE_W,
  DANCE_E,
  DANCE_R,
  DANCE_T,
  DANCE_Y,
  DANCE_U,
  DANCE_I,
  DANCE_O,
  DANCE_P,
  DANCE_A,
  DANCE_S,
  DANCE_D,
  DANCE_F,
  DANCE_G,
  DANCE_H,
  DANCE_J,
  DANCE_K,
  DANCE_L,
  DANCE_Z,
  DANCE_X,
  DANCE_C,
  DANCE_V,
  DANCE_B,
  DANCE_N,
  DANCE_M,
  DANCE_SPACE,
  DANCE_ENTER,
  DANCE_BSPACE,
  DANCE_TAB,
  DANCE_LEFT,
  DANCE_DOWN,
  DANCE_UP,
  DANCE_RIGHT,
  DANCE_DEL,
  DANCE_HOME,
  DANCE_END,
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
  DANCE_4,
  DANCE_5,
  DANCE_6,
  DANCE_7,
  DANCE_8,
  DANCE_9,
  DANCE_SLASH,
  DANCE_PLS,
  DANCE_MNS,
  DANCE_CBR,
  DANCE_PRN,
  DANCE_SBR,
  DANCE_SQT,
  DANCE_DQT,
  DANCE_ABK,
};

enum planck_layers {
  _BASE,
  _LOWER,
  _RAISE,
  _ADJUST,
  _FN,
};

// Create a global code capturing the current operating system for which to
// configure the keyboard. CUR_OS is changed whenver the user designates the
// proper OS keycode (see process_record_user). See the functions used in
// tap_dance_actions for how context switching is used for modifier keys.
enum os_code {
    MACOS = 0,
    WINDOWS,
    LINUX,
};
enum os_code CUR_OS;

#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [_BASE] = LAYOUT_planck_grid(
    TD(DANCE_Q), TD(DANCE_W), TD(DANCE_E), TD(DANCE_R), TD(DANCE_T),     KC_NO,           KC_NO, TD(DANCE_Y), TD(DANCE_U), TD(DANCE_I), TD(DANCE_O), TD(DANCE_P),
    TD(DANCE_A), TD(DANCE_S), TD(DANCE_D), TD(DANCE_F), TD(DANCE_G),     KC_NO,           KC_NO, TD(DANCE_H), TD(DANCE_J), TD(DANCE_K), TD(DANCE_L), KC_SCOLON,
    TD(DANCE_Z), TD(DANCE_X), TD(DANCE_C), TD(DANCE_V), TD(DANCE_B),     KC_NO,           KC_NO, TD(DANCE_N), TD(DANCE_M), KC_COMMA,    KC_DOT,      KC_QUOTE,
    KC_NO,       KC_NO,       KC_RCTRL,    LOWER,       TD(DANCE_SPACE), TD(DANCE_ENTER), KC_NO, KC_RSHIFT,   RAISE,       KC_RALT,     KC_NO,       KC_NO
  ),

  [_LOWER] = LAYOUT_planck_grid(
    TD(DANCE_TAB),    KC_NO,          KC_NO, KC_NO,          KC_NO, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          
    KC_ESCAPE,        KC_NO,          KC_NO, QUICK_SWITCH,   KC_NO, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          TD(DANCE_LEFT), TD(DANCE_DOWN), TD(DANCE_UP),   TD(DANCE_RIGHT),   
    TD(DANCE_BSPACE), TD(DANCE_DEL),  KC_NO, WIN_MAN,        KC_NO, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          TD(DANCE_HOME), TD(DANCE_END),  TAB_LEFT,       TAB_RIGHT,
    KC_TRANSPARENT,   KC_TRANSPARENT, KC_NO, KC_TRANSPARENT, KC_NO, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_RAISE] = LAYOUT_planck_grid(
    KC_AMPR,        KC_PIPE,        TD(DANCE_CBR),  KC_RCBR,        TD(DANCE_PLS),  KC_NO,          KC_NO, KC_UNDS,  TD(DANCE_7),    TD(DANCE_8),    TD(DANCE_9),    KC_ASTR,        
    TD(DANCE_ABK),  KC_RABK,        TD(DANCE_PRN),  KC_RPRN,        TD(DANCE_MNS),  KC_NO,          KC_NO, KC_EQUAL, TD(DANCE_4),    TD(DANCE_5),    TD(DANCE_6),    TD(DANCE_0),   
    TD(DANCE_SQT),  TD(DANCE_DQT),  TD(DANCE_SBR),  KC_RBRACKET,    KC_BSLASH,      KC_NO,          KC_NO, KC_DOT,   TD(DANCE_1),    TD(DANCE_2),    TD(DANCE_3),    TD(DANCE_SLASH),   
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO, KC_NO,    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT
  ),

  [_ADJUST] = LAYOUT_planck_grid(
    KC_MS_WH_LEFT, KC_MS_WH_DOWN,  KC_MS_UP,           KC_MS_WH_UP,    KC_MS_WH_RIGHT,    KC_TRANSPARENT,  KC_TRANSPARENT, KC_MEDIA_PLAY_PAUSE, KC_AUDIO_VOL_DOWN, KC_AUDIO_VOL_UP, KC_MEDIA_PREV_TRACK, KC_MEDIA_NEXT_TRACK,
    RGB_HUI,       KC_MS_LEFT,     KC_MS_DOWN,         KC_MS_RIGHT,    KC_LSHIFT,         KC_TRANSPARENT,  KC_TRANSPARENT, RGB_VAI,             KC_MS_BTN1,        KC_MS_BTN2,      KC_MS_BTN3,          KC_BRIGHTNESS_UP,
    RGB_HUD,       RGB_TOG,        TOGGLE_LAYER_COLOR, RGB_MOD,        KC_RCTRL,          KC_TRANSPARENT,  KC_TRANSPARENT, RGB_VAD,             KC_MS_ACCEL0,      KC_MS_ACCEL1,    KC_MS_ACCEL2,        KC_BRIGHTNESS_DOWN,
    KC_NO,         KC_TRANSPARENT, KC_NO,              KC_TRANSPARENT, OS_SWITCH_WINDOWS, OS_SWITCH_MACOS, KC_NO,          OS_SWITCH_LINUX,     KC_TRANSPARENT,    OSL(_FN),        KC_TRANSPARENT,      KC_NO
  ),

  [_FN] = LAYOUT_planck_grid(
    KC_F1,          KC_F2,          KC_F3,  KC_F4,  KC_NO, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO, KC_NO, KC_NO, KC_NO,          KC_NO,          
    KC_F5,          KC_F6,          KC_F7,  KC_F8,  KC_NO, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO, KC_NO, KC_NO, KC_NO,          KC_NO,   
    KC_F9,          KC_F10,         KC_F11, KC_F12, KC_NO, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO, KC_NO, KC_NO, KC_NO,          KC_NO,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,  KC_NO,  KC_NO, KC_NO,          KC_NO,          KC_NO, KC_NO, KC_NO, KC_TRANSPARENT, KC_TRANSPARENT
  ),
};

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();

  // Set default current operating system to MacOS.
  // Used for allowing context switching between operating systems upon user
  // switching OS contexts. See os_code enums below for more details.
  CUR_OS = MACOS;
}

#define RED {255,220,201}
#define ORANGE {14,255,255}
#define YELLOW {35,255,255}
#define GREEN {85,203,158}
#define BLUE {154,255,255}
#define PURPLE {202,255,164}
#define WHITE {0,0,255}
#define OFF {0,0,0}
const uint8_t PROGMEM ledmap[][DRIVER_LED_TOTAL][3] = {
    [0] = { BLUE, BLUE, BLUE,  BLUE,   BLUE,   OFF,  OFF,  BLUE,  BLUE,   BLUE,  BLUE, BLUE,
            BLUE, BLUE, BLUE,  BLUE,   BLUE,   BLUE, BLUE, BLUE,  BLUE,   BLUE,  BLUE, BLUE,
            BLUE, BLUE, BLUE,  BLUE,   BLUE,   OFF,  OFF,  BLUE,  BLUE,   BLUE,  BLUE, BLUE,
            OFF,  OFF,  GREEN, ORANGE, YELLOW, RED,        GREEN, ORANGE, GREEN, OFF,  OFF },

    [1] = { YELLOW, OFF,    OFF, OFF,    OFF, OFF,   OFF,   OFF,   OFF,    OFF,  OFF,  OFF,
            YELLOW, OFF,    OFF, GREEN,  OFF, OFF,   OFF,   OFF,   BLUE,   BLUE, BLUE, BLUE,
            YELLOW, YELLOW, OFF, GREEN,  OFF, GREEN, GREEN, OFF,   BLUE,   BLUE, BLUE, BLUE,
            OFF,    OFF,    OFF, ORANGE, OFF, RED,          GREEN, ORANGE, OFF,  OFF,  OFF },

    [2] = { BLUE, BLUE, BLUE,  BLUE,   BLUE,   PURPLE, PURPLE, BLUE, PURPLE, PURPLE, PURPLE, BLUE,
            BLUE, BLUE, BLUE,  BLUE,   BLUE,   OFF,    OFF,    BLUE, PURPLE, PURPLE, PURPLE, PURPLE,
            BLUE, BLUE, BLUE,  BLUE,   BLUE,   OFF,    OFF,    BLUE, PURPLE, PURPLE, PURPLE, BLUE,
            OFF,  OFF,  GREEN, ORANGE, YELLOW, RED,            OFF,  ORANGE, GREEN,  OFF,    OFF },

    [3] = { PURPLE, PURPLE, BLUE,   PURPLE, PURPLE, YELLOW, YELLOW, YELLOW, YELLOW, YELLOW, YELLOW, YELLOW,
            YELLOW, BLUE,   BLUE,   BLUE,   GREEN,  OFF,    OFF,    YELLOW, BLUE,   BLUE,   BLUE,   YELLOW,
            YELLOW, YELLOW, YELLOW, YELLOW, GREEN,  YELLOW, YELLOW, YELLOW, PURPLE, PURPLE, PURPLE, YELLOW,
            OFF,    OFF,    OFF,    ORANGE, WHITE,  WHITE,          WHITE,  ORANGE, ORANGE, OFF,    OFF },

    [4] = { BLUE, BLUE, BLUE, BLUE, OFF, OFF, OFF, OFF, OFF, OFF, OFF, OFF,
            BLUE, BLUE, BLUE, BLUE, OFF, OFF, OFF, OFF, OFF, OFF, OFF, OFF,
            BLUE, BLUE, BLUE, BLUE, OFF, OFF, OFF, OFF, OFF, OFF, OFF, OFF,
            OFF,  OFF,  OFF,  OFF,  OFF, OFF,      OFF, OFF, OFF, OFF, OFF },
};

void set_layer_color(int layer) {
  for (int i = 0; i < DRIVER_LED_TOTAL; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

void rgb_matrix_indicators_user(void) {
  if (keyboard_config.disable_layer_led) { return; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 1:
      set_layer_color(1);
      break;
    case 2:
      set_layer_color(2);
      break;
    case 3:
      set_layer_color(3);
      break;
    case 4:
      set_layer_color(4);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
}

bool handle_os_key(uint16_t macos_key, uint16_t default_key, keyrecord_t *record) {
  uint16_t code;
  switch(CUR_OS) {
    case MACOS: code = macos_key; break;
    default: code = default_key; break;
  }

  if (record->event.pressed) {
    register_code16(code);
  } else {
    unregister_code16(code);
  }

  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case OS_SWITCH_MACOS:
      if (record->event.pressed) {
        CUR_OS = MACOS;
      }
      return false;
    case OS_SWITCH_WINDOWS:
      if (record->event.pressed) {
        CUR_OS = WINDOWS;
      }
      return false;
    case OS_SWITCH_LINUX:
      if (record->event.pressed) {
        CUR_OS = LINUX;
      }
      return false;
    case QUICK_SWITCH: return handle_os_key(KC_RGUI, KC_RALT, record);
    case WIN_MAN: return handle_os_key(RCTL(KC_RALT), KC_LGUI, record);
    case TAB_LEFT: return handle_os_key(RALT(RGUI(KC_LEFT)), RCTL(KC_PGUP), record);
    case TAB_RIGHT: return handle_os_key(RALT(RGUI(KC_RIGHT)), RCTL(KC_PGDOWN), record);
  }
  return true;
}

#ifdef AUDIO_ENABLE
bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
    if (muse_mode) {
        if (IS_LAYER_ON(_RAISE)) {
            if (clockwise) {
                muse_offset++;
            } else {
                muse_offset--;
            }
        } else {
            if (clockwise) {
                muse_tempo+=1;
            } else {
                muse_tempo-=1;
            }
        }
    } else {
        if (clockwise) {
        #ifdef MOUSEKEY_ENABLE
            register_code(KC_MS_WH_DOWN);
            unregister_code(KC_MS_WH_DOWN);
        #else
            register_code(KC_PGDN);
            unregister_code(KC_PGDN);
        #endif
        } else {
        #ifdef MOUSEKEY_ENABLE
            register_code(KC_MS_WH_UP);
            unregister_code(KC_MS_WH_UP);
        #else
            register_code(KC_PGUP);
            unregister_code(KC_PGUP);
        #endif
        }
    }
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    }
#endif
}

bool music_mask_user(uint16_t keycode) {
    switch (keycode) {
    case RAISE:
    case LOWER:
        return false;
    default:
        return true;
    }
}
#endif

uint32_t layer_state_set_user(uint32_t state) {
    return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}

typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[56];

enum modifier {
    MOD_NONE = 0,
    CMD,
    CMD_ALT,
    SFT,
    ALT,
};

typedef struct {
    uint16_t code;
    uint16_t mod_code;
    enum modifier mod;
    char* str_sgl;
    char* str_dbl;
    uint8_t index;
} tap_dance_user_data_t;

uint8_t dance_step(qk_tap_dance_state_t *state);
uint8_t dance_step(qk_tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}

void set_mod_code(tap_dance_user_data_t *data) {
    switch(data->mod) {
        case CMD:
            switch(CUR_OS) {
                case MACOS: data->mod_code = LGUI(data->code); break;
                default: data->mod_code = RCTL(data->code); break;
            }
            break;
        case CMD_ALT:
            switch(CUR_OS) {
                case MACOS: data->mod_code = LGUI(data->code); break;
                default: data->mod_code = RALT(data->code); break;
            }
            break;
        case SFT: data->mod_code = RSFT(data->code); break;
        case ALT:
            switch(CUR_OS) {
                case MACOS: data->mod_code = RALT(data->code); break;
                default: data->mod_code = RCTL(data->code); break;
            }
            break;
        default: break;
    }
}

void on_dance(qk_tap_dance_state_t *state, void *user_data) {
    tap_dance_user_data_t *data = (tap_dance_user_data_t *)user_data;

    if(state->count < 3) {
        set_mod_code(data);
    }

    if(state->count == 3) {
        tap_code16(data->code);
        tap_code16(data->code);
        tap_code16(data->code);
    }

    if(state->count > 3) {
        tap_code16(data->code);
    }
}

void on_dance_finished(qk_tap_dance_state_t *state, void *user_data) {
    tap_dance_user_data_t *data = (tap_dance_user_data_t *)user_data;
    
    dance_state[data->index].step = dance_step(state);
    switch (dance_state[data->index].step) {
        case SINGLE_TAP: register_code16(data->code); break;
        case SINGLE_HOLD: register_code16(data->mod_code); break;
        case DOUBLE_TAP: register_code16(data->code); register_code16(data->code); break;
        case DOUBLE_HOLD: register_code16(data->code); break;
        case DOUBLE_SINGLE_TAP: tap_code16(data->code); register_code16(data->code);
    }
}

void on_dance_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);

    tap_dance_user_data_t *data = (tap_dance_user_data_t *)user_data;    
    switch (dance_state[data->index].step) {
        case SINGLE_TAP: unregister_code16(data->code); break;
        case SINGLE_HOLD: unregister_code16(data->mod_code); break;
        case DOUBLE_TAP: unregister_code16(data->code); break;
        case DOUBLE_HOLD: unregister_code16(data->code); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(data->code); break;
    }

    dance_state[data->index].step = 0;
}

void on_dance_finished_str(qk_tap_dance_state_t *state, void *user_data) {
    tap_dance_user_data_t *data = (tap_dance_user_data_t *)user_data;

    dance_state[data->index].step = dance_step(state);
    switch (dance_state[data->index].step) {
        case SINGLE_TAP: register_code16(data->code); break;
        case SINGLE_HOLD:
            send_string(data->str_sgl);
            SEND_STRING(SS_TAP(X_LEFT));
            break;
        case DOUBLE_TAP: register_code16(data->code); register_code16(data->code); break;
        case DOUBLE_HOLD:
            send_string(data->str_dbl);
            SEND_STRING(SS_TAP(X_LEFT) SS_TAP(X_LEFT));
            break;
        case DOUBLE_SINGLE_TAP: tap_code16(data->code); register_code16(data->code);
    }
}

void on_dance_reset_str(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);

    tap_dance_user_data_t *data = (tap_dance_user_data_t *)user_data;
    switch (dance_state[data->index].step) {
        case SINGLE_TAP: unregister_code16(data->code); break;
        case DOUBLE_TAP: unregister_code16(data->code); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(data->code); break;
    }

    dance_state[data->index].step = 0;
}

void on_dance_finished_abk(qk_tap_dance_state_t *state, void *user_data) {
    tap_dance_user_data_t *data = (tap_dance_user_data_t *)user_data;

    dance_state[data->index].step = dance_step(state);
    switch (dance_state[data->index].step) {
        case SINGLE_TAP: register_code16(data->code); break;
        case SINGLE_HOLD:
            send_string(data->str_sgl);
            SEND_STRING(SS_TAP(X_LEFT));
            break;
        case DOUBLE_TAP: register_code16(data->code); register_code16(data->code); break;
        case DOUBLE_SINGLE_TAP: tap_code16(data->code); register_code16(data->code);
    }
}

void on_dance_home(qk_tap_dance_state_t *state, void *user_data) {
    tap_dance_user_data_t *data = (tap_dance_user_data_t *)user_data;

    switch(CUR_OS) {
        case MACOS:
            data->code = RGUI(KC_LEFT);
            data->mod_code = RGUI(KC_UP);
            break;
        default:
            data->code = KC_HOME;
            data->mod_code = RCTL(KC_HOME);
            break;
    }

    if(state->count == 3) {
        tap_code16(data->code);
        tap_code16(data->code);
        tap_code16(data->code);
    }

    if(state->count > 3) {
        tap_code16(data->code);
    }
}

void on_dance_end(qk_tap_dance_state_t *state, void *user_data) {
    tap_dance_user_data_t *data = (tap_dance_user_data_t *)user_data;

    switch(CUR_OS) {
        case MACOS:
            data->code = RGUI(KC_RIGHT);
            data->mod_code = RGUI(KC_DOWN);
            break;
        default:
            data->code = KC_END;
            data->mod_code = RCTL(KC_END);
            break;
    }

    if(state->count == 3) {
        tap_code16(data->code);
        tap_code16(data->code);
        tap_code16(data->code);
    }

    if(state->count > 3) {
        tap_code16(data->code);
    }
}

#define ACTION_TAP_DANCE_DATA(code, mod, index) \
    { .fn = {on_dance, on_dance_finished, on_dance_reset}, .user_data = (void *)&((tap_dance_user_data_t){code, 0, mod, NULL, NULL, index}), }

#define ACTION_TAP_DANCE_DATA_STR(code, sgl, dbl, index) \
    { .fn = {on_dance, on_dance_finished_str, on_dance_reset_str}, .user_data = (void *)&((tap_dance_user_data_t){code, 0, MOD_NONE, sgl, dbl, index}), }

#define ACTION_TAP_DANCE_HOME(index) \
    { .fn = {on_dance_home, on_dance_finished, on_dance_reset}, .user_data = (void *)&((tap_dance_user_data_t){0, 0, MOD_NONE, NULL, NULL, index}), }

#define ACTION_TAP_DANCE_END(index) \
    { .fn = {on_dance_end, on_dance_finished, on_dance_reset}, .user_data = (void *)&((tap_dance_user_data_t){0, 0, MOD_NONE, NULL, NULL, index}), }

#define ACTION_TAP_DANCE_ABK(index) \
    { .fn = {on_dance, on_dance_finished_abk, on_dance_reset_str}, .user_data = (void *)&((tap_dance_user_data_t){KC_LABK, 0, MOD_NONE, "<>", NULL, index}), }

qk_tap_dance_action_t tap_dance_actions[] = {
        [DANCE_Q] = ACTION_TAP_DANCE_DATA(KC_Q, CMD, DANCE_Q),
        [DANCE_W] = ACTION_TAP_DANCE_DATA(KC_W, CMD, DANCE_W),
        [DANCE_E] = ACTION_TAP_DANCE_DATA(KC_E, CMD, DANCE_E),
        [DANCE_R] = ACTION_TAP_DANCE_DATA(KC_R, CMD, DANCE_R),
        [DANCE_T] = ACTION_TAP_DANCE_DATA(KC_T, CMD, DANCE_T),
        [DANCE_Y] = ACTION_TAP_DANCE_DATA(KC_Y, CMD, DANCE_Y),
        [DANCE_U] = ACTION_TAP_DANCE_DATA(KC_U, CMD, DANCE_U),
        [DANCE_I] = ACTION_TAP_DANCE_DATA(KC_I, CMD, DANCE_I),
        [DANCE_O] = ACTION_TAP_DANCE_DATA(KC_O, CMD, DANCE_O),
        [DANCE_P] = ACTION_TAP_DANCE_DATA(KC_P, CMD, DANCE_P),
        [DANCE_A] = ACTION_TAP_DANCE_DATA(KC_A, CMD, DANCE_A),
        [DANCE_S] = ACTION_TAP_DANCE_DATA(KC_S, CMD, DANCE_S),
        [DANCE_D] = ACTION_TAP_DANCE_DATA(KC_D, CMD, DANCE_D),
        [DANCE_F] = ACTION_TAP_DANCE_DATA(KC_F, CMD, DANCE_F),
        [DANCE_G] = ACTION_TAP_DANCE_DATA(KC_G, CMD, DANCE_G),
        [DANCE_H] = ACTION_TAP_DANCE_DATA(KC_H, CMD, DANCE_H),
        [DANCE_J] = ACTION_TAP_DANCE_DATA(KC_J, CMD, DANCE_J),
        [DANCE_K] = ACTION_TAP_DANCE_DATA(KC_K, CMD, DANCE_K),
        [DANCE_L] = ACTION_TAP_DANCE_DATA(KC_L, CMD, DANCE_L),
        [DANCE_Z] = ACTION_TAP_DANCE_DATA(KC_Z, CMD, DANCE_Z),
        [DANCE_X] = ACTION_TAP_DANCE_DATA(KC_X, CMD, DANCE_X),
        [DANCE_C] = ACTION_TAP_DANCE_DATA(KC_C, CMD, DANCE_C),
        [DANCE_V] = ACTION_TAP_DANCE_DATA(KC_V, CMD, DANCE_V),
        [DANCE_B] = ACTION_TAP_DANCE_DATA(KC_B, CMD, DANCE_B),
        [DANCE_N] = ACTION_TAP_DANCE_DATA(KC_N, CMD, DANCE_N),
        [DANCE_M] = ACTION_TAP_DANCE_DATA(KC_M, CMD, DANCE_M),
        [DANCE_SPACE] = ACTION_TAP_DANCE_DATA(KC_SPACE, CMD, DANCE_SPACE),
        [DANCE_ENTER] = ACTION_TAP_DANCE_DATA(KC_ENTER, CMD, DANCE_ENTER),
        [DANCE_BSPACE] = ACTION_TAP_DANCE_DATA(KC_BSPACE, ALT, DANCE_BSPACE),
        [DANCE_TAB] = ACTION_TAP_DANCE_DATA(KC_TAB, CMD_ALT, DANCE_TAB),
        [DANCE_LEFT] = ACTION_TAP_DANCE_DATA(KC_LEFT, ALT, DANCE_LEFT),
        [DANCE_DOWN] = ACTION_TAP_DANCE_DATA(KC_DOWN, ALT, DANCE_DOWN),
        [DANCE_UP] = ACTION_TAP_DANCE_DATA(KC_UP, ALT, DANCE_UP),
        [DANCE_RIGHT] = ACTION_TAP_DANCE_DATA(KC_RIGHT, ALT, DANCE_RIGHT),
        [DANCE_DEL] = ACTION_TAP_DANCE_DATA(KC_DELETE, ALT, DANCE_DEL),
        [DANCE_HOME] = ACTION_TAP_DANCE_HOME(DANCE_HOME),
        [DANCE_END] = ACTION_TAP_DANCE_END(DANCE_END),
        [DANCE_0] = ACTION_TAP_DANCE_DATA(KC_0, SFT, DANCE_0),
        [DANCE_1] = ACTION_TAP_DANCE_DATA(KC_1, SFT, DANCE_1),
        [DANCE_2] = ACTION_TAP_DANCE_DATA(KC_2, SFT, DANCE_2),
        [DANCE_3] = ACTION_TAP_DANCE_DATA(KC_3, SFT, DANCE_3),
        [DANCE_4] = ACTION_TAP_DANCE_DATA(KC_4, SFT, DANCE_4),
        [DANCE_5] = ACTION_TAP_DANCE_DATA(KC_5, SFT, DANCE_5),
        [DANCE_6] = ACTION_TAP_DANCE_DATA(KC_6, SFT, DANCE_6),
        [DANCE_7] = ACTION_TAP_DANCE_DATA(KC_7, SFT, DANCE_7),
        [DANCE_8] = ACTION_TAP_DANCE_DATA(KC_8, SFT, DANCE_8),
        [DANCE_9] = ACTION_TAP_DANCE_DATA(KC_9, SFT, DANCE_9),
        [DANCE_SLASH] = ACTION_TAP_DANCE_DATA(KC_SLASH, SFT, DANCE_SLASH),
        [DANCE_PLS] = ACTION_TAP_DANCE_DATA(KC_PLUS, CMD, DANCE_PLS),
        [DANCE_MNS] = ACTION_TAP_DANCE_DATA(KC_MINUS, CMD, DANCE_MNS),
        // Tap dance for macros: { -> {}, {};
        [DANCE_CBR] = ACTION_TAP_DANCE_DATA_STR(KC_LCBR, "{}", "{};", DANCE_CBR),
        // Tap dance for macros: ( -> (), ();
        [DANCE_PRN] = ACTION_TAP_DANCE_DATA_STR(KC_LPRN, "()", "();", DANCE_PRN),
        // Tap dance for macros: [ -> [], [];
        [DANCE_SBR] = ACTION_TAP_DANCE_DATA_STR(KC_LBRACKET, "[]", "[];", DANCE_SBR),
        // Tap dance for macros: ` -> '', '';
        [DANCE_SQT] = ACTION_TAP_DANCE_DATA_STR(KC_GRAVE, "''", "'';", DANCE_SQT),
        // Tap dance for macros: ~ -> "", "";
        [DANCE_DQT] = ACTION_TAP_DANCE_DATA_STR(KC_TILD, "\"\"", "\"\";", DANCE_DQT),
        // TAP dance for macros: < -> <>
        [DANCE_ABK] = ACTION_TAP_DANCE_ABK(DANCE_ABK),
};
