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
#define NO_BSLS_ALT KC_EQL
#define LSA_T(kc) MT(MOD_LSFT | MOD_LALT, kc)
#define BP_NDSH_MAC ALGR(KC_8)
#define SE_SECT_MAC ALGR(KC_6)
#define TOG_LCOL TOGGLE_LAYER_COLOR

#ifdef AUDIO_ENABLE
    float macos_song[][2] = MACOS_SONG;
    float windows_song[][2] = WINDOWS_SONG;
#endif

enum planck_keycodes {
  RGB_SLD = EZ_SAFE_RANGE,
};

enum tap_dance_codes {
    // MACOS TAP DANCE CODES
    TD_MQ = 0,
    TD_MW,
    TD_ME,
    TD_MR,
    TD_MT,
    TD_MY,
    TD_MU,
    TD_MI,
    TD_MO,
    TD_MP,
    TD_MA,
    TD_MS,
    TD_MD,
    TD_MF,
    TD_MG,
    TD_MH,
    TD_MJ,
    TD_MK,
    TD_ML,
    TD_MZ,
    TD_MX,
    TD_MC,
    TD_MV,
    TD_MB,
    TD_MN,
    TD_MM,
    TD_MTAB,
    TD_MBSPC,
    TD_MDEL,
    TD_MLEFT,
    TD_MDOWN,
    TD_MUP,
    TD_MRGHT,
    TD_MBEG,
    TD_MEND,
    TD_MPLUS,
    TD_MMINS,

    // WINDOWS TAP DANCE CODES
    TD_WQ,
    TD_WW,
    TD_WE,
    TD_WR,
    TD_WT,
    TD_WY,
    TD_WU,
    TD_WI,
    TD_WO,
    TD_WP,
    TD_WA,
    TD_WS,
    TD_WD,
    TD_WF,
    TD_WG,
    TD_WH,
    TD_WJ,
    TD_WK,
    TD_WL,
    TD_WZ,
    TD_WX,
    TD_WC,
    TD_WV,
    TD_WB,
    TD_WN,
    TD_WM,
    TD_WTAB,
    TD_WBSPC,
    TD_WDEL,
    TD_WLEFT,
    TD_WDOWN,
    TD_WUP,
    TD_WRGHT,
    TD_WBEG,
    TD_WEND,
    TD_WPLUS,
    TD_WMINS,

    // SHARED TAP DANCE CODES
    TD_SPC,
    TD_ENT,
    TD_0,
    TD_1,
    TD_2,
    TD_3,
    TD_4,
    TD_5,
    TD_6,
    TD_7,
    TD_8,
    TD_9,
    TD_SLSH,
    TD_CBR,
    TD_PRN,
    TD_BRC,
    TD_SQT,
    TD_DQT,
    TD_ABK,
};

enum planck_layers {
  _MBASE,
  _MLOWER,
  _MRAISE,
  _MADJUST,
  _WBASE,
  _WLOWER,
  _WRAISE,
  _WADJUST,
};

#define M_BASE TO(_MBASE)
#define M_LOWER MO(_MLOWER)
#define M_RAISE MO(_MRAISE)
#define W_BASE TO(_WBASE)
#define W_LOWER MO(_WLOWER)
#define W_RAISE MO(_WRAISE)
#define W_ADJUST MO(_WADJUST)
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [_MBASE] = LAYOUT_planck_grid(
    TD(TD_MQ), TD(TD_MW), TD(TD_ME), TD(TD_MR), TD(TD_MT),  KC_NO,      KC_NO, TD(TD_MY), TD(TD_MU), TD(TD_MI), TD(TD_MO), TD(TD_MP),    
    TD(TD_MA), TD(TD_MS), TD(TD_MD), TD(TD_MF), TD(TD_MG),  KC_NO,      KC_NO, TD(TD_MH), TD(TD_MJ), TD(TD_MK), TD(TD_ML), KC_SCLN,      
    TD(TD_MZ), TD(TD_MX), TD(TD_MC), TD(TD_MV), TD(TD_MB),  KC_NO,      KC_NO, TD(TD_MN), TD(TD_MM), KC_COMM,   KC_DOT,    KC_QUOT,       
    KC_NO,     KC_NO,     KC_RCTL,   M_LOWER,   TD(TD_SPC), TD(TD_ENT), KC_NO, KC_RSFT,   M_RAISE,   KC_RALT,   KC_NO,     KC_NO
  ),

  [_MLOWER] = LAYOUT_planck_grid(
    TD(TD_MTAB),  KC_NO,       KC_NO, KC_NO,         KC_NO, KC_TRNS, KC_TRNS, KC_NO,   KC_NO,        KC_NO,        KC_NO,               KC_NO,          
    KC_ESC,       KC_NO,       KC_NO, KC_RCMD,       KC_NO, KC_TRNS, KC_TRNS, KC_NO,   TD(TD_MLEFT), TD(TD_MDOWN), TD(TD_MUP),          TD(TD_MRGHT),   
    TD(TD_MBSPC), TD(TD_MDEL), KC_NO, RCTL(KC_ROPT), KC_NO, KC_TRNS, KC_TRNS, KC_NO,   TD(TD_MBEG),  TD(TD_MEND),  ROPT(RCMD(KC_LEFT)), ROPT(RCMD(KC_RGHT)),
    KC_TRNS,      KC_TRNS,     KC_NO, KC_TRNS,       KC_NO, KC_TRNS, KC_NO,   KC_TRNS, KC_TRNS,      KC_NO,        KC_TRNS,             KC_TRNS
  ),

  [_MRAISE] = LAYOUT_planck_grid(
    KC_AMPR,    KC_PIPE,    TD(TD_CBR), KC_RCBR, TD(TD_MPLUS), KC_NO,   KC_NO, KC_UNDS, TD(TD_7), TD(TD_8), TD(TD_9), KC_ASTR,        
    TD(TD_ABK), KC_RABK,    TD(TD_PRN), KC_RPRN, TD(TD_MMINS), KC_NO,   KC_NO, KC_EQL,  TD(TD_4), TD(TD_5), TD(TD_6), TD(TD_0),   
    TD(TD_SQT), TD(TD_DQT), TD(TD_BRC), KC_RBRC, KC_BSLS,      KC_NO,   KC_NO, KC_DOT,  TD(TD_1), TD(TD_2), TD(TD_3), TD(TD_SLSH),   
    KC_TRNS,    KC_TRNS,    KC_TRNS,    KC_TRNS, KC_TRNS,      KC_TRNS, KC_NO, KC_NO,   KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS
  ),

  [_MADJUST] = LAYOUT_planck_grid(
    KC_WH_L, KC_WH_D, KC_MS_UP, KC_WH_U, KC_WH_R, KC_TRNS, KC_TRNS, KC_MPLY, KC_VOLD, KC_VOLU, KC_MPRV, KC_MNXT,
    RGB_HUI, KC_MS_L, KC_MS_D,  KC_MS_R, KC_RSFT, KC_TRNS, KC_TRNS, RGB_VAI, KC_BTN1, KC_BTN3, KC_BTN2, KC_BRIU,
    RGB_HUD, RGB_TOG, TOG_LCOL, RGB_MOD, KC_RCTL, KC_TRNS, KC_TRNS, RGB_VAD, KC_ACL0, KC_ACL1, KC_ACL2, KC_BRID,
    KC_TRNS, KC_TRNS, KC_NO,    KC_TRNS, KC_NO,   W_BASE,  KC_NO,   KC_NO,   KC_TRNS, KC_NO,   KC_TRNS, KC_TRNS
  ),

  [_WBASE] = LAYOUT_planck_grid(
    TD(TD_WQ), TD(TD_WW), TD(TD_WE), TD(TD_WR), TD(TD_WT), KC_TRNS, KC_TRNS, TD(TD_WY), TD(TD_WU), TD(TD_WI), TD(TD_WO), TD(TD_WP),   
    TD(TD_WA), TD(TD_WS), TD(TD_WD), TD(TD_WF), TD(TD_WG), KC_TRNS, KC_TRNS, TD(TD_WH), TD(TD_WJ), TD(TD_WK), TD(TD_WL), KC_TRNS, 
    TD(TD_WZ), TD(TD_WX), TD(TD_WC), TD(TD_WV), TD(TD_WB), KC_TRNS, KC_TRNS, TD(TD_WN), TD(TD_WM), KC_TRNS,   KC_TRNS,   KC_TRNS, 
    KC_TRNS,   KC_TRNS,   KC_TRNS,   W_LOWER,   KC_TRNS,   KC_TRNS, KC_NO,   KC_TRNS,   W_RAISE,   KC_TRNS,   KC_TRNS,   KC_TRNS
  ),

  [_WLOWER] = LAYOUT_planck_grid(
    TD(TD_WTAB),  KC_NO,       KC_NO, KC_NO,   KC_NO, KC_TRNS, KC_TRNS, KC_NO,   KC_NO,        KC_NO,        KC_NO,         KC_NO,          
    KC_ESC,       KC_NO,       KC_NO, KC_RALT, KC_NO, KC_TRNS, KC_TRNS, KC_NO,   TD(TD_WLEFT), TD(TD_WDOWN), TD(TD_WUP),    TD(TD_WRGHT),   
    TD(TD_WBSPC), TD(TD_WDEL), KC_NO, KC_RWIN, KC_NO, KC_TRNS, KC_TRNS, KC_NO,   TD(TD_WBEG),  TD(TD_WEND),  RCTL(KC_PGUP), RCTL(KC_PGDN),
    KC_TRNS,      KC_TRNS,     KC_NO, KC_TRNS, KC_NO, KC_TRNS, KC_NO,   KC_TRNS, KC_TRNS,      KC_NO,        KC_TRNS,       KC_TRNS
  ),

  [_WRAISE] = LAYOUT_planck_grid(
    KC_AMPR,    KC_PIPE,    TD(TD_CBR), KC_RCBR,  TD(TD_WPLUS), KC_TRNS, KC_TRNS, KC_UNDS, TD(TD_7), TD(TD_8), TD(TD_9), KC_ASTR,        
    TD(TD_ABK), KC_RABK,    TD(TD_PRN), KC_RPRN,  TD(TD_WMINS), KC_TRNS, KC_TRNS, KC_EQL,  TD(TD_4), TD(TD_5), TD(TD_6), TD(TD_0),   
    TD(TD_SQT), TD(TD_DQT), TD(TD_BRC), KC_RBRC,  KC_BSLS,      KC_TRNS, KC_TRNS, KC_DOT,  TD(TD_1), TD(TD_2), TD(TD_3), TD(TD_SLSH),   
    KC_TRNS,    KC_TRNS,    KC_TRNS,    KC_TRNS,  KC_TRNS,      KC_TRNS, KC_NO,   KC_NO,   KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS
  ),

  [_WADJUST] = LAYOUT_planck_grid(
    KC_WH_L, KC_WH_D, KC_MS_UP, KC_WH_U, KC_WH_R, KC_TRNS, KC_TRNS, KC_MPLY, KC_VOLD, KC_VOLU, KC_MPRV, KC_MNXT,
    RGB_HUI, KC_MS_L, KC_MS_D,  KC_MS_R, KC_RSFT, KC_TRNS, KC_TRNS, RGB_VAI, KC_BTN1, KC_BTN3, KC_BTN2, KC_BRIU,
    RGB_HUD, RGB_TOG, TOG_LCOL, RGB_MOD, KC_RCTL, KC_TRNS, KC_TRNS, RGB_VAD, KC_ACL0, KC_ACL1, KC_ACL2, KC_BRID,
    KC_TRNS, KC_TRNS, KC_NO,    KC_TRNS, KC_NO,   M_BASE,  KC_NO,   KC_NO,   KC_TRNS, KC_NO,   KC_TRNS, KC_TRNS
  ),
};

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
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
    [_MBASE] = { BLUE, BLUE, BLUE,  BLUE,   BLUE,   OFF,  OFF,  BLUE,  BLUE,   BLUE,  BLUE, BLUE,
                 BLUE, BLUE, BLUE,  BLUE,   BLUE,   BLUE, BLUE, BLUE,  BLUE,   BLUE,  BLUE, BLUE,
                 BLUE, BLUE, BLUE,  BLUE,   BLUE,   OFF,  OFF,  BLUE,  BLUE,   BLUE,  BLUE, BLUE,
                 OFF,  OFF,  GREEN, ORANGE, YELLOW,     RED,    GREEN, ORANGE, GREEN, OFF,  OFF },

    [_MLOWER] = { YELLOW, OFF,    OFF, OFF,    OFF, OFF,   OFF,   OFF,   OFF,    OFF,  OFF,  OFF,
                  YELLOW, OFF,    OFF, GREEN,  OFF, OFF,   OFF,   OFF,   BLUE,   BLUE, BLUE, BLUE,
                  YELLOW, YELLOW, OFF, GREEN,  OFF, GREEN, GREEN, OFF,   BLUE,   BLUE, BLUE, BLUE,
                  OFF,    OFF,    OFF, ORANGE, OFF,     RED,      GREEN, ORANGE, OFF,  OFF,  OFF },

    [_MRAISE] = { BLUE, BLUE, BLUE,  BLUE,   BLUE,   PURPLE, PURPLE, BLUE, PURPLE, PURPLE, PURPLE, BLUE,
                  BLUE, BLUE, BLUE,  BLUE,   BLUE,   OFF,    OFF,    BLUE, PURPLE, PURPLE, PURPLE, PURPLE,
                  BLUE, BLUE, BLUE,  BLUE,   BLUE,   OFF,    OFF,    BLUE, PURPLE, PURPLE, PURPLE, BLUE,
                  OFF,  OFF,  GREEN, ORANGE, YELLOW,      RED,       OFF,  ORANGE, GREEN,  OFF,    OFF },

    [_MADJUST] = { PURPLE, PURPLE, BLUE,   PURPLE, PURPLE, YELLOW, YELLOW, YELLOW, YELLOW, YELLOW, YELLOW, YELLOW,
                   YELLOW, BLUE,   BLUE,   BLUE,   GREEN,  OFF,    OFF,    YELLOW, BLUE,   BLUE,   BLUE,   YELLOW,
                   YELLOW, YELLOW, YELLOW, YELLOW, GREEN,  YELLOW, YELLOW, YELLOW, PURPLE, PURPLE, PURPLE, YELLOW,
                   OFF,    OFF,    OFF,    ORANGE, OFF,       WHITE,       OFF,    ORANGE, OFF,    OFF,    OFF },

    [_WBASE] = { BLUE, BLUE, BLUE,  BLUE,   BLUE,   OFF,  OFF,  BLUE,  BLUE,   BLUE,  BLUE, BLUE,
                 BLUE, BLUE, BLUE,  BLUE,   BLUE,   BLUE, BLUE, BLUE,  BLUE,   BLUE,  BLUE, BLUE,
                 BLUE, BLUE, BLUE,  BLUE,   BLUE,   OFF,  OFF,  BLUE,  BLUE,   BLUE,  BLUE, BLUE,
                 OFF,  OFF,  GREEN, ORANGE, YELLOW,     RED,    GREEN, ORANGE, GREEN, OFF,  OFF },

    [_WLOWER] = { YELLOW, OFF,    OFF, OFF,    OFF, OFF,    OFF,    OFF,   OFF,    OFF,  OFF,  OFF,
                  YELLOW, OFF,    OFF, GREEN,  OFF, OFF,    OFF,    OFF,   BLUE,   BLUE, BLUE, BLUE,
                  YELLOW, YELLOW, OFF, GREEN,  OFF, GREEN,  GREEN,  OFF,   BLUE,   BLUE, BLUE, BLUE,
                  OFF,    OFF,    OFF, ORANGE, OFF,       RED,      GREEN, ORANGE, OFF,  OFF,  OFF },

    [_WRAISE] = { BLUE, BLUE, BLUE,  BLUE,   BLUE,   PURPLE, PURPLE, BLUE, PURPLE, PURPLE, PURPLE, BLUE,
                  BLUE, BLUE, BLUE,  BLUE,   BLUE,   OFF,    OFF,    BLUE, PURPLE, PURPLE, PURPLE, PURPLE,
                  BLUE, BLUE, BLUE,  BLUE,   BLUE,   OFF,    OFF,    BLUE, PURPLE, PURPLE, PURPLE, BLUE,
                  OFF,  OFF,  GREEN, ORANGE, YELLOW,      RED,       OFF,  ORANGE, GREEN,  OFF,    OFF },

    [_WADJUST] = { PURPLE, PURPLE, BLUE,   PURPLE, PURPLE, YELLOW, YELLOW, YELLOW, YELLOW, YELLOW, YELLOW, YELLOW,
                   YELLOW, BLUE,   BLUE,   BLUE,   GREEN,  OFF,    OFF,    YELLOW, BLUE,   BLUE,   BLUE,   YELLOW,
                   YELLOW, YELLOW, YELLOW, YELLOW, GREEN,  YELLOW, YELLOW, YELLOW, PURPLE, PURPLE, PURPLE, YELLOW,
                   OFF,    OFF,    OFF,    ORANGE, OFF,       WHITE,       OFF,    ORANGE, OFF,    OFF,    OFF },
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

  uint8_t layer = biton32(layer_state);
  switch (layer) {
    case _MBASE ... _WADJUST:
      set_layer_color(layer);
      break;
    default:
      if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
        rgb_matrix_set_color_all(0, 0, 0);
      }
      break;
  }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case M_BASE:
      if (record->event.pressed) {
#ifdef AUDIO_ENABLE
        PLAY_SONG(macos_song);
#endif
      }
      break;
    case W_BASE:
      if (record->event.pressed) {
#ifdef AUDIO_ENABLE
        PLAY_SONG(windows_song);
#endif
      }
      break;
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
        if (IS_LAYER_ON(_MRAISE)) {
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
            register_code(KC_WH_D);
            unregister_code(KC_WH_D);
        #else
            register_code(KC_PGDN);
            unregister_code(KC_PGDN);
        #endif
        } else {
        #ifdef MOUSEKEY_ENABLE
            register_code(KC_WH_U);
            unregister_code(KC_WH_U);
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
    // case RAISE:
    // case LOWER:
    //     return false;
    default:
        return true;
    }
}
#endif

uint32_t layer_state_set_user(uint32_t state) {
    // This gets the same behavior as tri-layer switching on the original
    // planck, but includes support for the windows layers. Should be half the
    // number of bits in a long.
    uint32_t threshold = 16;
    if (state >= threshold) {
        return update_tri_layer_state(state, _WLOWER, _WRAISE, _WADJUST);
    }
    return update_tri_layer_state(state, _MLOWER, _MRAISE, _MADJUST);
}

typedef struct {
    uint16_t code;
    uint16_t mod_code;
    uint8_t index;
} td_user_data_t;

typedef struct {
    uint16_t code;
    char* str_sgl;
    char* str_dbl;
    uint8_t index;
} td_user_data_str_t;

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

static tap dance_state[98];

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

void on_dance_finished(qk_tap_dance_state_t *state, void *user_data) {
    td_user_data_t *data = (td_user_data_t *)user_data;
    
    dance_state[data->index].step = dance_step(state);
    switch (dance_state[data->index].step) {
        case SINGLE_TAP: register_code16(data->code); break;
        case SINGLE_HOLD: register_code16(data->mod_code); break;
        case DOUBLE_TAP: register_code16(data->code); register_code16(data->code); break;
        case DOUBLE_HOLD: register_code16(data->code); break;
        case DOUBLE_SINGLE_TAP: tap_code16(data->code); register_code16(data->code); break;
        default: register_code16(data->code); break;
    }
}

void on_dance_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);

    td_user_data_t *data = (td_user_data_t *)user_data;    
    switch (dance_state[data->index].step) {
        case SINGLE_TAP: unregister_code16(data->code); break;
        case SINGLE_HOLD: unregister_code16(data->mod_code); break;
        case DOUBLE_TAP: unregister_code16(data->code); break;
        case DOUBLE_HOLD: unregister_code16(data->code); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(data->code); break;
        default: unregister_code16(data->code); break;
    }

    dance_state[data->index].step = 0;
}

void on_dance_finished_str(qk_tap_dance_state_t *state, void *user_data) {
    td_user_data_str_t *data = (td_user_data_str_t *)user_data;

    dance_state[data->index].step = dance_step(state);
    switch (dance_state[data->index].step) {
        case SINGLE_TAP: register_code16(data->code); break;
        case SINGLE_HOLD:
            if (data->str_sgl) {
                send_string(data->str_sgl);
                SEND_STRING(SS_TAP(X_LEFT));
            }
            break;
        case DOUBLE_TAP: register_code16(data->code); register_code16(data->code); break;
        case DOUBLE_HOLD:
            if (data->str_dbl) {
                send_string(data->str_dbl);
                SEND_STRING(SS_TAP(X_LEFT) SS_TAP(X_LEFT));
            }
            break;
        case DOUBLE_SINGLE_TAP: tap_code16(data->code); register_code16(data->code); break;
        default: register_code16(data->code); break;
    }
}

void on_dance_reset_str(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);

    td_user_data_str_t *data = (td_user_data_str_t *)user_data;
    switch (dance_state[data->index].step) {
        case SINGLE_TAP: unregister_code16(data->code); break;
        case SINGLE_HOLD: break;
        case DOUBLE_TAP: unregister_code16(data->code); break;
        case DOUBLE_HOLD: break;
        case DOUBLE_SINGLE_TAP: unregister_code16(data->code); break;
        default: unregister_code16(data->code); break;
    }

    dance_state[data->index].step = 0;
}

#define ACTION_TAP_DANCE_DATA(code, mod_code, index) \
    { .fn = {NULL, on_dance_finished, on_dance_reset}, .user_data = (void *)&((td_user_data_t){code, mod_code, index}) }

#define ACTION_TAP_DANCE_STR(code, sgl, dbl, index) \
    { .fn = {NULL, on_dance_finished_str, on_dance_reset_str}, .user_data = (void *)&((td_user_data_str_t){code, sgl, dbl, index}) }

qk_tap_dance_action_t tap_dance_actions[] = {
    // MACOS TAP DANCE ACTIONS
    [TD_MQ] = ACTION_TAP_DANCE_DATA(KC_Q, RCMD(KC_Q), TD_MQ),
    [TD_MW] = ACTION_TAP_DANCE_DATA(KC_W, RCMD(KC_W), TD_MW),
    [TD_ME] = ACTION_TAP_DANCE_DATA(KC_E, RCMD(KC_E), TD_ME),
    [TD_MR] = ACTION_TAP_DANCE_DATA(KC_R, RCMD(KC_R), TD_MR),
    [TD_MT] = ACTION_TAP_DANCE_DATA(KC_T, RCMD(KC_T), TD_MT),
    [TD_MY] = ACTION_TAP_DANCE_DATA(KC_Y, RCMD(KC_Y), TD_MY),
    [TD_MU] = ACTION_TAP_DANCE_DATA(KC_U, RCMD(KC_U), TD_MU),
    [TD_MI] = ACTION_TAP_DANCE_DATA(KC_I, RCMD(KC_I), TD_MI),
    [TD_MO] = ACTION_TAP_DANCE_DATA(KC_O, RCMD(KC_O), TD_MO),
    [TD_MP] = ACTION_TAP_DANCE_DATA(KC_P, RCMD(KC_P), TD_MP),
    [TD_MA] = ACTION_TAP_DANCE_DATA(KC_A, RCMD(KC_A), TD_MA),
    [TD_MS] = ACTION_TAP_DANCE_DATA(KC_S, RCMD(KC_S), TD_MS),
    [TD_MD] = ACTION_TAP_DANCE_DATA(KC_D, RCMD(KC_D), TD_MD),
    [TD_MF] = ACTION_TAP_DANCE_DATA(KC_F, RCMD(KC_F), TD_MF),
    [TD_MG] = ACTION_TAP_DANCE_DATA(KC_G, RCMD(KC_G), TD_MG),
    [TD_MH] = ACTION_TAP_DANCE_DATA(KC_H, RCMD(KC_H), TD_MH),
    [TD_MJ] = ACTION_TAP_DANCE_DATA(KC_J, RCMD(KC_J), TD_MJ),
    [TD_MK] = ACTION_TAP_DANCE_DATA(KC_K, RCMD(KC_K), TD_MK),
    [TD_ML] = ACTION_TAP_DANCE_DATA(KC_L, RCMD(KC_L), TD_ML),
    [TD_MZ] = ACTION_TAP_DANCE_DATA(KC_Z, RCMD(KC_Z), TD_MZ),
    [TD_MX] = ACTION_TAP_DANCE_DATA(KC_X, RCMD(KC_X), TD_MX),
    [TD_MC] = ACTION_TAP_DANCE_DATA(KC_C, RCMD(KC_C), TD_MC),
    [TD_MV] = ACTION_TAP_DANCE_DATA(KC_V, RCMD(KC_V), TD_MV),
    [TD_MB] = ACTION_TAP_DANCE_DATA(KC_B, RCMD(KC_B), TD_MB),
    [TD_MN] = ACTION_TAP_DANCE_DATA(KC_N, RCMD(KC_N), TD_MN),
    [TD_MM] = ACTION_TAP_DANCE_DATA(KC_M, RCMD(KC_M), TD_MM),
    [TD_MTAB] = ACTION_TAP_DANCE_DATA(KC_TAB, RCMD(KC_TAB), TD_MTAB),
    [TD_MBSPC] = ACTION_TAP_DANCE_DATA(KC_BSPC, ROPT(KC_BSPC), TD_MBSPC),
    [TD_MDEL] = ACTION_TAP_DANCE_DATA(KC_DEL, ROPT(KC_DEL), TD_MDEL),
    [TD_MLEFT] = ACTION_TAP_DANCE_DATA(KC_LEFT, ROPT(KC_LEFT), TD_MLEFT),
    [TD_MDOWN] = ACTION_TAP_DANCE_DATA(KC_DOWN, ROPT(KC_DOWN), TD_MDOWN),
    [TD_MUP] = ACTION_TAP_DANCE_DATA(KC_UP, ROPT(KC_UP), TD_MUP),
    [TD_MRGHT] = ACTION_TAP_DANCE_DATA(KC_RGHT, ROPT(KC_RGHT), TD_MRGHT),
    [TD_MBEG] = ACTION_TAP_DANCE_DATA(RCMD(KC_LEFT), RCMD(KC_UP), TD_MBEG),
    [TD_MEND] = ACTION_TAP_DANCE_DATA(RCMD(KC_RGHT), RCMD(KC_DOWN), TD_MEND),
    [TD_MPLUS] = ACTION_TAP_DANCE_DATA(KC_PLUS, RCMD(KC_PLUS), TD_MPLUS),
    [TD_MMINS] = ACTION_TAP_DANCE_DATA(KC_MINS, RCMD(KC_MINS), TD_MMINS),

    // WINDOWS TAP DANCE CODES
    [TD_WQ] = ACTION_TAP_DANCE_DATA(KC_Q, RCTL(KC_Q), TD_WQ),
    [TD_WW] = ACTION_TAP_DANCE_DATA(KC_W, RCTL(KC_W), TD_WW),
    [TD_WE] = ACTION_TAP_DANCE_DATA(KC_E, RCTL(KC_E), TD_WE),
    [TD_WR] = ACTION_TAP_DANCE_DATA(KC_R, RCTL(KC_R), TD_WR),
    [TD_WT] = ACTION_TAP_DANCE_DATA(KC_T, RCTL(KC_T), TD_WT),
    [TD_WY] = ACTION_TAP_DANCE_DATA(KC_Y, RCTL(KC_Y), TD_WY),
    [TD_WU] = ACTION_TAP_DANCE_DATA(KC_U, RCTL(KC_U), TD_WU),
    [TD_WI] = ACTION_TAP_DANCE_DATA(KC_I, RCTL(KC_I), TD_WI),
    [TD_WO] = ACTION_TAP_DANCE_DATA(KC_O, RCTL(KC_O), TD_WO),
    [TD_WP] = ACTION_TAP_DANCE_DATA(KC_P, RCTL(KC_P), TD_WP),
    [TD_WA] = ACTION_TAP_DANCE_DATA(KC_A, RCTL(KC_A), TD_WA),
    [TD_WS] = ACTION_TAP_DANCE_DATA(KC_S, RCTL(KC_S), TD_WS),
    [TD_WD] = ACTION_TAP_DANCE_DATA(KC_D, RCTL(KC_D), TD_WD),
    [TD_WF] = ACTION_TAP_DANCE_DATA(KC_F, RCTL(KC_F), TD_WF),
    [TD_WG] = ACTION_TAP_DANCE_DATA(KC_G, RCTL(KC_G), TD_WG),
    [TD_WH] = ACTION_TAP_DANCE_DATA(KC_H, RCTL(KC_H), TD_WH),
    [TD_WJ] = ACTION_TAP_DANCE_DATA(KC_J, RCTL(KC_J), TD_WJ),
    [TD_WK] = ACTION_TAP_DANCE_DATA(KC_K, RCTL(KC_K), TD_WK),
    [TD_WL] = ACTION_TAP_DANCE_DATA(KC_L, RCTL(KC_L), TD_WL),
    [TD_WZ] = ACTION_TAP_DANCE_DATA(KC_Z, RCTL(KC_Z), TD_WZ),
    [TD_WX] = ACTION_TAP_DANCE_DATA(KC_X, RCTL(KC_X), TD_WX),
    [TD_WC] = ACTION_TAP_DANCE_DATA(KC_C, RCTL(KC_C), TD_WC),
    [TD_WV] = ACTION_TAP_DANCE_DATA(KC_V, RCTL(KC_V), TD_WV),
    [TD_WB] = ACTION_TAP_DANCE_DATA(KC_B, RCTL(KC_B), TD_WB),
    [TD_WN] = ACTION_TAP_DANCE_DATA(KC_N, RCTL(KC_N), TD_WN),
    [TD_WM] = ACTION_TAP_DANCE_DATA(KC_M, RCTL(KC_M), TD_WM),
    [TD_WTAB] = ACTION_TAP_DANCE_DATA(KC_TAB, RALT(KC_TAB), TD_WTAB),
    [TD_WBSPC] = ACTION_TAP_DANCE_DATA(KC_BSPC, RCTL(KC_BSPC), TD_WBSPC),
    [TD_WDEL] = ACTION_TAP_DANCE_DATA(KC_DEL, RCTL(KC_DEL), TD_WDEL),
    [TD_WLEFT] = ACTION_TAP_DANCE_DATA(KC_LEFT, RCTL(KC_LEFT), TD_WLEFT),
    [TD_WDOWN] = ACTION_TAP_DANCE_DATA(KC_DOWN, RCTL(KC_DOWN), TD_WDOWN),
    [TD_WUP] = ACTION_TAP_DANCE_DATA(KC_UP, RCTL(KC_UP), TD_WUP),
    [TD_WRGHT] = ACTION_TAP_DANCE_DATA(KC_RGHT, RCTL(KC_RGHT), TD_WRGHT),
    [TD_WBEG] = ACTION_TAP_DANCE_DATA(KC_HOME, RCTL(KC_HOME), TD_WBEG),
    [TD_WEND] = ACTION_TAP_DANCE_DATA(KC_END, RCTL(KC_END), TD_WEND),
    [TD_WPLUS] = ACTION_TAP_DANCE_DATA(KC_PLUS, RCTL(KC_PLUS), TD_WPLUS),
    [TD_WMINS] = ACTION_TAP_DANCE_DATA(KC_MINS, RCTL(KC_MINS), TD_WMINS),

    // SHARED TAP DANCE ACTIONS
    [TD_SPC] = ACTION_TAP_DANCE_DATA(KC_SPC, RGUI(KC_SPC), TD_SPC),
    [TD_ENT] = ACTION_TAP_DANCE_DATA(KC_ENT, RSFT(KC_ENT), TD_ENT),
    [TD_0] = ACTION_TAP_DANCE_DATA(KC_0, RSFT(KC_0), TD_0),
    [TD_1] = ACTION_TAP_DANCE_DATA(KC_1, RSFT(KC_1), TD_1),
    [TD_2] = ACTION_TAP_DANCE_DATA(KC_2, RSFT(KC_2), TD_2),
    [TD_3] = ACTION_TAP_DANCE_DATA(KC_3, RSFT(KC_3), TD_3),
    [TD_4] = ACTION_TAP_DANCE_DATA(KC_4, RSFT(KC_4), TD_4),
    [TD_5] = ACTION_TAP_DANCE_DATA(KC_5, RSFT(KC_5), TD_5),
    [TD_6] = ACTION_TAP_DANCE_DATA(KC_6, RSFT(KC_6), TD_6),
    [TD_7] = ACTION_TAP_DANCE_DATA(KC_7, RSFT(KC_7), TD_7),
    [TD_8] = ACTION_TAP_DANCE_DATA(KC_8, RSFT(KC_8), TD_8),
    [TD_9] = ACTION_TAP_DANCE_DATA(KC_9, RSFT(KC_9), TD_9),
    [TD_SLSH] = ACTION_TAP_DANCE_DATA(KC_SLSH, RSFT(KC_SLSH), TD_SLSH),
    [TD_CBR] = ACTION_TAP_DANCE_STR(KC_LCBR, "{}", "{};", TD_CBR),
    [TD_PRN] = ACTION_TAP_DANCE_STR(KC_LPRN, "()", "();", TD_PRN),
    [TD_BRC] = ACTION_TAP_DANCE_STR(KC_LBRC, "[]", "[];", TD_BRC),
    [TD_SQT] = ACTION_TAP_DANCE_STR(KC_GRV, "''", "'';", TD_SQT),
    [TD_DQT] = ACTION_TAP_DANCE_STR(KC_TILD, "\"\"", "\"\";", TD_DQT),
    [TD_ABK] = ACTION_TAP_DANCE_STR(KC_LABK, "<>", NULL, TD_ABK),
};