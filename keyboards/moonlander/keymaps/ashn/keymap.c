#include QMK_KEYBOARD_H
#include "version.h"
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
#define MOON_LED_LEVEL LED_LEVEL

enum custom_keycodes {
  RGB_SLD = ML_SAFE_RANGE,
};

typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

// SYNC WARNING
// The length of dance_state must be equal to the number of tap_dance_codes.
static tap dance_state[53];

enum tap_dance_codes {
  DANCE_Q = 0,
  DANCE_W,
  DANCE_E,
  DANCE_R,
  DANCE_T,
  DANCE_A,
  DANCE_S,
  DANCE_D,
  DANCE_F,
  DANCE_G,
  DANCE_Z,
  DANCE_X,
  DANCE_C,
  DANCE_V,
  DANCE_B,
  DANCE_BSPACE,
  DANCE_Y,
  DANCE_U,
  DANCE_I,
  DANCE_O,
  DANCE_P,
  DANCE_H,
  DANCE_J,
  DANCE_K,
  DANCE_L,
  DANCE_N,
  DANCE_M,
  DANCE_ENTER,
  DANCE_TAB,
  DANCE_DEL,
  DANCE_LEFT,
  DANCE_DOWN,
  DANCE_UP,
  DANCE_RIGHT,
  DANCE_HOME,
  DANCE_END,
  DANCE_7,
  DANCE_8,
  DANCE_9,
  DANCE_4,
  DANCE_5,
  DANCE_6,
  DANCE_1,
  DANCE_2,
  DANCE_3,
  DANCE_SLASH,
  DANCE_CBR,
  DANCE_PRN,
  DANCE_SBR,
  DANCE_ABK,
  DANCE_SQT,
  DANCE_DQT,
  DANCE_0,
};
// END SYNC WARNING

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_moonlander(
    KC_NO, KC_NO,       KC_NO,       KC_NO,       KC_NO,       KC_NO,       KC_NO,                          KC_NO,           KC_NO,       KC_NO,       KC_NO,       KC_NO,       KC_NO,       KC_NO,          
    KC_NO, TD(DANCE_Q), TD(DANCE_W), TD(DANCE_E), TD(DANCE_R), TD(DANCE_T), KC_NO,                          KC_NO,           TD(DANCE_Y), TD(DANCE_U), TD(DANCE_I), TD(DANCE_O), TD(DANCE_P), KC_NO,          
    KC_NO, TD(DANCE_A), TD(DANCE_S), TD(DANCE_D), TD(DANCE_F), TD(DANCE_G), KC_NO,                          KC_NO,           TD(DANCE_H), TD(DANCE_J), TD(DANCE_K), TD(DANCE_L), KC_SCOLON,   KC_NO,          
    KC_NO, TD(DANCE_Z), TD(DANCE_X), TD(DANCE_C), TD(DANCE_V), TD(DANCE_B),                                                  TD(DANCE_N), TD(DANCE_M), KC_COMMA,    KC_DOT,      KC_QUOTE,    KC_NO,          
    KC_NO, KC_NO,       KC_NO,       KC_RCTRL,    MO(1),                    KC_LGUI,                        OSL(3),                       MO(2),       KC_RALT,     KC_NO,       KC_NO,       KC_NO,          
                                                               KC_SPACE,    TD(DANCE_BSPACE), KC_NO, KC_NO, TD(DANCE_ENTER), KC_LSHIFT
  ),
  [1] = LAYOUT_moonlander(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,  KC_TRANSPARENT, 
    KC_TRANSPARENT, TD(DANCE_TAB),  KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,           KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_ESCAPE,      KC_NO,          KC_NO,          KC_RALT,        KC_NO,          KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_NO,          TD(DANCE_LEFT), TD(DANCE_DOWN), TD(DANCE_UP),   TD(DANCE_RIGHT), KC_TRANSPARENT, 
    KC_TRANSPARENT, TD(DANCE_DEL),  KC_NO,          KC_NO,          KC_LGUI,        KC_NO,                                                                          KC_NO,          TD(DANCE_HOME), TD(DANCE_END),  RCTL(KC_PGUP),  RCTL(KC_PGDOWN), KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT,                 KC_NO,                                          KC_NO,                          MO(4),          KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT,  KC_TRANSPARENT, 
                                                                                    KC_NO,          KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT
  ),
  [2] = LAYOUT_moonlander(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,  KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_AMPR,        KC_PIPE,        TD(DANCE_CBR),  KC_RCBR,        KC_PLUS,        KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_UNDS,        TD(DANCE_7),    TD(DANCE_8),    TD(DANCE_9),    KC_ASTR,         KC_TRANSPARENT, 
    KC_TRANSPARENT, TD(DANCE_ABK),  KC_RABK,        TD(DANCE_PRN),  KC_RPRN,        TD(DANCE_SQT),  KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_EQUAL,       TD(DANCE_4),    TD(DANCE_5),    TD(DANCE_6),    TD(DANCE_0),     KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_GRAVE,       KC_TILD,        TD(DANCE_SBR),  KC_RBRACKET,    TD(DANCE_DQT),                                                                  KC_DOT,         TD(DANCE_1),    TD(DANCE_2),    TD(DANCE_3),    TD(DANCE_SLASH), KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, MO(4),                          KC_NO,                                          KC_NO,                          KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT,  KC_TRANSPARENT, 
                                                                                    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_NO
  ),
  [3] = LAYOUT_moonlander(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_NO,          KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_F5,          KC_F6,          KC_F7,          KC_F8,          KC_NO,          KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_F9,          KC_F10,         KC_F11,         KC_F12,         KC_NO,                                                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_NO,                          KC_NO,                                          KC_NO,                          KC_NO,          KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                                                    KC_NO,          KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_NO
  ),
  [4] = LAYOUT_moonlander(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT,      KC_TRANSPARENT,    KC_TRANSPARENT,  KC_TRANSPARENT,      KC_TRANSPARENT,      KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_MS_WH_LEFT,  KC_MS_WH_DOWN,  KC_MS_UP,           KC_MS_WH_UP,    KC_MS_WH_RIGHT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_MEDIA_PLAY_PAUSE, KC_AUDIO_VOL_DOWN, KC_AUDIO_VOL_UP, KC_MEDIA_PREV_TRACK, KC_MEDIA_NEXT_TRACK, KC_TRANSPARENT, 
    KC_TRANSPARENT, RGB_HUI,        KC_MS_LEFT,     KC_MS_DOWN,         KC_MS_RIGHT,    KC_LSHIFT,      KC_TRANSPARENT,                                 KC_TRANSPARENT, RGB_VAI,             KC_MS_BTN1,        KC_MS_BTN2,      KC_MS_BTN3,          KC_BRIGHTNESS_UP,    KC_TRANSPARENT, 
    KC_TRANSPARENT, RGB_HUD,        RGB_TOG,        TOGGLE_LAYER_COLOR, RGB_MOD,        KC_RCTRL,                                                                       RGB_VAD,             KC_MS_ACCEL0,      KC_MS_ACCEL1,    KC_MS_ACCEL2,        KC_BRIGHTNESS_DOWN,  KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,              KC_TRANSPARENT,                 WEBUSB_PAIR,                                    RESET,                               KC_TRANSPARENT,    KC_NO,           KC_TRANSPARENT,      KC_TRANSPARENT,      KC_TRANSPARENT, 
                                                                                        KC_NO,          KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_NO
  ),
};

extern bool g_suspend_state;
extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][DRIVER_LED_TOTAL][3] = {
    //      TOP                                                        BOTTOM
    [0] = { {0,0,0},      {0,0,0},       {0,0,0},       {0,0,0},       {0,0,0}, //  LEFT
            {0,0,0},      {154,255,255}, {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0},      {154,255,255}, {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0},      {154,255,255}, {154,255,255}, {154,255,255}, {85,203,158},
            {0,0,0},      {154,255,255}, {154,255,255}, {154,255,255}, {14,255,255},
            {0,0,0},      {154,255,255}, {154,255,255}, {154,255,255},
            {0,0,0},      {154,255,255}, {0,0,0}, // Left LED indicator strip
            {35,255,255}, {35,255,255},  {0,0,0},
            {0,0,255}, //                                                           RIGHT
            {0,0,0},      {0,0,0},       {0,0,0},       {0,0,0},       {0,0,0}, //  RIGHT
            {0,0,0},      {154,255,255}, {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0},      {154,255,255}, {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0},      {154,255,255}, {154,255,255}, {154,255,255}, {85,203,158},
            {0,0,0},      {154,255,255}, {154,255,255}, {154,255,255}, {14,255,255},
            {0,0,0},      {154,255,255}, {154,255,255}, {154,255,255},
            {0,0,0},      {154,255,255}, {0,0,0}, // Right LED indicator strip
            {85,203,158}, {255,220,201}, {0,0,0},
            {14,255,255} }, //                                                      LEFT

    //      TOP                                                       BOTTOM
    [1] = { {0,0,0},      {0,0,0},      {0,0,0},       {0,0,0},       {0,0,0}, //  LEFT
            {0,0,0},      {35,255,255}, {35,255,255},  {35,255,255},  {0,0,0},
            {0,0,0},      {0,0,0},      {0,0,0},       {0,0,0},       {0,0,0},
            {0,0,0},      {0,0,0},      {0,0,0},       {0,0,0},       {0,0,0},
            {0,0,0},      {0,0,0},      {85,203,158},  {85,203,158},  {14,255,255},
            {0,0,0},      {0,0,0},      {0,0,0},       {0,0,0},
            {0,0,0},      {0,0,0},      {85,203,158}, // Left LED indicator strip
            {0,0,0},      {0,0,0},      {0,0,0},
            {0,0,0}, //                                                            RIGHT
            {0,0,0},      {0,0,0},      {0,0,0},       {0,0,0},       {0,0,0}, //  RIGHT
            {0,0,0},      {0,0,0},      {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0},      {0,0,0},      {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0},      {0,0,0},      {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0},      {0,0,0},      {154,255,255}, {154,255,255}, {14,255,255},
            {0,0,0},      {0,0,0},      {0,0,0},       {0,0,0},
            {0,0,0},      {0,0,0},      {85,203,158},  // Right LED indicator strip
            {85,203,158}, {0,0,0},      {0,0,0},
            {0,0,0} }, //                                                          LEFT

    //      TOP                                                          BOTTOM
    [2] = { {0,0,0},        {0,0,0},       {0,0,0},       {0,0,0},       {0,0,0}, //  LEFT
            {0,0,0},        {154,255,255}, {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0},        {154,255,255}, {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0},        {154,255,255}, {154,255,255}, {154,255,255}, {85,203,158},
            {0,0,0},        {154,255,255}, {154,255,255}, {154,255,255}, {14,255,255},
            {0,0,0},        {154,255,255}, {154,255,255}, {154,255,255},
            {202,255,164},  {0,0,0},       {0,0,0}, // Left LED indicator strip
            {35,255,255},   {35,255,255},  {0,0,0},
            {0,0,0}, //                                                               RIGHT
            {0,0,0},        {0,0,0},       {0,0,0},       {0,0,0},       {0,0,0}, //  RIGHT
            {0,0,0},        {154,255,255}, {202,255,164}, {154,255,255}, {0,0,0},
            {0,0,0},        {202,255,164}, {202,255,164}, {202,255,164}, {0,0,0},
            {0,0,0},        {202,255,164}, {202,255,164}, {202,255,164}, {0,0,0},
            {0,0,0},        {202,255,164}, {202,255,164}, {202,255,164}, {14,255,255},
            {0,0,0},        {154,255,255}, {154,255,255}, {154,255,255},
            {202,255,164},  {0,0,0},       {0,0,0}, // Right LED indicator strip
            {0,0,0},        {0,0,0},       {0,0,0},
            {0,0,0} }, //                                                             LEFT

    //      TOP                                                   BOTTOM
    [3] = { {0,0,0}, {0,0,0},       {0,0,0},       {0,0,0},       {0,0,0}, // LEFT
            {0,0,0}, {154,255,255}, {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0}, {154,255,255}, {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0}, {154,255,255}, {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0}, {154,255,255}, {154,255,255}, {154,255,255}, {0,0,0},
            {0,0,0}, {0,0,0},       {0,0,0},       {0,0,0},
            {0,0,0}, {0,0,0},       {0,0,0}, // Left LED indicator strip
            {0,0,0}, {0,0,0},       {0,0,0},
            {0,0,0}, //                                                       RIGHT
            {0,0,0}, {0,0,0},       {0,0,0},       {0,0,0},       {0,0,0}, // RIGHT
            {0,0,0}, {0,0,0},       {0,0,0},       {0,0,0},       {0,0,0},
            {0,0,0}, {0,0,0},       {0,0,0},       {0,0,0},       {0,0,0},
            {0,0,0}, {0,0,0},       {0,0,0},       {0,0,0},       {0,0,0},
            {0,0,0}, {0,0,0},       {0,0,0},       {0,0,0},       {0,0,0},
            {0,0,0}, {0,0,0},       {0,0,0},       {0,0,0},
            {0,0,0}, {0,0,0},       {0,0,0}, // Right LED indicator strip
            {0,0,0}, {0,0,0},       {0,0,0},
            {0,0,0} }, //                                                     LEFT

    //      TOP                                                        BOTTOM
    [4] = { {0,0,0},      {0,0,0},       {0,0,0},       {0,0,0},       {0,0,0}, //  LEFT
            {0,0,0},      {202,255,164}, {35,255,255},  {35,255,255},  {0,0,0},
            {0,0,0},      {202,255,164}, {154,255,255}, {35,255,255},  {0,0,0},
            {0,0,0},      {154,255,255}, {154,255,255}, {35,255,255},  {0,0,0},
            {0,0,0},      {202,255,164}, {154,255,255}, {35,255,255},  {14,255,255},
            {0,0,0},      {202,255,164}, {85,203,158},  {85,203,158},
            {35,255,255}, {0,0,0},       {35,255,255}, // Left LED indicator strip
            {0,0,0},      {0,0,0},       {0,0,0},
            {0,0,255}, //                                                           RIGHT
            {0,0,0},      {0,0,0},       {0,0,0},       {0,0,0},       {0,0,0}, //  RIGHT
            {0,0,0},      {35,255,255},  {35,255,255},  {35,255,255},  {0,0,0},
            {0,0,0},      {35,255,255},  {154,255,255}, {202,255,164}, {0,0,0},
            {0,0,0},      {35,255,255},  {154,255,255}, {202,255,164}, {0,0,0},
            {0,0,0},      {35,255,255},  {154,255,255}, {202,255,164}, {14,255,255},
            {0,0,0},      {35,255,255},  {35,255,255},  {35,255,255},
            {35,255,255}, {0,0,0},       {35,255,255}, // Right LED indicator strip
            {0,0,0},      {0,0,0},       {0,0,0},
            {0,0,255} }, //                                                         LEFT

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
  if (g_suspend_state || keyboard_config.disable_layer_led) { return; }
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

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
  }
  return true;
}

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

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

void on_dance_q(qk_tap_dance_state_t *state, void *user_data);
void dance_q_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_q_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_q(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_Q);
        tap_code16(KC_Q);
        tap_code16(KC_Q);
    }
    if(state->count > 3) {
        tap_code16(KC_Q);
    }
}

void dance_q_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_Q].step = dance_step(state);
    switch (dance_state[DANCE_Q].step) {
        case SINGLE_TAP: register_code16(KC_Q); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_Q)); break;
        case DOUBLE_TAP: register_code16(KC_Q); register_code16(KC_Q); break;
        case DOUBLE_HOLD: register_code16(KC_Q); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_Q); register_code16(KC_Q);
    }
}

void dance_q_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_Q].step) {
        case SINGLE_TAP: unregister_code16(KC_Q); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_Q)); break;
        case DOUBLE_TAP: unregister_code16(KC_Q); break;
        case DOUBLE_HOLD: unregister_code16(KC_Q); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_Q); break;
    }
    dance_state[DANCE_Q].step = 0;
}
void on_dance_w(qk_tap_dance_state_t *state, void *user_data);
void dance_w_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_w_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_w(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_W);
        tap_code16(KC_W);
        tap_code16(KC_W);
    }
    if(state->count > 3) {
        tap_code16(KC_W);
    }
}

void dance_w_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_W].step = dance_step(state);
    switch (dance_state[DANCE_W].step) {
        case SINGLE_TAP: register_code16(KC_W); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_W)); break;
        case DOUBLE_TAP: register_code16(KC_W); register_code16(KC_W); break;
        case DOUBLE_HOLD: register_code16(KC_W); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_W); register_code16(KC_W);
    }
}

void dance_w_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_W].step) {
        case SINGLE_TAP: unregister_code16(KC_W); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_W)); break;
        case DOUBLE_TAP: unregister_code16(KC_W); break;
        case DOUBLE_HOLD: unregister_code16(KC_W); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_W); break;
    }
    dance_state[DANCE_W].step = 0;
}
void on_dance_e(qk_tap_dance_state_t *state, void *user_data);
void dance_e_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_e_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_e(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_E);
        tap_code16(KC_E);
        tap_code16(KC_E);
    }
    if(state->count > 3) {
        tap_code16(KC_E);
    }
}

void dance_e_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_E].step = dance_step(state);
    switch (dance_state[DANCE_E].step) {
        case SINGLE_TAP: register_code16(KC_E); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_E)); break;
        case DOUBLE_TAP: register_code16(KC_E); register_code16(KC_E); break;
        case DOUBLE_HOLD: register_code16(KC_E); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_E); register_code16(KC_E);
    }
}

void dance_e_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_E].step) {
        case SINGLE_TAP: unregister_code16(KC_E); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_E)); break;
        case DOUBLE_TAP: unregister_code16(KC_E); break;
        case DOUBLE_HOLD: unregister_code16(KC_E); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_E); break;
    }
    dance_state[DANCE_E].step = 0;
}
void on_dance_r(qk_tap_dance_state_t *state, void *user_data);
void dance_r_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_r_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_r(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_R);
        tap_code16(KC_R);
        tap_code16(KC_R);
    }
    if(state->count > 3) {
        tap_code16(KC_R);
    }
}

void dance_r_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_R].step = dance_step(state);
    switch (dance_state[DANCE_R].step) {
        case SINGLE_TAP: register_code16(KC_R); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_R)); break;
        case DOUBLE_TAP: register_code16(KC_R); register_code16(KC_R); break;
        case DOUBLE_HOLD: register_code16(KC_R); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_R); register_code16(KC_R);
    }
}

void dance_r_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_R].step) {
        case SINGLE_TAP: unregister_code16(KC_R); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_R)); break;
        case DOUBLE_TAP: unregister_code16(KC_R); break;
        case DOUBLE_HOLD: unregister_code16(KC_R); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_R); break;
    }
    dance_state[DANCE_R].step = 0;
}
void on_dance_t(qk_tap_dance_state_t *state, void *user_data);
void dance_t_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_t_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_t(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_T);
        tap_code16(KC_T);
        tap_code16(KC_T);
    }
    if(state->count > 3) {
        tap_code16(KC_T);
    }
}

void dance_t_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_T].step = dance_step(state);
    switch (dance_state[DANCE_T].step) {
        case SINGLE_TAP: register_code16(KC_T); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_T)); break;
        case DOUBLE_TAP: register_code16(KC_T); register_code16(KC_T); break;
        case DOUBLE_HOLD: register_code16(KC_T); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_T); register_code16(KC_T);
    }
}

void dance_t_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_T].step) {
        case SINGLE_TAP: unregister_code16(KC_T); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_T)); break;
        case DOUBLE_TAP: unregister_code16(KC_T); break;
        case DOUBLE_HOLD: unregister_code16(KC_T); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_T); break;
    }
    dance_state[DANCE_T].step = 0;
}
void on_dance_a(qk_tap_dance_state_t *state, void *user_data);
void dance_a_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_a_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_a(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_A);
        tap_code16(KC_A);
        tap_code16(KC_A);
    }
    if(state->count > 3) {
        tap_code16(KC_A);
    }
}

void dance_a_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_A].step = dance_step(state);
    switch (dance_state[DANCE_A].step) {
        case SINGLE_TAP: register_code16(KC_A); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_A)); break;
        case DOUBLE_TAP: register_code16(KC_A); register_code16(KC_A); break;
        case DOUBLE_HOLD: register_code16(KC_A); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_A); register_code16(KC_A);
    }
}

void dance_a_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_A].step) {
        case SINGLE_TAP: unregister_code16(KC_A); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_A)); break;
        case DOUBLE_TAP: unregister_code16(KC_A); break;
        case DOUBLE_HOLD: unregister_code16(KC_A); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_A); break;
    }
    dance_state[DANCE_A].step = 0;
}
void on_dance_s(qk_tap_dance_state_t *state, void *user_data);
void dance_s_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_s_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_s(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_S);
        tap_code16(KC_S);
        tap_code16(KC_S);
    }
    if(state->count > 3) {
        tap_code16(KC_S);
    }
}

void dance_s_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_S].step = dance_step(state);
    switch (dance_state[DANCE_S].step) {
        case SINGLE_TAP: register_code16(KC_S); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_S)); break;
        case DOUBLE_TAP: register_code16(KC_S); register_code16(KC_S); break;
        case DOUBLE_HOLD: register_code16(KC_S); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_S); register_code16(KC_S);
    }
}

void dance_s_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_S].step) {
        case SINGLE_TAP: unregister_code16(KC_S); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_S)); break;
        case DOUBLE_TAP: unregister_code16(KC_S); break;
        case DOUBLE_HOLD: unregister_code16(KC_S); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_S); break;
    }
    dance_state[DANCE_S].step = 0;
}
void on_dance_d(qk_tap_dance_state_t *state, void *user_data);
void dance_d_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_d_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_d(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_D);
        tap_code16(KC_D);
        tap_code16(KC_D);
    }
    if(state->count > 3) {
        tap_code16(KC_D);
    }
}

void dance_d_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_D].step = dance_step(state);
    switch (dance_state[DANCE_D].step) {
        case SINGLE_TAP: register_code16(KC_D); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_D)); break;
        case DOUBLE_TAP: register_code16(KC_D); register_code16(KC_D); break;
        case DOUBLE_HOLD: register_code16(KC_D); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_D); register_code16(KC_D);
    }
}

void dance_d_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_D].step) {
        case SINGLE_TAP: unregister_code16(KC_D); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_D)); break;
        case DOUBLE_TAP: unregister_code16(KC_D); break;
        case DOUBLE_HOLD: unregister_code16(KC_D); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_D); break;
    }
    dance_state[DANCE_D].step = 0;
}
void on_dance_f(qk_tap_dance_state_t *state, void *user_data);
void dance_f_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_f_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_f(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_F);
        tap_code16(KC_F);
        tap_code16(KC_F);
    }
    if(state->count > 3) {
        tap_code16(KC_F);
    }
}

void dance_f_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_F].step = dance_step(state);
    switch (dance_state[DANCE_F].step) {
        case SINGLE_TAP: register_code16(KC_F); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_F)); break;
        case DOUBLE_TAP: register_code16(KC_F); register_code16(KC_F); break;
        case DOUBLE_HOLD: register_code16(KC_F); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_F); register_code16(KC_F);
    }
}

void dance_f_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_F].step) {
        case SINGLE_TAP: unregister_code16(KC_F); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_F)); break;
        case DOUBLE_TAP: unregister_code16(KC_F); break;
        case DOUBLE_HOLD: unregister_code16(KC_F); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_F); break;
    }
    dance_state[DANCE_F].step = 0;
}
void on_dance_g(qk_tap_dance_state_t *state, void *user_data);
void dance_g_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_g_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_g(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_G);
        tap_code16(KC_G);
        tap_code16(KC_G);
    }
    if(state->count > 3) {
        tap_code16(KC_G);
    }
}

void dance_g_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_G].step = dance_step(state);
    switch (dance_state[DANCE_G].step) {
        case SINGLE_TAP: register_code16(KC_G); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_G)); break;
        case DOUBLE_TAP: register_code16(KC_G); register_code16(KC_G); break;
        case DOUBLE_HOLD: register_code16(KC_G); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_G); register_code16(KC_G);
    }
}

void dance_g_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_G].step) {
        case SINGLE_TAP: unregister_code16(KC_G); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_G)); break;
        case DOUBLE_TAP: unregister_code16(KC_G); break;
        case DOUBLE_HOLD: unregister_code16(KC_G); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_G); break;
    }
    dance_state[DANCE_G].step = 0;
}
void on_dance_z(qk_tap_dance_state_t *state, void *user_data);
void dance_z_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_z_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_z(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_Z);
        tap_code16(KC_Z);
        tap_code16(KC_Z);
    }
    if(state->count > 3) {
        tap_code16(KC_Z);
    }
}

void dance_z_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_Z].step = dance_step(state);
    switch (dance_state[DANCE_Z].step) {
        case SINGLE_TAP: register_code16(KC_Z); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_Z)); break;
        case DOUBLE_TAP: register_code16(KC_Z); register_code16(KC_Z); break;
        case DOUBLE_HOLD: register_code16(KC_Z); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_Z); register_code16(KC_Z);
    }
}

void dance_z_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_Z].step) {
        case SINGLE_TAP: unregister_code16(KC_Z); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_Z)); break;
        case DOUBLE_TAP: unregister_code16(KC_Z); break;
        case DOUBLE_HOLD: unregister_code16(KC_Z); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_Z); break;
    }
    dance_state[DANCE_Z].step = 0;
}
void on_dance_x(qk_tap_dance_state_t *state, void *user_data);
void dance_x_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_x_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_x(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_X);
        tap_code16(KC_X);
        tap_code16(KC_X);
    }
    if(state->count > 3) {
        tap_code16(KC_X);
    }
}

void dance_x_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_X].step = dance_step(state);
    switch (dance_state[DANCE_X].step) {
        case SINGLE_TAP: register_code16(KC_X); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_X)); break;
        case DOUBLE_TAP: register_code16(KC_X); register_code16(KC_X); break;
        case DOUBLE_HOLD: register_code16(KC_X); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_X); register_code16(KC_X);
    }
}

void dance_x_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_X].step) {
        case SINGLE_TAP: unregister_code16(KC_X); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_X)); break;
        case DOUBLE_TAP: unregister_code16(KC_X); break;
        case DOUBLE_HOLD: unregister_code16(KC_X); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_X); break;
    }
    dance_state[DANCE_X].step = 0;
}
void on_dance_c(qk_tap_dance_state_t *state, void *user_data);
void dance_c_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_c_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_c(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_C);
        tap_code16(KC_C);
        tap_code16(KC_C);
    }
    if(state->count > 3) {
        tap_code16(KC_C);
    }
}

void dance_c_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_C].step = dance_step(state);
    switch (dance_state[DANCE_C].step) {
        case SINGLE_TAP: register_code16(KC_C); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_C)); break;
        case DOUBLE_TAP: register_code16(KC_C); register_code16(KC_C); break;
        case DOUBLE_HOLD: register_code16(KC_C); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_C); register_code16(KC_C);
    }
}

void dance_c_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_C].step) {
        case SINGLE_TAP: unregister_code16(KC_C); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_C)); break;
        case DOUBLE_TAP: unregister_code16(KC_C); break;
        case DOUBLE_HOLD: unregister_code16(KC_C); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_C); break;
    }
    dance_state[DANCE_C].step = 0;
}
void on_dance_v(qk_tap_dance_state_t *state, void *user_data);
void dance_v_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_v_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_v(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_V);
        tap_code16(KC_V);
        tap_code16(KC_V);
    }
    if(state->count > 3) {
        tap_code16(KC_V);
    }
}

void dance_v_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_V].step = dance_step(state);
    switch (dance_state[DANCE_V].step) {
        case SINGLE_TAP: register_code16(KC_V); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_V)); break;
        case DOUBLE_TAP: register_code16(KC_V); register_code16(KC_V); break;
        case DOUBLE_HOLD: register_code16(KC_V); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_V); register_code16(KC_V);
    }
}

void dance_v_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_V].step) {
        case SINGLE_TAP: unregister_code16(KC_V); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_V)); break;
        case DOUBLE_TAP: unregister_code16(KC_V); break;
        case DOUBLE_HOLD: unregister_code16(KC_V); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_V); break;
    }
    dance_state[DANCE_V].step = 0;
}
void on_dance_b(qk_tap_dance_state_t *state, void *user_data);
void dance_b_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_b_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_b(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_B);
        tap_code16(KC_B);
        tap_code16(KC_B);
    }
    if(state->count > 3) {
        tap_code16(KC_B);
    }
}

void dance_b_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_B].step = dance_step(state);
    switch (dance_state[DANCE_B].step) {
        case SINGLE_TAP: register_code16(KC_B); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_B)); break;
        case DOUBLE_TAP: register_code16(KC_B); register_code16(KC_B); break;
        case DOUBLE_HOLD: register_code16(KC_B); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_B); register_code16(KC_B);
    }
}

void dance_b_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_B].step) {
        case SINGLE_TAP: unregister_code16(KC_B); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_B)); break;
        case DOUBLE_TAP: unregister_code16(KC_B); break;
        case DOUBLE_HOLD: unregister_code16(KC_B); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_B); break;
    }
    dance_state[DANCE_B].step = 0;
}
void on_dance_bspace(qk_tap_dance_state_t *state, void *user_data);
void dance_bspace_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_bspace_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_bspace(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_BSPACE);
        tap_code16(KC_BSPACE);
        tap_code16(KC_BSPACE);
    }
    if(state->count > 3) {
        tap_code16(KC_BSPACE);
    }
}

void dance_bspace_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_BSPACE].step = dance_step(state);
    switch (dance_state[DANCE_BSPACE].step) {
        case SINGLE_TAP: register_code16(KC_BSPACE); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_BSPACE)); break;
        case DOUBLE_TAP: register_code16(KC_BSPACE); register_code16(KC_BSPACE); break;
        case DOUBLE_HOLD: register_code16(KC_BSPACE); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_BSPACE); register_code16(KC_BSPACE);
    }
}

void dance_bspace_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_BSPACE].step) {
        case SINGLE_TAP: unregister_code16(KC_BSPACE); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_BSPACE)); break;
        case DOUBLE_TAP: unregister_code16(KC_BSPACE); break;
        case DOUBLE_HOLD: unregister_code16(KC_BSPACE); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_BSPACE); break;
    }
    dance_state[DANCE_BSPACE].step = 0;
}
void on_dance_y(qk_tap_dance_state_t *state, void *user_data);
void dance_y_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_y_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_y(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_Y);
        tap_code16(KC_Y);
        tap_code16(KC_Y);
    }
    if(state->count > 3) {
        tap_code16(KC_Y);
    }
}

void dance_y_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_Y].step = dance_step(state);
    switch (dance_state[DANCE_Y].step) {
        case SINGLE_TAP: register_code16(KC_Y); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_Y)); break;
        case DOUBLE_TAP: register_code16(KC_Y); register_code16(KC_Y); break;
        case DOUBLE_HOLD: register_code16(KC_Y); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_Y); register_code16(KC_Y);
    }
}

void dance_y_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_Y].step) {
        case SINGLE_TAP: unregister_code16(KC_Y); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_Y)); break;
        case DOUBLE_TAP: unregister_code16(KC_Y); break;
        case DOUBLE_HOLD: unregister_code16(KC_Y); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_Y); break;
    }
    dance_state[DANCE_Y].step = 0;
}
void on_dance_u(qk_tap_dance_state_t *state, void *user_data);
void dance_u_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_u_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_u(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_U);
        tap_code16(KC_U);
        tap_code16(KC_U);
    }
    if(state->count > 3) {
        tap_code16(KC_U);
    }
}

void dance_u_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_U].step = dance_step(state);
    switch (dance_state[DANCE_U].step) {
        case SINGLE_TAP: register_code16(KC_U); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_U)); break;
        case DOUBLE_TAP: register_code16(KC_U); register_code16(KC_U); break;
        case DOUBLE_HOLD: register_code16(KC_U); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_U); register_code16(KC_U);
    }
}

void dance_u_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_U].step) {
        case SINGLE_TAP: unregister_code16(KC_U); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_U)); break;
        case DOUBLE_TAP: unregister_code16(KC_U); break;
        case DOUBLE_HOLD: unregister_code16(KC_U); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_U); break;
    }
    dance_state[DANCE_U].step = 0;
}
void on_dance_i(qk_tap_dance_state_t *state, void *user_data);
void dance_i_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_i_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_i(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_I);
        tap_code16(KC_I);
        tap_code16(KC_I);
    }
    if(state->count > 3) {
        tap_code16(KC_I);
    }
}

void dance_i_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_I].step = dance_step(state);
    switch (dance_state[DANCE_I].step) {
        case SINGLE_TAP: register_code16(KC_I); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_I)); break;
        case DOUBLE_TAP: register_code16(KC_I); register_code16(KC_I); break;
        case DOUBLE_HOLD: register_code16(KC_I); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_I); register_code16(KC_I);
    }
}

void dance_i_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_I].step) {
        case SINGLE_TAP: unregister_code16(KC_I); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_I)); break;
        case DOUBLE_TAP: unregister_code16(KC_I); break;
        case DOUBLE_HOLD: unregister_code16(KC_I); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_I); break;
    }
    dance_state[DANCE_I].step = 0;
}
void on_dance_o(qk_tap_dance_state_t *state, void *user_data);
void dance_o_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_o_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_o(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_O);
        tap_code16(KC_O);
        tap_code16(KC_O);
    }
    if(state->count > 3) {
        tap_code16(KC_O);
    }
}

void dance_o_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_O].step = dance_step(state);
    switch (dance_state[DANCE_O].step) {
        case SINGLE_TAP: register_code16(KC_O); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_O)); break;
        case DOUBLE_TAP: register_code16(KC_O); register_code16(KC_O); break;
        case DOUBLE_HOLD: register_code16(KC_O); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_O); register_code16(KC_O);
    }
}

void dance_o_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_O].step) {
        case SINGLE_TAP: unregister_code16(KC_O); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_O)); break;
        case DOUBLE_TAP: unregister_code16(KC_O); break;
        case DOUBLE_HOLD: unregister_code16(KC_O); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_O); break;
    }
    dance_state[DANCE_O].step = 0;
}
void on_dance_p(qk_tap_dance_state_t *state, void *user_data);
void dance_p_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_p_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_p(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_P);
        tap_code16(KC_P);
        tap_code16(KC_P);
    }
    if(state->count > 3) {
        tap_code16(KC_P);
    }
}

void dance_p_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_P].step = dance_step(state);
    switch (dance_state[DANCE_P].step) {
        case SINGLE_TAP: register_code16(KC_P); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_P)); break;
        case DOUBLE_TAP: register_code16(KC_P); register_code16(KC_P); break;
        case DOUBLE_HOLD: register_code16(KC_P); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_P); register_code16(KC_P);
    }
}

void dance_p_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_P].step) {
        case SINGLE_TAP: unregister_code16(KC_P); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_P)); break;
        case DOUBLE_TAP: unregister_code16(KC_P); break;
        case DOUBLE_HOLD: unregister_code16(KC_P); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_P); break;
    }
    dance_state[DANCE_P].step = 0;
}
void on_dance_h(qk_tap_dance_state_t *state, void *user_data);
void dance_h_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_h_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_h(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_H);
        tap_code16(KC_H);
        tap_code16(KC_H);
    }
    if(state->count > 3) {
        tap_code16(KC_H);
    }
}

void dance_h_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_H].step = dance_step(state);
    switch (dance_state[DANCE_H].step) {
        case SINGLE_TAP: register_code16(KC_H); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_H)); break;
        case DOUBLE_TAP: register_code16(KC_H); register_code16(KC_H); break;
        case DOUBLE_HOLD: register_code16(KC_H); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_H); register_code16(KC_H);
    }
}

void dance_h_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_H].step) {
        case SINGLE_TAP: unregister_code16(KC_H); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_H)); break;
        case DOUBLE_TAP: unregister_code16(KC_H); break;
        case DOUBLE_HOLD: unregister_code16(KC_H); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_H); break;
    }
    dance_state[DANCE_H].step = 0;
}
void on_dance_j(qk_tap_dance_state_t *state, void *user_data);
void dance_j_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_j_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_j(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_J);
        tap_code16(KC_J);
        tap_code16(KC_J);
    }
    if(state->count > 3) {
        tap_code16(KC_J);
    }
}

void dance_j_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_J].step = dance_step(state);
    switch (dance_state[DANCE_J].step) {
        case SINGLE_TAP: register_code16(KC_J); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_J)); break;
        case DOUBLE_TAP: register_code16(KC_J); register_code16(KC_J); break;
        case DOUBLE_HOLD: register_code16(KC_J); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_J); register_code16(KC_J);
    }
}

void dance_j_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_J].step) {
        case SINGLE_TAP: unregister_code16(KC_J); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_J)); break;
        case DOUBLE_TAP: unregister_code16(KC_J); break;
        case DOUBLE_HOLD: unregister_code16(KC_J); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_J); break;
    }
    dance_state[DANCE_J].step = 0;
}
void on_dance_k(qk_tap_dance_state_t *state, void *user_data);
void dance_k_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_k_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_k(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_K);
        tap_code16(KC_K);
        tap_code16(KC_K);
    }
    if(state->count > 3) {
        tap_code16(KC_K);
    }
}

void dance_k_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_K].step = dance_step(state);
    switch (dance_state[DANCE_K].step) {
        case SINGLE_TAP: register_code16(KC_K); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_K)); break;
        case DOUBLE_TAP: register_code16(KC_K); register_code16(KC_K); break;
        case DOUBLE_HOLD: register_code16(KC_K); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_K); register_code16(KC_K);
    }
}

void dance_k_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_K].step) {
        case SINGLE_TAP: unregister_code16(KC_K); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_K)); break;
        case DOUBLE_TAP: unregister_code16(KC_K); break;
        case DOUBLE_HOLD: unregister_code16(KC_K); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_K); break;
    }
    dance_state[DANCE_K].step = 0;
}
void on_dance_l(qk_tap_dance_state_t *state, void *user_data);
void dance_l_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_l_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_l(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_L);
        tap_code16(KC_L);
        tap_code16(KC_L);
    }
    if(state->count > 3) {
        tap_code16(KC_L);
    }
}

void dance_l_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_L].step = dance_step(state);
    switch (dance_state[DANCE_L].step) {
        case SINGLE_TAP: register_code16(KC_L); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_L)); break;
        case DOUBLE_TAP: register_code16(KC_L); register_code16(KC_L); break;
        case DOUBLE_HOLD: register_code16(KC_L); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_L); register_code16(KC_L);
    }
}

void dance_l_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_L].step) {
        case SINGLE_TAP: unregister_code16(KC_L); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_L)); break;
        case DOUBLE_TAP: unregister_code16(KC_L); break;
        case DOUBLE_HOLD: unregister_code16(KC_L); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_L); break;
    }
    dance_state[DANCE_L].step = 0;
}
void on_dance_n(qk_tap_dance_state_t *state, void *user_data);
void dance_n_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_n_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_n(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_N);
        tap_code16(KC_N);
        tap_code16(KC_N);
    }
    if(state->count > 3) {
        tap_code16(KC_N);
    }
}

void dance_n_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_N].step = dance_step(state);
    switch (dance_state[DANCE_N].step) {
        case SINGLE_TAP: register_code16(KC_N); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_N)); break;
        case DOUBLE_TAP: register_code16(KC_N); register_code16(KC_N); break;
        case DOUBLE_HOLD: register_code16(KC_N); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_N); register_code16(KC_N);
    }
}

void dance_n_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_N].step) {
        case SINGLE_TAP: unregister_code16(KC_N); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_N)); break;
        case DOUBLE_TAP: unregister_code16(KC_N); break;
        case DOUBLE_HOLD: unregister_code16(KC_N); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_N); break;
    }
    dance_state[DANCE_N].step = 0;
}
void on_dance_m(qk_tap_dance_state_t *state, void *user_data);
void dance_m_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_m_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_m(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_M);
        tap_code16(KC_M);
        tap_code16(KC_M);
    }
    if(state->count > 3) {
        tap_code16(KC_M);
    }
}

void dance_m_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_M].step = dance_step(state);
    switch (dance_state[DANCE_M].step) {
        case SINGLE_TAP: register_code16(KC_M); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_M)); break;
        case DOUBLE_TAP: register_code16(KC_M); register_code16(KC_M); break;
        case DOUBLE_HOLD: register_code16(KC_M); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_M); register_code16(KC_M);
    }
}

void dance_m_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_M].step) {
        case SINGLE_TAP: unregister_code16(KC_M); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_M)); break;
        case DOUBLE_TAP: unregister_code16(KC_M); break;
        case DOUBLE_HOLD: unregister_code16(KC_M); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_M); break;
    }
    dance_state[DANCE_M].step = 0;
}
void on_dance_enter(qk_tap_dance_state_t *state, void *user_data);
void dance_enter_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_enter_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_enter(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_ENTER);
        tap_code16(KC_ENTER);
        tap_code16(KC_ENTER);
    }
    if(state->count > 3) {
        tap_code16(KC_ENTER);
    }
}

void dance_enter_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_ENTER].step = dance_step(state);
    switch (dance_state[DANCE_ENTER].step) {
        case SINGLE_TAP: register_code16(KC_ENTER); break;
        case SINGLE_HOLD: register_code16(LSFT(KC_ENTER)); break;
        case DOUBLE_TAP: register_code16(KC_ENTER); register_code16(KC_ENTER); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_ENTER); register_code16(KC_ENTER);
    }
}

void dance_enter_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_ENTER].step) {
        case SINGLE_TAP: unregister_code16(KC_ENTER); break;
        case SINGLE_HOLD: unregister_code16(LSFT(KC_ENTER)); break;
        case DOUBLE_TAP: unregister_code16(KC_ENTER); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_ENTER); break;
    }
    dance_state[DANCE_ENTER].step = 0;
}
void on_dance_tab(qk_tap_dance_state_t *state, void *user_data);
void dance_tab_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_tab_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_tab(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
    }
    if(state->count > 3) {
        tap_code16(KC_TAB);
    }
}

void dance_tab_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_TAB].step = dance_step(state);
    switch (dance_state[DANCE_TAB].step) {
        case SINGLE_TAP: register_code16(KC_TAB); break;
        case SINGLE_HOLD: register_code16(RALT(KC_TAB)); break;
        case DOUBLE_TAP: register_code16(KC_TAB); register_code16(KC_TAB); break;
        case DOUBLE_HOLD: register_code16(KC_TAB); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_TAB); register_code16(KC_TAB);
    }
}

void dance_tab_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_TAB].step) {
        case SINGLE_TAP: unregister_code16(KC_TAB); break;
        case SINGLE_HOLD: unregister_code16(RALT(KC_TAB)); break;
        case DOUBLE_TAP: unregister_code16(KC_TAB); break;
        case DOUBLE_HOLD: unregister_code16(KC_TAB); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_TAB); break;
    }
    dance_state[DANCE_TAB].step = 0;
}
void on_dance_del(qk_tap_dance_state_t *state, void *user_data);
void dance_del_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_del_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_del(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_DELETE);
        tap_code16(KC_DELETE);
        tap_code16(KC_DELETE);
    }
    if(state->count > 3) {
        tap_code16(KC_DELETE);
    }
}

void dance_del_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_DEL].step = dance_step(state);
    switch (dance_state[DANCE_DEL].step) {
        case SINGLE_TAP: register_code16(KC_DELETE); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_DELETE)); break;
        case DOUBLE_TAP: register_code16(KC_DELETE); register_code16(KC_DELETE); break;
        case DOUBLE_HOLD: register_code16(KC_DELETE); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_DELETE); register_code16(KC_DELETE);
    }
}

void dance_del_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_DEL].step) {
        case SINGLE_TAP: unregister_code16(KC_DELETE); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_DELETE)); break;
        case DOUBLE_TAP: unregister_code16(KC_DELETE); break;
        case DOUBLE_HOLD: unregister_code16(KC_DELETE); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_DELETE); break;
    }
    dance_state[DANCE_DEL].step = 0;
}
void on_dance_left(qk_tap_dance_state_t *state, void *user_data);
void dance_left_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_left_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_left(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_LEFT);
        tap_code16(KC_LEFT);
        tap_code16(KC_LEFT);
    }
    if(state->count > 3) {
        tap_code16(KC_LEFT);
    }
}

void dance_left_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_LEFT].step = dance_step(state);
    switch (dance_state[DANCE_LEFT].step) {
        case SINGLE_TAP: register_code16(KC_LEFT); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_LEFT)); break;
        case DOUBLE_TAP: register_code16(KC_LEFT); register_code16(KC_LEFT); break;
        case DOUBLE_HOLD: register_code16(KC_LEFT); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_LEFT); register_code16(KC_LEFT);
    }
}

void dance_left_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_LEFT].step) {
        case SINGLE_TAP: unregister_code16(KC_LEFT); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_LEFT)); break;
        case DOUBLE_TAP: unregister_code16(KC_LEFT); break;
        case DOUBLE_HOLD: unregister_code16(KC_LEFT); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_LEFT); break;
    }
    dance_state[DANCE_LEFT].step = 0;
}
void on_dance_down(qk_tap_dance_state_t *state, void *user_data);
void dance_down_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_down_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_down(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_DOWN);
        tap_code16(KC_DOWN);
        tap_code16(KC_DOWN);
    }
    if(state->count > 3) {
        tap_code16(KC_DOWN);
    }
}

void dance_down_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_DOWN].step = dance_step(state);
    switch (dance_state[DANCE_DOWN].step) {
        case SINGLE_TAP: register_code16(KC_DOWN); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_DOWN)); break;
        case DOUBLE_TAP: register_code16(KC_DOWN); register_code16(KC_DOWN); break;
        case DOUBLE_HOLD: register_code16(KC_DOWN); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_DOWN); register_code16(KC_DOWN);
    }
}

void dance_down_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_DOWN].step) {
        case SINGLE_TAP: unregister_code16(KC_DOWN); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_DOWN)); break;
        case DOUBLE_TAP: unregister_code16(KC_DOWN); break;
        case DOUBLE_HOLD: unregister_code16(KC_DOWN); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_DOWN); break;
    }
    dance_state[DANCE_DOWN].step = 0;
}
void on_dance_up(qk_tap_dance_state_t *state, void *user_data);
void dance_up_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_up_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_up(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_UP);
        tap_code16(KC_UP);
        tap_code16(KC_UP);
    }
    if(state->count > 3) {
        tap_code16(KC_UP);
    }
}

void dance_up_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_UP].step = dance_step(state);
    switch (dance_state[DANCE_UP].step) {
        case SINGLE_TAP: register_code16(KC_UP); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_UP)); break;
        case DOUBLE_TAP: register_code16(KC_UP); register_code16(KC_UP); break;
        case DOUBLE_HOLD: register_code16(KC_UP); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_UP); register_code16(KC_UP);
    }
}

void dance_up_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_UP].step) {
        case SINGLE_TAP: unregister_code16(KC_UP); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_UP)); break;
        case DOUBLE_TAP: unregister_code16(KC_UP); break;
        case DOUBLE_HOLD: unregister_code16(KC_UP); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_UP); break;
    }
    dance_state[DANCE_UP].step = 0;
}
void on_dance_right(qk_tap_dance_state_t *state, void *user_data);
void dance_right_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_right_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_right(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_RIGHT);
        tap_code16(KC_RIGHT);
        tap_code16(KC_RIGHT);
    }
    if(state->count > 3) {
        tap_code16(KC_RIGHT);
    }
}

void dance_right_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_RIGHT].step = dance_step(state);
    switch (dance_state[DANCE_RIGHT].step) {
        case SINGLE_TAP: register_code16(KC_RIGHT); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_RIGHT)); break;
        case DOUBLE_TAP: register_code16(KC_RIGHT); register_code16(KC_RIGHT); break;
        case DOUBLE_HOLD: register_code16(KC_RIGHT); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_RIGHT); register_code16(KC_RIGHT);
    }
}

void dance_right_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_RIGHT].step) {
        case SINGLE_TAP: unregister_code16(KC_RIGHT); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_RIGHT)); break;
        case DOUBLE_TAP: unregister_code16(KC_RIGHT); break;
        case DOUBLE_HOLD: unregister_code16(KC_RIGHT); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_RIGHT); break;
    }
    dance_state[DANCE_RIGHT].step = 0;
}
void on_dance_home(qk_tap_dance_state_t *state, void *user_data);
void dance_home_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_home_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_home(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_HOME);
        tap_code16(KC_HOME);
        tap_code16(KC_HOME);
    }
    if(state->count > 3) {
        tap_code16(KC_HOME);
    }
}

void dance_home_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_HOME].step = dance_step(state);
    switch (dance_state[DANCE_HOME].step) {
        case SINGLE_TAP: register_code16(KC_HOME); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_HOME)); break;
        case DOUBLE_TAP: register_code16(KC_HOME); register_code16(KC_HOME); break;
        case DOUBLE_HOLD: register_code16(KC_HOME); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_HOME); register_code16(KC_HOME);
    }
}

void dance_home_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_HOME].step) {
        case SINGLE_TAP: unregister_code16(KC_HOME); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_HOME)); break;
        case DOUBLE_TAP: unregister_code16(KC_HOME); break;
        case DOUBLE_HOLD: unregister_code16(KC_HOME); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_HOME); break;
    }
    dance_state[DANCE_HOME].step = 0;
}
void on_dance_end(qk_tap_dance_state_t *state, void *user_data);
void dance_end_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_end_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_end(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_END);
        tap_code16(KC_END);
        tap_code16(KC_END);
    }
    if(state->count > 3) {
        tap_code16(KC_END);
    }
}

void dance_end_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_END].step = dance_step(state);
    switch (dance_state[DANCE_END].step) {
        case SINGLE_TAP: register_code16(KC_END); break;
        case SINGLE_HOLD: register_code16(RCTL(KC_END)); break;
        case DOUBLE_TAP: register_code16(KC_END); register_code16(KC_END); break;
        case DOUBLE_HOLD: register_code16(KC_END); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_END); register_code16(KC_END);
    }
}

void dance_end_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_END].step) {
        case SINGLE_TAP: unregister_code16(KC_END); break;
        case SINGLE_HOLD: unregister_code16(RCTL(KC_END)); break;
        case DOUBLE_TAP: unregister_code16(KC_END); break;
        case DOUBLE_HOLD: unregister_code16(KC_END); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_END); break;
    }
    dance_state[DANCE_END].step = 0;
}
void on_dance_7(qk_tap_dance_state_t *state, void *user_data);
void dance_7_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_7_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_7(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_7);
        tap_code16(KC_7);
        tap_code16(KC_7);
    }
    if(state->count > 3) {
        tap_code16(KC_7);
    }
}

void dance_7_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_7].step = dance_step(state);
    switch (dance_state[DANCE_7].step) {
        case SINGLE_TAP: register_code16(KC_7); break;
        case SINGLE_HOLD: register_code16(LSFT(KC_7)); break;
        case DOUBLE_TAP: register_code16(KC_7); register_code16(KC_7); break;
        case DOUBLE_HOLD: register_code16(KC_7); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_7); register_code16(KC_7);
    }
}

void dance_7_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_7].step) {
        case SINGLE_TAP: unregister_code16(KC_7); break;
        case SINGLE_HOLD: unregister_code16(LSFT(KC_7)); break;
        case DOUBLE_TAP: unregister_code16(KC_7); break;
        case DOUBLE_HOLD: unregister_code16(KC_7); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_7); break;
    }
    dance_state[DANCE_7].step = 0;
}
void on_dance_8(qk_tap_dance_state_t *state, void *user_data);
void dance_8_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_8_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_8(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_8);
        tap_code16(KC_8);
        tap_code16(KC_8);
    }
    if(state->count > 3) {
        tap_code16(KC_8);
    }
}

void dance_8_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_8].step = dance_step(state);
    switch (dance_state[DANCE_8].step) {
        case SINGLE_TAP: register_code16(KC_8); break;
        case SINGLE_HOLD: register_code16(LSFT(KC_8)); break;
        case DOUBLE_TAP: register_code16(KC_8); register_code16(KC_8); break;
        case DOUBLE_HOLD: register_code16(KC_8); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_8); register_code16(KC_8);
    }
}

void dance_8_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_8].step) {
        case SINGLE_TAP: unregister_code16(KC_8); break;
        case SINGLE_HOLD: unregister_code16(LSFT(KC_8)); break;
        case DOUBLE_TAP: unregister_code16(KC_8); break;
        case DOUBLE_HOLD: unregister_code16(KC_8); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_8); break;
    }
    dance_state[DANCE_8].step = 0;
}
void on_dance_9(qk_tap_dance_state_t *state, void *user_data);
void dance_9_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_9_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_9(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_9);
        tap_code16(KC_9);
        tap_code16(KC_9);
    }
    if(state->count > 3) {
        tap_code16(KC_9);
    }
}

void dance_9_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_9].step = dance_step(state);
    switch (dance_state[DANCE_9].step) {
        case SINGLE_TAP: register_code16(KC_9); break;
        case SINGLE_HOLD: register_code16(LSFT(KC_9)); break;
        case DOUBLE_TAP: register_code16(KC_9); register_code16(KC_9); break;
        case DOUBLE_HOLD: register_code16(KC_9); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_9); register_code16(KC_9);
    }
}

void dance_9_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_9].step) {
        case SINGLE_TAP: unregister_code16(KC_9); break;
        case SINGLE_HOLD: unregister_code16(LSFT(KC_9)); break;
        case DOUBLE_TAP: unregister_code16(KC_9); break;
        case DOUBLE_HOLD: unregister_code16(KC_9); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_9); break;
    }
    dance_state[DANCE_9].step = 0;
}
void on_dance_4(qk_tap_dance_state_t *state, void *user_data);
void dance_4_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_4_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_4(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_4);
        tap_code16(KC_4);
        tap_code16(KC_4);
    }
    if(state->count > 3) {
        tap_code16(KC_4);
    }
}

void dance_4_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_4].step = dance_step(state);
    switch (dance_state[DANCE_4].step) {
        case SINGLE_TAP: register_code16(KC_4); break;
        case SINGLE_HOLD: register_code16(LSFT(KC_4)); break;
        case DOUBLE_TAP: register_code16(KC_4); register_code16(KC_4); break;
        case DOUBLE_HOLD: register_code16(KC_4); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_4); register_code16(KC_4);
    }
}

void dance_4_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_4].step) {
        case SINGLE_TAP: unregister_code16(KC_4); break;
        case SINGLE_HOLD: unregister_code16(LSFT(KC_4)); break;
        case DOUBLE_TAP: unregister_code16(KC_4); break;
        case DOUBLE_HOLD: unregister_code16(KC_4); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_4); break;
    }
    dance_state[DANCE_4].step = 0;
}
void on_dance_5(qk_tap_dance_state_t *state, void *user_data);
void dance_5_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_5_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_5(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_5);
        tap_code16(KC_5);
        tap_code16(KC_5);
    }
    if(state->count > 3) {
        tap_code16(KC_5);
    }
}

void dance_5_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_5].step = dance_step(state);
    switch (dance_state[DANCE_5].step) {
        case SINGLE_TAP: register_code16(KC_5); break;
        case SINGLE_HOLD: register_code16(LSFT(KC_5)); break;
        case DOUBLE_TAP: register_code16(KC_5); register_code16(KC_5); break;
        case DOUBLE_HOLD: register_code16(KC_5); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_5); register_code16(KC_5);
    }
}

void dance_5_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_5].step) {
        case SINGLE_TAP: unregister_code16(KC_5); break;
        case SINGLE_HOLD: unregister_code16(LSFT(KC_5)); break;
        case DOUBLE_TAP: unregister_code16(KC_5); break;
        case DOUBLE_HOLD: unregister_code16(KC_5); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_5); break;
    }
    dance_state[DANCE_5].step = 0;
}
void on_dance_6(qk_tap_dance_state_t *state, void *user_data);
void dance_6_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_6_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_6(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_6);
        tap_code16(KC_6);
        tap_code16(KC_6);
    }
    if(state->count > 3) {
        tap_code16(KC_6);
    }
}

void dance_6_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_6].step = dance_step(state);
    switch (dance_state[DANCE_6].step) {
        case SINGLE_TAP: register_code16(KC_6); break;
        case SINGLE_HOLD: register_code16(LSFT(KC_6)); break;
        case DOUBLE_TAP: register_code16(KC_6); register_code16(KC_6); break;
        case DOUBLE_HOLD: register_code16(KC_6); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_6); register_code16(KC_6);
    }
}

void dance_6_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_6].step) {
        case SINGLE_TAP: unregister_code16(KC_6); break;
        case SINGLE_HOLD: unregister_code16(LSFT(KC_6)); break;
        case DOUBLE_TAP: unregister_code16(KC_6); break;
        case DOUBLE_HOLD: unregister_code16(KC_6); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_6); break;
    }
    dance_state[DANCE_6].step = 0;
}
void on_dance_1(qk_tap_dance_state_t *state, void *user_data);
void dance_1_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_1_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_1(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_1);
        tap_code16(KC_1);
        tap_code16(KC_1);
    }
    if(state->count > 3) {
        tap_code16(KC_1);
    }
}

void dance_1_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_1].step = dance_step(state);
    switch (dance_state[DANCE_1].step) {
        case SINGLE_TAP: register_code16(KC_1); break;
        case SINGLE_HOLD: register_code16(LSFT(KC_1)); break;
        case DOUBLE_TAP: register_code16(KC_1); register_code16(KC_1); break;
        case DOUBLE_HOLD: register_code16(KC_1); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_1); register_code16(KC_1);
    }
}

void dance_1_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_1].step) {
        case SINGLE_TAP: unregister_code16(KC_1); break;
        case SINGLE_HOLD: unregister_code16(LSFT(KC_1)); break;
        case DOUBLE_TAP: unregister_code16(KC_1); break;
        case DOUBLE_HOLD: unregister_code16(KC_1); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_1); break;
    }
    dance_state[DANCE_1].step = 0;
}
void on_dance_2(qk_tap_dance_state_t *state, void *user_data);
void dance_2_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_2_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_2(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_2);
        tap_code16(KC_2);
        tap_code16(KC_2);
    }
    if(state->count > 3) {
        tap_code16(KC_2);
    }
}

void dance_2_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_2].step = dance_step(state);
    switch (dance_state[DANCE_2].step) {
        case SINGLE_TAP: register_code16(KC_2); break;
        case SINGLE_HOLD: register_code16(LSFT(KC_2)); break;
        case DOUBLE_TAP: register_code16(KC_2); register_code16(KC_2); break;
        case DOUBLE_HOLD: register_code16(KC_2); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_2); register_code16(KC_2);
    }
}

void dance_2_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_2].step) {
        case SINGLE_TAP: unregister_code16(KC_2); break;
        case SINGLE_HOLD: unregister_code16(LSFT(KC_2)); break;
        case DOUBLE_TAP: unregister_code16(KC_2); break;
        case DOUBLE_HOLD: unregister_code16(KC_2); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_2); break;
    }
    dance_state[DANCE_2].step = 0;
}
void on_dance_3(qk_tap_dance_state_t *state, void *user_data);
void dance_3_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_3_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_3(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_3);
        tap_code16(KC_3);
        tap_code16(KC_3);
    }
    if(state->count > 3) {
        tap_code16(KC_3);
    }
}

void dance_3_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_3].step = dance_step(state);
    switch (dance_state[DANCE_3].step) {
        case SINGLE_TAP: register_code16(KC_3); break;
        case SINGLE_HOLD: register_code16(LSFT(KC_3)); break;
        case DOUBLE_TAP: register_code16(KC_3); register_code16(KC_3); break;
        case DOUBLE_HOLD: register_code16(KC_3); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_3); register_code16(KC_3);
    }
}

void dance_3_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_3].step) {
        case SINGLE_TAP: unregister_code16(KC_3); break;
        case SINGLE_HOLD: unregister_code16(LSFT(KC_3)); break;
        case DOUBLE_TAP: unregister_code16(KC_3); break;
        case DOUBLE_HOLD: unregister_code16(KC_3); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_3); break;
    }
    dance_state[DANCE_3].step = 0;
}
void on_dance_slash(qk_tap_dance_state_t *state, void *user_data);
void dance_slash_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_slash_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_slash(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_SLASH);
        tap_code16(KC_SLASH);
        tap_code16(KC_SLASH);
    }
    if(state->count > 3) {
        tap_code16(KC_SLASH);
    }
}

void dance_slash_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_SLASH].step = dance_step(state);
    switch (dance_state[DANCE_SLASH].step) {
        case SINGLE_TAP: register_code16(KC_SLASH); break;
        case SINGLE_HOLD: register_code16(RSFT(KC_SLASH)); break;
        case DOUBLE_TAP: register_code16(KC_SLASH); register_code16(KC_SLASH); break;
        case DOUBLE_HOLD: register_code16(KC_SLASH); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_SLASH); register_code16(KC_SLASH);
    }
}

void dance_slash_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_SLASH].step) {
        case SINGLE_TAP: unregister_code16(KC_SLASH); break;
        case SINGLE_HOLD: unregister_code16(RSFT(KC_SLASH)); break;
        case DOUBLE_TAP: unregister_code16(KC_SLASH); break;
        case DOUBLE_HOLD: unregister_code16(KC_SLASH); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_SLASH); break;
    }
    dance_state[DANCE_SLASH].step = 0;
}

// Tap dance for macros: { -> {}, {};
void on_dance_cbr(qk_tap_dance_state_t *state, void *user_data);
void dance_cbr_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_cbr_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_cbr(qk_tap_dance_state_t *state, void *user_data) {
    if (state->count == 3) {
        tap_code16(KC_LCBR);
        tap_code16(KC_LCBR);
        tap_code16(KC_LCBR);
    }
    if (state->count > 3) {
        tap_code16(KC_LCBR);
    }
}

void dance_cbr_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_CBR].step = dance_step(state);
    switch (dance_state[DANCE_CBR].step) {
        case SINGLE_TAP: register_code16(KC_LCBR); break;
        case SINGLE_HOLD: SEND_STRING("{}" SS_TAP(X_LEFT)); break;
        case DOUBLE_TAP: register_code16(KC_LCBR); register_code16(KC_LCBR); break;
        case DOUBLE_HOLD: SEND_STRING("{};" SS_TAP(X_LEFT) SS_TAP(X_LEFT)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_LCBR); register_code16(KC_LCBR);
    }
}

void dance_cbr_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_CBR].step) {
        case SINGLE_TAP: unregister_code16(KC_LCBR); break;
        case DOUBLE_TAP: unregister_code16(KC_LCBR); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_LCBR); break;
    }
    dance_state[DANCE_CBR].step = 0;
}

// Tap dance for macros: ( -> (), ();
void on_dance_prn(qk_tap_dance_state_t *state, void *user_data);
void dance_prn_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_prn_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_prn(qk_tap_dance_state_t *state, void *user_data) {
    if (state->count == 3) {
        tap_code16(KC_LPRN);
        tap_code16(KC_LPRN);
        tap_code16(KC_LPRN);
    }
    if (state->count > 3) {
        tap_code16(KC_LPRN);
    }
}

void dance_prn_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_PRN].step = dance_step(state);
    switch (dance_state[DANCE_PRN].step) {
        case SINGLE_TAP: register_code16(KC_LPRN); break;
        case SINGLE_HOLD: SEND_STRING("()" SS_TAP(X_LEFT)); break;
        case DOUBLE_TAP: register_code16(KC_LPRN); register_code16(KC_LPRN); break;
        case DOUBLE_HOLD: SEND_STRING("();" SS_TAP(X_LEFT) SS_TAP(X_LEFT)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_LPRN); register_code16(KC_LPRN);
    }
}

void dance_prn_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_PRN].step) {
        case SINGLE_TAP: unregister_code16(KC_LPRN); break;
        case DOUBLE_TAP: unregister_code16(KC_LPRN); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_LPRN); break;
    }
    dance_state[DANCE_PRN].step = 0;
}

// Tap dance for macros: [ -> [], [];
void on_dance_sbr(qk_tap_dance_state_t *state, void *user_data);
void dance_sbr_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_sbr_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_sbr(qk_tap_dance_state_t *state, void *user_data) {
    if (state->count == 3) {
        tap_code16(KC_LBRACKET);
        tap_code16(KC_LBRACKET);
        tap_code16(KC_LBRACKET);
    }
    if (state->count > 3) {
        tap_code16(KC_LBRACKET);
    }
}

void dance_sbr_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_SBR].step = dance_step(state);
    switch (dance_state[DANCE_SBR].step) {
        case SINGLE_TAP: register_code16(KC_LBRACKET); break;
        case SINGLE_HOLD: SEND_STRING("[]" SS_TAP(X_LEFT)); break;
        case DOUBLE_TAP: register_code16(KC_LBRACKET); register_code16(KC_LBRACKET); break;
        case DOUBLE_HOLD: SEND_STRING("[];" SS_TAP(X_LEFT) SS_TAP(X_LEFT)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_LBRACKET); register_code16(KC_LBRACKET);
    }
}

void dance_sbr_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_SBR].step) {
        case SINGLE_TAP: unregister_code16(KC_LBRACKET); break;
        case DOUBLE_TAP: unregister_code16(KC_LBRACKET); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_LBRACKET); break;
    }
    dance_state[DANCE_SBR].step = 0;
}

// Tap dance for macro: < -> <>
void on_dance_abk(qk_tap_dance_state_t *state, void *user_data);
void dance_abk_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_abk_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_abk(qk_tap_dance_state_t *state, void *user_data) {
    if (state->count == 3) {
        tap_code16(KC_LABK);
        tap_code16(KC_LABK);
        tap_code16(KC_LABK);
    }
    if (state->count > 3) {
        tap_code16(KC_LABK);
    }
}

void dance_abk_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_ABK].step = dance_step(state);
    switch (dance_state[DANCE_ABK].step) {
        case SINGLE_TAP: register_code16(KC_LABK); break;
        case SINGLE_HOLD: SEND_STRING("<>" SS_TAP(X_LEFT)); break;
        case DOUBLE_TAP: register_code16(KC_LABK); register_code16(KC_LABK); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_LABK); register_code16(KC_LABK);
    }
}

void dance_abk_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_ABK].step) {
        case SINGLE_TAP: unregister_code16(KC_LABK); break;
        case DOUBLE_TAP: unregister_code16(KC_LABK); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_LABK); break;
    }
    dance_state[DANCE_ABK].step = 0;
}

// Tap dance for macros: - -> '', '';
void on_dance_sqt(qk_tap_dance_state_t *state, void *user_data);
void dance_sqt_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_sqt_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_sqt(qk_tap_dance_state_t *state, void *user_data) {
    if (state->count == 3) {
        tap_code16(KC_MINUS);
        tap_code16(KC_MINUS);
        tap_code16(KC_MINUS);
    }
    if (state->count > 3) {
        tap_code16(KC_MINUS);
    }
}

void dance_sqt_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_SQT].step = dance_step(state);
    switch (dance_state[DANCE_SQT].step) {
        case SINGLE_TAP: register_code16(KC_MINUS); break;
        case SINGLE_HOLD: SEND_STRING(SS_TAP(X_QUOTE)SS_TAP(X_QUOTE)SS_TAP(X_LEFT)); break;
        case DOUBLE_TAP: register_code16(KC_MINUS); register_code16(KC_MINUS); break;
        case DOUBLE_HOLD: SEND_STRING(SS_TAP(X_QUOTE)SS_TAP(X_QUOTE)";"SS_TAP(X_LEFT)SS_TAP(X_LEFT)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_MINUS); register_code16(KC_MINUS);
    }
}

void dance_sqt_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_SQT].step) {
        case SINGLE_TAP: unregister_code16(KC_MINUS); break;
        case DOUBLE_TAP: unregister_code16(KC_MINUS); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_MINUS); break;
    }
    dance_state[DANCE_SQT].step = 0;
}

// Tap dance for macros: \ -> "", "";
void on_dance_dqt(qk_tap_dance_state_t *state, void *user_data);
void dance_dqt_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_dqt_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_dqt(qk_tap_dance_state_t *state, void *user_data) {
    if (state->count == 3) {
        tap_code16(KC_BSLASH);
        tap_code16(KC_BSLASH);
        tap_code16(KC_BSLASH);
    }
    if (state->count > 3) {
        tap_code16(KC_BSLASH);
    }
}

void dance_dqt_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_DQT].step = dance_step(state);
    switch (dance_state[DANCE_DQT].step) {
        case SINGLE_TAP: register_code16(KC_BSLASH); break;
        case SINGLE_HOLD: SEND_STRING(SS_LSFT(SS_TAP(X_QUOTE))SS_LSFT(SS_TAP(X_QUOTE))SS_TAP(X_LEFT)); break;
        case DOUBLE_TAP: register_code16(KC_BSLASH); register_code16(KC_BSLASH); break;
        case DOUBLE_HOLD: SEND_STRING(SS_LSFT(SS_TAP(X_QUOTE))SS_LSFT(SS_TAP(X_QUOTE))";"SS_TAP(X_LEFT)SS_TAP(X_LEFT)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_BSLASH); register_code16(KC_BSLASH);
    }
}

void dance_dqt_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_DQT].step) {
        case SINGLE_TAP: unregister_code16(KC_BSLASH); break;
        case DOUBLE_TAP: unregister_code16(KC_BSLASH); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_BSLASH); break;
    }
    dance_state[DANCE_DQT].step = 0;
}

// Tap dance for numpad 0
void on_dance_0(qk_tap_dance_state_t *state, void *user_data);
void dance_0_finished(qk_tap_dance_state_t *state, void *user_data);
void dance_0_reset(qk_tap_dance_state_t *state, void *user_data);

void on_dance_0(qk_tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_0);
        tap_code16(KC_0);
        tap_code16(KC_0);
    }
    if(state->count > 3) {
        tap_code16(KC_0);
    }
}

void dance_0_finished(qk_tap_dance_state_t *state, void *user_data) {
    dance_state[DANCE_0].step = dance_step(state);
    switch (dance_state[DANCE_0].step) {
        case SINGLE_TAP: register_code16(KC_0); break;
        case SINGLE_HOLD: register_code16(RSFT(KC_0)); break;
        case DOUBLE_TAP: register_code16(KC_0); register_code16(KC_0); break;
        case DOUBLE_HOLD: register_code16(KC_0); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_0); register_code16(KC_0);
    }
}

void dance_0_reset(qk_tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[DANCE_0].step) {
        case SINGLE_TAP: unregister_code16(KC_0); break;
        case SINGLE_HOLD: unregister_code16(RSFT(KC_0)); break;
        case DOUBLE_TAP: unregister_code16(KC_0); break;
        case DOUBLE_HOLD: unregister_code16(KC_0); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_0); break;
    }
    dance_state[DANCE_0].step = 0;
}

qk_tap_dance_action_t tap_dance_actions[] = {
        [DANCE_Q] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_q, dance_q_finished, dance_q_reset),
        [DANCE_W] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_w, dance_w_finished, dance_w_reset),
        [DANCE_E] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_e, dance_e_finished, dance_e_reset),
        [DANCE_R] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_r, dance_r_finished, dance_r_reset),
        [DANCE_T] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_t, dance_t_finished, dance_t_reset),
        [DANCE_A] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_a, dance_a_finished, dance_a_reset),
        [DANCE_S] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_s, dance_s_finished, dance_s_reset),
        [DANCE_D] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_d, dance_d_finished, dance_d_reset),
        [DANCE_F] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_f, dance_f_finished, dance_f_reset),
        [DANCE_G] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_g, dance_g_finished, dance_g_reset),
        [DANCE_Z] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_z, dance_z_finished, dance_z_reset),
        [DANCE_X] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_x, dance_x_finished, dance_x_reset),
        [DANCE_C] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_c, dance_c_finished, dance_c_reset),
        [DANCE_V] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_v, dance_v_finished, dance_v_reset),
        [DANCE_B] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_b, dance_b_finished, dance_b_reset),
        [DANCE_BSPACE] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_bspace, dance_bspace_finished, dance_bspace_reset),
        [DANCE_Y] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_y, dance_y_finished, dance_y_reset),
        [DANCE_U] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_u, dance_u_finished, dance_u_reset),
        [DANCE_I] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_i, dance_i_finished, dance_i_reset),
        [DANCE_O] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_o, dance_o_finished, dance_o_reset),
        [DANCE_P] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_p, dance_p_finished, dance_p_reset),
        [DANCE_H] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_h, dance_h_finished, dance_h_reset),
        [DANCE_J] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_j, dance_j_finished, dance_j_reset),
        [DANCE_K] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_k, dance_k_finished, dance_k_reset),
        [DANCE_L] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_l, dance_l_finished, dance_l_reset),
        [DANCE_N] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_n, dance_n_finished, dance_n_reset),
        [DANCE_M] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_m, dance_m_finished, dance_m_reset),
        [DANCE_ENTER] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_enter, dance_enter_finished, dance_enter_reset),
        [DANCE_TAB] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_tab, dance_tab_finished, dance_tab_reset),
        [DANCE_DEL] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_del, dance_del_finished, dance_del_reset),
        [DANCE_LEFT] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_left, dance_left_finished, dance_left_reset),
        [DANCE_DOWN] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_down, dance_down_finished, dance_down_reset),
        [DANCE_UP] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_up, dance_up_finished, dance_up_reset),
        [DANCE_RIGHT] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_right, dance_right_finished, dance_right_reset),
        [DANCE_HOME] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_home, dance_home_finished, dance_home_reset),
        [DANCE_END] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_end, dance_end_finished, dance_end_reset),
        [DANCE_7] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_7, dance_7_finished, dance_7_reset),
        [DANCE_8] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_8, dance_8_finished, dance_8_reset),
        [DANCE_9] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_9, dance_9_finished, dance_9_reset),
        [DANCE_4] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_4, dance_4_finished, dance_4_reset),
        [DANCE_5] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_5, dance_5_finished, dance_5_reset),
        [DANCE_6] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_6, dance_6_finished, dance_6_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
        [DANCE_SLASH] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_slash, dance_slash_finished, dance_slash_reset),
        [DANCE_CBR] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_cbr, dance_cbr_finished, dance_cbr_reset),
        [DANCE_PRN] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_prn, dance_prn_finished, dance_prn_reset),
        [DANCE_SBR] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_sbr, dance_sbr_finished, dance_sbr_reset),
        [DANCE_ABK] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_abk, dance_abk_finished, dance_abk_reset),
        [DANCE_SQT] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_sqt, dance_sqt_finished, dance_sqt_reset),
        [DANCE_DQT] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_dqt, dance_dqt_finished, dance_dqt_reset),
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
};
