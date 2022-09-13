#include "quantum.h"
#include "os_swap.h"

/* Windows and Linux are considered default operating systems. */
static enum operating_system CUR_OS = WINLIN;

static inline uint16_t _get_swap_keycode(uint16_t index) {
    switch(CUR_OS) {
        case WINLIN: return os_swap_mappings[index][WINLIN];
        case MAC_OS: return os_swap_mappings[index][MACOS];
    }
}

static inline bool _handle_toggle(keyrecord_t *record) {
    if (!record->event.pressed) {
         return true;
    }

    switch (CUR_OS) {
        case WINLIN: CUR_OS = MACOS; break;
        case MACOS: CUR_OS = WINLIN; break;
    }

    return false;
}

static inline bool _handle_swap_keycode(uint16_t index, keyrecord_t *record) {
    uint16_t code = _get_swap_keycode(index);

    if (record->event.pressed) {
        register_code16(code);
    } else {
        unregister_code16(code);
    }

    return false;
}

bool process_os_swap(uint16_t keycode, keyrecord_t *record) {
    switch(keycode) {
        case OS_SWAP_TOGGLE:
            return _handle_toggle(record);
        case OS_SWAP_MIN ... OS_SWAP_MAX:
            return _handle_swap_keycode(keycode - OS_SWAP_MIN, record);
    }

    return true;
}

uint16_t os_swap_keycode(uint16_t keycode) {
    switch(keycode) {
        case OS_SWAP_MIN ... OS_SWAP_MAX:;
            return _get_swap_keycode(keycode - OS_SWAP_MIN);
    }
    
    return KC_NO;
}