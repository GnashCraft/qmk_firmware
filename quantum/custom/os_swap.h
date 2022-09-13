#pragma once

#ifdef OS_SWAP_ENABLE

#include <inttypes.h>
#include "action.h"


#define OS_COUNT 2
enum operating_system {
    WINLIN = 0,
    MACOS,
};

extern uint16_t os_swap_mappings[][OS_COUNT];

bool process_os_swap(uint16_t keycode, keyrecord_t *record);
uint16_t os_swap_keycode(uint16_t index);

#define OS(n) (OS_SWAP_MIN | ((n)&0xFF))
#define OS_SWAP_ENTRY(index, kc0, kc1) [index] = {[WINLIN] = kc0, [MACOS] = kc1}

#else

#define OS(n) KC_NO
#define OS_SWAP_ENTRY {}

#endif