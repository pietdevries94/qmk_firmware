#include QMK_KEYBOARD_H
#include "rgb_matrix_user.h"

enum alt_keycodes {
    L_BRI = SAFE_RANGE, //LED Brightness Increase
    L_BRD,              //LED Brightness Decrease
    L_PTN,              //LED Pattern Select Next
    L_PTP,              //LED Pattern Select Previous
    L_PSI,              //LED Pattern Speed Increase
    L_PSD,              //LED Pattern Speed Decrease
    L_T_MD,             //LED Toggle Mode
    L_T_ONF,            //LED Toggle On / Off
    L_ON,               //LED On
    L_OFF,              //LED Off
    L_T_BR,             //LED Toggle Breath Effect
    L_T_PTD,            //LED Toggle Scrolling Pattern Direction
    U_T_AUTO,           //USB Extra Port Toggle Auto Detect / Always Active
    U_T_AGCR,           //USB Toggle Automatic GCR control
    DBG_TOG,            //DEBUG Toggle On / Off
    DBG_MTRX,           //DEBUG Toggle Matrix Prints
    DBG_KBD,            //DEBUG Toggle Keyboard Prints
    DBG_MOU,            //DEBUG Toggle Mouse Prints
    MD_BOOT,            //Restart into bootloader after hold timeout
};

enum {
  TD_F13_F14 = 0,
  TD_F15_F16 = 1,
  TD_F17_F18 = 2,
  TD_F19_F20 = 3,
  TD_F21_F22 = 4,
  TD_F23_F24 = 5,
};

#define TG_NKRO MAGIC_TOGGLE_NKRO //Toggle 6KRO / NKRO mode

keymap_config_t keymap_config;

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [0] = LAYOUT(
        KC_ESC,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC, TD(TD_F19_F20), \
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC, KC_BSLS, TD(TD_F21_F22), \
        KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,          KC_ENT,  TD(TD_F23_F24), \
        KC_LSPO, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_RSPC,          KC_UP,   KC_DEL, \
        KC_LCTL, KC_LALT, KC_LGUI,                            KC_SPC,                             MO(1),   KC_RCTL, KC_LEFT, KC_DOWN, KC_RGHT  \
    ),
    [1] = LAYOUT(
        KC_GRV,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  KC_TRNS, TD(TD_F13_F14), \
        L_T_BR,  L_PSD,   L_BRI,   L_PSI,   KC_TRNS, KC_TRNS, KC_TRNS, U_T_AUTO,U_T_AGCR,KC_TRNS, KC_PSCR, KC_SLCK, KC_PAUS, KC_TRNS, TD(TD_F15_F16), \
        L_T_PTD, L_PTP,   L_BRD,   L_PTN,   KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,          KC_TRNS, TD(TD_F17_F18), \
        KC_TRNS, L_T_MD,  L_T_ONF, KC_TRNS, KC_TRNS, MD_BOOT, TG_NKRO, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,          KC_PGUP, KC_TRNS, \
        KC_TRNS, KC_TRNS, KC_TRNS,                            KC_TRNS,                            KC_TRNS, KC_TRNS, KC_HOME, KC_PGDN, KC_END  \
    ),
    /*
    [X] = LAYOUT(
        KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, \
        KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, \
        KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,          KC_TRNS, KC_TRNS, \
        KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,          KC_TRNS, KC_TRNS, \
        KC_TRNS, KC_TRNS, KC_TRNS,                            KC_TRNS,                            KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS  \
    ),
    */
};

const uint16_t PROGMEM fn_actions[] = {

};

// Runs just one time when the keyboard initializes.
void matrix_init_user(void) {
};

// Runs constantly in the background, in a loop.
void matrix_scan_user(void) {
};

#define MODS_SHIFT  (keyboard_report->mods & MOD_BIT(KC_LSHIFT) || keyboard_report->mods & MOD_BIT(KC_RSHIFT))
#define MODS_CTRL  (keyboard_report->mods & MOD_BIT(KC_LCTL) || keyboard_report->mods & MOD_BIT(KC_RCTRL))
#define MODS_ALT  (keyboard_report->mods & MOD_BIT(KC_LALT) || keyboard_report->mods & MOD_BIT(KC_RALT))

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    static uint32_t key_timer;

    switch (keycode) {
        case L_BRI:
            if (record->event.pressed) {
                if (LED_GCR_STEP > LED_GCR_MAX - gcr_desired) gcr_desired = LED_GCR_MAX;
                else gcr_desired += LED_GCR_STEP;
                if (led_animation_breathing) gcr_breathe = gcr_desired;
            }
            return false;
        case L_BRD:
            if (record->event.pressed) {
                if (LED_GCR_STEP > gcr_desired) gcr_desired = 0;
                else gcr_desired -= LED_GCR_STEP;
                if (led_animation_breathing) gcr_breathe = gcr_desired;
            }
            return false;
        case L_PTN:
            if (record->event.pressed) {
                if (led_animation_id == led_setups_count - 1) led_animation_id = 0;
                else led_animation_id++;
            }
            return false;
        case L_PTP:
            if (record->event.pressed) {
                if (led_animation_id == 0) led_animation_id = led_setups_count - 1;
                else led_animation_id--;
            }
            return false;
        case L_PSI:
            if (record->event.pressed) {
                led_animation_speed += ANIMATION_SPEED_STEP;
            }
            return false;
        case L_PSD:
            if (record->event.pressed) {
                led_animation_speed -= ANIMATION_SPEED_STEP;
                if (led_animation_speed < 0) led_animation_speed = 0;
            }
            return false;
        case L_T_MD:
            if (record->event.pressed) {
                led_lighting_mode++;
                if (led_lighting_mode > LED_MODE_MAX_INDEX) led_lighting_mode = LED_MODE_NORMAL;
            }
            return false;
        case L_T_ONF:
            if (record->event.pressed) {
                led_enabled = !led_enabled;
                I2C3733_Control_Set(led_enabled);
            }
            return false;
        case L_ON:
            if (record->event.pressed) {
                led_enabled = 1;
                I2C3733_Control_Set(led_enabled);
            }
            return false;
        case L_OFF:
            if (record->event.pressed) {
                led_enabled = 0;
                I2C3733_Control_Set(led_enabled);
            }
            return false;
        case L_T_BR:
            if (record->event.pressed) {
                led_animation_breathing = !led_animation_breathing;
                if (led_animation_breathing) {
                    gcr_breathe = gcr_desired;
                    led_animation_breathe_cur = BREATHE_MIN_STEP;
                    breathe_dir = 1;
                }
            }
            return false;
        case L_T_PTD:
            if (record->event.pressed) {
                led_animation_direction = !led_animation_direction;
            }
            return false;
        case U_T_AUTO:
            if (record->event.pressed && MODS_SHIFT && MODS_CTRL) {
                TOGGLE_FLAG_AND_PRINT(usb_extra_manual, "USB extra port manual mode");
            }
            return false;
        case U_T_AGCR:
            if (record->event.pressed && MODS_SHIFT && MODS_CTRL) {
                TOGGLE_FLAG_AND_PRINT(usb_gcr_auto, "USB GCR auto mode");
            }
            return false;
        case DBG_TOG:
            if (record->event.pressed) {
                TOGGLE_FLAG_AND_PRINT(debug_enable, "Debug mode");
            }
            return false;
        case DBG_MTRX:
            if (record->event.pressed) {
                TOGGLE_FLAG_AND_PRINT(debug_matrix, "Debug matrix");
            }
            return false;
        case DBG_KBD:
            if (record->event.pressed) {
                TOGGLE_FLAG_AND_PRINT(debug_keyboard, "Debug keyboard");
            }
            return false;
        case DBG_MOU:
            if (record->event.pressed) {
                TOGGLE_FLAG_AND_PRINT(debug_mouse, "Debug mouse");
            }
            return false;
        case MD_BOOT:
            if (record->event.pressed) {
                key_timer = timer_read32();
            } else {
                if (timer_elapsed32(key_timer) >= 500) {
                    reset_keyboard();
                }
            }
            return false;
        default:
            return true; //Process all other keycodes normally
    }
}

// All tapdance functions are defined here
// void register_hyper (void) {
//     register_code (KC_LSFT);
//     register_code (KC_LCTL);
//     register_code (KC_LALT);
//     register_code (KC_LGUI);
// }

// void unregister_hyper (void) {
//     unregister_code (KC_LSFT);
//     unregister_code (KC_LCTL);
//     unregister_code (KC_LALT);
//     unregister_code (KC_LGUI);
// }

void tap_dance_f13_f14_start (qk_tap_dance_state_t *state, void *user_data) {
//  register_hyper ();
  if (state->count == 1) {
    register_code (KC_F13);
  } else {
    register_code (KC_F14);
  }
}

void tap_dance_f13_f14_stop (qk_tap_dance_state_t *state, void *user_data) {
//  unregister_hyper ();
  if (state->count == 1) {
    unregister_code (KC_F13);
  } else {
    unregister_code (KC_F14);
  }
}

void tap_dance_f15_f16_start (qk_tap_dance_state_t *state, void *user_data) {
//  register_hyper ();
  if (state->count == 1) {
    register_code (KC_F15);
  } else {
    register_code (KC_F16);
  }
}

void tap_dance_f15_f16_stop (qk_tap_dance_state_t *state, void *user_data) {
//  unregister_hyper ();
  if (state->count == 1) {
    unregister_code (KC_F15);
  } else {
    unregister_code (KC_F16);
  }
}

void tap_dance_f17_f18_start (qk_tap_dance_state_t *state, void *user_data) {
//  register_hyper ();
  if (state->count == 1) {
    register_code (KC_F17);
  } else {
    register_code (KC_F18);
  }
}

void tap_dance_f17_f18_stop (qk_tap_dance_state_t *state, void *user_data) {
//  unregister_hyper ();
  if (state->count == 1) {
    unregister_code (KC_F17);
  } else {
    unregister_code (KC_F18);
  }
}

void tap_dance_f19_f20_start (qk_tap_dance_state_t *state, void *user_data) {
//  register_hyper ();
  if (state->count == 1) {
    register_code (KC_F19);
  } else {
    register_code (KC_F20);
  }
}

void tap_dance_f19_f20_stop (qk_tap_dance_state_t *state, void *user_data) {
//  unregister_hyper ();
  if (state->count == 1) {
    unregister_code (KC_F19);
  } else {
    unregister_code (KC_F20);
  }
}

void tap_dance_f21_f22_start (qk_tap_dance_state_t *state, void *user_data) {
//  register_hyper ();
  if (state->count == 1) {
    register_code (KC_F21);
  } else {
    register_code (KC_F22);
  }
}

void tap_dance_f21_f22_stop (qk_tap_dance_state_t *state, void *user_data) {
//  unregister_hyper ();
  if (state->count == 1) {
    unregister_code (KC_F21);
  } else {
    unregister_code (KC_F22);
  }
}

void tap_dance_f23_f24_start (qk_tap_dance_state_t *state, void *user_data) {
//  register_hyper ();
  if (state->count == 1) {
    register_code (KC_F23);
  } else {
    register_code (KC_F24);
  }
}

void tap_dance_f23_f24_stop (qk_tap_dance_state_t *state, void *user_data) {
// //  unregister_hyper ();
  if (state->count == 1) {
    unregister_code (KC_F23);
  } else {
    unregister_code (KC_F24);
  }
}

//All tap dance functions would go here. Only showing this one.
qk_tap_dance_action_t tap_dance_actions[] = {
 [TD_F13_F14] = ACTION_TAP_DANCE_FN_ADVANCED (NULL, tap_dance_f13_f14_start, tap_dance_f13_f14_stop),
 [TD_F15_F16] = ACTION_TAP_DANCE_FN_ADVANCED (NULL, tap_dance_f15_f16_start, tap_dance_f15_f16_stop),
 [TD_F17_F18] = ACTION_TAP_DANCE_FN_ADVANCED (NULL, tap_dance_f17_f18_start, tap_dance_f17_f18_stop),
 [TD_F19_F20] = ACTION_TAP_DANCE_FN_ADVANCED (NULL, tap_dance_f19_f20_start, tap_dance_f19_f20_stop),
 [TD_F21_F22] = ACTION_TAP_DANCE_FN_ADVANCED (NULL, tap_dance_f21_f22_start, tap_dance_f21_f22_stop),
 [TD_F23_F24] = ACTION_TAP_DANCE_FN_ADVANCED (NULL, tap_dance_f23_f24_start, tap_dance_f23_f24_stop)
};
