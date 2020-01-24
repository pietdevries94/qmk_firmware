// !!! DO NOT ADD #pragma once !!! //

// Step 1.
// Declare custom effects using the RGB_MATRIX_EFFECT macro
// (note the lack of semicolon after the macro!)
RGB_MATRIX_EFFECT(my_cool_effect)

// Step 2.
// Define effects inside the `RGB_MATRIX_CUSTOM_EFFECT_IMPLS` ifdef block
#ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS

// e.g: A simple effect, self-contained within a single method
static bool my_cool_effect(effect_params_t* params) {
  RGB_MATRIX_USE_LIMITS(led_min, led_max);
  for (uint8_t i = led_min; i < led_max; i++) {
    if (i > 66) {
      // underglow
      rgb_matrix_set_color(i, 0x99, 0xff, 0x00);
    } else {
      // keys
      rgb_matrix_set_color(i, 0xff, 0xc0, 0x00);
    }
  }
  return led_max < DRIVER_LED_TOTAL;
}

#endif // RGB_MATRIX_CUSTOM_EFFECT_IMPLS
