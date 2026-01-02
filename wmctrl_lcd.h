#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct wm_hw_s;

void wm_lcd_init(struct wm_hw_s *hw);
void wm_lcd_clear(struct wm_hw_s *hw);
void wm_lcd_set_cursor(struct wm_hw_s *hw, uint8_t row, uint8_t col);
void wm_lcd_print(struct wm_hw_s *hw, const char *s);

#ifdef __cplusplus
}
#endif
