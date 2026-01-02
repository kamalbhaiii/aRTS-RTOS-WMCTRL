#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* GPIO minors (must match your board registration) */
enum
{
  WM_GPIO_DOOR      = 0,  /* PB5 */
  WM_GPIO_VIB       = 1,  /* PB4 */
  WM_GPIO_RELAY     = 2,  /* PA5 */
  WM_GPIO_BUZZER    = 3,  /* PA9 */
  WM_GPIO_MODE_BTN  = 4,  /* PC1 */
  WM_GPIO_B1_USER   = 5,  /* PC13 */

  WM_GPIO_L298_IN1  = 6,  /* PB2 */
  WM_GPIO_L298_IN2  = 7,  /* PB1 */

  WM_GPIO_LCD_RS    = 8,  /* PA0 */
  WM_GPIO_LCD_E     = 9,  /* PA1 */
  WM_GPIO_LCD_D4    = 10, /* PA4 */
  WM_GPIO_LCD_D5    = 11, /* PB0 */
  WM_GPIO_LCD_D6    = 12, /* PB7 */
  WM_GPIO_LCD_D7    = 13  /* PB6 */
};

/* With FD-per-block=8 we must assume we may only have 1..2 FDs available.
 * Therefore we keep no persistent FDs at all.
 */
struct wm_hw_s
{
  const char *adc_dev_preferred; /* may be NULL; we will probe adc0/adc1 */
};

int  wm_hw_init(struct wm_hw_s *hw, const char *adc_dev_preferred);
void wm_hw_close(struct wm_hw_s *hw);

uint32_t wm_hw_now_ms(void);

/* Opens/closes internally on each call */
int  wm_hw_gpio_read(struct wm_hw_s *hw, int minor, bool *val);
int  wm_hw_gpio_write(struct wm_hw_s *hw, int minor, bool val);

/* Returns last_percent if ADC not available; sets *adc_ok accordingly */
uint32_t wm_hw_adc_read_percent_live(struct wm_hw_s *hw,
                                     uint8_t chan,
                                     uint32_t last_percent,
                                     bool *adc_ok);

#ifdef __cplusplus
}
#endif
