/****************************************************************************
 * examples/wmctrl/wmctrl_lcd.c
 *
 * HD44780 16x2, 4-bit mode over GPIO minors 8..13
 * Uses temporary open/close writes to keep FD usage <= 8.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <unistd.h>

#include <nuttx/arch.h>  /* up_mdelay/up_udelay */

#include "wmctrl_hw.h"
#include "wmctrl_lcd.h"

/* LCD pin minors (must match your board mapping) */
#define LCD_RS  WM_GPIO_LCD_RS
#define LCD_E   WM_GPIO_LCD_E
#define LCD_D4  WM_GPIO_LCD_D4
#define LCD_D5  WM_GPIO_LCD_D5
#define LCD_D6  WM_GPIO_LCD_D6
#define LCD_D7  WM_GPIO_LCD_D7

static void lcd_gpio(struct wm_hw_s *hw, int minor, bool v)
{
  (void)wm_hw_gpio_write(hw, minor, v);
}

static void lcd_pulse_e(struct wm_hw_s *hw)
{
  lcd_gpio(hw, LCD_E, 1);
  up_udelay(50);
  lcd_gpio(hw, LCD_E, 0);
  up_udelay(50);
}

static void lcd_write4(struct wm_hw_s *hw, uint8_t data4)
{
  lcd_gpio(hw, LCD_D4, (data4 >> 0) & 1);
  lcd_gpio(hw, LCD_D5, (data4 >> 1) & 1);
  lcd_gpio(hw, LCD_D6, (data4 >> 2) & 1);
  lcd_gpio(hw, LCD_D7, (data4 >> 3) & 1);
  lcd_pulse_e(hw);
}

static void lcd_cmd(struct wm_hw_s *hw, uint8_t cmd)
{
  lcd_gpio(hw, LCD_RS, 0);
  lcd_write4(hw, (uint8_t)(cmd >> 4));
  lcd_write4(hw, (uint8_t)(cmd & 0x0F));
  up_mdelay(2);
}

static void lcd_data(struct wm_hw_s *hw, uint8_t d)
{
  lcd_gpio(hw, LCD_RS, 1);
  lcd_write4(hw, (uint8_t)(d >> 4));
  lcd_write4(hw, (uint8_t)(d & 0x0F));
}

void wm_lcd_init(struct wm_hw_s *hw)
{
  up_mdelay(40);

  lcd_gpio(hw, LCD_RS, 0);

  lcd_write4(hw, 0x03); up_mdelay(5);
  lcd_write4(hw, 0x03); up_mdelay(5);
  lcd_write4(hw, 0x03); up_mdelay(5);
  lcd_write4(hw, 0x02); up_mdelay(5);

  lcd_cmd(hw, 0x28); /* 4-bit, 2 lines */
  lcd_cmd(hw, 0x08); /* display off */
  lcd_cmd(hw, 0x01); up_mdelay(2); /* clear */
  lcd_cmd(hw, 0x06); /* entry mode */
  lcd_cmd(hw, 0x0C); /* display on */
}

void wm_lcd_clear(struct wm_hw_s *hw)
{
  lcd_cmd(hw, 0x01);
  up_mdelay(2);
}

void wm_lcd_set_cursor(struct wm_hw_s *hw, uint8_t row, uint8_t col)
{
  uint8_t addr = (row == 0 ? 0x00 : 0x40) + col;
  lcd_cmd(hw, (uint8_t)(0x80 | addr));
}

void wm_lcd_print(struct wm_hw_s *hw, const char *s)
{
  while (s && *s)
    {
      lcd_data(hw, (uint8_t)*s++);
    }
}
