/****************************************************************************
 * examples/wmctrl/wmctrl_hw.c
 *
 * FD-safe hardware helpers (CONFIG_NFILE_DESCRIPTORS_PER_BLOCK=8 compatible):
 *  - NO persistent open() descriptors
 *  - Each GPIO/ADC operation opens the node, performs ioctl/read, then closes.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include <nuttx/ioexpander/gpio.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/clock.h>

#include "wmctrl_hw.h"

uint32_t wm_hw_now_ms(void)
{
  return (uint32_t)TICK2MSEC(clock_systime_ticks());
}

int wm_hw_init(struct wm_hw_s *hw, const char *adc_dev_preferred)
{
  if (!hw)
    {
      return -EINVAL;
    }

  memset(hw, 0, sizeof(*hw));
  hw->adc_dev_preferred = adc_dev_preferred;

  /* Do not open anything here; just accept configuration. */
  return 0;
}

void wm_hw_close(struct wm_hw_s *hw)
{
  (void)hw;
  /* Nothing to close; we keep no persistent FDs. */
}

static int gpio_open_minor(int minor)
{
  char path[16];
  snprintf(path, sizeof(path), "/dev/gpio%d", minor);

  int fd = open(path, O_RDWR);
  if (fd < 0)
    {
      return -errno;
    }

  return fd;
}

int wm_hw_gpio_read(struct wm_hw_s *hw, int minor, bool *val)
{
  (void)hw;

  if (!val)
    {
      return -EINVAL;
    }

  int fd = gpio_open_minor(minor);
  if (fd < 0)
    {
      /* Return negative errno */
      return fd;
    }

  bool v = false;
  int ret = ioctl(fd, GPIOC_READ, (unsigned long)((uintptr_t)&v));
  int err = (ret < 0) ? -errno : 0;

  close(fd);

  if (err < 0)
    {
      return err;
    }

  *val = v;
  return 0;
}

int wm_hw_gpio_write(struct wm_hw_s *hw, int minor, bool val)
{
  (void)hw;

  int fd = gpio_open_minor(minor);
  if (fd < 0)
    {
      return fd;
    }

  int ret = ioctl(fd, GPIOC_WRITE, (unsigned long)val);
  int err = (ret < 0) ? -errno : 0;

  close(fd);
  return err;
}

static int adc_try_open(const char *dev, bool nonblock)
{
  int flags = O_RDONLY;
  if (nonblock) flags |= O_NONBLOCK;

  int fd = open(dev, flags);
  if (fd < 0)
    {
      return -errno;
    }

  return fd;
}

static int adc_open_best(const char *preferred)
{
  /* Preferred first (if provided), then common fallbacks. */
  const char *candidates[3] = { preferred, "/dev/adc1", "/dev/adc0" };

  for (int i = 0; i < 3; i++)
    {
      const char *dev = candidates[i];
      if (!dev || dev[0] == '\0')
        {
          continue;
        }

      /* Try blocking open first (some drivers dislike NONBLOCK), then nonblock */
      int fd = adc_try_open(dev, false);
      if (fd >= 0) return fd;

      fd = adc_try_open(dev, true);
      if (fd >= 0) return fd;
    }

  return -ENOENT;
}

uint32_t wm_hw_adc_read_percent_live(struct wm_hw_s *hw,
                                     uint8_t chan,
                                     uint32_t last_percent,
                                     bool *adc_ok)
{
  if (adc_ok) *adc_ok = false;
  if (!hw)
    {
      return last_percent;
    }

  int fd = adc_open_best(hw->adc_dev_preferred);
  if (fd < 0)
    {
      /* ADC unavailable */
      return last_percent;
    }

  if (adc_ok) *adc_ok = true;

  /* Trigger sampling if supported (ignore error) */
  (void)ioctl(fd, ANIOC_TRIGGER, 0);

  struct adc_msg_s msgs[16];
  uint32_t newest = last_percent;
  bool got = false;

  while (1)
    {
      ssize_t nread = read(fd, msgs, sizeof(msgs));
      if (nread < 0)
        {
          if (errno == EAGAIN)
            {
              break;
            }
          /* Any other error: stop and keep last */
          break;
        }

      size_t nmsgs = (size_t)nread / sizeof(struct adc_msg_s);
      if (nmsgs == 0)
        {
          break;
        }

      for (size_t i = 0; i < nmsgs; i++)
        {
          if (msgs[i].am_channel == chan)
            {
              uint32_t raw = msgs[i].am_data;
              if (raw > 4095) raw = 4095;
              newest = (raw * 100UL) / 4095UL;
              got = true;
            }
        }

      if ((size_t)nread < sizeof(msgs))
        {
          break;
        }
    }

  close(fd);
  return got ? newest : last_percent;
}
