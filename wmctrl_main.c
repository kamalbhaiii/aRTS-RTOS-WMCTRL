/****************************************************************************
 * examples/wmctrl/wmctrl_main.c
 *
 * NuttX 12.6.0 - NUCLEO-G431RB washing machine controller (wmctrl)
 *
 * FD-safe version for CONFIG_NFILE_DESCRIPTORS_PER_BLOCK=8:
 *   - Keeps ONLY Relay + Buzzer persistent open (2 FDs)
 *   - All other GPIO and ADC operations are open->ioctl/read->close in wmctrl_hw.c
 *   - LCD is stable: no periodic clear; update only changed 16-char lines
 *   - ADC is REQUIRED to start; if ADC goes unavailable mid-cycle -> FAULT_WATER latch
 *   - Water level influences transitions (fill/drain targets) and can fault
 *   - Vibration faults are pulse-tolerant (accumulator) and resume when quiet stable
 *   - Motor driver uses L298 IN1/IN2 via GPIO minors 6/7 (ENA assumed tied HIGH)
 *   - Low-latency loop: sleeps until next deadline (reduces perceived lag)
 *
 * UPDATE (requested):
 *   - Removes current debug logs (keeps only init/critical errors)
 *   - Fixes live RPM display on 16x2 LCD by fitting within 16 chars:
 *       Line2: "Wxx D:C V:Y R240"  (exactly 16 chars)
 *   - Adds built-in Evaluation/Validation (no extra hardware):
 *       * task lateness stats (INPUT/FSM/ADC/LCD/BUZZ)
 *       * faults counters, adc_ok drops, water min/max
 *       * state time accounting + transition counts
 *       * vibration activity stats
 *       * on-demand dump: hold B1 ~1.2s in IDLE
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

#include "wmctrl_hw.h"
#include "wmctrl_lcd.h"

/* -------------------------- Configuration -------------------------- */

#define WM_ADC_DEV_PREF            "/dev/adc1"   /* probe adc1 then adc0 */
#define WM_WATER_ADC_CH            6

/* Task cadences */
#define INPUT_SCAN_MS              5
#define FSM_TICK_MS                10
#define BUZZER_TICK_MS             20
#define ADC_TICK_MS                120
#define LCD_UPDATE_MS              250

/* Debounce */
#define DOOR_DEBOUNCE_MS           25
#define MODE_DEBOUNCE_MS           20
#define B1_DEBOUNCE_MS             20
#define VIB_DEBOUNCE_MS            10

/* Button semantics for your NuttX mapping */
#define MODE_PRESSED_VALUE         0   /* /dev/gpio4: default 1, press 0 */
#define B1_PRESSED_VALUE           1   /* /dev/gpio5: default 0, press 1 */

/* Water thresholds */
#define WATER_START_MIN_PCT        80
#define WATER_WASH_TARGET_PCT      90
#define WATER_RINSE_TARGET_PCT     85
#define WATER_DRAIN_TARGET_PCT     20

#define MIN_FILL_MS                4000
#define MIN_DRAIN_MS               4000
#define MIN_RINSE_FILL_MS          3000

/* Vibration */
#define VIB_ACTIVE_VALUE           1
#define VIB_FAULT_MS               1200
#define VIB_CLEAR_STABLE_MS        250

/* Program UI */
#define MSG_SHOW_MS                900
#define DONE_SCREEN_MS             2500

/* Agitation */
#define AGITATE_ON_MS              600
#define AGITATE_OFF_MS             300

/* Outputs */
#define RELAY_ON_VALUE             1
#define RELAY_OFF_VALUE            0

#define BUZZER_ON_VALUE            0  /* active-low */
#define BUZZER_OFF_VALUE           1

/* Optional wiring pulse test (no logs) */
#define STARTUP_MOTOR_PULSE_TEST   0
#define STARTUP_MOTOR_PULSE_MS     250

/* Built-in evaluation / validation */
#define WM_EVAL_ENABLE             1
#define WM_EVAL_DUMP_HOLD_MS       1200  /* hold B1 this long in IDLE to dump report */

/* -------------------------- GPIO minors (board mapping) -------------------------- */

enum
{
  GPIO_DOOR      = WM_GPIO_DOOR,     /* 0 */
  GPIO_VIB       = WM_GPIO_VIB,      /* 1 */
  GPIO_RELAY     = WM_GPIO_RELAY,    /* 2 */
  GPIO_BUZZER    = WM_GPIO_BUZZER,   /* 3 */
  GPIO_MODE      = WM_GPIO_MODE_BTN, /* 4 */
  GPIO_B1        = WM_GPIO_B1_USER,  /* 5 */

  GPIO_L298_IN1  = WM_GPIO_L298_IN1, /* 6 */
  GPIO_L298_IN2  = WM_GPIO_L298_IN2  /* 7 */
};

/* -------------------------- Time helpers -------------------------- */

static uint32_t now_ms(void)
{
  return wm_hw_now_ms();
}

static inline bool time_due(uint32_t now, uint32_t deadline)
{
  return (int32_t)(now - deadline) >= 0;
}

static inline uint32_t time_min_u32(uint32_t a, uint32_t b)
{
  return (int32_t)(a - b) <= 0 ? a : b;
}

static inline uint32_t u32_absdiff(uint32_t a, uint32_t b)
{
  return (a >= b) ? (a - b) : (b - a);
}

/* -------------------------- Persistent outputs (2 FDs) -------------------------- */

struct wm_out_s
{
  int fd_relay;
  int fd_buzzer;
};

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

static int out_init(struct wm_out_s *o)
{
  if (!o) return -EINVAL;

  o->fd_relay  = -1;
  o->fd_buzzer = -1;

  o->fd_relay = gpio_open_minor(GPIO_RELAY);
  if (o->fd_relay < 0)
    {
      printf("wmctrl: open /dev/gpio%d failed: errno=%d\n", GPIO_RELAY, -o->fd_relay);
      return o->fd_relay;
    }

  o->fd_buzzer = gpio_open_minor(GPIO_BUZZER);
  if (o->fd_buzzer < 0)
    {
      printf("wmctrl: open /dev/gpio%d failed: errno=%d\n", GPIO_BUZZER, -o->fd_buzzer);
      close(o->fd_relay);
      o->fd_relay = -1;
      return o->fd_buzzer;
    }

  (void)ioctl(o->fd_relay,  GPIOC_WRITE, (unsigned long)RELAY_OFF_VALUE);
  (void)ioctl(o->fd_buzzer, GPIOC_WRITE, (unsigned long)BUZZER_OFF_VALUE);
  return 0;
}

static void out_deinit(struct wm_out_s *o)
{
  if (!o) return;
  if (o->fd_relay  >= 0) { close(o->fd_relay);  o->fd_relay  = -1; }
  if (o->fd_buzzer >= 0) { close(o->fd_buzzer); o->fd_buzzer = -1; }
}

static inline void Relay_Set(struct wm_out_s *o, bool on)
{
  (void)ioctl(o->fd_relay, GPIOC_WRITE, (unsigned long)(on ? RELAY_ON_VALUE : RELAY_OFF_VALUE));
}

static inline void Buzzer_Set(struct wm_out_s *o, bool on)
{
  (void)ioctl(o->fd_buzzer, GPIOC_WRITE, (unsigned long)(on ? BUZZER_ON_VALUE : BUZZER_OFF_VALUE));
}

/* -------------------------- Logic types -------------------------- */

typedef enum
{
  WM_IDLE = 0,
  WM_FILLING,
  WM_WASHING,
  WM_DRAINING_1,
  WM_RINSE_FILL,
  WM_RINSE_AGITATE,
  WM_DRAINING_2,
  WM_SPINNING,
  WM_DONE,
  WM_FAULT
} wm_state_t;

typedef enum
{
  MOTOR_MODE_OFF  = 0,
  MOTOR_MODE_WASH = 1,
  MOTOR_MODE_SPIN = 2
} motor_mode_t;

typedef enum
{
  BUZZER_PATTERN_NONE = 0,
  BUZZER_PATTERN_DOUBLE_BEEP,
  BUZZER_PATTERN_LONG_BEEP,
  BUZZER_PATTERN_ALARM,
  BUZZER_PATTERN_SHORT_BEEP
} buzzer_pattern_t;

typedef enum
{
  PROG_QUICK = 0,
  PROG_NORMAL,
  PROG_HEAVY,
  PROG_COUNT
} wm_program_t;

typedef struct
{
  const char *name;
  uint8_t rinseTotal;
  uint32_t fill_ms, wash_ms, drain1_ms;
  uint32_t rinseFill_ms, rinseAgitate_ms, drain2_ms;
  uint32_t spin_ms;
} program_ms_t;

typedef enum
{
  FAULT_NONE = 0,
  FAULT_DOOR_OPEN,
  FAULT_VIBRATION,
  FAULT_WATER,
  FAULT_USER_ABORT
} fault_reason_t;

static const program_ms_t g_programs[PROG_COUNT] =
{
  { "QCK", 1,  9000, 18000,  9000,  8000, 12000,  9000, 12000 },
  { "NRM", 2, 11000, 22000, 10000,  9000, 14000, 10000, 15000 },
  { "HVY", 3, 13000, 26000, 11000, 10000, 16000, 11000, 18000 }
};

static const char *state_short(wm_state_t s)
{
  switch (s)
    {
      case WM_IDLE:          return "IDLE";
      case WM_FILLING:       return "FILL";
      case WM_WASHING:       return "WASH";
      case WM_DRAINING_1:    return "DRN1";
      case WM_RINSE_FILL:    return "R-FL";
      case WM_RINSE_AGITATE: return "R-AG";
      case WM_DRAINING_2:    return "DRN2";
      case WM_SPINNING:      return "SPIN";
      case WM_DONE:          return "DONE";
      case WM_FAULT:         return "FAIL";
      default:               return "UNK ";
    }
}

static const char *fault_short(fault_reason_t r)
{
  switch (r)
    {
      case FAULT_DOOR_OPEN:  return "DOOR";
      case FAULT_VIBRATION:  return "VIB";
      case FAULT_WATER:      return "WATR";
      case FAULT_USER_ABORT: return "ABRT";
      case FAULT_NONE:
      default:               return "NONE";
    }
}

/* -------------------------- Debounce -------------------------- */

static bool DebounceLevel(bool raw, uint32_t now, uint32_t debounce_ms,
                          bool *stable, bool *last_raw, uint32_t *last_change_ms)
{
  if (raw != *last_raw)
    {
      *last_raw = raw;
      *last_change_ms = now;
    }

  if ((uint32_t)(now - *last_change_ms) >= debounce_ms)
    {
      *stable = *last_raw;
    }

  return *stable;
}

/* -------------------------- Inputs (open/close in wmctrl_hw.c) -------------------------- */

static bool DoorClosed_Raw(struct wm_hw_s *hw)
{
  bool v = false;
  (void)wm_hw_gpio_read(hw, GPIO_DOOR, &v);
  return (v == 0); /* LOW=closed */
}

static bool VibActive_Raw(struct wm_hw_s *hw)
{
  bool v = false;
  (void)wm_hw_gpio_read(hw, GPIO_VIB, &v);
  return (v == VIB_ACTIVE_VALUE);
}

static bool ModePressed_Raw(struct wm_hw_s *hw)
{
  bool v = false;
  (void)wm_hw_gpio_read(hw, GPIO_MODE, &v);
  return ((int)v == MODE_PRESSED_VALUE);
}

static bool B1Pressed_Raw(struct wm_hw_s *hw)
{
  bool v = false;
  (void)wm_hw_gpio_read(hw, GPIO_B1, &v);
  return ((int)v == B1_PRESSED_VALUE);
}

/* -------------------------- Motor (L298) -------------------------- */
/* Note: ENA is assumed tied HIGH externally. We only drive IN1/IN2.   */

static int L298_Write(struct wm_hw_s *hw, bool in1, bool in2)
{
  int r1 = wm_hw_gpio_write(hw, GPIO_L298_IN1, in1);
  int r2 = wm_hw_gpio_write(hw, GPIO_L298_IN2, in2);

  if (r1 < 0) return r1;
  if (r2 < 0) return r2;
  return 0;
}

static int L298_Stop(struct wm_hw_s *hw)
{
  return L298_Write(hw, false, false);
}

static int L298_SetDir(struct wm_hw_s *hw, bool forward)
{
  if (forward) return L298_Write(hw, true, false);
  return L298_Write(hw, false, true);
}

static int Motor_ApplyOutputs(struct wm_hw_s *hw, motor_mode_t mode, uint32_t now)
{
  if (mode == MOTOR_MODE_OFF)
    {
      return L298_Stop(hw);
    }

  bool forward = true;

  if (mode == MOTOR_MODE_WASH)
    {
      /* reverse every ~2s */
      forward = ((now / 2000U) % 2U) == 0U;
    }

  return L298_SetDir(hw, forward);
}

/* -------------------------- Remaining time -------------------------- */

static uint32_t state_planned_ms(wm_state_t st, const program_ms_t *p)
{
  switch (st)
    {
      case WM_FILLING:       return p->fill_ms;
      case WM_WASHING:       return p->wash_ms;
      case WM_DRAINING_1:    return p->drain1_ms;
      case WM_RINSE_FILL:    return p->rinseFill_ms;
      case WM_RINSE_AGITATE: return p->rinseAgitate_ms;
      case WM_DRAINING_2:    return p->drain2_ms;
      case WM_SPINNING:      return p->spin_ms;
      case WM_DONE:          return DONE_SCREEN_MS;
      default:               return 0;
    }
}

static uint32_t total_remaining_ms(wm_state_t st, uint32_t inState_ms,
                                  const program_ms_t *p, uint8_t rinseIndex)
{
#define ADD_ONE_RINSE() do { rem += p->rinseFill_ms; rem += p->rinseAgitate_ms; rem += p->drain2_ms; } while(0)

  uint32_t rem = 0;

  if (st == WM_DONE)  return (inState_ms >= DONE_SCREEN_MS) ? 0 : (DONE_SCREEN_MS - inState_ms);
  if (st == WM_FAULT) return 0;
  if (st == WM_IDLE)  return 0;

  uint32_t planned = state_planned_ms(st, p);
  uint32_t cur_rem = (inState_ms >= planned) ? 0 : (planned - inState_ms);
  rem += cur_rem;

  switch (st)
    {
      case WM_FILLING:
        rem += p->wash_ms + p->drain1_ms;
        for (uint8_t i = 0; i < p->rinseTotal; i++) ADD_ONE_RINSE();
        rem += p->spin_ms;
        break;

      case WM_WASHING:
        rem += p->drain1_ms;
        for (uint8_t i = 0; i < p->rinseTotal; i++) ADD_ONE_RINSE();
        rem += p->spin_ms;
        break;

      case WM_DRAINING_1:
        for (uint8_t i = 0; i < p->rinseTotal; i++) ADD_ONE_RINSE();
        rem += p->spin_ms;
        break;

      case WM_RINSE_FILL:
      case WM_RINSE_AGITATE:
      case WM_DRAINING_2:
        {
          uint8_t total = p->rinseTotal;
          uint8_t done  = rinseIndex;
          if (done > total) done = total;

          uint8_t remaining_including_current = (total > done) ? (uint8_t)(total - done) : 0;

          if (remaining_including_current > 0)
            {
              if (st == WM_RINSE_FILL)         rem += p->rinseAgitate_ms + p->drain2_ms;
              else if (st == WM_RINSE_AGITATE) rem += p->drain2_ms;

              for (uint8_t i = 0; i < (uint8_t)(remaining_including_current - 1); i++) ADD_ONE_RINSE();
            }

          rem += p->spin_ms;
        }
        break;

      default:
        break;
    }

#undef ADD_ONE_RINSE
  return rem;
}

/* -------------------------- LCD stable helpers -------------------------- */

static void make_line16(char out[17], const char *src)
{
  size_t n = (src ? strlen(src) : 0);
  if (n > 16) n = 16;
  memset(out, ' ', 16);
  if (n > 0) memcpy(out, src, n);
  out[16] = '\0';
}

static void lcd_write_line16(struct wm_hw_s *hw, uint8_t row, const char line16[17])
{
  wm_lcd_set_cursor(hw, row, 0);
  wm_lcd_print(hw, line16);
}

/* -------------------------- Buzzer patterns -------------------------- */

static buzzer_pattern_t s_buzzerPattern = BUZZER_PATTERN_NONE;
static uint32_t         s_buzzerTick    = 0;

static void Buzzer_StartPattern(struct wm_out_s *o, buzzer_pattern_t pattern)
{
  s_buzzerPattern = pattern;
  s_buzzerTick    = 0;
  Buzzer_Set(o, false);
}

static void Buzzer_Update(struct wm_out_s *o)
{
  bool on = false;

  switch (s_buzzerPattern)
    {
      case BUZZER_PATTERN_NONE:
        on = false;
        break;

      case BUZZER_PATTERN_DOUBLE_BEEP:
        on = (s_buzzerTick == 0 || s_buzzerTick == 2);
        if (s_buzzerTick >= 4) s_buzzerPattern = BUZZER_PATTERN_NONE;
        break;

      case BUZZER_PATTERN_LONG_BEEP:
        on = (s_buzzerTick < 8);
        if (s_buzzerTick >= 8) s_buzzerPattern = BUZZER_PATTERN_NONE;
        break;

      case BUZZER_PATTERN_ALARM:
        on = ((s_buzzerTick % 2U) == 0U);
        if (s_buzzerTick >= 14) s_buzzerPattern = BUZZER_PATTERN_NONE;
        break;

      case BUZZER_PATTERN_SHORT_BEEP:
        on = (s_buzzerTick == 0);
        if (s_buzzerTick >= 1) s_buzzerPattern = BUZZER_PATTERN_NONE;
        break;
    }

  Buzzer_Set(o, on);
  if (s_buzzerPattern != BUZZER_PATTERN_NONE) s_buzzerTick++;
}

/* -------------------------- Fault latch helper -------------------------- */

static void latch_fault(struct wm_out_s *out,
                        bool *faultLatched,
                        fault_reason_t *faultReason,
                        wm_state_t *state,
                        fault_reason_t reason,
                        uint32_t now,
                        uint32_t stateStart_ms,
                        uint8_t rinseIndex,
                        wm_state_t *savedState,
                        uint32_t *savedInState_ms,
                        uint8_t *savedRinseIndex,
                        char msg1[64],
                        char msg2[64],
                        bool *showMsg,
                        uint32_t *msgStart_ms)
{
  *savedState      = *state;
  *savedInState_ms = (now - stateStart_ms);
  *savedRinseIndex = rinseIndex;

  *faultLatched = true;
  *faultReason  = reason;
  *state        = WM_FAULT;

  Relay_Set(out, false);
  Buzzer_StartPattern(out, BUZZER_PATTERN_ALARM);

  switch (reason)
    {
      case FAULT_DOOR_OPEN:
        snprintf(msg1, 64, "DOOR OPEN!");
        snprintf(msg2, 64, "Close to resume");
        break;
      case FAULT_VIBRATION:
        snprintf(msg1, 64, "VIBRATION!");
        snprintf(msg2, 64, "Wait to resume");
        break;
      case FAULT_WATER:
        snprintf(msg1, 64, "WATER/ADC!");
        snprintf(msg2, 64, "Check sensor");
        break;
      case FAULT_USER_ABORT:
        snprintf(msg1, 64, "ABORTED");
        snprintf(msg2, 64, "Back to IDLE");
        break;
      default:
        snprintf(msg1, 64, "FAULT");
        snprintf(msg2, 64, "Resume...");
        break;
    }

  *showMsg = true;
  *msgStart_ms = now;
}

/* -------------------------- EVALUATION / VALIDATION -------------------------- */
#if WM_EVAL_ENABLE

typedef struct
{
  uint32_t runs;
  uint32_t max_late_ms;
  uint32_t sum_late_ms;
  uint32_t late_over_1ms;
  uint32_t late_over_5ms;
  uint32_t late_over_10ms;
} eval_task_t;

typedef struct
{
  uint32_t boot_ms;

  eval_task_t t_input;
  eval_task_t t_fsm;
  eval_task_t t_adc;
  eval_task_t t_lcd;
  eval_task_t t_buzz;

  uint32_t adc_ok_drops;
  uint32_t water_min;
  uint32_t water_max;

  uint32_t vib_active_ms;
  uint32_t vib_acc_peak_ms;

  uint32_t fault_door_cnt;
  uint32_t fault_vib_cnt;
  uint32_t fault_water_cnt;
  uint32_t fault_abort_cnt;

  uint32_t cycle_starts;
  uint32_t cycle_completes;
  uint32_t pauses;
  uint32_t resumes;

  uint32_t state_enter_cnt[WM_FAULT + 1];
  uint32_t state_time_ms[WM_FAULT + 1];
  wm_state_t last_state;
  uint32_t   last_state_enter_ms;

  uint32_t eval_dump_cnt;
} eval_s;

static void eval_task_update(eval_task_t *t, uint32_t late_ms)
{
  if (!t) return;
  t->runs++;
  t->sum_late_ms += late_ms;
  if (late_ms > t->max_late_ms) t->max_late_ms = late_ms;
  if (late_ms > 1)  t->late_over_1ms++;
  if (late_ms > 5)  t->late_over_5ms++;
  if (late_ms > 10) t->late_over_10ms++;
}

static void eval_state_enter(eval_s *e, wm_state_t st, uint32_t now)
{
  if (!e) return;

  if (e->last_state_enter_ms != 0 && e->last_state <= WM_FAULT)
    {
      e->state_time_ms[e->last_state] += (now - e->last_state_enter_ms);
    }

  e->last_state = st;
  e->last_state_enter_ms = now;
  if (st <= WM_FAULT) e->state_enter_cnt[st]++;
}

static void eval_dump(const eval_s *e, const program_ms_t *prog, uint32_t now,
                      wm_state_t state, bool paused, bool faultLatched,
                      fault_reason_t faultReason, bool doorClosed, bool vibActive,
                      bool adc_ok, uint32_t waterPct, int target_rpm, int motor_rpm)
{
  if (!e) return;

  uint32_t up = now - e->boot_ms;

  printf("\n=== WMCTRL VALIDATION DUMP #%lu ===\n", (unsigned long)e->eval_dump_cnt);
  printf("Uptime: %lu ms  (%.1f s)\n", (unsigned long)up, (double)up / 1000.0);
  printf("State: %s  paused=%d  faultLatched=%d(%s)  door=%c  vib=%d\n",
         state_short(state),
         paused ? 1 : 0,
         faultLatched ? 1 : 0,
         fault_short(faultReason),
         doorClosed ? 'C' : 'O',
         vibActive ? 1 : 0);
  printf("ADC: ok=%d  water=%lu%%  min=%lu%% max=%lu%%  ok_drops=%lu\n",
         adc_ok ? 1 : 0,
         (unsigned long)waterPct,
         (unsigned long)e->water_min,
         (unsigned long)e->water_max,
         (unsigned long)e->adc_ok_drops);
  printf("Motor(UI): target=%d rpm=%d  program=%s\n", target_rpm, motor_rpm, prog ? prog->name : "???");

  printf("\nTask lateness (ms): runs / max / avg / >1ms / >5ms / >10ms\n");
#define P_TASK(lbl, t) do { \
  uint32_t avg = (t.runs ? (t.sum_late_ms / t.runs) : 0); \
  printf("  %-5s: %6lu / %3lu / %3lu / %6lu / %6lu / %6lu\n", \
         lbl, (unsigned long)t.runs, (unsigned long)t.max_late_ms, (unsigned long)avg, \
         (unsigned long)t.late_over_1ms, (unsigned long)t.late_over_5ms, (unsigned long)t.late_over_10ms); \
} while(0)
  P_TASK("INPUT", e->t_input);
  P_TASK("FSM",   e->t_fsm);
  P_TASK("ADC",   e->t_adc);
  P_TASK("LCD",   e->t_lcd);
  P_TASK("BUZZ",  e->t_buzz);
#undef P_TASK

  printf("\nCounters:\n");
  printf("  cycle_starts=%lu completes=%lu pauses=%lu resumes=%lu\n",
         (unsigned long)e->cycle_starts, (unsigned long)e->cycle_completes,
         (unsigned long)e->pauses, (unsigned long)e->resumes);
  printf("  faults: door=%lu vib=%lu water=%lu abort=%lu\n",
         (unsigned long)e->fault_door_cnt, (unsigned long)e->fault_vib_cnt,
         (unsigned long)e->fault_water_cnt, (unsigned long)e->fault_abort_cnt);

  printf("\nState time (ms) and entries:\n");
  for (int i = 0; i <= (int)WM_FAULT; i++)
    {
      printf("  %-4s: time=%8lu  enters=%6lu\n",
             state_short((wm_state_t)i),
             (unsigned long)e->state_time_ms[i],
             (unsigned long)e->state_enter_cnt[i]);
    }

  printf("\nVibration stats:\n");
  printf("  vib_active_ms=%lu  vib_acc_peak_ms=%lu\n",
         (unsigned long)e->vib_active_ms,
         (unsigned long)e->vib_acc_peak_ms);

  printf("=== END DUMP ===\n\n");
}

#endif /* WM_EVAL_ENABLE */

/* -------------------------- MAIN -------------------------- */

int wmctrl_main(int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  struct wm_hw_s hw;
  (void)wm_hw_init(&hw, WM_ADC_DEV_PREF);

  struct wm_out_s out;
  int ret = out_init(&out);
  if (ret < 0)
    {
      printf("wmctrl: outputs init failed: %d\n", ret);
      return 1;
    }

  Relay_Set(&out, false);
  Buzzer_Set(&out, false);

  /* Ensure motor is off */
  (void)L298_Stop(&hw);

  wm_lcd_init(&hw);

#if STARTUP_MOTOR_PULSE_TEST
  (void)L298_Write(&hw, true, false);
  usleep(STARTUP_MOTOR_PULSE_MS * 1000U);
  (void)L298_Write(&hw, false, true);
  usleep(STARTUP_MOTOR_PULSE_MS * 1000U);
  (void)L298_Stop(&hw);
#endif

  /* LCD stable buffers */
  char last0[17]; memset(last0, 0, sizeof(last0));
  char last1[17]; memset(last1, 0, sizeof(last1));

  {
    char l0[17], l1[17];
    make_line16(l0, "Boot OK");
    make_line16(l1, "wmctrl");
    lcd_write_line16(&hw, 0, l0);
    lcd_write_line16(&hw, 1, l1);
    memcpy(last0, l0, 17);
    memcpy(last1, l1, 17);
  }

  /* State */
  wm_state_t state = WM_IDLE;

  wm_program_t selected = PROG_NORMAL;
  const program_ms_t *prog = &g_programs[selected];

  bool cycleRunning = false;
  bool paused = false;

  uint32_t now = now_ms();

  /* Debounce storage */
  bool doorStable=false, doorLastRaw=false; uint32_t doorLastChange=now;
  bool modeStable=false, modeLastRaw=false; uint32_t modeLastChange=now;
  bool b1Stable=false,   b1LastRaw=false;   uint32_t b1LastChange=now;
  bool vibStable=false,  vibLastRaw=false;  uint32_t vibLastChange=now;

  bool doorClosed = DebounceLevel(DoorClosed_Raw(&hw), now, DOOR_DEBOUNCE_MS, &doorStable, &doorLastRaw, &doorLastChange);
  bool lastDoorClosed = doorClosed;

  bool modePressed = DebounceLevel(ModePressed_Raw(&hw), now, MODE_DEBOUNCE_MS, &modeStable, &modeLastRaw, &modeLastChange);
  bool lastModePressed = modePressed;

  bool b1Pressed = DebounceLevel(B1Pressed_Raw(&hw), now, B1_DEBOUNCE_MS, &b1Stable, &b1LastRaw, &b1LastChange);
  bool lastB1Pressed = b1Pressed;

  bool vibActive = DebounceLevel(VibActive_Raw(&hw), now, VIB_DEBOUNCE_MS, &vibStable, &vibLastRaw, &vibLastChange);

  uint32_t stateStart_ms = now;
  uint32_t inState_ms = 0;
  uint8_t  rinseIndex = 0;

  /* Fault latch and resume */
  bool faultLatched = false;
  fault_reason_t faultReason = FAULT_NONE;

  wm_state_t savedState = WM_IDLE;
  uint32_t   savedInState_ms = 0;
  uint8_t    savedRinseIndex = 0;

  uint32_t vibClearStart_ms = 0;

  /* Pulse-tolerant vibration accumulator */
  uint32_t vib_acc_ms = 0;

  motor_mode_t motor_mode = MOTOR_MODE_OFF;
  int motor_rpm = 0;
  int target_rpm = 0;
  uint32_t lastMotorUpdate_ms = now;

  /* UI messages */
  bool showMsg = false;
  uint32_t msgStart_ms = 0;
  char msg1[64] = {0}, msg2[64] = {0};

  /* ADC */
  uint32_t levelPercent = 0;
  bool adc_ok = false;
  bool last_adc_ok = false;

  /* B1 long-press abort */
  uint32_t b1Down_ms = 0;

#if WM_EVAL_ENABLE
  eval_s eval;
  memset(&eval, 0, sizeof(eval));
  eval.boot_ms = now;
  eval.water_min = 100;
  eval.water_max = 0;
  eval.last_state = state;
  eval.last_state_enter_ms = now;
  eval.state_enter_cnt[state]++;
#endif

  /* Schedulers */
  uint32_t next_input = now;
  uint32_t next_fsm   = now;
  uint32_t next_adc   = now;
  uint32_t next_lcd   = now;
  uint32_t next_buzz  = now;

  Buzzer_StartPattern(&out, BUZZER_PATTERN_NONE);

  while (1)
    {
      now = now_ms();

      /* ADC tick */
      if (time_due(now, next_adc))
        {
          uint32_t late = (now >= next_adc) ? (now - next_adc) : 0;

#if WM_EVAL_ENABLE
          eval_task_update(&eval.t_adc, late);
#endif
          next_adc += ADC_TICK_MS;

          levelPercent = wm_hw_adc_read_percent_live(&hw, WM_WATER_ADC_CH, levelPercent, &adc_ok);

#if WM_EVAL_ENABLE
          if (last_adc_ok && !adc_ok) eval.adc_ok_drops++;
          last_adc_ok = adc_ok;

          if (adc_ok)
            {
              if (levelPercent < eval.water_min) eval.water_min = levelPercent;
              if (levelPercent > eval.water_max) eval.water_max = levelPercent;
            }
#endif

          bool inActiveCycle = (state != WM_IDLE && state != WM_DONE && state != WM_FAULT);
          if (!faultLatched && inActiveCycle && !paused)
            {
              if (!adc_ok)
                {
#if WM_EVAL_ENABLE
                  eval.fault_water_cnt++;
#endif
                  latch_fault(&out,
                              &faultLatched, &faultReason, &state,
                              FAULT_WATER,
                              now, stateStart_ms, rinseIndex,
                              &savedState, &savedInState_ms, &savedRinseIndex,
                              msg1, msg2, &showMsg, &msgStart_ms);
                  motor_mode = MOTOR_MODE_OFF;
                  target_rpm = 0;

#if WM_EVAL_ENABLE
                  eval_state_enter(&eval, state, now);
#endif
                }
            }
        }

      /* Input scan tick */
      if (time_due(now, next_input))
        {
          uint32_t late = (now >= next_input) ? (now - next_input) : 0;

#if WM_EVAL_ENABLE
          eval_task_update(&eval.t_input, late);
#endif
          next_input += INPUT_SCAN_MS;

          doorClosed  = DebounceLevel(DoorClosed_Raw(&hw),  now, DOOR_DEBOUNCE_MS, &doorStable, &doorLastRaw, &doorLastChange);
          modePressed = DebounceLevel(ModePressed_Raw(&hw), now, MODE_DEBOUNCE_MS, &modeStable, &modeLastRaw, &modeLastChange);
          b1Pressed   = DebounceLevel(B1Pressed_Raw(&hw),   now, B1_DEBOUNCE_MS,   &b1Stable,   &b1LastRaw,   &b1LastChange);
          vibActive   = DebounceLevel(VibActive_Raw(&hw),   now, VIB_DEBOUNCE_MS,  &vibStable,  &vibLastRaw,  &vibLastChange);

          bool modeEdge   = (modePressed && !lastModePressed);
          lastModePressed = modePressed;

          bool b1DownEdge = (b1Pressed && !lastB1Pressed);
          bool b1UpEdge   = (!b1Pressed && lastB1Pressed);
          lastB1Pressed   = b1Pressed;

          if (b1DownEdge) b1Down_ms = now;

          /* MODE cycle (idle only) */
          if (modeEdge && state == WM_IDLE && !cycleRunning && !paused && !faultLatched)
            {
              selected = (wm_program_t)((selected + 1) % PROG_COUNT);
              prog = &g_programs[selected];

              snprintf(msg1, sizeof(msg1), "MODE: %s", prog->name);
              snprintf(msg2, sizeof(msg2), "B1 start");
              showMsg = true;
              msgStart_ms = now;
              Buzzer_StartPattern(&out, BUZZER_PATTERN_SHORT_BEEP);
            }

          /* Door opens during active cycle -> latch fault and resume on close */
          if (doorClosed != lastDoorClosed)
            {
              if (!doorClosed && (cycleRunning || paused) && !faultLatched)
                {
#if WM_EVAL_ENABLE
                  eval.fault_door_cnt++;
#endif
                  latch_fault(&out,
                              &faultLatched, &faultReason, &state,
                              FAULT_DOOR_OPEN,
                              now, stateStart_ms, rinseIndex,
                              &savedState, &savedInState_ms, &savedRinseIndex,
                              msg1, msg2, &showMsg, &msgStart_ms);
                  motor_mode = MOTOR_MODE_OFF;
                  target_rpm = 0;

#if WM_EVAL_ENABLE
                  eval_state_enter(&eval, state, now);
#endif
                }

              lastDoorClosed = doorClosed;
            }

          /* B1 action on release */
          if (b1UpEdge)
            {
              uint32_t held = now - b1Down_ms;
              bool isLong = (held >= 350U);

              /* Validation dump trigger: hold B1 long in IDLE (and not starting a cycle) */
#if WM_EVAL_ENABLE
              if (state == WM_IDLE && !cycleRunning && !paused && !faultLatched && held >= WM_EVAL_DUMP_HOLD_MS)
                {
                  eval.eval_dump_cnt++;
                  /* small UI hint */
                  snprintf(msg1, sizeof(msg1), "EVAL DUMP");
                  snprintf(msg2, sizeof(msg2), "See console");
                  showMsg = true; msgStart_ms = now;

                  eval_dump(&eval, prog, now, state, paused, faultLatched, faultReason,
                            doorClosed, vibActive, adc_ok, levelPercent, target_rpm, motor_rpm);
                  continue;
                }
#endif

              if (!cycleRunning && !paused && !faultLatched && state == WM_IDLE)
                {
                  if (!doorClosed)
                    {
                      snprintf(msg1, sizeof(msg1), "Close Door");
                      snprintf(msg2, sizeof(msg2), "to Start");
                      showMsg = true; msgStart_ms = now;
                      Buzzer_StartPattern(&out, BUZZER_PATTERN_DOUBLE_BEEP);
                    }
                  else if (!adc_ok)
                    {
                      snprintf(msg1, sizeof(msg1), "ADC ERROR");
                      snprintf(msg2, sizeof(msg2), "Check /dev/adc");
                      showMsg = true; msgStart_ms = now;
                      Buzzer_StartPattern(&out, BUZZER_PATTERN_ALARM);
                    }
                  else if (levelPercent < WATER_START_MIN_PCT)
                    {
                      snprintf(msg1, sizeof(msg1), "Water too low");
                      snprintf(msg2, sizeof(msg2), "Need >=80%%");
                      showMsg = true; msgStart_ms = now;
                      Buzzer_StartPattern(&out, BUZZER_PATTERN_ALARM);
                    }
                  else
                    {
                      cycleRunning = true;
                      paused = false;
                      faultLatched = false;
                      faultReason = FAULT_NONE;
                      rinseIndex = 0;
                      vib_acc_ms = 0;
                      vibClearStart_ms = 0;

                      state = WM_FILLING;
                      stateStart_ms = now;

#if WM_EVAL_ENABLE
                      eval.cycle_starts++;
                      eval_state_enter(&eval, state, now);
#endif

                      Buzzer_StartPattern(&out, BUZZER_PATTERN_DOUBLE_BEEP);
                    }
                }
              else
                {
                  if (!faultLatched && !isLong)
                    {
                      paused = !paused;

#if WM_EVAL_ENABLE
                      if (paused) eval.pauses++; else eval.resumes++;
#endif

                      if (paused)
                        {
                          Relay_Set(&out, false);
                          motor_mode = MOTOR_MODE_OFF;
                          target_rpm = 0;
                          snprintf(msg1, sizeof(msg1), "PAUSED");
                          snprintf(msg2, sizeof(msg2), "B1 resume");
                        }
                      else
                        {
                          snprintf(msg1, sizeof(msg1), "RESUMED");
                          snprintf(msg2, sizeof(msg2), "Running...");
                        }

                      showMsg = true; msgStart_ms = now;
                      Buzzer_StartPattern(&out, BUZZER_PATTERN_SHORT_BEEP);
                    }
                  else if (!faultLatched && isLong)
                    {
#if WM_EVAL_ENABLE
                      eval.fault_abort_cnt++;
#endif
                      latch_fault(&out,
                                  &faultLatched, &faultReason, &state,
                                  FAULT_USER_ABORT,
                                  now, stateStart_ms, rinseIndex,
                                  &savedState, &savedInState_ms, &savedRinseIndex,
                                  msg1, msg2, &showMsg, &msgStart_ms);

                      cycleRunning = false;
                      paused = false;
                      motor_mode = MOTOR_MODE_OFF;
                      target_rpm = 0;

#if WM_EVAL_ENABLE
                      eval_state_enter(&eval, state, now);
#endif
                    }
                }
            }
        }

      /* Buzzer tick */
      if (time_due(now, next_buzz))
        {
          uint32_t late = (now >= next_buzz) ? (now - next_buzz) : 0;

#if WM_EVAL_ENABLE
          eval_task_update(&eval.t_buzz, late);
#endif
          next_buzz += BUZZER_TICK_MS;
          Buzzer_Update(&out);
        }

      /* FSM tick */
      if (time_due(now, next_fsm))
        {
          uint32_t late = (now >= next_fsm) ? (now - next_fsm) : 0;

#if WM_EVAL_ENABLE
          eval_task_update(&eval.t_fsm, late);
#endif
          next_fsm += FSM_TICK_MS;
          inState_ms = now - stateStart_ms;

          bool inActiveCycle = (state != WM_IDLE && state != WM_DONE && state != WM_FAULT);

          /* Vibration accumulation (pulse tolerant) */
          if (!faultLatched && (cycleRunning || inActiveCycle) && !paused)
            {
#if WM_EVAL_ENABLE
              if (vibActive) eval.vib_active_ms += FSM_TICK_MS;
#endif
              if (vibActive)
                {
                  if (vib_acc_ms < VIB_FAULT_MS + 5000U) vib_acc_ms += FSM_TICK_MS;
                }
              else
                {
                  if (vib_acc_ms >= FSM_TICK_MS) vib_acc_ms -= FSM_TICK_MS;
                  else vib_acc_ms = 0;
                }

#if WM_EVAL_ENABLE
              if (vib_acc_ms > eval.vib_acc_peak_ms) eval.vib_acc_peak_ms = vib_acc_ms;
#endif

              if (vib_acc_ms >= VIB_FAULT_MS)
                {
#if WM_EVAL_ENABLE
                  eval.fault_vib_cnt++;
#endif
                  latch_fault(&out,
                              &faultLatched, &faultReason, &state,
                              FAULT_VIBRATION,
                              now, stateStart_ms, rinseIndex,
                              &savedState, &savedInState_ms, &savedRinseIndex,
                              msg1, msg2, &showMsg, &msgStart_ms);

                  motor_mode = MOTOR_MODE_OFF;
                  target_rpm = 0;

#if WM_EVAL_ENABLE
                  eval_state_enter(&eval, state, now);
#endif
                }
            }
          else
            {
              vib_acc_ms = 0;
            }

          /* Fault-latched behavior */
          if (faultLatched)
            {
              Relay_Set(&out, false);
              motor_mode = MOTOR_MODE_OFF;
              target_rpm = 0;

              bool canResume = true;

              if (faultReason == FAULT_DOOR_OPEN)
                {
                  if (!doorClosed) canResume = false;
                }
              else if (faultReason == FAULT_VIBRATION)
                {
                  if (vibActive)
                    {
                      vibClearStart_ms = 0;
                      canResume = false;
                    }
                  else
                    {
                      if (vibClearStart_ms == 0) vibClearStart_ms = now;
                      if ((uint32_t)(now - vibClearStart_ms) < VIB_CLEAR_STABLE_MS) canResume = false;
                    }
                }
              else if (faultReason == FAULT_WATER)
                {
                  if (!adc_ok) canResume = false;
                }
              else if (faultReason == FAULT_USER_ABORT)
                {
                  state = WM_IDLE;
                  faultLatched = false;
                  faultReason = FAULT_NONE;
                  cycleRunning = false;
                  paused = false;
                  stateStart_ms = now;
                  vib_acc_ms = 0;

#if WM_EVAL_ENABLE
                  eval_state_enter(&eval, state, now);
#endif
                  continue;
                }

              if (canResume)
                {
                  state = savedState;
                  rinseIndex = savedRinseIndex;
                  stateStart_ms = now - savedInState_ms;

                  faultLatched = false;
                  faultReason = FAULT_NONE;
                  vib_acc_ms = 0;
                  vibClearStart_ms = 0;

                  paused = false;
                  Buzzer_StartPattern(&out, BUZZER_PATTERN_DOUBLE_BEEP);

                  snprintf(msg1, sizeof(msg1), "RESUMED");
                  snprintf(msg2, sizeof(msg2), "Continuing...");
                  showMsg = true;
                  msgStart_ms = now;

#if WM_EVAL_ENABLE
                  eval_state_enter(&eval, state, now);
#endif
                }
            }

          /* Pause safety */
          if (paused && !faultLatched)
            {
              Relay_Set(&out, false);
              motor_mode = MOTOR_MODE_OFF;
              target_rpm = 0;
            }

          /* Program transitions */
          if (!paused && !faultLatched)
            {
              wm_state_t prev = state;

              switch (state)
                {
                  case WM_IDLE:
                    cycleRunning = false;
                    motor_mode = MOTOR_MODE_OFF;
                    target_rpm = 0;
                    Relay_Set(&out, false);
                    rinseIndex = 0;
                    break;

                  case WM_FILLING:
                    motor_mode = MOTOR_MODE_OFF;
                    target_rpm = 0;
                    Relay_Set(&out, false);

                    if (inState_ms >= MIN_FILL_MS)
                      {
                        if (!adc_ok)
                          {
#if WM_EVAL_ENABLE
                            eval.fault_water_cnt++;
#endif
                            latch_fault(&out, &faultLatched, &faultReason, &state,
                                        FAULT_WATER, now, stateStart_ms, rinseIndex,
                                        &savedState, &savedInState_ms, &savedRinseIndex,
                                        msg1, msg2, &showMsg, &msgStart_ms);
                            break;
                          }

                        if (levelPercent >= WATER_WASH_TARGET_PCT || inState_ms >= prog->fill_ms)
                          {
                            state = WM_WASHING;
                            stateStart_ms = now;
                          }
                      }
                    break;

                  case WM_WASHING:
                    motor_mode = MOTOR_MODE_WASH;
                    Relay_Set(&out, true);

                    {
                      uint32_t phase = (inState_ms % (AGITATE_ON_MS + AGITATE_OFF_MS));
                      target_rpm = (phase < AGITATE_ON_MS) ? 240 : 0;
                    }

                    if (inState_ms >= prog->wash_ms)
                      {
                        Relay_Set(&out, false);
                        state = WM_DRAINING_1;
                        stateStart_ms = now;
                      }
                    break;

                  case WM_DRAINING_1:
                    Relay_Set(&out, false);
                    motor_mode = MOTOR_MODE_WASH;
                    target_rpm = 170;

                    if (inState_ms >= MIN_DRAIN_MS)
                      {
                        if (!adc_ok)
                          {
#if WM_EVAL_ENABLE
                            eval.fault_water_cnt++;
#endif
                            latch_fault(&out, &faultLatched, &faultReason, &state,
                                        FAULT_WATER, now, stateStart_ms, rinseIndex,
                                        &savedState, &savedInState_ms, &savedRinseIndex,
                                        msg1, msg2, &showMsg, &msgStart_ms);
                            break;
                          }

                        if (levelPercent <= WATER_DRAIN_TARGET_PCT || inState_ms >= prog->drain1_ms)
                          {
                            state = WM_RINSE_FILL;
                            stateStart_ms = now;
                          }
                      }
                    break;

                  case WM_RINSE_FILL:
                    Relay_Set(&out, false);
                    motor_mode = MOTOR_MODE_OFF;
                    target_rpm = 0;

                    if (inState_ms >= MIN_RINSE_FILL_MS)
                      {
                        if (!adc_ok)
                          {
#if WM_EVAL_ENABLE
                            eval.fault_water_cnt++;
#endif
                            latch_fault(&out, &faultLatched, &faultReason, &state,
                                        FAULT_WATER, now, stateStart_ms, rinseIndex,
                                        &savedState, &savedInState_ms, &savedRinseIndex,
                                        msg1, msg2, &showMsg, &msgStart_ms);
                            break;
                          }

                        if (levelPercent >= WATER_RINSE_TARGET_PCT || inState_ms >= prog->rinseFill_ms)
                          {
                            state = WM_RINSE_AGITATE;
                            stateStart_ms = now;
                          }
                      }
                    break;

                  case WM_RINSE_AGITATE:
                    Relay_Set(&out, false);
                    motor_mode = MOTOR_MODE_WASH;

                    {
                      uint32_t phase = (inState_ms % (AGITATE_ON_MS + AGITATE_OFF_MS));
                      target_rpm = (phase < AGITATE_ON_MS) ? 220 : 0;
                    }

                    if (inState_ms >= prog->rinseAgitate_ms)
                      {
                        state = WM_DRAINING_2;
                        stateStart_ms = now;
                      }
                    break;

                  case WM_DRAINING_2:
                    Relay_Set(&out, false);
                    motor_mode = MOTOR_MODE_WASH;
                    target_rpm = 170;

                    if (inState_ms >= MIN_DRAIN_MS)
                      {
                        if (!adc_ok)
                          {
#if WM_EVAL_ENABLE
                            eval.fault_water_cnt++;
#endif
                            latch_fault(&out, &faultLatched, &faultReason, &state,
                                        FAULT_WATER, now, stateStart_ms, rinseIndex,
                                        &savedState, &savedInState_ms, &savedRinseIndex,
                                        msg1, msg2, &showMsg, &msgStart_ms);
                            break;
                          }

                        if (levelPercent <= WATER_DRAIN_TARGET_PCT || inState_ms >= prog->drain2_ms)
                          {
                            rinseIndex++;
                            if (rinseIndex < prog->rinseTotal)
                              {
                                state = WM_RINSE_FILL;
                                stateStart_ms = now;
                              }
                            else
                              {
                                state = WM_SPINNING;
                                stateStart_ms = now;
                              }
                          }
                      }
                    break;

                  case WM_SPINNING:
                    Relay_Set(&out, false);
                    motor_mode = MOTOR_MODE_SPIN;

                    {
                      uint32_t half = (prog->spin_ms / 2U);
                      if (half == 0) half = 1;

                      if (inState_ms < half)
                        target_rpm = 350 + (int)((550L * (long)inState_ms) / (long)half);
                      else
                        {
                          uint32_t t2 = inState_ms - half;
                          target_rpm = 900 - (int)((900L * (long)t2) / (long)half);
                          if (target_rpm < 0) target_rpm = 0;
                        }
                    }

                    if (inState_ms >= prog->spin_ms)
                      {
                        state = WM_DONE;
                        stateStart_ms = now;
                        cycleRunning = false;
                        Buzzer_StartPattern(&out, BUZZER_PATTERN_LONG_BEEP);

#if WM_EVAL_ENABLE
                        eval.cycle_completes++;
#endif
                      }
                    break;

                  case WM_DONE:
                    Relay_Set(&out, false);
                    motor_mode = MOTOR_MODE_OFF;
                    target_rpm = 0;

                    if (inState_ms >= DONE_SCREEN_MS)
                      {
                        state = WM_IDLE;
                        stateStart_ms = now;
                      }
                    break;

                  default:
                    state = WM_IDLE;
                    stateStart_ms = now;
                    cycleRunning = false;
                    paused = false;
                    Relay_Set(&out, false);
                    motor_mode = MOTOR_MODE_OFF;
                    target_rpm = 0;
                    motor_rpm = 0;
                    rinseIndex = 0;
                    break;
                }

#if WM_EVAL_ENABLE
              if (prev != state)
                {
                  eval_state_enter(&eval, state, now);
                }
#endif
            }

          /* RPM smoothing (UI only) */
          if ((uint32_t)(now - lastMotorUpdate_ms) >= 60)
            {
              lastMotorUpdate_ms = now;

              if (motor_mode == MOTOR_MODE_OFF || target_rpm == 0)
                {
                  if (motor_rpm > 0)
                    {
                      motor_rpm -= 120;
                      if (motor_rpm < 0) motor_rpm = 0;
                    }
                }
              else
                {
                  int step = (motor_mode == MOTOR_MODE_SPIN) ? 140 : 110;
                  if (motor_rpm < target_rpm)
                    {
                      motor_rpm += step;
                      if (motor_rpm > target_rpm) motor_rpm = target_rpm;
                    }
                  else if (motor_rpm > target_rpm)
                    {
                      motor_rpm -= step;
                      if (motor_rpm < target_rpm) motor_rpm = target_rpm;
                    }
                }
            }

          /* Apply motor outputs (no debug logs) */
          if (!paused && !faultLatched && motor_mode != MOTOR_MODE_OFF && target_rpm > 0)
            {
              (void)Motor_ApplyOutputs(&hw, motor_mode, now);
            }
          else
            {
              (void)Motor_ApplyOutputs(&hw, MOTOR_MODE_OFF, now);
            }
        }

      /* LCD update tick (stable; no clears) */
      if (time_due(now, next_lcd))
        {
          uint32_t late = (now >= next_lcd) ? (now - next_lcd) : 0;

#if WM_EVAL_ENABLE
          eval_task_update(&eval.t_lcd, late);
#endif
          next_lcd += LCD_UPDATE_MS;

          char out0[17], out1[17];
          char tmp0[80], tmp1[80];

          if (showMsg && (uint32_t)(now - msgStart_ms) < MSG_SHOW_MS)
            {
              make_line16(out0, msg1);
              make_line16(out1, msg2);
            }
          else
            {
              if (showMsg) showMsg = false;

              wm_state_t dispState = state;
              uint32_t dispInState = inState_ms;
              uint8_t  dispRinse   = rinseIndex;

              if (faultLatched)
                {
                  dispState   = savedState;
                  dispInState = savedInState_ms;
                  dispRinse   = savedRinseIndex;
                }

              uint32_t remMs  = total_remaining_ms(dispState, dispInState, prog, dispRinse);
              uint32_t remSec = (remMs + 999U) / 1000U;

              snprintf(tmp0, sizeof(tmp0), "%s %-4s T:%3lus",
                       prog->name, state_short(dispState), (unsigned long)remSec);

              char wbuf[4];
              if (adc_ok) snprintf(wbuf, sizeof(wbuf), "%02lu", (unsigned long)levelPercent);
              else        snprintf(wbuf, sizeof(wbuf), "--");

              /* EXACT 16-char fit after padding/truncation:
               * "Wxx D:C V:Y R240"
               */
              int rpmShow = motor_rpm;
              if (rpmShow < 0) rpmShow = 0;
              if (rpmShow > 999) rpmShow = 999;

              snprintf(tmp1, sizeof(tmp1), "W%s D:%c V:%c R%03d",
                       wbuf,
                       doorClosed ? 'C' : 'O',
                       vibActive ? 'Y' : 'N',
                       rpmShow);

              if (faultLatched)
                {
                  snprintf(tmp0, sizeof(tmp0), "FAULT:%-4s %-4s",
                           fault_short(faultReason), state_short(savedState));
                }

              make_line16(out0, tmp0);
              make_line16(out1, tmp1);
            }

          if (memcmp(out0, last0, 17) != 0)
            {
              lcd_write_line16(&hw, 0, out0);
              memcpy(last0, out0, 17);
            }

          if (memcmp(out1, last1, 17) != 0)
            {
              lcd_write_line16(&hw, 1, out1);
              memcpy(last1, out1, 17);
            }
        }

      /* Sleep until next deadline to reduce lag/jitter */
      uint32_t next = next_input;
      next = time_min_u32(next, next_fsm);
      next = time_min_u32(next, next_adc);
      next = time_min_u32(next, next_lcd);
      next = time_min_u32(next, next_buzz);

      int32_t dt = (int32_t)(next - now);
      if (dt > 0)
        {
          usleep((useconds_t)dt * 1000U);
        }
      else
        {
          usleep(200);
        }
    }

  /* not reached */
  out_deinit(&out);
  wm_hw_close(&hw);
  return 0;
}
