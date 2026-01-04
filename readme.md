# Real-Time Operating System for Washing Machine Control
#### Reasearch Question: How does the choice of a Real-Time Operating System affect scheduling determinism and timing accuracy in single-core embedded control systems for household appliances such as washing machines?

This is a NuttX example application that emulates a washing machine controller on the STM32 NUCLEO-G431RB. It uses a cooperative time-triggered loop (single thread) to manage inputs, a finite state machine (FSM), LCD updates, and a buzzer while staying within tight file-descriptor limits.

The implementation targets NuttX 12.6.0 and is compatible with newer 12.x releases. It is designed to run with `CONFIG_NFILE_DESCRIPTORS_PER_BLOCK=8`, which is why most GPIO/ADC operations open -> operate -> close.

## Features
- Programs: Quick, Normal, Heavy with parameterized phase durations.
- FSM states: idle, filling, washing, draining, rinsing (fill/agitate), spinning, done, fault.
- Safety: door-open latch, vibration latch, ADC/water sensor fault.
- LCD UI: stable 16x2 updates (no periodic clear), fixed-width formatting.
- Buzzer patterns for start/resume/fault/done.
- Built-in validation dump (hold B1 in IDLE) with task lateness, faults, water stats, and state timing.
- FD-safe hardware access for tight descriptor limits.

## Hardware
Target board: STM32 NUCLEO-G431RB

![Hardware Pinout Map](assets/Hardware%20Pinout%20Map.png)

### Inputs
- Door sensor: GPIO minor 0 (PB5) (active-low, LOW = closed)
- Vibration sensor: GPIO minor 1 (PB4) (active-high)
- Mode button: GPIO minor 4 (PC1) (pressed = 0)
- User button B1: GPIO minor 5 (PC13) (pressed = 1)
- Water level sensor: ADC channel 6 (0-4095 mapped to 0-100%)

### Outputs
- Relay (valve/pump): GPIO minor 2 (PA5)
- Buzzer (active-low): GPIO minor 3 (PA9)
- Motor driver (L298 IN1/IN2): GPIO minors 6/7 (PB2/PB1)
- LCD (HD44780 16x2, 4-bit): GPIO minors 8-13
  - RS: 8 (PA0)
  - E: 9 (PA1)
  - D4: 10 (PA4)
  - D5: 11 (PB0)
  - D6: 12 (PB7)
  - D7: 13 (PB6)

Power: USB, 3.3V logic.

## Software Structure
- `wmctrl_main.c`: main loop, FSM, UI, buzzer, evaluation dump.
- `wmctrl_hw.c/.h`: FD-safe GPIO/ADC helpers and time source.
- `wmctrl_lcd.c/.h`: HD44780 4-bit driver using GPIO minors.
- `Kconfig`, `Makefile`, `Make.defs`: NuttX app configuration.

## Architecture Overview
The controller is a single-threaded, time-triggered loop with independent deadlines. Each cycle:
- Scans inputs with debouncing.
- Updates the FSM and outputs (relay, motor direction, buzzer).
- Samples ADC water level.
- Updates LCD only when content changes.
- Sleeps until the next earliest deadline to reduce jitter.

The design avoids dynamic allocation in the loop and keeps file descriptors low:
- Relay and buzzer are kept open persistently (2 FDs).
- GPIO reads/writes and ADC sampling open/close on each access.

![System Architecture](assets/System%20Architecture.png)
Use the following prompt verbatim with your preferred diagram or design tool (e.g., Figma, draw.io, Lucidchart, Excalidraw, Mermaid+renderer, or an AI image generator that supports diagram style).

```
You are creating four professional, publication-quality engineering diagrams for a NuttX-based washing machine controller running on STM32 NUCLEO-G431RB. The diagrams must be clean, precise, and visually consistent as a set.

Global style requirements:
- Canvas: 1600px width, 16:9 aspect ratio, export as PNG.
- Visual style: flat vector, minimal gradients, subtle blue/gray palette (no neon).
- Typography: clean technical sans-serif, consistent size and weight across all diagrams.
- Labels: concise but specific; avoid abbreviations unless defined.
- Arrows: consistent thickness and arrowheads; directionality should be unambiguous.
- Layout: generous spacing, alignment to a simple grid, no overlapping text.
- Include a small title in the top-left of each diagram.

Diagram 1: System Architecture Overview
- Title: "WMCTRL System Architecture"
- Show three major columns: Inputs, Controller, Outputs.
- Inputs block (left): Door Sensor (PB5), Vibration Sensor (PB4), Mode Button (PC1), User Button B1 (PC13), Water Level ADC (ADC1 ch6).
- Controller block (center): "wmctrl_main.c" with sub-blocks: Time-Triggered Loop, Debounce, FSM, Evaluation/Validation, LCD Formatting, Buzzer Patterns.
- Outputs block (right): Relay (PA5), Motor Driver L298 (IN1 PB2, IN2 PB1), Buzzer (PA9), LCD 16x2 (RS PA0, E PA1, D4 PA4, D5 PB0, D6 PB7, D7 PB6).
- Draw data-flow arrows from inputs to controller, and controller to outputs.
- Annotate task cadences near the loop: Input 5ms, FSM 10ms, Buzzer 20ms, ADC 120ms, LCD 250ms.
- Add a small callout box: "FD-safe access: open -> ioctl/read -> close (GPIO/ADC)".

Diagram 2: FSM State Diagram
- Title: "WMCTRL FSM"
- Nodes: IDLE, FILLING, WASHING, DRAINING_1, RINSE_FILL, RINSE_AGITATE, DRAINING_2, SPINNING, DONE, FAULT.
- Use directional arrows with labels:
  - IDLE -> FILLING: "B1 start"
  - FILLING -> WASHING: "water >= target OR fill timeout"
  - WASHING -> DRAINING_1: "wash timeout"
  - DRAINING_1 -> RINSE_FILL: "water <= drain target OR drain timeout"
  - RINSE_FILL -> RINSE_AGITATE: "water >= rinse target OR fill timeout"
  - RINSE_AGITATE -> DRAINING_2: "rinse timeout"
  - DRAINING_2 -> RINSE_FILL: "more rinses"
  - DRAINING_2 -> SPINNING: "rinse complete"
  - SPINNING -> DONE: "spin timeout"
  - DONE -> IDLE: "done screen timeout"
- Fault transitions:
  - Any state -> FAULT for "Door Open", "Vibration", or "ADC/Water".
  - FAULT -> previous state when condition clears; FAULT -> IDLE on user abort.
- Use dashed arrows for fault transitions and solid arrows for normal flow.

Diagram 3: Timing Cadence Timeline
- Title: "Task Cadences (1s window)"
- Horizontal timeline from 0ms to 1000ms with tick marks every 100ms.
- Draw five horizontal lanes:
  - Input Scan (5ms)
  - FSM Tick (10ms)
  - Buzzer Tick (20ms)
  - ADC Sample (120ms)
  - LCD Update (250ms)
- Represent each task as repeated small blocks at their interval (not every single tick; show pattern clearly).
- Add a note: "Loop sleeps until nearest next deadline to reduce jitter."

Diagram 4: Hardware Pinout Map
- Title: "NUCLEO-G431RB Pin Mapping (wmctrl)"
- Draw the MCU/board as a rectangle, with labeled pins on left/right edges.
- List each signal with its pin and GPIO minor:
  - Door Sensor -> PB5 (GPIO0)
  - Vibration Sensor -> PB4 (GPIO1)
  - Relay -> PA5 (GPIO2)
  - Buzzer -> PA9 (GPIO3)
  - Mode Button -> PC1 (GPIO4)
  - User Button B1 -> PC13 (GPIO5)
  - L298 IN1 -> PB2 (GPIO6)
  - L298 IN2 -> PB1 (GPIO7)
  - LCD RS -> PA0 (GPIO8)
  - LCD E -> PA1 (GPIO9)
  - LCD D4 -> PA4 (GPIO10)
  - LCD D5 -> PB0 (GPIO11)
  - LCD D6 -> PB7 (GPIO12)
  - LCD D7 -> PB6 (GPIO13)
  - Water Level -> ADC1 channel 6
- Add a footnote: "Logic level: 3.3V, power via USB."
```

## NuttX Configuration
Enable the app and set its parameters in `menuconfig`:
- `EXAMPLES_WMCTRL`: enable wmctrl example
- `EXAMPLES_WMCTRL_PROGNAME` (default: `wmctrl`)
- `EXAMPLES_WMCTRL_PRIORITY` (default: 100)
- `EXAMPLES_WMCTRL_STACKSIZE` (default: 4096)

Make sure the board support package registers the GPIO minors listed above and exposes the ADC device (`/dev/adc1` preferred, falls back to `/dev/adc0`).

## Build and Flash (NuttX)
Typical NuttX workflow:
1. Configure NuttX for your board.
2. Enable the `EXAMPLES_WMCTRL` app in `menuconfig`.
3. Build and flash.

Once the board is running NuttX, use the shell to launch:
```
wmctrl
```

## Running the Controller
1. **Serial Console:**
   - Connect via USB (virtual COM port).
   - Example: `picocom -b 115200 /dev/ttyACM0` (replace with your device).
   - At the NSH prompt, run `wmctrl`.

2. **Program Selection:**
   - Press the Mode button to cycle Quick/Normal/Heavy.
   - Press B1 to start the cycle.

3. **During Operation:**
   - LCD shows program/state/remaining time on line 1.
   - Line 2 shows water %, door/vibration, and RPM indicator.
   - Press B1 to pause/resume; long press B1 to abort.

4. **Diagnostics:**
   - In IDLE, hold B1 for ~1.2s to print a validation dump to the console.

## LCD Format
Line 1 (16 chars):
- `QCK WASH T: 23s`
- `NRM SPIN T: 05s`

Line 2 (16 chars):
- `W85 D:C V:N R240`
  - `Wxx`: water percent (or `--` if ADC unavailable)
  - `D:C`/`D:O`: door closed/open
  - `V:Y`/`V:N`: vibration active
  - `Rxxx`: UI-only RPM estimate (0-999)

Fault display replaces line 1 with:
- `FAULT:VIB WASH`

## State Machine Overview
![FSM State Diagram](assets/FSM%20State%20Diagram.png)

States:
- `IDLE` -> waiting for user
- `FILL` -> water fill until threshold or timeout
- `WASH` -> agitation pattern
- `DRN1` -> drain after wash
- `R-FL` -> rinse fill
- `R-AG` -> rinse agitation
- `DRN2` -> drain between rinses
- `SPIN` -> ramp up/down spin
- `DONE` -> completion message
- `FAIL` -> fault latched

Programs (ms):
- Quick: fill 9000, wash 18000, drain1 9000, rinse fill 8000, rinse agitate 12000, drain2 9000, spin 12000
- Normal: fill 11000, wash 22000, drain1 10000, rinse fill 9000, rinse agitate 14000, drain2 10000, spin 15000
- Heavy: fill 13000, wash 26000, drain1 11000, rinse fill 10000, rinse agitate 16000, drain2 11000, spin 18000

Water thresholds (percent):
- Start min: 80
- Wash target: 90
- Rinse target: 85
- Drain target: 20

## Timing Cadences
The controller uses a time-triggered loop with separate deadlines:
- Input scan: 5 ms
- FSM tick: 10 ms
- Buzzer tick: 20 ms
- ADC sampling: 120 ms
- LCD update: 250 ms

![Timing Cadence Diagram](assets/Timing%20Cadence%20Diagram.png)

Debounce windows:
- Door: 25 ms
- Mode: 20 ms
- B1: 20 ms
- Vibration: 10 ms

## Motor Behavior (UI vs. Outputs)
- Motor control uses L298 IN1/IN2 direction only (ENA assumed tied HIGH).
- `MOTOR_MODE_WASH` reverses direction every ~2s; `MOTOR_MODE_SPIN` ramps the UI RPM.
- RPM on LCD is an estimate for user feedback; no tachometer is used.

## Fault Handling
Faults are latched and require conditions to clear:
- Door open: closes before resume.
- Vibration: clears after stable quiet for 250 ms.
- Water/ADC: ADC must become available again.
- User abort: returns directly to IDLE.

While faulted:
- Relay off, motor off, buzzer alarm pattern.
- State is saved and resumed when allowed.

## Evaluation / Validation Dump
Hold B1 in IDLE (~1.2s) to print metrics:
- Task lateness stats (runs, max/avg lateness, counts over 1/5/10 ms)
- ADC ok drops and water min/max
- Fault counters and cycle counts
- Time spent per state and entry counts
- Vibration activity stats

This is useful for timing validation and runtime characterization.

## Notes on FD Usage
NuttX can be configured with very few file descriptors. To stay safe:
- Only the relay and buzzer are kept open persistently (2 FDs).
- All other GPIO and ADC operations open/close on each access.

## Troubleshooting
- No LCD output: verify GPIO minors 8-13 are registered and wiring matches the LCD 4-bit pins.
- No ADC updates: confirm `/dev/adc1` or `/dev/adc0` exists and channel 6 is enabled.
- Buttons inverted: check `MODE_PRESSED_VALUE` and `B1_PRESSED_VALUE` in `wmctrl_main.c`.

## Contributing
Contributions welcome. Please follow NuttX coding guidelines and keep FD usage constraints in mind. If you add new peripherals, ensure GPIO minors are properly registered and keep the 16x2 LCD formatting within 16 columns.

## Team
- Member 1: Kamal Sharma, [1541791]
- Member 2: Varshitha Ramamurthy, [1542037]
- Member 3: Fasla Puthiyaveettil Abdul Kader, [1542036]

## Acknowledgement
Thanks to Professor Matthias Deegener and Frankfurt University of Applied Sciences for guidance, feedback, and support throughout this project.

## License
Apache License 2.0 (see LICENSE file).

## References
- Project Paper: [Link to PDF or LaTeX in repo]
- NuttX Docs: https://nuttx.apache.org/docs/latest/
- Board Support: https://nuttx.apache.org/docs/latest/platforms/arm/stm32g4/boards/nucleo-g431rb/index.html
- NuttX Running Guide: https://nuttx.apache.org/docs/latest/quickstart/running.html
- NuttX Installation: https://nuttx.apache.org/docs/latest/quickstart/install.html
- NuttX Detailed Support: https://nuttx.apache.org/docs/12.3.0/introduction/detailed_support.html
- Installing NuttX Guide: https://bertvoldenuit.github.io/Nuttx-mdwiki/en/#!pages/getting-started/installing_nuttx.md
- STM32 Nucleo User Manual: https://www.st.com/resource/en/user_manual/um2505-stm32g4-nucleo64-boards-mb1367-stmicroelectronics.pdf
- NuttX on STM32F769: https://www.cocoacrumbs.com/blog/2021-10-25-nuttx-on-the-stm32f769-disco-board/

For issues, open a GitHub ticket.
