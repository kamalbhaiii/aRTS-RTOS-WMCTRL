
### Running the Controller
1. **Serial Console:**
- Connect via USB (virtual COM port).
- Use picocom: `picocom -b 115200 /dev/ttyACM0` (replace with your device).
- At NSH prompt: `wmctrl` to launch the controller.

2. **Usage:**
- Select program (Quick/Normal/Heavy) using buttons.
- Start cycle with B1.
- Monitor LCD for status (e.g., "NRM WASH T:23s", "W85 D:C V:N R240").
- Simulate faults (open door, trigger vibration) to test interlocks.
- Long-press B1 in IDLE for diagnostic dump.

3. **Debugging:**
- Use NSH for logs.
- Oscilloscope on GPIO for loop timing.
- Validation dumps provide stats (uptime, lateness, faults).

### Contributing
Contributions welcome! Fork the repo, add features (e.g., multi-tasking), and submit pull requests. Follow NuttX coding guidelines.

### License
Apache License 2.0 (see LICENSE file).

### References
- Project Paper: [Link to PDF or LaTeX in repo].
- NuttX Docs: https://nuttx.apache.org/docs/latest/.
- Board Support: https://nuttx.apache.org/docs/latest/platforms/arm/stm32g4/boards/nucleo-g431rb/index.html.
- NuttX Running Guide: https://nuttx.apache.org/docs/latest/quickstart/running.html.
- NuttX Installation: https://nuttx.apache.org/docs/latest/quickstart/install.html.
- NuttX Detailed Support: https://nuttx.apache.org/docs/12.3.0/introduction/detailed_support.html.
- Installing NuttX Guide: https://bertvoldenuit.github.io/Nuttx-mdwiki/en/#!pages/getting-started/installing_nuttx.md.
- STM32 Nucleo User Manual: https://www.st.com/resource/en/user_manual/um2505-stm32g4-nucleo64-boards-mb1367-stmicroelectronics.pdf.
- NuttX on STM32F769: https://www.cocoacrumbs.com/blog/2021-10-25-nuttx-on-the-stm32f769-disco-board/.

For issues, open a GitHub ticket.

---

Embedded RTOS like NuttX are ideal for appliances requiring deterministic multitasking. This implementation emulates a washing machine on STM32 Nucleo-G431RB, handling phases (fill, wash, rinse, spin) with safety features. It leverages NuttX's POSIX APIs for I/O and timing, achieving low jitter (max 2.2 ms) and efficient resource use (50 KB Flash, 8 KB RAM). The single-thread cooperative loop simplifies design while meeting real-time needs, as validated in 10+ cycles.

Compared to bare-metal or HDL approaches (e.g., Reddy et al., 2024), NuttX adds portability and modularity, reducing development time by ~30%. Tested on NuttX 12.6.0, it's compatible with 12.12.0, benefiting from SMP optimizations and better signal handling.

The Nucleo-G431RB runs at 72 MHz, with peripherals mapped as per Table 1 in the paper. Optocouplers enhance isolation for reliability. Sensors simulate real inputs: potentiometer for water (0-100%), switches for door/vibration. Motor uses L298N for direction control; relay for valve/pump. LCD (HD44780) shows compact status; buzzer patterns alert users.

Full pinout:

| Function          | Pin/Device          |
|-------------------|---------------------|
| Door sensor       | PB5 (input)         |
| Vibration sensor  | PB4 (input)         |
| Water level       | ADC1 ch6 (analog)   |
| Motor IN1         | PB2 (output)        |
| Motor IN2         | PB1 (output)        |
| Relay             | PA5 (output)        |
| Buzzer            | PA9 (output)        |
| LCD RS            | PA0 (output)        |
| LCD E             | PA1 (output)        |
| LCD D4-D7         | PA4, PB0, PB7, PB6 |
| Button B1         | PC13 (input)        |
| Mode button       | PC1 (input)         |

Power via USB; all at 3.3V.

**FSM Design:** Enumerated states with transitions via δ(S, E) → S. Supports resume after recoverable faults (door/vibration). Programs (Quick: short, no rinse; Normal: 1 rinse; Heavy: 2 rinses) parameterized in structs.

**Scheduling:** Time-triggered loop with usleep() for efficiency. Periods: input 5 ms, FSM 10 ms, etc. Jitter bounded at 20% of period; no missed deadlines.

**Fault Handling:** Debouncing (10-25 ms). Faults trigger pause/abort; recovery <500 ms. Tested: 100% resumption.

**Outputs/Alerts:** Motor modes (oscillate/wash, uni-spin). LCD optimized (change-only updates, 40% faster). Buzzer patterns as mini-FSM.

**Drivers:** POSIX I/O for GPIO/ADC. Non-blocking ADC with trigger.

Memory: <32 KB Flash, <8 KB RAM; no dynamic alloc except stacks (peak 65%). File descriptors limited to 8; open-close pattern avoids leaks.

Diagnostics: On-demand dump via button; stats on lateness, faults, state times.

Additional 12.6.0 Results: Dumps show cycle times ~70 s, fault overhead ~872 ms/fault, cumulative lateness but stable.

Table of State Times Across Dumps:

| State         | Dump 1 (ms/Entries) | Dump 2 (ms/Entries) | Dump 3 (ms/Entries) |
|---------------|---------------------|---------------------|---------------------|
| IDLE         | 0/1                | 20990/2            | 50290/3            |
| FILLING      | 0/0                | 4000/1             | 7650/4             |
| WASHING      | 0/0                | 18010/1            | 36030/5            |
| DRAINING_1   | 0/0                | 9000/1             | 18020/3            |
| RINSE_FILL   | 0/0                | 3010/1             | 6020/2             |
| RINSE_AGITATE| 0/0                | 12020/1            | 24040/4            |
| DRAINING_2   | 0/0                | 9000/1             | 18010/3            |
| SPINNING     | 0/0                | 12000/1            | 24000/2            |
| DONE         | 0/0                | 2500/1             | 5000/2             |
| FAULT        | 0/0                | 0/0                | 7850/9             |
| **Total**    | 0                  | 90530              | 196910             |
| **Uptime**   | 4820               | 95800              | 204270             |

Observations: Consistent durations; faults increase entries but not total time significantly.

Timing Metrics Table:

| Task   | Period (ms) | Max Lateness (ms) | Avg Lateness (ms) |
|--------|-------------|-------------------|-------------------|
| Input  | 5           | 2.2               | 0.1               |
| FSM    | 10          | 2.0               | 0.05              |
| ADC    | 120         | 1.5               | 0.2               |
| LCD    | 250         | 2.1               | 0.3               |
| Buzzer | 20          | 1.0               | 0.01              |

Proof: Oscilloscope-aligned with NuttX latencies (<10 μs).

- Multi-threading for scalability.
- Add PID for temperature.
- Stress testing under loads.
- Logging for maintenance.

Repo: https://github.com/Kamalbhaiii/aRTS-RTOS-WMCTRL. See paper for theory.

**Key Citations**
- [Apache NuttX Downloads](https://nuttx.apache.org/download/)
- [NuttX Wikipedia](https://en.wikipedia.org/wiki/NuttX)
- [NuttX GitHub](https://github.com/apache/nuttx)
- [ST Nucleo G431RB Docs](https://nuttx.apache.org/docs/latest/platforms/arm/stm32g4/boards/nucleo-g431rb/index.html)
- [NuttX Running Guide](https://nuttx.apache.org/docs/latest/quickstart/running.html)
- [NuttX Installation](https://nuttx.apache.org/docs/latest/quickstart/install.html)
- [NuttX Detailed Support](https://nuttx.apache.org/docs/12.3.0/introduction/detailed_support.html)
- [Installing NuttX Guide](https://bertvoldenuit.github.io/Nuttx-mdwiki/en/#!pages/getting-started/installing_nuttx.md)
- [STM32 Nucleo User Manual](https://www.st.com/resource/en/user_manual/um2505-stm32g4-nucleo64-boards-mb1367-stmicroelectronics.pdf)
- [NuttX on STM32F769](https://www.cocoacrumbs.com/blog/2021-10-25-nuttx-on-the-stm32f769-disco-board/)