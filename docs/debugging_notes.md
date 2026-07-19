# Debugging Notes

Technical issues encountered during development on the **NUCLEO-F429ZI (STM32F429ZI)** and how they were resolved.

## HSE_BYPASS vs HSE_ON — Hang in Error_Handler

**Symptom:** Firmware hung in `Error_Handler()` immediately after clock configuration, and CAN bitrate was completely wrong when it did run.

**Cause:** On NUCLEO boards like the F429ZI, the HSE isn't driven by a classic quartz crystal on OSC_IN/OSC_OUT. Instead, the integrated ST-LINK's MCO (Master Clock Output) generates an 8 MHz clock signal and injects it directly onto the OSC_IN pin.

This changes how HSE must be configured:
- A real crystal needs an oscillator circuit (internal feedback resistor enabled) → **crystal/resonator mode**.
- An already-generated external clock signal (like the one from the ST-LINK) must be injected directly, bypassing the internal oscillator circuit → **bypass mode**.

Configuring HSE in crystal mode on a NUCLEO board makes the MCU try to drive its own oscillation circuit with a signal that isn't a real crystal — the clock either fails to start or starts incorrectly, resulting in a wrong CAN bitrate (this is what caused the 250 kbps configuration issues).

**Fix:** Set `RCC_HSE_BYPASS` instead of `RCC_HSE_ON` in the clock configuration (CubeMX RCC settings) — required on any NUCLEO board with the ST-LINK-driven clock.

---

## CubeIDE Debug Mode Blocking Serial Port

**Symptom:** Dashboard couldn't connect to the STM32 over UART while the board was flashed via CubeIDE's debug session.

**Cause:** CubeIDE's debugger holds the ST-Link's virtual COM port open during a debug session, blocking any other application (including the dashboard) from accessing it.

**Fix:** Flash via `st-flash` + `arm-none-eabi-objcopy` instead of CubeIDE's Run/Debug, freeing the port for the dashboard immediately after flashing.

---

## Parasitic UART Frames from Legacy Reference Loop

**Symptom:** Dashboard occasionally displayed corrupted sensor values with no clear pattern.

**Cause:** A legacy professor reference loop left in an early firmware version was still transmitting on the same UART line, interleaving its own frames with the intended ones.

**Fix:** Removed the legacy loop entirely from the firmware.
