# SAM-Multislope — Project Context

## What This Is

Bare-metal firmware for the **SAMC21J18A** (ARM Cortex-M0+, 48 MHz, 256 KB Flash, 32 KB RAM).
The codebase originated as a proof-of-concept on an Atmel SAMC21 Xplained Pro but now targets
**ATSAM C21J 1.0**, a custom proto-board that gives full control over the MCU pins.

The long-term goal is a **multi-slope ADC** implementation — still far out. The immediate priority
is building a solid, reusable base of peripheral utilities.

---

## Hardware — Custom Proto Board

| Resource | Details |
|---|---|
| MCU | ATSAMC21J18A-AU (64-pin, 3.3 V / 5 V tolerant I/O) |
| Crystal 1 | 24 MHz — PA14 (XIN), PA15 (XOUT) |
| Crystal 2 | 32.768 kHz footprint — PA00 (XIN32), PA01 (XOUT32); layout tested, not used by firmware |
| UART→USB | PB30 (TX) and PB31 (RX) → COM5; serial monitor tested OK at **1 Mbps** / `1000000` baud (CH340 + ADUM1201) |
| Debug | Atmel ICE via SWD **or** JLink EDU Mini v2.0 — both recognise the chip |
| User LEDs | 3 active-high LEDs, jumperable to any MCU pin |
| User Buttons | 3 active-high buttons with external pull-downs, jumperable to any MCU pin |
| **Currently wired** | PB23 (LED), PB22 (BTN) — everything else disconnected for now |

### Pin Notes
- PA14/PA15 are reserved for the 24 MHz crystal — **do not use for GPIO or peripherals**.
- PA00/PA01 are not used by current firmware; the 32.768 kHz crystal footprint was only validated.
- PB30/PB31 are reserved for the UART→USB bridge (COM5).
- LEDs and buttons are jumperable test fixtures; `pins.hpp` records current bench wiring, not
  permanent board constraints.
- Xplained Pro pin constraints are obsolete. Pick the peripheral/pad mapping that best fits the
  custom board, then jumper test fixtures as needed.

---

## Build System

**PlatformIO** with a fully custom environment (no Arduino, no ASF).

Key `platformio.ini` settings:

```ini
board = samC21_xpro          ; used only for MCU identification
board_build.mcu = samc21j18a
board_build.f_cpu = 48000000L
board_build.ldscript = linker/samc21j18.ld
build_flags = -std=gnu++23 -DDONT_USE_CMSIS_INIT ...
upload_protocol = jlink
monitor_port = COM5
monitor_speed = 1000000
```

- Toolchain: `arm-gnu-toolchain-15.2.rel1` (mingw hosted)
- C++ standard: **C++23** (`-std=gnu++23`)
- CMSIS startup disabled; custom `startup_samc21.c` provides the vector table and reset handler
- Flash wait states: **2 cycles** (required at 48 MHz)
- No RTOS — bare polling loop

---

## Source Layout

```
src/
├── main.cpp            — init sequence + serial heartbeat loop
├── pins.hpp            — fixed UART pins + current jumper/test assignments
├── init.hpp            — clock init (OSC48M @ 48 MHz, XOSC 24 MHz on GCLK1)
├── waveform.hpp        — active TCC0 PWM test waveform at 375 kHz
└── core/               — reusable utility layer (the good stuff)
    ├── pin.hpp         — template GPIO abstraction Pin<Bank, Index>
    ├── uart.hpp        — SERCOM UART traits
    ├── uart_int.hpp    — interrupt-driven UART driver
    ├── serial_port.hpp — serial wrapper with print methods
    ├── bytestream.hpp  — C++23 std::to_chars based number formatting
    ├── line_reader.hpp — CLI line buffer with echo, backspace, timeout
    ├── ring_buffer.hpp — power-of-2 FIFO template RingBuffer<T, N>
    ├── timebase.hpp/.cpp — SysTick 1 kHz → millis() / micros()
    ├── startup_samc21.c — vector table, reset handler, BSS/data init
    └── syscalls.cpp    — newlib stubs (_write/_read → Serial)
```

---

## Core Utilities Worth Keeping

| Utility | Status | Notes |
|---|---|---|
| `Pin<Bank,N>` | Good | Zero-overhead GPIO template |
| `RingBuffer<T,N>` | Good | Power-of-2 FIFO, ISR-safe push/pop |
| `Uart<Interrupt,...>` | Good | Stats tracking, non-blocking read/write |
| `SerialPort` | Good | Print mixin; `extern SerialType Serial` |
| `Timebase` | Good | SysTick millis/micros, IRQ-safe |
| `LineReader` | Good | CLI input with echo/backspace/timeout |
| `ByteStream` print | Good | Hex/bin/dec via `std::to_chars` — the right alternative to printf (no newlib formatter, no flash bloat) |
| `startup_samc21.c` | Good | Keep — custom vector table |
| `syscalls.cpp` | Good | Newlib I/O → Serial |

### Removed Experiments

Old `ac.hpp`, `events.hpp`, `luts.hpp`, and `heartbeat.hpp` experiments were removed from `src/`;
use git history if that lab-note material is ever needed again.

---

## Conventions

- **No Arduino, no ASF, no HAL** — direct register access via CMSIS headers.
- **Static struct pattern** for peripheral init: `struct Foo { static void init(); };`
- **Templates over inheritance** for zero-cost abstraction.
- **C++23** — concepts, `std::to_chars`, structured bindings are all available.
- No dynamic allocation. No exceptions. No RTTI.
- Peripheral SERCOM selection: map required pads to available pins first, then pick SERCOM.

---

## Current Checkpoint

- UART is on PB30/PB31 via SERCOM5; current firmware and PlatformIO monitor use `1000000` baud,
  tested OK as a **1 Mbps** serial monitor link.
- `GCLK0` remains OSC48M @ 48 MHz; `GCLK1` uses the 24 MHz crystal for the active TCC0 waveform.
- The 32.768 kHz crystal is not started or used.
- Xplained Pro AC/EVSYS/CCL/heartbeat experiments were removed; `waveform.hpp` is the only active
  TCC0 test sketch.
