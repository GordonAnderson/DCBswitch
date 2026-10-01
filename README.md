# DCBswitch

Firmware for the **DCBswitch** MIPS module (Teensy 4.0), built with PlatformIO.
Ported from the Arduino IDE. Current version: **1.2** (Sept 2, 2024).

The module can run inside a MIPS system (over the TWI/I2C bus) or standalone
(commands over USB serial).

## Features

- 4 DC bias outputs, ±250 V, with voltage readback, supply on/off and readback-error trip protection
- 4 digital inputs (Q–T) and 4 digital outputs (A–D)
- 2 high-speed FET switches, optionally following a digital input
- 2 pulse generators: arm/trigger inputs, delay, skip count, width, period, pulse count, re-trigger,
  and trigger output on a digital output, FET switch or DC bias channel
- 2-channel ramp generator: up to 10 (time, voltage) points per channel, period, cycle count, external trigger
- Settings and calibration saved to on-board flash (LittleFS)
- MIPS bus interface with EEPROM emulation and serial-over-TWI

## Build and upload

Requires [PlatformIO](https://platformio.org/).

```sh
pio run                 # build
pio run -t upload       # build and flash the Teensy
pio device monitor      # USB serial console
```

Dependencies are fetched from GitHub by `platformio.ini`:
[GAACE_Core](https://github.com/GordonAnderson/GAACE_Core) and
[ArduinoThread](https://github.com/GordonAnderson/ArduinoThread). They are not pinned to a
revision, so pin them (`repo.git#<tag-or-commit>`) before a release.

## Layout

| Path | Purpose |
|------|---------|
| `src/DCBswitch.cpp` | Application: setup/loop, command table and handlers, pulse and ramp generators, TWI handling |
| `include/DCBswitch.h` | Pin map, TWI command codes, settings structures, handler prototypes |
| `src/DIN.cpp`, `include/DIN.h` | Digital input class: one pin interrupt fanning out to several callbacks |
| `src/Hardware.cpp`, `include/Hardware.h` | DAC/ADC update helpers for the AD5592 |
| `src/TWIext.cpp`, `include/TWIext.h` | Helpers to read/write multi-byte values on the TWI interface |
| `src/DCBswitchV1.1.hex` | Prebuilt V1.1 firmware (historical) |

## Hardware (Teensy 4.0 pins)

| Function | Pins |
|----------|------|
| Digital outputs A–D | 23, 22, 21, 20 |
| Digital inputs Q–T | 17, 16, 15, 14 |
| AD5592 chip select | 10 |
| FET switches 1, 2 | 9, 8 |
| HV supply off | 5 |

AD5592 channels 0–3 are the DC bias DAC outputs and 4–7 the readback ADCs.

## Host commands

Send ASCII commands over USB serial (or over TWI after the serial command). Channel-indexed commands take the channel as
the first argument. Use the built-in help for the full list with descriptions. Main groups:

| Group | Commands |
|-------|----------|
| System | `GVER`, `?NAME`, `SAVE`, `RESTORE`, `FORMAT`, `STWIADD`/`GTWIADD` (hex, valid 0x08–0x77, applied after `SAVE` and reset) |
| DC bias | `SDCBS`/`GDCBS`, `GDCBSV` (readback), `SDCPWR`/`GDCPWR`, `?RBTST`, `CALCH` |
| Digital IO and FETs | `SDIO`/`GDIO`, `SSW`/`GSW`, `PSW`, `STRGSW`/`GTRGSW` |
| Pulse generator | `SPG*` / `GPG*`: `ENA`, `RET`, `ARMT`, `ARML`, `TRG`, `TRGL`, `SKIP`, `DLY`, `WDTH`, `TRGO`, `TRGF`, `OUTCH`, `PV`, `NUM`, `PER` |
| Ramp generator | `SRPENA`/`GRPENA`, `SRPTRG`, `SRPTRGL`, `SRPDCBC`, `SRPCHENA`, `SRAMP`/`GRAMP`, `SRPPER`, `SRPCYC` (and `G` forms) |

Example: pulse generator 1, 5 µs wide, on DC bias channel 2 at 100 V:

```
SPGWDTH,1,5
SPGOUTCH,1,2
SPGPV,1,100
SPGENA,1,TRUE
```

## TWI (MIPS bus) interface

The module is an I2C slave (default address 0x54). Because MIPS expects a serial EEPROM, one address emulates it:

- A first byte with MSB clear is a one-byte EEPROM address; a lone address returns 32 bytes of the settings structure, extra bytes are written.
- `0x80` followed by a 16-bit address selects an extended address.
- A first byte with MSB set and non-zero low bits is a command (see `include/DCBswitch.h`), e.g. `0x27` switches to serial command mode (ESC leaves it).

## Safety notes

- The HV supply is off at boot. If a readback differs from its setpoint by more than `TRIPVOLTAGE` (5 V) after the holdoff, the supply is switched off.
- Pulse generator and ramp settings are applied immediately; keep outputs disconnected when experimenting.

## Known issues / to review

Found during the PlatformIO port review and left unchanged because they need hardware or MIPS host knowledge:

- `myfs.begin(0x1024 * 64)` in `setup()`: 0x1024 looks like a typo for 1024. Changing it alters the flash file system size, so saved settings would be lost.
- `GET_DCBS_VRB` (0x42) is defined but unused; the voltage readbacks are returned for `GET_DCBS_RBTST` (0x46). Confirm which code MIPS uses.
- `pulseGenPulse()` raises a user setting (`PG[ch].width`) when it is below the timing correction.
- Several ISR-shared fields (`dcbs.PG[].enable`, DIN callback tables) are not `volatile`.
- Enabling the ramp generator twice with an external trigger attaches the callback twice (harmless, `DIN` ignores duplicates).
- No automated tests; `test/` is empty. Command parsing could be covered with the GAACE_Core test setup.

## License

Not yet specified.
