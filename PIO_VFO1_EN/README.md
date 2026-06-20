# PIO_VFO — Raspberry Pi Pico PIO 7MHz I/Q VFO

A 7MHz-band quadrature (I/Q) VFO using the PIO of the Raspberry Pi Pico.

## Features / requirement mapping

| Requirement | Implementation |
|------|------|
| Oscillates 7.000-7.200 MHz | `FREQ_MIN` / `FREQ_MAX` |
| Step 100Hz / 1000Hz toggled by a push switch | toggled by `PIN_STEP_SW = GP6` |
| I/Q 90-degree quadrature output | 4-phase Gray code to GP2(I)/GP3(Q) via PIO |
| Accuracy +-10Hz of the indicated value | high-speed clkdiv dithering by core1 + `CAL_PPM` calibration |
| Variable tuning by rotary encoder (custom lib, GP0/GP1) | `RotaryEncoder.h/.cpp` |
| Frequency display OLED 128x32 (U8g2lib.h) | `U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C` |

## Wiring

```
Rotary encoder       A  --- GP0
                     B  --- GP1
                     C(COM) --- GND
Step-toggle push SW     --- GP6  ---|press to GND|--- GND   (internal pull-up)

RF output  I --- GP2
           Q --- GP3   (* I/Q low-pass -> buffer/level conversion recommended. 3.3V logic square wave)

OLED(SSD1306 128x32, I2C)
        SDA --- GP4
        SCL --- GP5
        VCC --- 3V3
        GND --- GND
```

> GP2/GP3 are 3.3V logic square waves. When using them as a local oscillator for
> a receiver, insert an LPF / comparator or buffer (74AC04 etc.) between stages.

## Build environment

- **arduino-pico (Earle Philhower) core** (supports PIO/SDK API, dual core, `Wire.setSDA/SCL`)
- Library: **U8g2** (install from the Library Manager)
- Board: `Raspberry Pi Pico` or `Raspberry Pi Pico 2`
- Place the 4 files (`PIO_VFO.ino` / `RotaryEncoder.h` / `RotaryEncoder.cpp` / `README.md`)
  in the same sketch folder and compile.

> The PIO program is not provided as a separate `.pio` file; it is embedded
> in `PIO_VFO.ino` as a hand-assembled instruction array (no pioasm needed).

## Principle of frequency generation

```
f_rf = f_sys / (4 * clkdiv)
```

The PIO is run at 4xf_rf and outputs the Gray code `00->01->11->10` on two pins.
GP2(I) and GP3(Q) are always square waves offset by 1 sample = 90 degrees (the phase difference is exact).

clkdiv has 1/256 steps, so at 7MHz 1 LSB is about 6kHz, which is coarse. Therefore **core1**
switches between two adjacent clkdiv values at high speed with a 1st-order delta-sigma, making the
**average** division ratio continuously variable to home in on the target frequency (achieving +-10Hz).

## Calibration (required to meet +-10Hz)

+-10Hz = **+-1.4ppm**. The Pico's standard crystal is +-10 to 30ppm (= +-70 to 210Hz @ 7MHz), so
**the absolute-accuracy requirement cannot be met without calibration**.

1. Set `g_freq` to, for example, 7.100000MHz.
2. Measure with a frequency counter / calibrated receiver.
3. Compute the error in ppm: `ppm = (measured - indicated) / indicated * 1e6`
4. Enter that value into `CAL_PPM` in `PIO_VFO.ino` and re-flash.

If long-term and temperature stability are required, replace the Pico's XOSC with a **TCXO**.

## Notes / known limitations

- Small spurs from dither switching appear roughly 1MHz away from the carrier.
  If this is an issue for narrowband use, make core1 a 2nd-order (MASH) delta-sigma,
  or adjust `delayMicroseconds`.
- If the encoder produces 2 counts per detent, etc., call
  `enc.setStepsPerDetent(2);` in `setup()` to adjust.
