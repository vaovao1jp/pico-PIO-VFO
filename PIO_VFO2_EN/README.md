# PIO_VFO2 — 128x64 dial-UI version

A version that keeps the oscillator section of `PIO_VFO` unchanged but extends the display to an **OLED 128x64**.

## Display layout

```
+----------------------------+
|        v  (center=current freq)  |  <- pointer marker + pointer line at top
|   7.09  \|/  7.11          |  <- arc dial (only the top of a large circle)
|  -----| | | |-----         |     ticks: 1k=thin / 5k=medium / 10k=long+label
|                            |     the ticks rotate left/right as the frequency changes
|       7.100.000            |  <- frequency (profont17, centered)
|        STEP: 100 Hz        |  <- step (5x8, centered)
+----------------------------+
```

- The center of the circle is below (outside) the screen, so only the top arc is visible — an "old-radio dial" look.
- The center pointer line always points at the current oscillation frequency, and the dial scale scrolls.
- Raising the frequency makes the scale flow to the left (higher frequencies line up on the right).

## Adjustable parameters (top of PIO_VFO2.ino)

| Constant | Role | Default |
|------|------|------|
| `DIAL_CY` / `DIAL_R` | Center position / radius of the circle. Arc apex y = CY-R | 90 / 86 |
| `DIAL_RAD_PER_HZ` | Sensitivity of frequency -> rotation angle. Larger = the scale flows faster | 5.0e-5 |
| `DIAL_TH_LIMIT` | Angular range of the drawn arc (+-rad) | 0.95 |

- To make the ticks "denser" -> lower `DIAL_RAD_PER_HZ` (e.g. 3.0e-5).
- If labels are too crowded / too sparse -> change the 10kHz decision interval in `drawTicks()`.

## Wiring / build / calibration

Same as `PIO_VFO` (128x32 version). Connect a 128x64 SSD1306 OLED to I2C (GP4/GP5).
The library is U8g2. Calibrate `CAL_PPM` with the same procedure (currently set to 16.43).

> If the OLED driver is an SH1106 etc., change the U8g2 constructor to
> `U8G2_SH1106_128X64_NONAME_F_HW_I2C` or similar.
