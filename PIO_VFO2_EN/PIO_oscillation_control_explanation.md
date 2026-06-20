# How the PIO oscillation control works + the CAL_PPM calibration procedure

An explanation of how a 7MHz-band I/Q VFO is generated with the Raspberry Pi Pico's PIO.
It builds up from the bottom — "how the waveform is made -> how the frequency is set -> how accuracy is achieved" —
and finally shows how to find `CAL_PPM`, which determines the absolute accuracy.

---

## 0. The big picture

| Layer | Owner | Role |
|---|---|---|
| (1) PIO program (4 instructions) | PIO SM0 | Produces the "shape" of the I/Q square waves on GP2/GP3 |
| (2) clkdiv (clock division) | Hardware | Execution speed of the SM = coarse frequency adjustment |
| (3) Dithering | core1 | Rapidly wobbles clkdiv for fine frequency adjustment (resolution) |
| (4) CAL_PPM | core0 computation | Corrects the crystal's ppm offset to ensure absolute accuracy |

Basic equation:

```
f_rf = f_sys / (4 x clkdiv)
```

---

## 1. The PIO program: making the waveform (SET instruction)

The core is just these 4 instructions.

```cpp
static const uint16_t quad_instr[] = {
  0xe000,   // set pins, 0b00   -> I=0, Q=0
  0xe001,   // set pins, 0b01   -> I=1, Q=0
  0xe003,   // set pins, 0b11   -> I=1, Q=1
  0xe002,   // set pins, 0b10   -> I=0, Q=1
};
```

### Bit allocation of the SET instruction (16-bit fixed length)

Example: `set pins, 1` = `0xE001` = `1110 0000 0000 0001`

```
 bit: 15 14 13 | 12 11 10  9  8 |  7  6  5 |  4  3  2  1  0
      --------- --------------- ---------- ---------------
       1  1  1 |  0  0  0  0  0 |  0  0  0 |  0  0  0  0  1
      [opcode]   [Delay/SideSet]  [Dest]       [Data]
       111=SET     0=execute 1 cycle  000=PINS    00001=output value 1
```

| Bits | Field | Description |
|---|---|---|
| [15:13] | opcode | fixed `111`, the SET instruction |
| [12:8] | Delay / SideSet | number of wait cycles inserted afterward (0-31). All zero here -> **1 instruction = 1 cycle** |
| [7:5] | Dest | `000`=PINS / `001`=X / `010`=Y / `100`=PINDIRS. Here it is PINS |
| [4:0] | Data | immediate value (0-31). This is the value driven onto the pins |

The 4 instructions differ only in the data field:

```
0xE000 -> set pins, 0  (I=0,Q=0)
0xE001 -> set pins, 1  (I=1,Q=0)
0xE003 -> set pins, 3  (I=1,Q=1)
0xE002 -> set pins, 2  (I=0,Q=1)
```

**Key points**

1. **The data can only be an immediate (constant)**. SET cannot output a register or computed result. That is why the 4 phases are laid out as "4 fixed instructions."
2. **Which pins are driven is decided by configuration**. `sm_config_set_set_pins(&c, PIN_RF_I, 2)` declares "SET targets = 2 pins starting at GP2" -> data bit0 -> GP2(I), bit1 -> GP3(Q). The data is 5 bits, so up to 5 pins.
3. **Direction is set separately**. `pio_sm_set_consecutive_pindirs(...)` makes them outputs at initialization. The SET in the loop only handles the value (PINS).
4. The Delay field can extend the length of each phase, but it changes the number of cycles per period and affects the `f_rf` calculation, so all are set to 0.

### I/Q output waveform (90-degree phase difference)

1 instruction = 1 clock, and the 4 instructions repeat via `.wrap`.

```
step:        0     1     2     3     0     1     2     3
SET instr:  00    01    11    10    00    01    11    10
                 +-----------+           +-----------+
I(GP2): ---------+           +-----------+           +----
                       +-----------+           +-----------
Q(GP3): ---------------+           +-----------+
                 |<-90->|
                 |<-------- 1 period = 4 steps -------->|
```

- I is High at steps 1,2; Q is High at steps 2,3.
- **Q always lags I by one step = always exactly 90 degrees**. Because the offset is by sample units, the phase difference does not depend on frequency.
- `00->01->11->10` (Gray code) is used so that two pins do not change simultaneously between adjacent steps, **preventing glitches (whiskers)**.

---

## 2. clkdiv: coarse frequency adjustment

The SM execution speed is `f_sys` (e.g. 125MHz) divided by **clkdiv**. clkdiv is a **16-bit integer + 8-bit fraction (1/256 steps)**.

```cpp
double div = (fsys / (4.0 * freqHz)) * corr;   // compute the target div as a real number
double scaled = div * 256.0;                    // to 1/256 units
uint32_t n = floor(scaled);                     // the lower clkdiv value
double   w = scaled - n;                        // fractional part (0-1)
```

The register format is `[31:16]=integer, [15:8]=fraction`.

**The problem**: at 7MHz clkdiv is about 4.46. With 1/256 steps, one step is about **6kHz**, so this alone cannot even produce a 100Hz step. The fraction `w` cannot be discarded.

---

## 3. Delta-sigma: wobbling clkdiv to get resolution (core1)

Switch between the two adjacent clkdiv values (`n` and `n+1`) extremely fast, making **the average the real number D = n + w** (1st-order delta-sigma = error-feedback type). core1 runs this exclusively.

```cpp
void loop1() {
  static uint32_t acc = 0;
  uint32_t old = acc;
  acc += g_w;                                    // g_w = w x 2^32
  uint32_t reg = (acc < old) ? g_reg1 : g_reg0;  // the larger div only on overflow
  VFO_PIO->sm[VFO_SM].clkdiv = reg;
  delayMicroseconds(1);
}
```

### Signal flow

```
 w*2^32 --->(+)---> acc(32bit) ---> overflow? >=2^32 ---> c[k] in {0,1} ---> D = n + c[k]
            ^         |
            +-- z^-1 -+   (mod 2^32: subtract 2^32 on carry, carrying the error over to next time)
```

### Recurrence

```
acc[k] = (acc[k-1] + W) mod 2^32            W = round(w * 2^32)
c[k]   = 1  (when acc[k-1] + W >= 2^32) / 0 (otherwise)
D[k]   = n + c[k]
```

The code `old=acc; acc+=W; carry=(acc<old);` is exactly this overflow detection.

### Why the average matches exactly

`acc` is a perfect counter that wraps at 2^32. After N additions, the number of overflows = `floor(N*W / 2^32)`.

```
carry rate = W / 2^32 = w         (exact as N -> infinity)
D_avg      = n*(1-w) + (n+1)*w = n + w   <- exactly as intended
```

### The output frequency matches exactly too (thinking in terms of phase)

Each RF period always advances the phase by 2pi. The time of one period is `4*D[k]/f_sys`. The total time T over K periods is

```
T = sum 4*D[k]/f_sys = (4/f_sys)*K*(n+w)
average frequency = 2pi*K / (2pi*T) = f_sys / (4*(n+w)) = target value
```

-> the average "frequency (spectral line)" lands on the target.

### Resolution

`w` has 32-bit precision (1/2^32 steps). Converted to frequency,

```
df ~ (4*f_rf^2 / f_sys) * (1/256)/2^32 ~ 1.4 uHz    (f_rf=7MHz, f_sys=125MHz)
```

**On the order of microhertz.** Orders of magnitude of headroom relative to the +-10Hz requirement. What determines accuracy is not the resolution but the crystal ppm (-> chapter 5).

### Noise shaping

Expressed with the normalized accumulator `e[k]=acc[k]/2^32`,

```
c[k] = w + (e[k-1] - e[k]) = w - delta_e[k]
```

The quantization error appears as a first difference (derivative) -> the noise transfer function is `NTF = 1 - z^-1` (high-pass). **The noise near the carrier is pushed down small**, and the error is driven to high frequencies. That is why the average is accurate and the vicinity is clean.

### Weakness of the 1st order and improvements

When `w` is a simple fraction, a regular pattern arises and produces discrete spurs. Since the update is about 1MHz, the spurs are distributed about 1MHz away from the carrier and have little effect near 7MHz. To suppress them further:

- **Make it 2nd order (MASH 1-1)** -> `NTF=(1-z^-1)^2` further reduces the near noise
- **Update faster** (but if the clkdiv rewrite is too fast, the divider keeps getting reset, so there is an upper limit)

---

## 4. Synchronization between cores

`loop1()` (core1) reads `g_w`, `g_reg0`, `g_reg1`, which `setFrequency()` (core0) writes, every microsecond.

- **A single 32-bit variable is atomic**: a 4-byte-aligned 32-bit read/write is one instruction (LDR/STR). No tearing occurs. `volatile` is added to forbid the compiler's register-caching optimization.
- **No cache-coherency problem**: the RP2040 has no data cache, and both cores access SRAM directly. A write by core0 is immediately visible to core1.
- **The only weakness = misalignment of a group of variables**: because the 3 variables are written separately, reading in between writes can give "new w + old reg." But the result is just using a slightly different division ratio for 1us, which is **harmless**. Frequency changes happen at the rate a person turns a knob, so it deliberately uses only volatile without locking (to avoid overhead in core1's fast loop).

For strict consistency:

| Method | Mechanism |
|---|---|
| Double buffering (recommended, lightweight) | Prepare 2 sets of structs, write to the unused side -> finally **atomically switch a 1-word index** |
| seqlock | Increment a counter before and after writing; the reader re-reads until before and after match |
| HW spinlock | The SIO hardware spinlock (SDK's mutex / critical_section) |
| Inter-core FIFO | Pass via `multicore_fifo_push/pop` |

```cpp
// Essence of double buffering
struct DivSet { uint32_t reg0, reg1, w; };
volatile DivSet buf[2];
volatile uint8_t active = 0;

// core0:
uint8_t nx = active ^ 1;
buf[nx] = {reg0, reg1, wfix};
active = nx;                          // only this 1-word switch is atomic
// core1:
const volatile DivSet &d = buf[active];   // always a consistent set
```

- **Startup order**: the `delay(50)` at the top of `setup1()` lets core0's first `setFrequency()` run first. In addition, `g_reg0/g_reg1` are given valid initial values from the start, so even if core1 runs ahead it never writes an invalid division ratio.
- **Ownership of the PIO register**: after running, only core1 ever writes clkdiv. The structure ensures the two cores never fight over the same PIO register, avoiding contention.

---

## 5. CAL_PPM: calibrating the absolute accuracy

### Why it is needed

(1)-(3) are **relative** computations based on `f_sys`. If the crystal behind `f_sys` is off by some ppm, the output is off too.

- Required accuracy +-10Hz @ 7MHz = **+-1.4ppm**
- The Pico's standard crystal is **+-10 to 30ppm (= +-70 to 210Hz)** -> the absolute accuracy cannot be met without calibration

`CAL_PPM` is the correction value that cancels the crystal error `e`.

```cpp
double corr = 1.0 + (CAL_PPM * 1e-6);
double div  = (fsys / (4.0 * freqHz)) * corr;   // multiply div by corr to cancel the crystal error
```

### The reasoning behind the computation

Let the actual system clock be `f_sys_true = f_sys_nom*(1+e)`. The corrected output is then

```
f_out = f_sys_true / (4*div)
      = f_indicated * (1 + e) / (1 + CAL_PPM*1e-6)
```

When `CAL_PPM*1e-6 = e`, `f_out = f_indicated` (a match).
That is, **`CAL_PPM`[ppm] = the crystal's relative error `e` x 10^6**.

### Procedure A: finding it from scratch (measure with CAL_PPM = 0)

1. Set `CAL_PPM = 0.0` and flash it.
2. Display the indicated frequency `f_ind` (e.g. 7,100,000Hz).
3. Measure the output `f_meas` (frequency counter, etc.).
4. Compute with the formula below and set `CAL_PPM`.

```
CAL_PPM = (f_meas - f_ind) / f_ind x 1,000,000
```

> Sign: **positive if measured is high**, negative if low.

**Example**: `f_ind = 7,100,000`, `f_meas = 7,100,100` (+100Hz)

```
CAL_PPM = 100 / 7,100,000 x 1e6 = +14.08
```

### Procedure B: refining from an already-set value (incremental)

Measure while keeping the current `CAL_PPM = C0` and add the residual. For a tiny residual, the following approximation is sufficient.

```
CAL_PPM_new ~ C0 + (f_meas - f_ind) / f_ind x 1,000,000
```

Exact formula (when the residual is large):

```
CAL_PPM_new = [ (f_meas / f_ind) x (1 + C0*1e-6) - 1 ] x 1,000,000
```

**Example**: `C0 = 14.08`, re-measuring `f_ind = 7,100,000`, `f_meas = 7,100,017` (+17Hz residual)

```
CAL_PPM_new ~ 14.08 + 17 / 7,100,000 x 1e6 = 14.08 + 2.39 = 16.47
```

-> the project's configured value `16.43` corresponds to the result of this "zero-calibration -> add the residual" process.

### Measurement tips

- **Reference**: a frequency counter (ideally GPS-disciplined / OCXO-referenced), a calibrated receiver, or a zero-beat against a standard radio signal such as WWV/JJY.
- **You may measure at any frequency**: the error is in ppm (proportional), so it is the same anywhere in the band. But reading more digits improves accuracy. At 7MHz with 1Hz resolution, the ppm resolution is `1/7e6 ~ 0.14ppm`.
- **Measure after warm-up**: a crystal moves by a few ppm with temperature. Let it sit a while after power-on before measuring.
- **It differs per unit**: due to crystal unit-to-unit variation, `CAL_PPM` differs per board. Re-measure when you change boards.
- **If long-term stability is needed, use a TCXO** to replace the XOSC. That reduces the temperature variation itself.

---

## Summary

- **The shape is the PIO instruction (the SET immediate)**: a 4-phase Gray code offsets I/Q by exactly 90 degrees.
- **The frequency is clkdiv**: `f_rf = f_sys/(4*clkdiv)`. But 1/256 steps are coarse.
- **The resolution is delta-sigma**: an average of two values makes a real-number division ratio, fixing the average frequency on the target (uHz resolution, clean vicinity).
- **Between cores, 32-bit atomicity + volatile is enough** (use double buffering for strictness).
- **The absolute accuracy is CAL_PPM**: `CAL_PPM = (f_meas - f_ind)/f_ind x 1e6` (add the residual). This is the only part that depends on the unit and temperature.
