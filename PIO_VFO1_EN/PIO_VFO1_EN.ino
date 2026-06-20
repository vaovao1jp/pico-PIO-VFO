/*
 * PIO_VFO.ino  —  7MHz-band I/Q VFO using the Raspberry Pi Pico PIO
 * June 17, 2026  JR3XNW
 * ============================================================================
 *  Features / requirement mapping
 *    - Oscillates 7.000 - 7.200 MHz                        -> FREQ_MIN/MAX
 *    - Step 100Hz / 1000Hz toggled by a push switch        -> GP6
 *    - I/Q 90-degree quadrature output (GP2=I, GP3=Q)      -> PIO 4-phase generation
 *    - Frequency accuracy target +-10Hz                    -> core1 clkdiv dither
 *    - Frequency tuning by rotary encoder (GP0/GP1, custom lib) -> RotaryEncoder
 *    - Frequency display OLED 128x32 (U8g2lib.h)           -> I2C0 GP4/GP5
 *
 *  Target core: arduino-pico (Earle Philhower) board package
 *           Board: "Raspberry Pi Pico" / "Pico 2"
 * ----------------------------------------------------------------------------
 *  How the frequency is generated (important)
 *   The PIO state machine runs at 4*f_rf and outputs a 4-phase Gray code
 *     00 -> 01 -> 11 -> 10 (=0,1,3,2)
 *   on two pins. This makes GP2(I) and GP3(Q) square waves that are offset by
 *   exactly one sample = 90 degrees (the phase difference is exact by design).
 *
 *   f_rf = f_sys / (4 * clkdiv)
 *
 *   clkdiv has only a 16-bit integer + 8-bit fractional part (1/256 steps), so
 *   at 7MHz 1 LSB is about 6kHz coarse and cannot meet +-10Hz. Therefore core1
 *   rapidly switches between two adjacent clkdiv values (1st-order delta-sigma /
 *   dither) to set the average division ratio continuously and home in on the
 *   target frequency.
 *
 *  About absolute accuracy
 *   +-10Hz (= +-1.4ppm) is impossible without calibration using the Pico's
 *   standard crystal (+-10 to 30ppm). Measure with a frequency counter etc. and
 *   tune CAL_PPM. If stability is needed, replace the XOSC with a TCXO.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "RotaryEncoder.h"

// ----------------------------------------------------------------------------
// Pin definitions
// ----------------------------------------------------------------------------
static const uint8_t PIN_ENC_A = 0;   // Rotary encoder phase A
static const uint8_t PIN_ENC_B = 1;   // Rotary encoder phase B
static const uint8_t PIN_RF_I  = 2;   // I output (PIO)
static const uint8_t PIN_RF_Q  = 3;   // Q output (PIO)  *must be the next consecutive pin after GP2
static const uint8_t PIN_SDA   = 4;   // OLED I2C SDA
static const uint8_t PIN_SCL   = 5;   // OLED I2C SCL
static const uint8_t PIN_STEP_SW = 6; // Step-toggle push switch (pressed = GND)

// ----------------------------------------------------------------------------
// Frequency parameters
// ----------------------------------------------------------------------------
static const uint32_t FREQ_MIN  = 7000000UL;   // 7.000 MHz
static const uint32_t FREQ_MAX  = 7200000UL;   // 7.200 MHz
static const uint32_t FREQ_INIT = 7100000UL;   // 7.100 MHz at startup

// Frequency calibration (ppm). Positive if the measured frequency is high, negative if low.
//   ppm = (measured - indicated) / indicated * 1e6
//   e.g.) when the measured value is +100Hz higher (7.100100MHz) than the indicated 7.100000MHz:
//       100 / 7100000 * 1e6 = +14.08ppm  -> CAL_PPM = 14.08
//   * Since it is a proportional error, the entire 7.0-7.2MHz range is corrected almost simultaneously.
//      For a tighter result, compute the formula above with your own measured value and replace it.
static const double CAL_PPM = 16.43;

// Tuning step (100Hz / 1000Hz)
static const uint32_t STEP_A = 100UL;
static const uint32_t STEP_B = 1000UL;

// ----------------------------------------------------------------------------
// PIO configuration
//   Hand-assembled 4-instruction program (set pins, ...)
//   SET PINS, d = 0xE000 | d   (no delay / side-set)
// ----------------------------------------------------------------------------
static const uint16_t quad_instr[] = {
  0xe000,   // set pins, 0b00
  0xe001,   // set pins, 0b01   (I=1,Q=0)
  0xe003,   // set pins, 0b11   (I=1,Q=1)
  0xe002,   // set pins, 0b10   (I=0,Q=1)
};
static const struct pio_program quad_program = {
  .instructions = quad_instr,
  .length       = 4,
  .origin       = -1,
};

#define VFO_PIO  pio0
#define VFO_SM   0
static uint   g_pio_offset = 0;

// ----------------------------------------------------------------------------
// core0 <-> core1 shared (for clkdiv dithering)
//   g_reg0 / g_reg1 : two adjacent clkdiv register values
//   g_w             : probability of choosing g_reg1 (32-bit fixed point, 0..0xFFFFFFFF)
// ----------------------------------------------------------------------------
volatile uint32_t g_reg0 = (4u << 16) | (118u << 8);  // safe initial value (about 4.46)
volatile uint32_t g_reg1 = (4u << 16) | (119u << 8);
volatile uint32_t g_w    = 0;

// ----------------------------------------------------------------------------
// Global state
// ----------------------------------------------------------------------------
RotaryEncoder enc(PIN_ENC_A, PIN_ENC_B);

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

uint32_t g_freq = FREQ_INIT;
uint32_t g_step = STEP_A;
bool     g_dirty = true;

// ============================================================================
// Set frequency: from the target f_rf, compute the two clkdiv values + weight w
// and store them into the shared variables.
// ============================================================================
void setFrequency(uint32_t freqHz) {
  double fsys = (double)clock_get_hz(clk_sys);          // actual system clock
  double corr = 1.0 + (CAL_PPM * 1e-6);                 // calibration factor

  // f_rf = fsys / (4*div)  ->  div = fsys / (4*f_rf)
  // Calibration: assume the actual frequency is off by a factor of corr, and multiply div by corr to cancel it
  double div = (fsys / (4.0 * (double)freqHz)) * corr;

  if (div < 1.0)    div = 1.0;        // clkdiv lower limit
  if (div > 65535.0) div = 65535.0;   // 16-bit upper limit of the integer part

  // Convert div to 1/256 units. n = lower clkdiv (x256), w = interpolation between n and n+1
  double scaled = div * 256.0;
  uint32_t n    = (uint32_t)floor(scaled);
  double   w    = scaled - (double)n;          // 0..1

  uint32_t n1 = n + 1;

  uint16_t i0 = (uint16_t)(n  >> 8), f0 = (uint16_t)(n  & 0xFF);
  uint16_t i1 = (uint16_t)(n1 >> 8), f1 = (uint16_t)(n1 & 0xFF);

  // RP2040 CLKDIV register: [31:16]=integer, [15:8]=fraction, [7:0]=0
  uint32_t reg0 = ((uint32_t)i0 << 16) | ((uint32_t)f0 << 8);
  uint32_t reg1 = ((uint32_t)i1 << 16) | ((uint32_t)f1 << 8);

  // Update w then the reg values in order so core1 can pick them up safely
  g_w    = (uint32_t)(w * 4294967296.0);   // 2^32
  g_reg0 = reg0;
  g_reg1 = reg1;
}

// ============================================================================
// PIO initialization
// ============================================================================
void vfoPioInit() {
  g_pio_offset = pio_add_program(VFO_PIO, &quad_program);

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, g_pio_offset + 0, g_pio_offset + 3);
  sm_config_set_set_pins(&c, PIN_RF_I, 2);   // SET targets = GP2,GP3

  // Place the GPIOs under PIO control
  pio_gpio_init(VFO_PIO, PIN_RF_I);
  pio_gpio_init(VFO_PIO, PIN_RF_Q);
  pio_sm_set_consecutive_pindirs(VFO_PIO, VFO_SM, PIN_RF_I, 2, true); // output

  pio_sm_init(VFO_PIO, VFO_SM, g_pio_offset, &c);

  // Apply the initial frequency's clkdiv before starting
  setFrequency(g_freq);
  VFO_PIO->sm[VFO_SM].clkdiv = g_reg0;

  pio_sm_set_enabled(VFO_PIO, VFO_SM, true);
}

// ============================================================================
// OLED display
// ============================================================================
void drawDisplay() {
  char fbuf[16];
  char sbuf[16];

  // 7100000 -> "7.100.000"
  uint32_t f = g_freq;
  snprintf(fbuf, sizeof(fbuf), "%lu.%03lu.%03lu",
           (unsigned long)(f / 1000000UL),
           (unsigned long)((f / 1000UL) % 1000UL),
           (unsigned long)(f % 1000UL));

  if (g_step >= 1000) snprintf(sbuf, sizeof(sbuf), "STEP: 1 kHz");
  else                snprintf(sbuf, sizeof(sbuf), "STEP: 100 Hz");

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_profont17_tr);   // larger digits
  u8g2.drawStr(2, 16, fbuf);
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(2, 30, sbuf);
  u8g2.drawStr(96, 30, "MHz");
  u8g2.sendBuffer();
}

// ============================================================================
// Step-toggle switch (simple debounce, toggle on falling edge)
// ============================================================================
bool readStepButton() {
  static bool     lastStable = HIGH;
  static bool     lastRead   = HIGH;
  static uint32_t lastChange = 0;

  bool r = digitalRead(PIN_STEP_SW);
  uint32_t now = millis();

  if (r != lastRead) { lastRead = r; lastChange = now; }

  if ((now - lastChange) > 20 && r != lastStable) {
    lastStable = r;
    if (lastStable == LOW) return true;   // the moment it is pressed
  }
  return false;
}

// ============================================================================
// core0: setup / loop  (all UI)
// ============================================================================
void setup() {
  pinMode(PIN_STEP_SW, INPUT_PULLUP);

  // OLED (I2C0)
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();
  u8g2.setBusClock(400000);
  u8g2.begin();

  // Encoder
  enc.begin();

  // Start the PIO VFO
  vfoPioInit();

  drawDisplay();
}

void loop() {
  bool changed = false;

  // --- Encoder ---
  long d = enc.getSteps();
  if (d != 0) {
    int64_t nf = (int64_t)g_freq + (int64_t)d * (int64_t)g_step;
    if (nf < (int64_t)FREQ_MIN) nf = FREQ_MIN;
    if (nf > (int64_t)FREQ_MAX) nf = FREQ_MAX;
    if ((uint32_t)nf != g_freq) {
      g_freq = (uint32_t)nf;
      setFrequency(g_freq);
      changed = true;
    }
  }

  // --- Step-toggle switch ---
  if (readStepButton()) {
    g_step = (g_step == STEP_A) ? STEP_B : STEP_A;
    changed = true;
  }

  if (changed) drawDisplay();
}

// ============================================================================
// core1: clkdiv dithering (1st-order delta-sigma)
//   Add weight w to acc, and select the upper clkdiv (reg1) only when it carries.
//   The average division ratio = reg0 + w*(1/256), so the frequency can be homed in continuously.
//   Updated every 1us (switching spurs appear ~1MHz away, so they are far enough from 7MHz).
// ============================================================================
void setup1() {
  // Wait a little for core0's setFrequency to run
  delay(50);
}

void loop1() {
  static uint32_t acc = 0;

  uint32_t w  = g_w;
  uint32_t r0 = g_reg0;
  uint32_t r1 = g_reg1;

  uint32_t old = acc;
  acc += w;
  uint32_t reg = (acc < old) ? r1 : r0;   // carry detection

  VFO_PIO->sm[VFO_SM].clkdiv = reg;

  delayMicroseconds(1);
}
