/*
 * PIO_VFO.ino  —  Raspberry Pi Pico PIO による 7MHz 帯 I/Q VFO
 * 2026年6月17日 JR3XNW
 * ============================================================================
 *  機能 / 要件対応
 *    - 7.000 〜 7.200 MHz を発振                           -> FREQ_MIN/MAX
 *    - ステップ 100Hz / 1000Hz をプッシュSWで切替             -> GP6
 *    - I/Q 90度位相差出力 (GP2=I, GP3=Q)                    -> PIO 4位相生成
 *    - 周波数精度 目標値 ±10Hz                               -> core1 clkdiv ディザ
 *    - ロータリーエンコーダで周波数可変 (GP0/GP1, 自作lib)      -> RotaryEncoder
 *    - 周波数表示 OLED 128x32 (U8g2lib.h)                  -> I2C0 GP4/GP5
 *
 *  対象コア: arduino-pico (Earle Philhower) ボードパッケージ
 *           ボード: "Raspberry Pi Pico" / "Pico 2"
 * ----------------------------------------------------------------------------
 *  ■ 周波数生成の仕組み（重要）
 *   PIO ステートマシンを 4*f_rf で回し、2本のピンに 4位相グレイコード
 *     00 -> 01 -> 11 -> 10 (=0,1,3,2)
 *   を出力する。これで GP2(I) と GP3(Q) が「ちょうど1サンプル=90度」ずれた
 *   方形波になる（位相差は原理的に正確）。
 *
 *   f_rf = f_sys / (4 * clkdiv)
 *
 *   clkdiv は 16bit整数 + 8bit小数(=1/256刻み) しかないため、7MHz では
 *   1LSB ≈ 6kHz の粗さになり ±10Hz を満たせない。そこで core1 で
 *   隣り合う2つの clkdiv 値を高速に切り替え（1次デルタシグマ/ディザ）、
 *   平均分周比を連続的に設定して目標周波数へ追い込む。
 *
 *  ■ 絶対精度について
 *   ±10Hz(=±1.4ppm) は Pico 標準水晶(±10〜30ppm)では校正なしに不可能。
 *   周波数カウンタ等で実測し CAL_PPM を合わせ込むこと。安定度が必要なら
 *   XOSC を TCXO へ置き換える。
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "RotaryEncoder.h"

// ----------------------------------------------------------------------------
// ピン定義
// ----------------------------------------------------------------------------
static const uint8_t PIN_ENC_A = 0;   // ロータリーエンコーダ A相
static const uint8_t PIN_ENC_B = 1;   // ロータリーエンコーダ B相
static const uint8_t PIN_RF_I  = 2;   // I 出力 (PIO)
static const uint8_t PIN_RF_Q  = 3;   // Q 出力 (PIO)  ※GP2の次の連番が必須
static const uint8_t PIN_SDA   = 4;   // OLED I2C SDA
static const uint8_t PIN_SCL   = 5;   // OLED I2C SCL
static const uint8_t PIN_STEP_SW = 6; // ステップ切替プッシュSW（GND押下）

// ----------------------------------------------------------------------------
// 周波数パラメータ
// ----------------------------------------------------------------------------
static const uint32_t FREQ_MIN  = 7000000UL;   // 7.000 MHz
static const uint32_t FREQ_MAX  = 7200000UL;   // 7.200 MHz
static const uint32_t FREQ_INIT = 7100000UL;   // 起動時 7.100 MHz

// 周波数校正(ppm)。実測周波数が高ければ正、低ければ負の値を入れる。
//   ppm = (実測 - 指示) / 指示 * 1e6
//   例) 指示7.100000MHz に対し実測が +100Hz 高い(7.100100MHz)場合:
//       100 / 7100000 * 1e6 = +14.08ppm  → CAL_PPM = 14.08
//   ※ 比例誤差なので 7.0〜7.2MHz 全域でほぼ同時に補正される。
//      より厳密に追い込むなら、自分の実測値で上式を計算して差し替えること。
static const double CAL_PPM = 16.43;

// 可変ステップ（100Hz / 1000Hz）
static const uint32_t STEP_A = 100UL;
static const uint32_t STEP_B = 1000UL;

// ----------------------------------------------------------------------------
// PIO 設定
//   ハンドアセンブルした 4命令プログラム（set pins, ...）
//   SET PINS, d = 0xE000 | d   （ディレイ/サイドセット無し）
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
// core0 <-> core1 共有（clkdiv ディザ用）
//   g_reg0 / g_reg1 : 隣接する2つの clkdiv レジスタ値
//   g_w             : g_reg1 を選ぶ確率（32bit 固定小数, 0..0xFFFFFFFF）
// ----------------------------------------------------------------------------
volatile uint32_t g_reg0 = (4u << 16) | (118u << 8);  // 安全な初期値(約4.46)
volatile uint32_t g_reg1 = (4u << 16) | (119u << 8);
volatile uint32_t g_w    = 0;

// ----------------------------------------------------------------------------
// グローバル状態
// ----------------------------------------------------------------------------
RotaryEncoder enc(PIN_ENC_A, PIN_ENC_B);

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

uint32_t g_freq = FREQ_INIT;
uint32_t g_step = STEP_A;
bool     g_dirty = true;

// ============================================================================
// 周波数設定: 目標 f_rf から clkdiv 2値 + 重み w を計算して共有変数へ
// ============================================================================
void setFrequency(uint32_t freqHz) {
  double fsys = (double)clock_get_hz(clk_sys);          // 実 system clock
  double corr = 1.0 + (CAL_PPM * 1e-6);                 // 校正係数

  // f_rf = fsys / (4*div)  →  div = fsys / (4*f_rf)
  // 校正: 実周波数を corr 倍ずれと仮定し、div を corr 倍して打ち消す
  double div = (fsys / (4.0 * (double)freqHz)) * corr;

  if (div < 1.0)    div = 1.0;        // clkdiv 下限
  if (div > 65535.0) div = 65535.0;   // 整数部 16bit 上限

  // div を 1/256 単位へ。n = 下側の clkdiv(×256), w = n と n+1 の按分
  double scaled = div * 256.0;
  uint32_t n    = (uint32_t)floor(scaled);
  double   w    = scaled - (double)n;          // 0..1

  uint32_t n1 = n + 1;

  uint16_t i0 = (uint16_t)(n  >> 8), f0 = (uint16_t)(n  & 0xFF);
  uint16_t i1 = (uint16_t)(n1 >> 8), f1 = (uint16_t)(n1 & 0xFF);

  // RP2040 CLKDIV レジスタ: [31:16]=整数, [15:8]=小数, [7:0]=0
  uint32_t reg0 = ((uint32_t)i0 << 16) | ((uint32_t)f0 << 8);
  uint32_t reg1 = ((uint32_t)i1 << 16) | ((uint32_t)f1 << 8);

  // core1 が安全に拾えるよう w → reg を順に更新
  g_w    = (uint32_t)(w * 4294967296.0);   // 2^32
  g_reg0 = reg0;
  g_reg1 = reg1;
}

// ============================================================================
// PIO 初期化
// ============================================================================
void vfoPioInit() {
  g_pio_offset = pio_add_program(VFO_PIO, &quad_program);

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, g_pio_offset + 0, g_pio_offset + 3);
  sm_config_set_set_pins(&c, PIN_RF_I, 2);   // SET 対象 = GP2,GP3

  // GPIO を PIO 制御下へ
  pio_gpio_init(VFO_PIO, PIN_RF_I);
  pio_gpio_init(VFO_PIO, PIN_RF_Q);
  pio_sm_set_consecutive_pindirs(VFO_PIO, VFO_SM, PIN_RF_I, 2, true); // 出力

  pio_sm_init(VFO_PIO, VFO_SM, g_pio_offset, &c);

  // 初期周波数の clkdiv を反映してから始動
  setFrequency(g_freq);
  VFO_PIO->sm[VFO_SM].clkdiv = g_reg0;

  pio_sm_set_enabled(VFO_PIO, VFO_SM, true);
}

// ============================================================================
// OLED 表示
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
  u8g2.setFont(u8g2_font_profont17_tr);   // 数字大きめ
  u8g2.drawStr(2, 16, fbuf);
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(2, 30, sbuf);
  u8g2.drawStr(96, 30, "MHz");
  u8g2.sendBuffer();
}

// ============================================================================
// ステップ切替スイッチ（簡易デバウンス, 立ち下がりトグル）
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
    if (lastStable == LOW) return true;   // 押された瞬間
  }
  return false;
}

// ============================================================================
// core0: setup / loop  （UI 全般）
// ============================================================================
void setup() {
  pinMode(PIN_STEP_SW, INPUT_PULLUP);

  // OLED (I2C0)
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();
  u8g2.setBusClock(400000);
  u8g2.begin();

  // エンコーダ
  enc.begin();

  // PIO VFO 始動
  vfoPioInit();

  drawDisplay();
}

void loop() {
  bool changed = false;

  // --- エンコーダ ---
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

  // --- ステップ切替SW ---
  if (readStepButton()) {
    g_step = (g_step == STEP_A) ? STEP_B : STEP_A;
    changed = true;
  }

  if (changed) drawDisplay();
}

// ============================================================================
// core1: clkdiv ディザリング（1次デルタシグマ）
//   acc に重み w を加算し、桁上がりした時だけ上側 clkdiv(reg1) を選ぶ。
//   平均分周比 = reg0 + w*(1/256) となり、周波数を連続的に追い込める。
//   1us 間隔で更新（切替スプリアスは ~1MHz 離れに出るので 7MHz から十分離れる）。
// ============================================================================
void setup1() {
  // core0 の setFrequency が走るのを少し待つ
  delay(50);
}

void loop1() {
  static uint32_t acc = 0;

  uint32_t w  = g_w;
  uint32_t r0 = g_reg0;
  uint32_t r1 = g_reg1;

  uint32_t old = acc;
  acc += w;
  uint32_t reg = (acc < old) ? r1 : r0;   // 桁上がり検出

  VFO_PIO->sm[VFO_SM].clkdiv = reg;

  delayMicroseconds(1);
}
