# PIO_VFO — Raspberry Pi Pico PIO 7MHz I/Q VFO

ラズベリーパイ Pico の PIO を使った 7MHz 帯 直交(I/Q)VFO。

## 機能 / 要件対応

| 要件 | 実装 |
|------|------|
| 7.000〜7.200 MHz を発振 | `FREQ_MIN` / `FREQ_MAX` |
| ステップ 100Hz / 1000Hz をプッシュSWで切替 | `PIN_STEP_SW = GP6` でトグル |
| I/Q 90度位相差出力 | PIO で 4位相グレイコードを GP2(I)/GP3(Q) へ |
| 精度 指示値 ±10Hz | core1 による clkdiv 高速ディザリング + `CAL_PPM` 校正 |
| ロータリーエンコーダで可変（自作lib, GP0/GP1）| `RotaryEncoder.h/.cpp` |
| 周波数表示 OLED128x32（U8g2lib.h）| `U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C` |

## 配線

```
ロータリーエンコーダ  A  --- GP0
                      B  --- GP1
                      C(COM) --- GND
ステップ切替プッシュSW    --- GP6  ---|押下でGND|--- GND   (内蔵プルアップ)

RF 出力  I --- GP2
         Q --- GP3   (※ I/Q ローパス→バッファ/レベル変換を推奨。3.3Vロジック方形波)

OLED(SSD1306 128x32, I2C)
        SDA --- GP4
        SCL --- GP5
        VCC --- 3V3
        GND --- GND
```

> GP2/GP3 は 3.3V ロジックの方形波です。受信機の局発として使う場合は
> 段間に LPF / コンパレータ or バッファ(74AC04 等)を入れてください。

## ビルド環境

- **arduino-pico (Earle Philhower) コア**（PIO/SDK API・dualコア・`Wire.setSDA/SCL` 対応）
- ライブラリ: **U8g2** (ライブラリマネージャからインストール)
- ボード: `Raspberry Pi Pico` または `Raspberry Pi Pico 2`
- 4ファイル(`PIO_VFO.ino` / `RotaryEncoder.h` / `RotaryEncoder.cpp` / `README.md`)を
  同じスケッチフォルダに置いてコンパイル。

> PIO プログラムは `.pio` を別途用意せず、`PIO_VFO.ino` 内に
> ハンドアセンブル済み命令配列として埋め込んでいる（pioasm 不要）。

## 周波数生成の原理

```
f_rf = f_sys / (4 * clkdiv)
```

PIO を 4×f_rf で回し、2ピンに `00→01→11→10` のグレイコードを出力。
GP2(I) と GP3(Q) は常に 1サンプル = 90度 ずれた方形波になる（位相差は厳密）。

clkdiv は 1/256 刻みで 7MHz では 1LSB≈6kHz と粗い。そこで **core1** が
隣接する2つの clkdiv 値を 1次デルタシグマで高速切替し、**平均**分周比を
連続可変にして目標周波数へ追い込む（±10Hz 達成）。

## 校正（±10Hz を満たすために必須）

±10Hz = **±1.4ppm**。Pico 標準水晶は ±10〜30ppm(=±70〜210Hz@7MHz) あるため、
**校正なしでは絶対精度の要件を満たせません**。

1. `g_freq` を例えば 7.100000MHz に設定。
2. 周波数カウンタ / 校正された受信機で実測。
3. 誤差 ppm を計算: `ppm = (実測 - 指示) / 指示 * 1e6`
4. `PIO_VFO.ino` の `CAL_PPM` にその値を入れて再書き込み。

長期安定度・温度安定度が必要なら Pico の XOSC を **TCXO** に換装してください。

## 注意 / 既知の制約

- ディザ切替による微小スプリアスが搬送波から ~1MHz 程度離れた所に出ます。
  狭帯域用途で気になる場合は core1 を 2次(MASH)デルタシグマ化、もしくは
  `delayMicroseconds` を調整してください。
- エンコーダが 1 デテントで 2 カウント等になる場合は
  `enc.setStepsPerDetent(2);` を `setup()` で呼んで調整。
