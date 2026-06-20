# VFO using the PIO of the Raspberry Pi Pico / ラズベリーパイ pico の PIO で VFO
## JR3XNW 20/june/2026

---

## Introduction: about the folders / はじめに：フォルダの説明

**EN**

- `PIO_VFO1_EN` -> English version
- `PIO_VFO2_EN` -> English version
- `PIO_VFO1_J` -> Japanese version
- `PIO_VFO2_J` -> Japanese version

**JA / 日本語**

- `PIO_VFO1_EN` →英語版
- `PIO_VFO2_EN` →英語版
- `PIO_VFO1_J` →日本語版
- `PIO_VFO2_J` →日本語版

---

## About each program / 各プログラムについて

**EN**

`PIO_VFO1` is the basic PIO oscillation program; the display shows only the frequency and the step frequency. It uses a 128x32 OLED.

`PIO_VFO2` has no change to the PIO oscillation program; the display adds a dial indication to the frequency and step frequency. It uses a 128x64 OLED.

**JA / 日本語**

`PIO_VFO1` は、PIO発振プログラムの基本であり、ディスプレイは周波数表示とステップ周波数のみ。OLED 128x32 を使用。

`PIO_VFO2` は、PIO発振プログラムに変更はなく、ディスプレイは周波数表示とステップ周波数にダイアル表示を追加。OLED 128x64 を使用。

---

## Detailed oscillation control / 詳しい発振制御について

**EN**

`PIO_VFO2_EN` folder:
`PIO_oscillation_control_explanation.md` is the English explanation of the oscillation control.

`PIO_VFO2_J` folder:
`PIO発振制御_解説.md` is the Japanese explanation of the oscillation control.

The `README.md` in each folder explains how to use it.

Among these, calibration is especially important. By accurately measuring the VFO's oscillation frequency with a frequency counter or the like and calibrating it, you can compensate for the clock oscillation-frequency differences caused by Pico unit-to-unit variation, and accurate-frequency oscillation can be expected!

Details are also described in the sketch comments, so please refer to them.

**JA / 日本語**

`PIO_VFO2_EN` フォルダ：
`PIO_oscillation_control_explanation.md` は、英語版の発振制御解説文

`PIO_VFO2_J` フォルダ：
`PIO発振制御_解説.md` は、日本語版の発振制御解説文

各フォルダにある `README.md` は使い方の説明です。

なかでも、特に校正が重要です。Pico の個体差によるクロック発振周波数の違いを、VFO の発振周波数を周波数カウンタなどで正確に測定し、校正することで、正確な周波数の発振が期待できます！

スケッチのコメントでも詳しく記述しているので参考にしてください。

---

## Supplementary note / 補足事項

### Theoretical limit in hardware/algorithm (oscillatable frequency range) / ハードウェア／アルゴリズム上の理論限界（発振可能周波数範囲）

**EN**

The actual limit is determined by the clamping of `clkdiv` and the system clock `f_sys`.

- div lower limit = 1.0 -> maximum frequency = `f_sys / 4`
- div upper limit = 65535 -> minimum frequency = `f_sys / (4 x 65535)`

Since `f_sys` depends on the board's default operating clock:

**JA / 日本語**

実際の限界は `clkdiv` のクランプと system clock `f_sys` で決まる。

- div の下限 = 1.0 → 最高周波数 = `f_sys / 4`
- div の上限 = 65535 → 最低周波数 = `f_sys / (4 × 65535)`

`f_sys` はボードのデフォルト動作クロックに依存するので、

#### For the Raspberry Pi Pico (RP2040, f_sys = 125 MHz) / Raspberry Pi Pico（RP2040, f_sys = 125 MHz）の場合

| | Calculation / 計算 | Frequency / 周波数 |
|---|---|---|
| Maximum / 最高 | 125 MHz ÷ 4 | 31.25 MHz |
| Minimum / 最低 | 125 MHz ÷ (4×65535) | approx. 477 Hz / 約 477 Hz |

#### For the Raspberry Pi Pico 2 (RP2350, f_sys = 150 MHz) / Raspberry Pi Pico 2（RP2350, f_sys = 150 MHz）の場合

| | Calculation / 計算 | Frequency / 周波数 |
|---|---|---|
| Maximum / 最高 | 150 MHz ÷ 4 | 37.5 MHz |
| Minimum / 最低 | 150 MHz ÷ (4×65535) | approx. 572 Hz / 約 572 Hz |
