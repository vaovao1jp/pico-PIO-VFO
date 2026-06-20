/*
 * RotaryEncoder.h
 * ----------------------------------------------------------------------------
 * ロータリーエンコーダ用 自作ライブラリ（Raspberry Pi Pico / arduino-pico 用）
 *
 *  - GPIO0 = A相, GPIO1 = B相 を想定（ピンは begin で指定可能）
 *  - 割り込み駆動 + 4-bit 遷移テーブルによるグリッチ耐性のある方式
 *  - クリック(デテント)単位で +1 / -1 を返す
 *
 * 使い方:
 *      RotaryEncoder enc(0, 1);     // A=GP0, B=GP1
 *      void setup(){ enc.begin(); }
 *      void loop(){ long d = enc.getSteps(); ... }
 * ----------------------------------------------------------------------------
 */
#pragma once
#include <Arduino.h>

class RotaryEncoder {
public:
  // pinA = A相, pinB = B相
  RotaryEncoder(uint8_t pinA, uint8_t pinB);

  // ピン初期化 + 割り込み登録
  void begin();

  // 前回呼び出しからのデテント差分（符号付き）。読むとゼロにリセットされる。
  long getSteps();

  // 起動からの絶対位置（デテント数）
  long position();

  // 割り込みハンドラ本体（内部用）
  void handleInterrupt();

  // エンコーダの 1 デテント当たりの遷移数。
  // 多くの機種は 4。半分しか進まない/倍進む場合は 2 に変更する。
  void setStepsPerDetent(uint8_t n) { _stepsPerDetent = n; }

private:
  uint8_t _pinA, _pinB;
  uint8_t _stepsPerDetent;
  uint8_t _prevState;
  volatile int8_t _subcount;
  volatile long   _position;
  volatile long   _delta;

  static RotaryEncoder* _instance;   // 単一インスタンス用 ISR トランポリン
  static void isrTrampoline();
};
