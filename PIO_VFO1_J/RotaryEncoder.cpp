/*
 * RotaryEncoder.cpp  —  自作ロータリーエンコーダライブラリ実装
 * ----------------------------------------------------------------------------
 * グレイコードの 2bit 状態 (A,B) について、前回状態(2bit)と今回状態(2bit)を
 * 連結した 4bit を添字にして +1/-1/0 を引く「直交デコードテーブル」方式。
 * チャタリングや取りこぼしに強く、CW/CCW を確実に判定できる。
 * ----------------------------------------------------------------------------
 */
#include "RotaryEncoder.h"

RotaryEncoder* RotaryEncoder::_instance = nullptr;

// 直交エンコーダ遷移テーブル（添字 = (前状態<<2)|今状態）
//   有効な 1 ステップ進み =+1, 戻り =-1, それ以外(無効/停止)=0
static const int8_t QEM[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

RotaryEncoder::RotaryEncoder(uint8_t pinA, uint8_t pinB)
: _pinA(pinA), _pinB(pinB),
  _stepsPerDetent(4),
  _prevState(0),
  _subcount(0),
  _position(0),
  _delta(0) {}

void RotaryEncoder::begin() {
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);

  // 起動時の状態を初期値に
  _prevState = (digitalRead(_pinA) << 1) | digitalRead(_pinB);

  _instance = this;
  attachInterrupt(digitalPinToInterrupt(_pinA), isrTrampoline, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinB), isrTrampoline, CHANGE);
}

void RotaryEncoder::isrTrampoline() {
  if (_instance) _instance->handleInterrupt();
}

void RotaryEncoder::handleInterrupt() {
  uint8_t s   = (digitalRead(_pinA) << 1) | digitalRead(_pinB);   // 今の (A,B)
  uint8_t idx = ((_prevState << 2) | s) & 0x0F;                   // 4bit 添字
  _prevState  = s;

  _subcount += QEM[idx];

  if (_subcount >= _stepsPerDetent) {        // 1 デテント分 CW
    _subcount = 0;
    _position++;
    _delta++;
  } else if (_subcount <= -(int8_t)_stepsPerDetent) {  // 1 デテント分 CCW
    _subcount = 0;
    _position--;
    _delta--;
  }
}

long RotaryEncoder::getSteps() {
  noInterrupts();
  long d = _delta;
  _delta = 0;
  interrupts();
  return d;
}

long RotaryEncoder::position() {
  noInterrupts();
  long p = _position;
  interrupts();
  return p;
}
