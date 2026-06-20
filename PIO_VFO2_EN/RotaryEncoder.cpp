/*
 * RotaryEncoder.cpp  —  custom rotary-encoder library implementation
 * ----------------------------------------------------------------------------
 * A "quadrature decode table" scheme: for the 2-bit Gray-code state (A,B),
 * concatenate the previous state (2 bits) and the current state (2 bits) into a
 * 4-bit index to look up +1/-1/0. Robust against chatter and missed steps, and
 * reliably determines CW/CCW.
 * ----------------------------------------------------------------------------
 */
#include "RotaryEncoder.h"

RotaryEncoder* RotaryEncoder::_instance = nullptr;

// Quadrature encoder transition table (index = (prevState<<2)|currState)
//   a valid 1-step advance = +1, step back = -1, otherwise (invalid/stopped) = 0
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

  // Use the state at startup as the initial value
  _prevState = (digitalRead(_pinA) << 1) | digitalRead(_pinB);

  _instance = this;
  attachInterrupt(digitalPinToInterrupt(_pinA), isrTrampoline, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinB), isrTrampoline, CHANGE);
}

void RotaryEncoder::isrTrampoline() {
  if (_instance) _instance->handleInterrupt();
}

void RotaryEncoder::handleInterrupt() {
  uint8_t s   = (digitalRead(_pinA) << 1) | digitalRead(_pinB);   // current (A,B)
  uint8_t idx = ((_prevState << 2) | s) & 0x0F;                   // 4-bit index
  _prevState  = s;

  _subcount += QEM[idx];

  if (_subcount >= _stepsPerDetent) {        // one detent CW
    _subcount = 0;
    _position++;
    _delta++;
  } else if (_subcount <= -(int8_t)_stepsPerDetent) {  // one detent CCW
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
