/*
 * RotaryEncoder.h
 * ----------------------------------------------------------------------------
 * Custom rotary-encoder library (for Raspberry Pi Pico / arduino-pico)
 *
 *  - Assumes GPIO0 = phase A, GPIO1 = phase B (pins can be specified in begin)
 *  - Interrupt-driven + glitch-resistant scheme using a 4-bit transition table
 *  - Returns +1 / -1 per click (detent)
 *
 * Usage:
 *      RotaryEncoder enc(0, 1);     // A=GP0, B=GP1
 *      void setup(){ enc.begin(); }
 *      void loop(){ long d = enc.getSteps(); ... }
 * ----------------------------------------------------------------------------
 */
#pragma once
#include <Arduino.h>

class RotaryEncoder {
public:
  // pinA = phase A, pinB = phase B
  RotaryEncoder(uint8_t pinA, uint8_t pinB);

  // Pin initialization + interrupt registration
  void begin();

  // Detent delta since the last call (signed). Reading it resets it to zero.
  long getSteps();

  // Absolute position since startup (number of detents)
  long position();

  // The interrupt handler body (internal use)
  void handleInterrupt();

  // Number of transitions per detent of the encoder.
  // Many models use 4. If it advances only half / advances double, change it to 2.
  void setStepsPerDetent(uint8_t n) { _stepsPerDetent = n; }

private:
  uint8_t _pinA, _pinB;
  uint8_t _stepsPerDetent;
  uint8_t _prevState;
  volatile int8_t _subcount;
  volatile long   _position;
  volatile long   _delta;

  static RotaryEncoder* _instance;   // ISR trampoline for a single instance
  static void isrTrampoline();
};
