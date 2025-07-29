// STLM20W87F temperature sensor reader for ESP‑WROOM‑32E (GPIO39 / SENSOR_VN)
//
// ────────────────────────── Wiring ────────────────────────────
// STLM20W87F VCC  → 3V3 (2.4–5.5 V)
// STLM20W87F GND  → GND
// STLM20W87F VOUT → GPIO39 (ADC1_CH3 / SENSOR_VN) — input‑only pin
// Optional 0.1 µF bypass capacitor across VCC & GND.  Add RC filtering
// close to the sensor if your power stage is very noisy.
//
// ───────────────────── Board / core settings ──────────────────
// • Select an "ESP32" board in Arduino IDE.
// • *Recommended*: Arduino‑ESP32 core v2.0.5 or newer — gives
//   analogReadMilliVolts() **and** analogSetPinAttenuation().
// • *Older cores (≤1.0.x)* are still supported; the sketch falls back to
//   the legacy analogSetAttenuation() API automatically (see below).
// • No external libraries required.
//
// ─────────────────── Theory of operation (recap) ───────────────
// The STLM20 outputs a voltage roughly linear with temperature.  At VCC ≈3.3 V
//   VOUT ≈ –11.77 mV / °C × T  +  1.8577 V       (datasheet Table 2, –40…110 °C)
// Inverse:  T = (1.8577 V – VOUT) ⁄ 0.01177 V/°C.
// A 2nd‑order model covers –55…130 °C (see functions below).
//
// ───────────────────────── Includes ───────────────────────────
#include <Arduino.h>
#include <math.h>

// Newer cores already pull this in via <Arduino.h>, but older ones don’t.
#if __has_include("esp32-hal-adc.h")
  #include "esp32-hal-adc.h"   // adc_atten_t, ADC_11db, analogSetPinAttenuation()
#elif __has_include("driver/adc.h")
  #include "driver/adc.h"      // fallback (IDF style)
#endif

// ───────────────────── Sketch configuration ───────────────────
const int     VTEMP_PIN    = 39;   // GPIO39 / ADC1_CH3
const uint8_t NUM_SAMPLES  = 16;   // simple moving average to tame noise

// ─────────────── Helper: choose best attenuation API ──────────
#ifndef USE_PIN_ATTEN
  #if defined(analogSetPinAttenuation)
    #define USE_PIN_ATTEN 1   // per‑pin API available (core ≥2.0.5)
  #else
    #define USE_PIN_ATTEN 0   // fall back to global API
  #endif
#endif

// ──────────────────────── Utilities ───────────────────────────
static uint32_t readMilliVoltsAveraged(uint8_t samples = NUM_SAMPLES) {
  uint64_t sum = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    sum += analogReadMilliVolts(VTEMP_PIN);
  }
  return sum / samples;
}

// Linear approximation, –40 °C … 110 °C
static inline float temperatureC_linear(float v) {
  // VOUT = –11.77 mV/°C * T + 1.8577 V  →  T = (1.8577 – V)/0.01177
  return (1.8577f - v) / 0.01177f;
}

// 2nd‑order polynomial (better for –55 °C … 130 °C)
static inline float temperatureC_parabolic(float v) {
  const float a = -3.88e-6f;
  const float b = -1.15e-2f;
  const float c = 1.8639f - v;
  float disc = b * b - 4.0f * a * c;
  if (disc < 0) return NAN;               // outside sensor limits
  return (-b - sqrtf(disc)) / (2.0f * a); // negative root is physical
}

// ────────────────── Arduino setup / loop ──────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);   // let USB‑CDC come up

  // Configure ADC attenuation so the full‑scale voltage covers ≈3.3 V.
  #if USE_PIN_ATTEN
    analogSetPinAttenuation(VTEMP_PIN, ADC_11db);   // per‑pin (core ≥2.0.5)
  #else
    analogSetAttenuation(ADC_11db);                 // global (old cores)
  #endif

  Serial.println("STLM20W87F temperature demo\n");
}

void loop() {
  uint32_t mV = readMilliVoltsAveraged();
  float v  = mV / 1000.0f;                // volts
  float tC = temperatureC_linear(v);      // choose linear or parabolic

  Serial.printf("Vout: %.4f V → %.2f °C\n", v, tC);
  delay(100);
}
