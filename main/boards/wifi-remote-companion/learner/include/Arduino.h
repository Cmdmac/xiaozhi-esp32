// Arduino.h - Minimal Arduino-compatibility shim for the IRremoteESP8266
// (learner) port to ESP-IDF 5.5.2 / ESP32-C3.
//
// Design notes:
//   * Reuses the shims already provided by HeatpumpIRCompat.h (micros/millis/
//     delay/pinMode/digitalRead/digitalWrite/INPUT/OUTPUT/HIGH/LOW/boolean/
//     byte/PROGMEM/PSTR/F...), so this header INCLUDES it instead of
//     redefining.  Defining the HeatpumpIRCompat include guard here is NOT
//     needed: the guard inside HeatpumpIRCompat.h handles double inclusion.
//   * Declares the ESP32 Arduino core version macros so that the
//     IRremoteESP8266 ESP32 code takes the "core v3" path (V3PLUS).  That path
//     only needs timerBegin(freq)/timerWrite/timerAlarm/timerAttachInterrupt/
//     timerDetachInterrupt/timerEnd and hw_timer_t, which we provide on top of
//     the IDF gptimer driver (the raw-register hack of old Arduino cores does
//     not match the ESP32-C3 timer group layout and is compiled out).
//   * attachInterrupt()/detachInterrupt() map onto the IDF GPIO ISR service
//     with a tiny dispatch table (IRrecv's ISR is already USE_IRAM_ATTR).
//
// The ISR-context calls (gpio_intr -> timerWrite/timerAlarm) go through the
// gptimer driver; build with CONFIG_GPTIMER_ISR_IRAM_SAFE=y so those driver
// functions are IRAM-resident and safe to call from an ISR.

#ifndef ARDUINO_H_SHIM
#define ARDUINO_H_SHIM

#include "HeatpumpIRCompat.h"  // micros/millis/delay/pinMode/digitalRead/... + PROGMEM/PSTR/F/...

#include <string>
#include <cstdint>
#include <cassert>

#include <esp_system.h>       // esp_restart()
#include <esp_attr.h>         // IRAM_ATTR
#include <driver/gpio.h>
#include "driver/gptimer.h"

// ---------------------------------------------------------------------------
// ESP32 Arduino core version - force "core v3" semantics for IRremoteESP8266.
// NOTE: IDF does not define the `ESP32` macro for ESP32-C3, but the
// IRremoteESP8266 ESP32 capture code is guarded by `#if defined(ESP32)`.  We
// define it here so that code compiles on this target.
#ifndef ESP32
#define ESP32 1
#endif

#ifndef ESP_ARDUINO_VERSION_VAL
#define ESP_ARDUINO_VERSION_VAL(major, minor, patch) (((major) << 16) | ((minor) << 8) | (patch))
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 3
#define ESP_ARDUINO_VERSION_MINOR 0
#define ESP_ARDUINO_VERSION_PATCH 0
#define ESP_ARDUINO_VERSION ESP_ARDUINO_VERSION_VAL(3, 0, 0)
#endif

// IRremoteESP8266.h aliases String to std::string; keep a matching alias here
// for translation units that include <Arduino.h> but not that header first.
#ifndef String
typedef std::string String;
#endif

// IRtext.h forward-declares this type; provide the declaration for IRtext.cpp.
class __FlashStringHelper;

// GPIO modes (INPUT/OUTPUT/HIGH/LOW come from HeatpumpIRCompat.h)
#ifndef INPUT_PULLUP
#define INPUT_PULLUP 0x10
#endif

// Interrupt mode constants (values are opaque; the shim always uses ANYEDGE)
#ifndef CHANGE
#define CHANGE 1
#endif
#ifndef RISING
#define RISING 2
#endif
#ifndef FALLING
#define FALLING 3
#endif

// ESP.restart() used by IRrecv on fatal allocation failure.
// Arduino defines `EspClass` and a global instance `ESP`; IRrecv calls
// `ESP.restart()` with a dot, so we need an object, not a namespace.
class EspClass {
 public:
  static void restart() { esp_restart(); }
};
static EspClass ESP;

// ---------------------------------------------------------------------------
// hw_timer_t - Arduino-ESP32-core v3 style hardware timer on top of gptimer.
// ---------------------------------------------------------------------------
typedef struct hw_timer_s {
  gptimer_handle_t handle{nullptr};
  uint32_t resolution_hz{1000000};
  bool enabled{false};  // gptimer_enable() called
  bool running{false};  // gptimer_start() called
  void (*fn)(void){nullptr};
} hw_timer_t;

static bool IRAM_ATTR timer_isr_trampoline(gptimer_handle_t handle,
                                           const gptimer_alarm_event_data_t *edata,
                                           void *ctx) {
  hw_timer_t *t = static_cast<hw_timer_t *>(ctx);
  if (t && t->fn) {
    t->fn();
  }
  return false;  // let the driver handle reload behaviour
}

// core v3 signature: timerBegin(frequency_hz)
inline hw_timer_t *timerBegin(uint32_t freq) {
  hw_timer_t *t = new hw_timer_t();
  t->resolution_hz = freq;
  gptimer_config_t cfg = {};
  cfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  cfg.resolution_hz = freq;
  cfg.flags.intr_shared = true;
  if (gptimer_new_timer(&cfg, &t->handle) != ESP_OK) {
    delete t;
    return nullptr;
  }
  return t;
}

inline void timerWrite(hw_timer_t *t, uint64_t value) {
  if (t && t->handle) gptimer_set_raw_count(t->handle, value);
}

// core v3 signature: timerAlarm(timer, alarm_us, autoreload, countUp)
inline void timerAlarm(hw_timer_t *t, uint64_t alarm_us, bool autoreload,
                       uint64_t countUp) {
  (void)countUp;
  if (!t || !t->handle) return;
  gptimer_alarm_config_t alarm = {};
  alarm.alarm_count = alarm_us;
  alarm.reload_count = 0;
  alarm.flags.auto_reload_on_alarm = autoreload;
  gptimer_set_alarm_action(t->handle, &alarm);
  if (!t->running) {
    if (gptimer_start(t->handle) == ESP_OK) t->running = true;
  }
}

// core v3 signature: timerAttachInterrupt(timer, callback)
inline void timerAttachInterrupt(hw_timer_t *t, void (*fn)(void)) {
  if (!t || !t->handle) return;
  t->fn = fn;
  gptimer_event_callbacks_t cbs = {};
  cbs.on_alarm = timer_isr_trampoline;
  gptimer_register_event_callbacks(t->handle, &cbs, t);
  if (gptimer_enable(t->handle) == ESP_OK) t->enabled = true;
}

inline void timerDetachInterrupt(hw_timer_t *t) {
  if (!t) return;
  t->fn = nullptr;
  if (t->enabled && t->handle) {
    gptimer_disable(t->handle);
    t->enabled = false;
  }
}

inline void timerEnd(hw_timer_t *t) {
  if (!t) return;
  timerDetachInterrupt(t);
  if (t->running && t->handle) {
    gptimer_stop(t->handle);
    t->running = false;
  }
  if (t->handle) gptimer_del_timer(t->handle);
  delete t;
}

// ---------------------------------------------------------------------------
// attachInterrupt/detachInterrupt - GPIO ISR service dispatch table.
// ---------------------------------------------------------------------------
namespace arduino_shim {
static void (*g_isr_fns[64])(void) = {nullptr};

static void IRAM_ATTR isr_dispatch(void *arg) {
  const int pin = static_cast<int>(reinterpret_cast<intptr_t>(arg));
  if (pin >= 0 && pin < 64) {
    void (*fn)(void) = g_isr_fns[pin];
    if (fn) fn();
  }
}
}  // namespace arduino_shim

inline void attachInterrupt(uint8_t pin, void (*fn)(void), int mode) {
  (void)mode;  // IRrecv only uses CHANGE; we always use ANYEDGE
  esp_err_t err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE("ArduinoShim", "gpio_install_isr_service: %s", esp_err_to_name(err));
  }
  arduino_shim::g_isr_fns[pin] = fn;
  gpio_config_t io = {};
  io.pin_bit_mask = (1ULL << pin);
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_ANYEDGE;
  gpio_config(&io);
  gpio_isr_handler_add(static_cast<gpio_num_t>(pin), arduino_shim::isr_dispatch,
                       reinterpret_cast<void *>(static_cast<intptr_t>(pin)));
}

inline void detachInterrupt(uint8_t pin) {
  gpio_isr_handler_remove(static_cast<gpio_num_t>(pin));
  arduino_shim::g_isr_fns[pin] = nullptr;
}

#endif  // ARDUINO_H_SHIM
