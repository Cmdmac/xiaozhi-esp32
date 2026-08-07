// remote_transmitter.h - Standalone ESP-IDF RMT-based IR/RF transmitter.
//
// Extracted from ESPHome's remote_transmitter component and stripped of
// ESPHome framework dependencies. Targets ESP32-C3 + ESP-IDF 5.5.2.
//
// Key features:
//   * Uses the new rmt_simple_encoder callback API available since IDF 5.5.1
//     (mandatory path on 5.5.2). The callback streams a vector of half-words
//     (15-bit duration + 1-bit level) into RMT symbols on demand, supporting
//     arbitrary send_times repetition without pre-expanding the buffer.
//   * Carrier modulation applied via rmt_apply_carrier() (hardware).
//   * Optional non-blocking mode uses esp_timer to signal completion.
//
// Original ESPHome source:
//   esphome/components/remote_transmitter/remote_transmitter.h
//   esphome/components/remote_transmitter/remote_transmitter_rmt.cpp
//   esphome/components/remote_base/remote_base.h (RemoteTransmitData)
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <vector>
#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <esp_timer.h>
#include "esp_idf_version.h"

static_assert(ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 1),
              "heatpump_ir_idf requires ESP-IDF >= 5.5.1 (5.5.2 recommended). "
              "Earlier versions lack the rmt_simple_encoder callback API.");

namespace heatpump_ir_tx {

// A sequence of mark (positive) / space (negative) durations in microseconds,
// together with an optional carrier frequency (Hz) for hardware modulation.
// Mirrors ESPHome's remote_base::RemoteTransmitData but without protobuf/base64
// helpers (not needed for a transmit-only library).
class RemoteTransmitData {
 public:
  void mark(uint32_t us) { data_.push_back(static_cast<int32_t>(us)); }
  void space(uint32_t us) { data_.push_back(-static_cast<int32_t>(us)); }
  void item(uint32_t mark_us, uint32_t space_us) {
    mark(mark_us);
    space(space_us);
  }
  void reserve(size_t n) { data_.reserve(n); }
  void set_carrier_frequency(uint32_t hz) { carrier_frequency_ = hz; }
  uint32_t get_carrier_frequency() const { return carrier_frequency_; }
  const std::vector<int32_t>& get_data() const { return data_; }
  void reset() {
    data_.clear();
    carrier_frequency_ = 0;
  }

 private:
  std::vector<int32_t> data_{};
  uint32_t carrier_frequency_{0};
};

// 15-bit duration + 1-bit level, packed into a 16-bit half-word. Matches the
// IDF 5.5.1+ rmt_simple_encoder callback contract. Two half-words combine
// into one 32-bit rmt_symbol_word_t via `.val` (little-endian: sym_0 | sym_1<<16).
// Declared at namespace scope so the free rmt_simple_encoder callback (which
// runs in ISR context and cannot be a class member) can cast its arguments.
union SymbolHalf {
  struct {
    uint16_t duration : 15;
    uint16_t level : 1;
  };
  uint16_t val;
};
static_assert(sizeof(SymbolHalf) == sizeof(uint16_t), "SymbolHalf must be 16-bit");

struct EncoderStore {
  uint32_t times;   // remaining repetitions
  uint32_t index;   // current index into the half-word buffer
};

// RMT-backed transmitter for ESP32-C3.
//
// Lifecycle:
//   RemoteTransmitter tx;
//   tx.begin(GPIO_NUM_3, /*rmt_symbols=*/48, /*clock_hz=*/1'000'000);
//   tx.set_carrier_duty_percent(33);
//   RemoteTransmitData d;
//   d.set_carrier_frequency(38000);
//   d.mark(9000); d.space(4500); d.item(560, 1690); ...
//   tx.send(d, /*times=*/1, /*wait_ms=*/0);
class RemoteTransmitter {
 public:
  RemoteTransmitter() = default;
  ~RemoteTransmitter();

  // Configure and enable the RMT TX channel. Must be called once before send().
  //   pin:               GPIO connected to the IR LED / 433MHz modulator.
  //   rmt_symbols:       RMT memory block size in symbols (48 is a safe value
  //                      for ESP32-C3, matches the ESPHome default).
  //   clock_resolution_hz: RMT tick frequency. 1 MHz gives 1 us resolution.
  //   with_dma:          ESP32-C3 has no RMT DMA; leave false.
  esp_err_t begin(gpio_num_t pin,
                  uint32_t rmt_symbols = 48,
                  uint32_t clock_resolution_hz = 1'000'000,
                  bool with_dma = false);

  // Release the RMT channel and encoder.
  void end();

  // 1..100. Only matters when carrier frequency != 0.
  void set_carrier_duty_percent(uint8_t percent) { carrier_duty_percent_ = percent; }

  // Static output level at end-of-transmission. Default false (idle low).
  void set_eot_level(bool level) { eot_level_ = level; }

  // When enabled, send() returns immediately after submitting to RMT and a
  // one-shot esp_timer calls on_complete when the queue drains. When disabled
  // (default), send() blocks until the transmission is finished.
  void set_non_blocking(bool enable);

  // Optional completion callback (non-blocking mode). Invoked from the
  // esp_timer task context.
  void set_on_complete(void (*cb)(void*), void* arg) {
    on_complete_cb_ = cb;
    on_complete_arg_ = arg;
  }

  // Transmit `data` `send_times` times, pausing `send_wait_ms` ms between
  // repeats. Carrier frequency comes from data.get_carrier_frequency().
  esp_err_t send(const RemoteTransmitData& data,
                 uint32_t send_times = 1,
                 uint32_t send_wait_ms = 0);

  // Force a steady level on the pin (bypasses the encoder). Useful for idle.
  esp_err_t digital_write(bool level);

  bool is_failed() const { return failed_; }
  esp_err_t last_error() const { return last_error_; }

 private:
  esp_err_t configure_rmt_();
  void wait_for_rmt_();
  uint32_t from_microseconds_(uint32_t us) const {
    return us * (clock_resolution_ / 100000u) / 10;
  }

  gpio_num_t pin_{GPIO_NUM_NC};
  uint32_t rmt_symbols_{48};
  uint32_t clock_resolution_{1'000'000};
  uint8_t carrier_duty_percent_{50};
  bool with_dma_{false};
  bool eot_level_{false};
  bool inverted_{false};
  bool non_blocking_{false};
  bool initialized_{false};
  bool failed_{false};
  esp_err_t last_error_{ESP_OK};

  rmt_channel_handle_t channel_{nullptr};
  rmt_encoder_handle_t encoder_{nullptr};
  uint32_t current_carrier_frequency_{38000};

  std::vector<SymbolHalf> rmt_temp_{};
  EncoderStore store_{};

  // Non-blocking completion timer.
  esp_timer_handle_t complete_timer_{nullptr};
  void (*on_complete_cb_)(void*){nullptr};
  void* on_complete_arg_{nullptr};
};

}  // namespace heatpump_ir_tx
