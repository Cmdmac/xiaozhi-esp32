// remote_transmitter.cpp - ESP-IDF RMT implementation.
//
// Ported from ESPHome's remote_transmitter_rmt.cpp, IDF >= 5.5.1 branch.
// All ESPHome-specific helpers (Component scheduler, status_set_warning,
// InternalGPIOPin, Trigger<>) have been replaced with ESP-IDF primitives.
#include "remote_transmitter.h"

#include <algorithm>
#include <cstring>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace heatpump_ir_tx {

static const char* TAG = "ir_tx";

// Maximum duration expressible in a single RMT half-symbol (15-bit field).
static constexpr uint32_t RMT_SYMBOL_DURATION_MAX = 0x7FFF;

// IRAM_ATTR: this runs in the RMT ISR context. Only touch internal-RAM data.
// Streams half-words from the input buffer two at a time, packing them into
// 32-bit rmt_symbol_word_t. When the buffer is exhausted, either repeats
// (times > 1) or signals completion via *done.
static size_t IRAM_ATTR encoder_callback(const void* data, size_t size,
                                         size_t /*written*/, size_t free,
                                         rmt_symbol_word_t* symbols, bool* done,
                                         void* arg) {
  auto* store = static_cast<EncoderStore*>(arg);
  const auto* encoded = static_cast<const SymbolHalf*>(data);
  size_t length = size / sizeof(SymbolHalf);
  size_t count = 0;

  for (size_t i = 0; i < free; i++) {
    uint16_t sym_0 = encoded[store->index++].val;
    if (store->index >= length) {
      store->index = 0;
      store->times--;
      if (store->times == 0) {
        *done = true;
        symbols[count++].val = sym_0;
        return count;
      }
    }
    uint16_t sym_1 = encoded[store->index++].val;
    if (store->index >= length) {
      store->index = 0;
      store->times--;
      if (store->times == 0) {
        *done = true;
        symbols[count++].val = sym_0 | (static_cast<uint32_t>(sym_1) << 16);
        return count;
      }
    }
    symbols[count++].val = sym_0 | (static_cast<uint32_t>(sym_1) << 16);
  }
  *done = false;
  return count;
}

RemoteTransmitter::~RemoteTransmitter() { end(); }

esp_err_t RemoteTransmitter::begin(gpio_num_t pin, uint32_t rmt_symbols,
                                   uint32_t clock_resolution_hz,
                                   bool with_dma) {
  pin_ = pin;
  rmt_symbols_ = rmt_symbols;
  clock_resolution_ = clock_resolution_hz;
  with_dma_ = with_dma;

  esp_err_t err = configure_rmt_();
  if (err != ESP_OK) {
    failed_ = true;
    last_error_ = err;
    return err;
  }

  if (non_blocking_) {
    esp_timer_create_args_t args = {};
    args.callback = [](void* self) {
      static_cast<RemoteTransmitter*>(self)->wait_for_rmt_();
    };
    args.arg = this;
    args.name = "ir_tx_complete";
    esp_err_t te = esp_timer_create(&args, &complete_timer_);
    if (te != ESP_OK) {
      ESP_LOGW(TAG, "esp_timer_create failed: %s", esp_err_to_name(te));
      non_blocking_ = false;
    }
  }
  return ESP_OK;
}

void RemoteTransmitter::end() {
  if (complete_timer_) {
    esp_timer_stop(complete_timer_);
    esp_timer_delete(complete_timer_);
    complete_timer_ = nullptr;
  }
  if (encoder_) {
    rmt_del_encoder(encoder_);
    encoder_ = nullptr;
  }
  if (channel_) {
    rmt_disable(channel_);
    rmt_del_channel(channel_);
    channel_ = nullptr;
  }
  initialized_ = false;
}

void RemoteTransmitter::set_non_blocking(bool enable) {
  if (enable == non_blocking_) return;
  if (initialized_) {
    ESP_LOGW(TAG, "set_non_blocking() ignored: transmitter already initialized");
    return;
  }
  non_blocking_ = enable;
}

esp_err_t RemoteTransmitter::digital_write(bool value) {
  SymbolHalf symbol{};
  symbol.duration = 1;
  symbol.level = value ? 1 : 0;
  rmt_transmit_config_t config;
  memset(&config, 0, sizeof(config));
  config.flags.eot_level = value;
  store_.times = 1;
  store_.index = 0;
  esp_err_t err = rmt_transmit(channel_, encoder_, &symbol, sizeof(symbol), &config);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "rmt_transmit(digital_write) failed: %s", esp_err_to_name(err));
    return err;
  }
  return rmt_tx_wait_all_done(channel_, -1);
}

esp_err_t RemoteTransmitter::configure_rmt_() {
  rmt_tx_channel_config_t channel;
  memset(&channel, 0, sizeof(channel));
  channel.clk_src = RMT_CLK_SRC_DEFAULT;
  channel.resolution_hz = clock_resolution_;
  channel.gpio_num = pin_;
  channel.mem_block_symbols = rmt_symbols_;
  channel.trans_queue_depth = 1;
  channel.flags.invert_out = 0;
  channel.flags.with_dma = with_dma_;
  channel.intr_priority = 0;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
  channel.flags.io_loop_back = 0;
  channel.flags.io_od_mode = 0;
#endif
  esp_err_t err = rmt_new_tx_channel(&channel, &channel_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "rmt_new_tx_channel failed: %s", esp_err_to_name(err));
    return err;
  }

  rmt_simple_encoder_config_t encoder;
  memset(&encoder, 0, sizeof(encoder));
  encoder.callback = encoder_callback;
  encoder.arg = &store_;
  encoder.min_chunk_size = 1;
  err = rmt_new_simple_encoder(&encoder, &encoder_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "rmt_new_simple_encoder failed: %s", esp_err_to_name(err));
    rmt_del_channel(channel_);
    channel_ = nullptr;
    return err;
  }

  err = rmt_enable(channel_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(err));
    rmt_del_encoder(encoder_);
    encoder_ = nullptr;
    rmt_del_channel(channel_);
    channel_ = nullptr;
    return err;
  }

  // Park the pin at eot_level_ until first transmit.
  digital_write(eot_level_);
  initialized_ = true;

  // Apply initial carrier (38 kHz default).
  if (current_carrier_frequency_ == 0 || carrier_duty_percent_ == 100) {
    err = rmt_apply_carrier(channel_, nullptr);
  } else {
    rmt_carrier_config_t carrier;
    memset(&carrier, 0, sizeof(carrier));
    carrier.frequency_hz = current_carrier_frequency_;
    carrier.duty_cycle = static_cast<float>(carrier_duty_percent_) / 100.0f;
    carrier.flags.polarity_active_low = inverted_;
    carrier.flags.always_on = 1;
    err = rmt_apply_carrier(channel_, &carrier);
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "rmt_apply_carrier failed: %s", esp_err_to_name(err));
  }
  return err;
}

void RemoteTransmitter::wait_for_rmt_() {
  esp_err_t err = rmt_tx_wait_all_done(channel_, -1);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "rmt_tx_wait_all_done failed: %s", esp_err_to_name(err));
  }
  if (on_complete_cb_) on_complete_cb_(on_complete_arg_);
}

esp_err_t RemoteTransmitter::send(const RemoteTransmitData& data,
                                  uint32_t send_times, uint32_t send_wait_ms) {
  if (failed_ || !initialized_) {
    return ESP_ERR_INVALID_STATE;
  }

  // In non-blocking mode, ensure any prior transmission has flushed.
  if (non_blocking_ && complete_timer_) {
    esp_timer_stop(complete_timer_);
    wait_for_rmt_();
  }

  // Reconfigure carrier if it changed.
  uint32_t new_carrier = data.get_carrier_frequency();
  if (new_carrier != current_carrier_frequency_) {
    current_carrier_frequency_ = new_carrier;
    esp_err_t err;
    if (new_carrier == 0 || carrier_duty_percent_ == 100) {
      err = rmt_apply_carrier(channel_, nullptr);
    } else {
      rmt_carrier_config_t carrier;
      memset(&carrier, 0, sizeof(carrier));
      carrier.frequency_hz = new_carrier;
      carrier.duty_cycle = static_cast<float>(carrier_duty_percent_) / 100.0f;
      carrier.flags.polarity_active_low = inverted_;
      carrier.flags.always_on = 1;
      err = rmt_apply_carrier(channel_, &carrier);
    }
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "rmt_apply_carrier failed: %s", esp_err_to_name(err));
      return err;
    }
  }

  rmt_temp_.clear();
  rmt_temp_.reserve(data.get_data().size() + 4);

  // Inter-repeat gap (encoded as low-level half-symbols so the encoder can
  // loop seamlessly). Skipped on first iteration by the encoder (store_.index
  // is set to offset below).
  uint32_t wait_ticks = from_microseconds_(send_wait_ms * 1000);
  size_t offset = 0;
  while (wait_ticks > 0) {
    uint32_t dur = std::min(wait_ticks, RMT_SYMBOL_DURATION_MAX);
    SymbolHalf s{};
    s.duration = static_cast<uint16_t>(dur);
    s.level = eot_level_ ? 1 : 0;
    rmt_temp_.push_back(s);
    wait_ticks -= dur;
    offset++;
  }

  // Encode data: each mark/space may exceed 15-bit and must be split.
  uint64_t total_duration_us = send_wait_ms * 1000 * (send_times - 1);
  for (int32_t value : data.get_data()) {
    bool level = value >= 0;
    if (!level) value = -value;
    total_duration_us += static_cast<uint64_t>(value) * send_times;
    int32_t ticks = static_cast<int32_t>(from_microseconds_(static_cast<uint32_t>(value)));
    while (ticks > 0) {
      int32_t dur = std::min(ticks, static_cast<int32_t>(RMT_SYMBOL_DURATION_MAX));
      SymbolHalf s{};
      s.duration = static_cast<uint16_t>(dur);
      s.level = (level ^ inverted_) ? 1 : 0;
      rmt_temp_.push_back(s);
      ticks -= dur;
    }
  }

  if (rmt_temp_.empty()) {
    ESP_LOGE(TAG, "send(): empty data");
    return ESP_ERR_INVALID_ARG;
  }

  rmt_transmit_config_t config;
  memset(&config, 0, sizeof(config));
  config.flags.eot_level = eot_level_;
  store_.times = send_times;
  store_.index = offset;  // skip the inter-repeat gap on the first pass
  esp_err_t err = rmt_transmit(channel_, encoder_, rmt_temp_.data(),
                               rmt_temp_.size() * sizeof(SymbolHalf), &config);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "rmt_transmit failed: %s", esp_err_to_name(err));
    return err;
  }

  if (non_blocking_ && complete_timer_) {
    // Add a small grace period to the calculated duration so the timer fires
    // after the hardware has actually drained.
    int64_t timeout_us = static_cast<int64_t>(total_duration_us) + 1000;
    esp_timer_start_once(complete_timer_, timeout_us);
  } else {
    wait_for_rmt_();
  }
  return ESP_OK;
}

}  // namespace heatpump_ir_tx
