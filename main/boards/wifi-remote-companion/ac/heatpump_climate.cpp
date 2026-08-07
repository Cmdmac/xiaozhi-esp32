// heatpump_climate.cpp - Implementation of the climate-to-IR bridge.
//
// Ported from ESPHome's heatpumpir/heatpumpir.cpp. The HeatpumpIR library
// headers are included ONLY in this translation unit to isolate their
// unnamespaced defines (millis(), etc.) from the rest of an IDF application.
#include "heatpump_climate.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>

#include <esp_log.h>

// External HeatpumpIR library (ToniA/arduino-HeatpumpIR).
// 只引入 Panasonic 家族 (DKE/JKE/NKE/EKE/LKE/CKP/AltDKE), 不引入全部协议以控制固件体积
#include <IRSender.h>
#include <PanasonicHeatpumpIR.h>
#include <PanasonicCKPHeatpumpIR.h>
#include <PanasonicAltDKEHeatpumpIR.h>

// 清理协议头文件里与 Protocol 枚举同名的历史遗留宏（IDF 编译兼容）
#undef PANASONIC_DKE
#undef PANASONIC_JKE
#undef PANASONIC_NKE
#undef PANASONIC_LKE
#undef PANASONIC_EKE
// HeatpumpIRCompat.h (ESP-IDF 分支) 定义了 LOW/HIGH 宏, 与 ClimateFanMode::LOW/HIGH 冲突
#undef LOW
#undef HIGH

namespace heatpump_ir_tx {

static const char* TAG = "heatpump_climate";

// Bridge between our RemoteTransmitter and the HeatpumpIR library's IRSender
// interface. HeatpumpIR calls mark()/space()/setFrequency() to build a frame;
// space(0) is the library's convention for "frame complete, flush now".
class IRSenderIDF : public IRSender {
 public:
  explicit IRSenderIDF(RemoteTransmitter* tx) : IRSender(0), tx_(tx) {}

  // HeatpumpIR passes frequency in kHz; RemoteTransmitData expects Hz.
  void setFrequency(int frequency) override {
    data_.set_carrier_frequency(static_cast<uint32_t>(frequency) * 1000u);
  }

  void space(int space_length) override {
    if (space_length != 0) {
      data_.space(static_cast<uint32_t>(space_length));
    } else {
      // Flush: send what we have accumulated, then reset for the next frame.
      tx_->send(data_, 1, 0);
      data_.reset();
    }
  }

  void mark(int mark_length) override {
    data_.mark(static_cast<uint32_t>(mark_length));
  }

 private:
  RemoteTransmitter* tx_;
  RemoteTransmitData data_{};
};

// Factory table mapping Protocol -> HeatpumpIR subclass constructor.
// 仅保留 Panasonic 家族 (本机空调), 其余协议不编译以控制固件体积
static const std::map<Protocol, std::function<HeatpumpIR*()>>& protocol_constructors() {
  static const std::map<Protocol, std::function<HeatpumpIR*()>> table = {
      {Protocol::PANASONIC_CKP, []() { return new PanasonicCKPHeatpumpIR(); }},
      {Protocol::PANASONIC_DKE, []() { return new PanasonicDKEHeatpumpIR(); }},
      {Protocol::PANASONIC_EKE, []() { return new PanasonicEKEHeatpumpIR(); }},
      {Protocol::PANASONIC_JKE, []() { return new PanasonicJKEHeatpumpIR(); }},
      {Protocol::PANASONIC_LKE, []() { return new PanasonicLKEHeatpumpIR(); }},
      {Protocol::PANASONIC_NKE, []() { return new PanasonicNKEHeatpumpIR(); }},
      {Protocol::PANASONIC_ALTDKE, []() { return new PanasonicAltDKEHeatpumpIR(); }},
  };
  return table;
}

HeatPumpClimate::~HeatPumpClimate() { release_protocol_(); }

void HeatPumpClimate::release_protocol_() {
  delete heatpump_ir_;
  heatpump_ir_ = nullptr;
}

esp_err_t HeatPumpClimate::begin(RemoteTransmitter* tx, Protocol protocol) {
  tx_ = tx;
  return set_protocol(protocol);
}

esp_err_t HeatPumpClimate::set_protocol(Protocol protocol) {
  protocol_ = protocol;
  release_protocol_();
  const auto& table = protocol_constructors();
  auto it = table.find(protocol);
  if (it == table.end()) {
    ESP_LOGE(TAG, "unknown protocol (%u)", static_cast<unsigned>(protocol));
    return ESP_ERR_NOT_SUPPORTED;
  }
  heatpump_ir_ = it->second();
  return ESP_OK;
}

esp_err_t HeatPumpClimate::transmit_state() {
  if (!tx_ || !heatpump_ir_) {
    ESP_LOGE(TAG, "transmit_state: not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t power_mode_cmd;
  uint8_t operating_mode_cmd;
  uint8_t temperature_cmd;
  uint8_t fan_speed_cmd;
  uint8_t swing_v_cmd;
  uint8_t swing_h_cmd;

  // Vertical direction / swing
  switch (default_vertical_) {
    case VerticalDirection::AUTO:    swing_v_cmd = VDIR_AUTO;    break;
    case VerticalDirection::UP:      swing_v_cmd = VDIR_UP;      break;
    case VerticalDirection::MUP:     swing_v_cmd = VDIR_MUP;     break;
    case VerticalDirection::MIDDLE:  swing_v_cmd = VDIR_MIDDLE;  break;
    case VerticalDirection::MDOWN:   swing_v_cmd = VDIR_MDOWN;   break;
    case VerticalDirection::DOWN:    swing_v_cmd = VDIR_DOWN;    break;
    default:
      ESP_LOGE(TAG, "invalid vertical default");
      return ESP_ERR_INVALID_ARG;
  }
  if (state_.swing == ClimateSwingMode::VERTICAL ||
      state_.swing == ClimateSwingMode::BOTH) {
    swing_v_cmd = VDIR_SWING;
  }

  // Horizontal direction / swing
  switch (default_horizontal_) {
    case HorizontalDirection::AUTO:   swing_h_cmd = HDIR_AUTO;   break;
    case HorizontalDirection::MIDDLE: swing_h_cmd = HDIR_MIDDLE; break;
    case HorizontalDirection::LEFT:   swing_h_cmd = HDIR_LEFT;   break;
    case HorizontalDirection::MLEFT:  swing_h_cmd = HDIR_MLEFT;  break;
    case HorizontalDirection::MRIGHT: swing_h_cmd = HDIR_MRIGHT; break;
    case HorizontalDirection::RIGHT:  swing_h_cmd = HDIR_RIGHT;  break;
    default:
      ESP_LOGE(TAG, "invalid horizontal default");
      return ESP_ERR_INVALID_ARG;
  }
  if (state_.swing == ClimateSwingMode::HORIZONTAL ||
      state_.swing == ClimateSwingMode::BOTH) {
    swing_h_cmd = HDIR_SWING;
  }

  // Fan speed
  switch (state_.fan) {
    case ClimateFanMode::LOW:    fan_speed_cmd = FAN_2;    break;
    case ClimateFanMode::MEDIUM: fan_speed_cmd = FAN_3;    break;
    case ClimateFanMode::HIGH:   fan_speed_cmd = FAN_4;    break;
    case ClimateFanMode::AUTO:
    default:                     fan_speed_cmd = FAN_AUTO; break;
  }

  // Operating mode
  switch (state_.mode) {
    case ClimateMode::COOL:
      power_mode_cmd = POWER_ON;
      operating_mode_cmd = MODE_COOL;
      break;
    case ClimateMode::HEAT:
      power_mode_cmd = POWER_ON;
      operating_mode_cmd = MODE_HEAT;
      break;
    // Map HEAT_COOL to hardware AUTO mode (automatic heat/cool changeover).
    case ClimateMode::HEAT_COOL:
    case ClimateMode::OFF:
      // Note: OFF is handled below with POWER_OFF; mode falls back to AUTO.
      power_mode_cmd = POWER_ON;
      operating_mode_cmd = MODE_AUTO;
      break;
    case ClimateMode::FAN_ONLY:
      power_mode_cmd = POWER_ON;
      operating_mode_cmd = MODE_FAN;
      break;
    case ClimateMode::DRY:
      power_mode_cmd = POWER_ON;
      operating_mode_cmd = MODE_DRY;
      break;
    default:
      power_mode_cmd = POWER_ON;
      operating_mode_cmd = MODE_AUTO;
      break;
  }
  if (state_.mode == ClimateMode::OFF) {
    power_mode_cmd = POWER_OFF;
    operating_mode_cmd = MODE_AUTO;
  }

  temperature_cmd = static_cast<uint8_t>(
      std::clamp(state_.target_temperature, min_temperature_, max_temperature_));

  IRSenderIDF sender(tx_);
  heatpump_ir_->send(sender, power_mode_cmd, operating_mode_cmd, fan_speed_cmd,
                     temperature_cmd, swing_v_cmd, swing_h_cmd);
  return ESP_OK;
}

}  // namespace heatpump_ir_tx
