// heatpump_climate.h - Air-conditioner IR control built on top of the
// external HeatpumpIR library (ToniA/arduino-HeatpumpIR) and our RemoteTransmitter.
//
// Extracted from ESPHome's heatpumpir component, with the ClimateIR/Climate
// base classes, sensor callbacks and receiver path removed. The user holds a
// ClimateState struct and calls transmit_state() to emit the corresponding IR
// frame through the bound RemoteTransmitter.
//
// Original ESPHome source:
//   esphome/components/heatpumpir/heatpumpir.h
//   esphome/components/heatpumpir/heatpumpir.cpp
//   esphome/components/climate_ir/climate_ir.h  (transmit_state contract)
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <esp_err.h>

#include "remote_transmitter.h"

// Forward-declare the HeatpumpIR class from the external library. We cannot
// include its header here because it pulls in unnamespaced defines (e.g.
// millis()) that clash with ESP-IDF / application code. The implementation
// file includes the full header in an isolated translation unit.
class HeatpumpIR;

namespace heatpump_ir_tx {

// Air-conditioner protocol selector. Mirrors ESPHome's heatpumpir::Protocol
// enum; keep the integer values in sync with that enum so that configuration
// strings map 1:1.
enum class Protocol : uint8_t {
  AUX,
  BALLU,
  CARRIER_MCA,
  CARRIER_NQV,
  DAIKIN_ARC417,
  DAIKIN_ARC480,
  DAIKIN,
  ELECTROLUXYAL,
  FUEGO,
  FUJITSU_AWYZ,
  GREE,
  GREEYAA,
  GREEYAN,
  GREEYAC,
  GREEYT,
  GREEYAP,
  HISENSE_AUD,
  HITACHI,
  HYUNDAI,
  IVT,
  MIDEA,
  MITSUBISHI_FA,
  MITSUBISHI_FD,
  MITSUBISHI_FE,
  MITSUBISHI_HEAVY_FDTC,
  MITSUBISHI_HEAVY_ZJ,
  MITSUBISHI_HEAVY_ZM,
  MITSUBISHI_HEAVY_ZMP,
  MITSUBISHI_KJ,
  MITSUBISHI_MSC,
  MITSUBISHI_MSY,
  MITSUBISHI_SEZ,
  PANASONIC_CKP,
  PANASONIC_DKE,
  PANASONIC_EKE,
  PANASONIC_JKE,
  PANASONIC_LKE,
  PANASONIC_NKE,
  SAMSUNG_AQV,
  SAMSUNG_FJM,
  SHARP,
  TOSHIBA_DAISEIKAI,
  TOSHIBA,
  ZHLT01,
  NIBE,
  QLIMA_1,
  QLIMA_2,
  SAMSUNG_AQV12MSAN,
  ZHJG01,
  AIRWAY,
  BGH_AUD,
  PANASONIC_ALTDKE,
  PHILCO_PHS32,
  VAILLANTVAI8,
  R51M,
};

enum class HorizontalDirection : uint8_t {
  AUTO = 0,
  MIDDLE = 1,
  LEFT = 2,
  MLEFT = 3,
  MRIGHT = 4,
  RIGHT = 5,
};

enum class VerticalDirection : uint8_t {
  AUTO = 0,
  UP = 1,
  MUP = 2,
  MIDDLE = 3,
  MDOWN = 4,
  DOWN = 5,
};

enum class ClimateMode : uint8_t {
  OFF,
  COOL,
  HEAT,
  // Automatic heat/cool changeover based on target vs current temperature.
  HEAT_COOL,
  FAN_ONLY,
  DRY,
};

enum class ClimateFanMode : uint8_t {
  AUTO,
  LOW,
  MEDIUM,
  HIGH,
};

enum class ClimateSwingMode : uint8_t {
  OFF,
  HORIZONTAL,
  VERTICAL,
  BOTH,
};

// Logical state of the AC. Set these fields then call transmit_state().
struct ClimateState {
  ClimateMode mode{ClimateMode::OFF};
  ClimateFanMode fan{ClimateFanMode::AUTO};
  ClimateSwingMode swing{ClimateSwingMode::OFF};
  float target_temperature{24.0f};   // Celsius, clamped to [min,max] at send time
  float current_temperature{NAN};     // optional, only used by some protocols
};

// Translates a ClimateState into an IR frame and pushes it through the bound
// RemoteTransmitter. Construction of the underlying HeatpumpIR object is done
// lazily in set_protocol().
class HeatPumpClimate {
 public:
  HeatPumpClimate() = default;
  ~HeatPumpClimate();

  // Bind a transmitter (must outlive this object) and instantiate the protocol
  // handler. Returns ESP_ERR_NOT_SUPPORTED if the protocol is unknown.
  esp_err_t begin(RemoteTransmitter* tx, Protocol protocol);

  // Switch protocol at runtime; recreates the protocol handler.
  esp_err_t set_protocol(Protocol protocol);

  void set_min_temperature(float t) { min_temperature_ = t; }
  void set_max_temperature(float t) { max_temperature_ = t; }
  void set_vertical_default(VerticalDirection v) { default_vertical_ = v; }
  void set_horizontal_default(HorizontalDirection h) { default_horizontal_ = h; }

  ClimateState& state() { return state_; }
  const ClimateState& state() const { return state_; }

  // Encode state_ into a RemoteTransmitData and transmit it once.
  esp_err_t transmit_state();

  // Convenience: update state_ in one call and transmit immediately.
  esp_err_t set_state_and_send(const ClimateState& s) {
    state_ = s;
    return transmit_state();
  }

 private:
  RemoteTransmitter* tx_{nullptr};
  Protocol protocol_{Protocol::PANASONIC_LKE};
  float min_temperature_{16.0f};
  float max_temperature_{30.0f};
  VerticalDirection default_vertical_{VerticalDirection::AUTO};
  HorizontalDirection default_horizontal_{HorizontalDirection::AUTO};
  ClimateState state_{};
  HeatpumpIR* heatpump_ir_{nullptr};

  void release_protocol_();
};

}  // namespace heatpump_ir_tx
