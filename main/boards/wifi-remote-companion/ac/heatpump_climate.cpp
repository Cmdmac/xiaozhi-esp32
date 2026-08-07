// heatpump_climate.cpp - Implementation of the climate-to-IR bridge.
//
// Ported from ESPHome's heatpumpir/heatpumpir.cpp. The HeatpumpIR library
// headers are included ONLY in this translation unit to isolate their
// unnamespaced defines (millis(), etc.) from the rest of an IDF application.
//
// Full protocol support: the factory table below registers every protocol in
// heatpump_climate.h (mirroring ESPHome's CLIMATE_IR_PROTOCOL_FACTORY), so the
// AC brand can be selected at runtime through Protocol / set_protocol().
#include "heatpump_climate.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>

#include <esp_log.h>

// External HeatpumpIR library (ToniA/arduino-HeatpumpIR).
#include <IRSender.h>
#include <HeatpumpIR.h>
#include <AIRWAYHeatpumpIR.h>
#include <AUXHeatpumpIR.h>
#include <BGHHeatpumpIR.h>
#include <BalluHeatpumpIR.h>
#include <CarrierHeatpumpIR.h>
#include <DaikinHeatpumpARC417IR.h>
#include <DaikinHeatpumpARC480A14IR.h>
#include <DaikinHeatpumpIR.h>
#include <ElectroluxHeatpumpIR.h>
#include <FuegoHeatpumpIR.h>
#include <FujitsuHeatpumpIR.h>
#include <GreeHeatpumpIR.h>
#include <HisenseHeatpumpIR.h>
#include <HitachiHeatpumpIR.h>
#include <HyundaiHeatpumpIR.h>
#include <IVTHeatpumpIR.h>
#include <MideaHeatpumpIR.h>
#include <MitsubishiHeatpumpIR.h>
#include <MitsubishiHeavyFDTCHeatpumpIR.h>
#include <MitsubishiHeavyHeatpumpIR.h>
#include <MitsubishiMSCHeatpumpIR.h>
#include <MitsubishiSEZKDXXHeatpumpIR.h>
#include <NibeHeatpumpIR.h>
#include <PanasonicAltDKEHeatpumpIR.h>
#include <PanasonicCKPHeatpumpIR.h>
#include <PanasonicHeatpumpIR.h>
#include <PhilcoPHS32HeatpumpIR.h>
#include <R51MHeatpumpIR.h>
#include <SamsungHeatpumpIR.h>
#include <SharpHeatpumpIR.h>
#include <ToshibaDaiseikaiHeatpumpIR.h>
#include <ToshibaHeatpumpIR.h>
#include <VaillantHeatpumpIR.h>
#include <ZHJG01HeatpumpIR.h>
#include <ZHLT01HeatpumpIR.h>

// 清理协议头文件里与 Protocol 枚举同名的历史遗留宏（IDF 编译兼容）
#undef PANASONIC_DKE
#undef PANASONIC_JKE
#undef PANASONIC_NKE
#undef PANASONIC_LKE
#undef PANASONIC_EKE
#undef MITSUBISHI_FA
#undef MITSUBISHI_FD
#undef MITSUBISHI_FE
#undef MITSUBISHI_KJ
#undef MITSUBISHI_MSY
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
// 完整覆盖 heatpump_climate.h 中的全部协议 (参考 ESPHome heatpumpir 组件)
static const std::map<Protocol, std::function<HeatpumpIR*()>>& protocol_constructors() {
  static const std::map<Protocol, std::function<HeatpumpIR*()>> table = {
      {Protocol::AUX, []() { return new AUXHeatpumpIR(); }},
      {Protocol::BALLU, []() { return new BalluHeatpumpIR(); }},
      {Protocol::CARRIER_MCA, []() { return new CarrierMCAHeatpumpIR(); }},
      {Protocol::CARRIER_NQV, []() { return new CarrierNQVHeatpumpIR(); }},
      {Protocol::DAIKIN_ARC417, []() { return new DaikinHeatpumpARC417IR(); }},
      {Protocol::DAIKIN_ARC480, []() { return new DaikinHeatpumpARC480A14IR(); }},
      {Protocol::DAIKIN, []() { return new DaikinHeatpumpIR(); }},
      {Protocol::ELECTROLUXYAL, []() { return new ElectroluxYALHeatpumpIR(); }},
      {Protocol::FUEGO, []() { return new FuegoHeatpumpIR(); }},
      {Protocol::FUJITSU_AWYZ, []() { return new FujitsuHeatpumpIR(); }},
      {Protocol::GREE, []() { return new GreeGenericHeatpumpIR(); }},
      {Protocol::GREEYAA, []() { return new GreeYAAHeatpumpIR(); }},
      {Protocol::GREEYAN, []() { return new GreeYANHeatpumpIR(); }},
      {Protocol::GREEYAC, []() { return new GreeYACHeatpumpIR(); }},
      {Protocol::GREEYT, []() { return new GreeYTHeatpumpIR(); }},
      {Protocol::GREEYAP, []() { return new GreeYAPHeatpumpIR(); }},
      {Protocol::HISENSE_AUD, []() { return new HisenseHeatpumpIR(); }},
      {Protocol::HITACHI, []() { return new HitachiHeatpumpIR(); }},
      {Protocol::HYUNDAI, []() { return new HyundaiHeatpumpIR(); }},
      {Protocol::IVT, []() { return new IVTHeatpumpIR(); }},
      {Protocol::MIDEA, []() { return new MideaHeatpumpIR(); }},
      {Protocol::MITSUBISHI_FA, []() { return new MitsubishiFAHeatpumpIR(); }},
      {Protocol::MITSUBISHI_FD, []() { return new MitsubishiFDHeatpumpIR(); }},
      {Protocol::MITSUBISHI_FE, []() { return new MitsubishiFEHeatpumpIR(); }},
      {Protocol::MITSUBISHI_HEAVY_FDTC, []() { return new MitsubishiHeavyFDTCHeatpumpIR(); }},
      {Protocol::MITSUBISHI_HEAVY_ZJ, []() { return new MitsubishiHeavyZJHeatpumpIR(); }},
      {Protocol::MITSUBISHI_HEAVY_ZM, []() { return new MitsubishiHeavyZMHeatpumpIR(); }},
      {Protocol::MITSUBISHI_HEAVY_ZMP, []() { return new MitsubishiHeavyZMPHeatpumpIR(); }},
      {Protocol::MITSUBISHI_KJ, []() { return new MitsubishiKJHeatpumpIR(); }},
      {Protocol::MITSUBISHI_MSC, []() { return new MitsubishiMSCHeatpumpIR(); }},
      {Protocol::MITSUBISHI_MSY, []() { return new MitsubishiMSYHeatpumpIR(); }},
      {Protocol::MITSUBISHI_SEZ, []() { return new MitsubishiSEZKDXXHeatpumpIR(); }},
      {Protocol::PANASONIC_CKP, []() { return new PanasonicCKPHeatpumpIR(); }},
      {Protocol::PANASONIC_DKE, []() { return new PanasonicDKEHeatpumpIR(); }},
      {Protocol::PANASONIC_EKE, []() { return new PanasonicEKEHeatpumpIR(); }},
      {Protocol::PANASONIC_JKE, []() { return new PanasonicJKEHeatpumpIR(); }},
      {Protocol::PANASONIC_LKE, []() { return new PanasonicLKEHeatpumpIR(); }},
      {Protocol::PANASONIC_NKE, []() { return new PanasonicNKEHeatpumpIR(); }},
      {Protocol::SAMSUNG_AQV, []() { return new SamsungAQVHeatpumpIR(); }},
      {Protocol::SAMSUNG_FJM, []() { return new SamsungFJMHeatpumpIR(); }},
      {Protocol::SHARP, []() { return new SharpHeatpumpIR(); }},
      {Protocol::TOSHIBA_DAISEIKAI, []() { return new ToshibaDaiseikaiHeatpumpIR(); }},
      {Protocol::TOSHIBA, []() { return new ToshibaHeatpumpIR(); }},
      {Protocol::ZHLT01, []() { return new ZHLT01HeatpumpIR(); }},
      {Protocol::NIBE, []() { return new NibeHeatpumpIR(); }},
      {Protocol::QLIMA_1, []() { return new Qlima1HeatpumpIR(); }},
      {Protocol::QLIMA_2, []() { return new Qlima2HeatpumpIR(); }},
      {Protocol::SAMSUNG_AQV12MSAN, []() { return new SamsungAQV12MSANHeatpumpIR(); }},
      {Protocol::ZHJG01, []() { return new ZHJG01HeatpumpIR(); }},
      {Protocol::AIRWAY, []() { return new AIRWAYHeatpumpIR(); }},
      {Protocol::BGH_AUD, []() { return new BGHHeatpumpIR(); }},
      {Protocol::PANASONIC_ALTDKE, []() { return new PanasonicAltDKEHeatpumpIR(); }},
      {Protocol::PHILCO_PHS32, []() { return new PhilcoPHS32HeatpumpIR(); }},
      {Protocol::VAILLANTVAI8, []() { return new VaillantHeatpumpIR(); }},
      {Protocol::R51M, []() { return new R51MHeatpumpIR(); }},
  };
  return table;
}

// Protocol 名称 <-> 枚举值映射表 (小写, ESPHome 风格)。单份数据源, 供两个方向使用。
namespace {
struct ProtocolName {
  const char* name;
  Protocol proto;
};
const ProtocolName kProtocols[] = {
    {"aux", Protocol::AUX},
    {"ballu", Protocol::BALLU},
    {"carrier_mca", Protocol::CARRIER_MCA},
    {"carrier_nqv", Protocol::CARRIER_NQV},
    {"daikin_arc417", Protocol::DAIKIN_ARC417},
    {"daikin_arc480", Protocol::DAIKIN_ARC480},
    {"daikin", Protocol::DAIKIN},
    {"electroluxyal", Protocol::ELECTROLUXYAL},
    {"fuego", Protocol::FUEGO},
    {"fujitsu_awy", Protocol::FUJITSU_AWYZ},
    {"fujitsu", Protocol::FUJITSU_AWYZ},
    {"gree", Protocol::GREE},
    {"greeyaa", Protocol::GREEYAA},
    {"greeyan", Protocol::GREEYAN},
    {"greeyac", Protocol::GREEYAC},
    {"greeyt", Protocol::GREEYT},
    {"greeyap", Protocol::GREEYAP},
    {"hisense_aud", Protocol::HISENSE_AUD},
    {"hitachi", Protocol::HITACHI},
    {"hyundai", Protocol::HYUNDAI},
    {"ivt", Protocol::IVT},
    {"midea", Protocol::MIDEA},
    {"mitsubishi_fa", Protocol::MITSUBISHI_FA},
    {"mitsubishi_fd", Protocol::MITSUBISHI_FD},
    {"mitsubishi_fe", Protocol::MITSUBISHI_FE},
    {"mitsubishi_heavy_fdtc", Protocol::MITSUBISHI_HEAVY_FDTC},
    {"mitsubishi_heavy_zj", Protocol::MITSUBISHI_HEAVY_ZJ},
    {"mitsubishi_heavy_zm", Protocol::MITSUBISHI_HEAVY_ZM},
    {"mitsubishi_heavy_zmp", Protocol::MITSUBISHI_HEAVY_ZMP},
    {"mitsubishi_kj", Protocol::MITSUBISHI_KJ},
    {"mitsubishi_msc", Protocol::MITSUBISHI_MSC},
    {"mitsubishi_msy", Protocol::MITSUBISHI_MSY},
    {"mitsubishi_sez", Protocol::MITSUBISHI_SEZ},
    {"panasonic_ckp", Protocol::PANASONIC_CKP},
    {"panasonic_dke", Protocol::PANASONIC_DKE},
    {"panasonic_eke", Protocol::PANASONIC_EKE},
    {"panasonic_jke", Protocol::PANASONIC_JKE},
    {"panasonic_lke", Protocol::PANASONIC_LKE},
    {"panasonic_nke", Protocol::PANASONIC_NKE},
    {"samsung_aqv", Protocol::SAMSUNG_AQV},
    {"samsung_fjm", Protocol::SAMSUNG_FJM},
    {"sharp", Protocol::SHARP},
    {"toshiba_daiseikai", Protocol::TOSHIBA_DAISEIKAI},
    {"toshiba", Protocol::TOSHIBA},
    {"zhlt01", Protocol::ZHLT01},
    {"nibe", Protocol::NIBE},
    {"qlima_1", Protocol::QLIMA_1},
    {"qlima_2", Protocol::QLIMA_2},
    {"samsung_aqv12msan", Protocol::SAMSUNG_AQV12MSAN},
    {"zhjg01", Protocol::ZHJG01},
    {"airway", Protocol::AIRWAY},
    {"bgh_aud", Protocol::BGH_AUD},
    {"panasonic_altdke", Protocol::PANASONIC_ALTDKE},
    {"philco_phs32", Protocol::PHILCO_PHS32},
    {"vaillantvai8", Protocol::VAILLANTVAI8},
    {"r51m", Protocol::R51M},
};
}  // namespace

// 协议名字符串 -> Protocol; 未知时回退到默认协议
Protocol protocol_from_string(const char* name) {
  for (auto& p : kProtocols) {
    if (strcmp(p.name, name) == 0) return p.proto;
  }
  return Protocol::PANASONIC_LKE;  // 默认
}

// Protocol -> 规范协议名字符串; 未知时回退到默认名
const char* protocol_to_string(Protocol protocol) {
  for (auto& p : kProtocols) {
    if (p.proto == protocol) return p.name;
  }
  return "panasonic_lke";  // 默认
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
