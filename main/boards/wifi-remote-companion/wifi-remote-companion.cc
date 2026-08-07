#include "wifi_board.h"
#include "adc_pdm_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include <esp_log.h>

#include "device_state.h"
#include "alarm_manager.h"
#include "codecs/mix_audio_codec.h"

#include "nvs_flash.h"

// ===== IR 遥控伴侣模块 (学习 + 全协议空调控制) =====
// 注意: heatpump_climate.h 必须先于 IRLearner.h 包含——IRLearner.h 通过
// Arduino 兼容层会定义 LOW/HIGH 宏, 会破坏 heatpump_climate.h 中的枚举声明
#include <remote_transmitter.h>
#include <heatpump_climate.h>
#include <IRLearner.h>

// HeatpumpIRCompat.h 定义的 LOW/HIGH 宏与代码中其它枚举值同名, 取消定义
#undef LOW
#undef HIGH

#ifdef CONFIG_ESP_HI_WEB_CONTROL_ENABLED
#include "esp_hi_web_control.h"
#endif //CONFIG_ESP_HI_WEB_CONTROL_ENABLED

#define TAG "WIFI-REMOTE-COMPANION"

// 学习结果 NVS 持久化
#define IR_NVS_NAMESPACE "ir_keys"
#define IR_NVS_KEY       "learned"
#define IR_NVS_AC_PROTOCOL_KEY "ac_protocol"

using heatpump_ir_tx::RemoteTransmitter;
using heatpump_ir_tx::RemoteTransmitData;
using heatpump_ir_tx::HeatPumpClimate;

class WifiRemoteCompanion : public WifiBoard {
private:
    Button boot_button_;
    Button audio_wake_button_;
    Button move_wake_button_;
    MixAudioCodec* mix_audio_codec_ = nullptr;

    // 红外遥控硬件
    RemoteTransmitter ir_tx_;
    HeatPumpClimate climate_;
    std::string ac_protocol_name_{""};  // 当前空调品牌协议名
    bool ac_protocol_configured_ = false;            // 是否已显式设置过品牌(NVS/set_protocol)
    IRLearner* ir_learner_ = nullptr;
    TaskHandle_t ir_task_ = nullptr;

    static void IRLoopTask(void* arg) {
        auto* self = static_cast<WifiRemoteCompanion*>(arg);
        uint32_t idle_ticks = 0;
        bool was_learning = false;
        while (true) {
            if (self->ir_learner_) {
                self->ir_learner_->loop();
            }
            // 学习会话刚结束(isLearning true->false) → 自动保存学习结果到 NVS
            bool learning = self->ir_learner_ ? self->ir_learner_->isLearning() : false;
            if (was_learning && !learning) {
                self->SaveIRKeysToNVS();
            }
            was_learning = learning;

            // 无学习/回放任务时, 空闲约 1 秒(50*20ms)后自挂起, 由 EnsureIRLoopTask 唤醒,
            // 避免一直空转; 挂起前留 1 秒余量以消除与 MCP 回调(启动学习)之间的竞态
            if (self->ir_learner_ && !learning) {
                if (++idle_ticks >= 50) {
                    idle_ticks = 0;
                    vTaskSuspend(nullptr);
                }
            } else {
                idle_ticks = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    // 按需创建/唤醒 ir_loop 任务 (开始学习或回放前调用)
    void EnsureIRLoopTask() {
        if (ir_task_ == nullptr) {
            xTaskCreate(IRLoopTask, "ir_loop", 6144, this, 1, &ir_task_);
        } else {
            vTaskResume(ir_task_);
        }
    }

    // ===== 空调品牌协议持久化 =====
    void SaveACProtocolToNVS(const std::string& name) {
        nvs_handle_t h;
        if (nvs_open(IR_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
        esp_err_t err = nvs_set_str(h, IR_NVS_AC_PROTOCOL_KEY, name.c_str());
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "空调品牌协议已保存: %s", name.c_str());
        } else {
            ESP_LOGE(TAG, "保存空调品牌协议失败: %s", esp_err_to_name(err));
        }
    }

    std::string LoadACProtocolFromNVS() {
        nvs_handle_t h;
        if (nvs_open(IR_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
            return "";
        }
        char buf[32] = {0};
        size_t len = sizeof(buf);
        esp_err_t err = nvs_get_str(h, IR_NVS_AC_PROTOCOL_KEY, buf, &len);
        nvs_close(h);
        if (err != ESP_OK) return "";
        return std::string(buf);
    }

    // 清除已保存的空调品牌协议
    void ClearACProtocolFromNVS() {
        nvs_handle_t h;
        if (nvs_open(IR_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
        esp_err_t err = nvs_erase_key(h, IR_NVS_AC_PROTOCOL_KEY);
        if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;  // 本来就没有, 不算错误
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "空调品牌协议已清除");
        } else {
            ESP_LOGE(TAG, "清除空调品牌协议失败: %s", esp_err_to_name(err));
        }
    }

    // ===== 学习结果 NVS 持久化 =====
    static void PutU16(std::vector<uint8_t>& buf, size_t& off, uint16_t v) {
        buf[off++] = static_cast<uint8_t>(v & 0xFF);
        buf[off++] = static_cast<uint8_t>((v >> 8) & 0xFF);
    }

    static uint16_t GetU16(const std::vector<uint8_t>& buf, size_t& off) {
        uint16_t v = static_cast<uint16_t>(buf[off]) |
                     (static_cast<uint16_t>(buf[off + 1]) << 8);
        off += 2;
        return v;
    }

    // 将已学习按键(仅 isLearned 且有原始码)序列化到 NVS
    void SaveIRKeysToNVS() {
        if (ir_learner_ == nullptr) return;
        auto& keys = ir_learner_->getKeys();
        uint16_t count = 0;
        for (auto& k : keys) {
            if (k.isLearned && !k.rawData.empty()) count++;
        }
        nvs_handle_t h;
        if (nvs_open(IR_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
            ESP_LOGE(TAG, "nvs_open(%s) failed", IR_NVS_NAMESPACE);
            return;
        }
        if (count == 0) {
            nvs_erase_key(h, IR_NVS_KEY);  // 无已学习按键 → 清空
            nvs_commit(h);
            nvs_close(h);
            ESP_LOGI(TAG, "无已学习按键, 已清空 NVS");
            return;
        }
        // 布局: [count u16] 每键: [name_len u16][name][raw_count u16][raw u16...][freq u16]
        size_t total = 2;
        for (auto& k : keys) {
            if (!k.isLearned || k.rawData.empty()) continue;
            total += 2 + k.name.size() + 2 + k.rawData.size() * 2 + 2;
        }
        std::vector<uint8_t> buf(total);
        size_t off = 0;
        PutU16(buf, off, count);
        for (auto& k : keys) {
            if (!k.isLearned || k.rawData.empty()) continue;
            PutU16(buf, off, static_cast<uint16_t>(k.name.size()));
            memcpy(buf.data() + off, k.name.data(), k.name.size());
            off += k.name.size();
            PutU16(buf, off, static_cast<uint16_t>(k.rawData.size()));
            for (auto v : k.rawData) PutU16(buf, off, v);
            PutU16(buf, off, k.frequency);
        }
        esp_err_t err = nvs_set_blob(h, IR_NVS_KEY, buf.data(), total);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "保存按键到 NVS 失败: %s", esp_err_to_name(err));
            return;
        }
        ESP_LOGI(TAG, "已保存 %d 个按键到 NVS (%d bytes)", count, (int)total);
    }

    // 开机从 NVS 恢复上次学习结果
    void LoadIRKeysFromNVS() {
        if (ir_learner_ == nullptr) return;
        nvs_handle_t h;
        if (nvs_open(IR_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
            return;  // 从未保存过
        }
        size_t len = 0;
        if (nvs_get_blob(h, IR_NVS_KEY, nullptr, &len) != ESP_OK || len < 2) {
            nvs_close(h);
            return;
        }
        std::vector<uint8_t> buf(len);
        if (nvs_get_blob(h, IR_NVS_KEY, buf.data(), &len) != ESP_OK) {
            nvs_close(h);
            return;
        }
        nvs_close(h);

        size_t off = 0;
        uint16_t count = GetU16(buf, off);
        uint16_t loaded = 0;
        for (uint16_t i = 0; i < count && off + 2 <= len; i++) {
            uint16_t name_len = GetU16(buf, off);
            if (off + name_len > len) break;
            std::string name(reinterpret_cast<const char*>(buf.data() + off), name_len);
            off += name_len;
            if (off + 2 > len) break;
            uint16_t raw_count = GetU16(buf, off);
            if (off + static_cast<size_t>(raw_count) * 2 > len) break;
            std::vector<uint16_t> raw(raw_count);
            for (uint16_t j = 0; j < raw_count; j++) raw[j] = GetU16(buf, off);
            if (off + 2 > len) break;
            uint16_t freq = GetU16(buf, off);

            IRRawKey k;
            k.name = name;
            k.rawData = raw;
            k.frequency = freq;
            k.isLearned = true;
            ir_learner_->getKeys().push_back(k);
            loaded++;
        }
        if (loaded > 0) {
            ESP_LOGI(TAG, "已从 NVS 恢复 %d 个按键", loaded);
        }
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto &app = Application::GetInstance();
            // During startup (before connected), pressing BOOT button enters Wi-Fi config mode without reboot
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeIR() {
        ESP_LOGI(TAG, "Initialize IR: TX=GPIO%d RX=GPIO%d", IR_TX_GPIO, IR_RX_GPIO);

        esp_err_t err = ir_tx_.begin(IR_TX_GPIO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "IR transmitter begin failed: %s", esp_err_to_name(err));
        }
        ir_tx_.set_carrier_duty_percent(33);

        // 空调控制: 默认协议 Panasonic DKE(本机空调型号), 可通过 self.ac.set/set_protocol 切换品牌并持久化
        esp_err_t cerr = climate_.begin(&ir_tx_, heatpump_ir_tx::Protocol::PANASONIC_DKE);
        if (cerr != ESP_OK) {
            ESP_LOGE(TAG, "AC climate begin failed: %s", esp_err_to_name(cerr));
        }
        climate_.set_min_temperature(16.0f);
        climate_.set_max_temperature(30.0f);

        // 开机恢复上次保存的空调品牌协议
        std::string saved_proto = LoadACProtocolFromNVS();
        if (!saved_proto.empty()) {
            auto proto = heatpump_ir_tx::protocol_from_string(saved_proto.c_str());
            if (climate_.set_protocol(proto) == ESP_OK) {
                ac_protocol_name_ = heatpump_ir_tx::protocol_to_string(proto);
                ac_protocol_configured_ = true;  // 持久化过的品牌同样视为已配置
                ESP_LOGI(TAG, "空调品牌协议(已保存): %s", ac_protocol_name_.c_str());
            }
        }

        // 红外学习: 接收用 GPIO 中断, 回放共用 RMT 发射器
        ir_learner_ = new IRLearner(IR_RX_GPIO, IR_TX_GPIO);
        ir_learner_->setup();
        ir_learner_->setIRSender(new IRSenderRMT(&ir_tx_));

        // 开机从 NVS 恢复上次学习结果 (学习完成后由 ir_loop 任务自动保存)
        LoadIRKeysFromNVS();

        // 开机预置默认学习按键(空调), 保证按键列表始终已初始化;
        // 之后可通过 self.ir.learn_start 的 preset/keys 参数覆盖
        // ir_learner_->addTargetKey("电源");
        // ir_learner_->addTargetKey("模式");
        // ir_learner_->addTargetKey("温度+");
        // ir_learner_->addTargetKey("温度-");
        // ir_learner_->addTargetKey("风速");
        // ir_learner_->addTargetKey("上下扫风");
        // ir_learner_->addTargetKey("左右扫风");
        // ir_learner_->addTargetKey("定时");
        //         ESP_LOGI(TAG, "预置学习按键: 电源/模式/温度+/温度-/风速/上下扫风/左右扫风/定时");
        // 注意: ir_loop 任务不在此创建, 首次学习/回放时才按需启动 (EnsureIRLoopTask)
        EnsureIRLoopTask();
        ESP_LOGI(TAG, "IR learner ready");
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();

        // ========== 空调控制 (全协议) ==========
        mcp_server.AddTool("self.ac.set_protocol",
            "设置并记住空调品牌协议(保存到 NVS, 之后 self.ac.set 无需再传 protocol)。重要约束: 只有在用户明确说出空调品牌/型号时才能调用本工具; 禁止猜测、推断或默认空调品牌(如用户只说'打开空调'而未告知品牌, 绝不能调用本工具, 必须先向用户询问'你的空调是什么品牌')。protocol 可选: aux/ballu/carrier_mca/carrier_nqv/daikin_arc417/daikin_arc480/daikin/electroluxyal/fuego/fujitsu/gree/greeyaa/greeyan/greeyac/greeyt/greeyap/hisense_aud/hitachi/hyundai/ivt/midea/mitsubishi_fa/mitsubishi_fd/mitsubishi_fe/mitsubishi_heavy_fdtc/mitsubishi_heavy_zj/mitsubishi_heavy_zm/mitsubishi_heavy_zmp/mitsubishi_kj/mitsubishi_msc/mitsubishi_msy/mitsubishi_sez/panasonic_ckp/panasonic_dke/panasonic_eke/panasonic_jke/panasonic_lke/panasonic_nke/samsung_aqv/samsung_fjm/sharp/toshiba_daiseikai/toshiba/zhlt01/nibe/qlima_1/qlima_2/samsung_aqv12msan/zhjg01/airway/bgh_aud/panasonic_altdke/philco_phs32/vaillantvai8/r51m",
            PropertyList({
                Property("protocol", kPropertyTypeString, std::string(""))
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string protocol = properties["protocol"].value<std::string>();
                if (protocol.empty()) {
                    return "{\"success\": false, \"message\": \"未提供空调品牌 protocol, 请先询问用户空调品牌后再调用本工具\"}";
                }
                auto proto = heatpump_ir_tx::protocol_from_string(protocol.c_str());
                esp_err_t err = climate_.set_protocol(proto);
                if (err != ESP_OK) {
                    char resp[160];
                    snprintf(resp, sizeof(resp), "{\"success\": false, \"message\": \"protocol not supported: %s\"}", protocol.c_str());
                    return std::string(resp);
                }
                ac_protocol_name_ = heatpump_ir_tx::protocol_to_string(proto);
                ac_protocol_configured_ = true;
                SaveACProtocolToNVS(ac_protocol_name_);
                char resp[256];
                snprintf(resp, sizeof(resp), "{\"success\": true, \"message\": \"空调品牌已设为 %s 并保存\"}", ac_protocol_name_.c_str());
                return std::string(resp);
            });

        mcp_server.AddTool("self.ac.set",
            "设置空调状态并通过红外发送控制信号。protocol 可空(使用已设置的空调品牌)或直接指定品牌; 若从未设置过品牌, 本工具会拒绝发送并提示先调用 self.ac.set_protocol 告知品牌。mode: off/cool/heat/auto/fan/dry; 重要: 用户要求'打开/开启空调'时, 必须传开机模式(如 cool/heat/auto/fan), 绝不能因为 self.ac.get 返回的当前状态是 off 就发送 off(off 只在用户明确说'关闭/关机'时才传); temperature: 16-30; fan: auto/low/medium/high; swing: off/horizontal/vertical/both",
            PropertyList({
                Property("protocol", kPropertyTypeString, std::string("")),
                Property("mode", kPropertyTypeString, std::string("cool")),
                Property("temperature", kPropertyTypeInteger, 24, 16, 30),
                Property("fan", kPropertyTypeString, std::string("auto")),
                Property("swing", kPropertyTypeString, std::string("off"))
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                // 若传入了 protocol 则切换并记住; 否则必须已显式设置过品牌才能发送
                std::string protocol = properties["protocol"].value<std::string>();
                if (!protocol.empty()) {
                    auto proto = heatpump_ir_tx::protocol_from_string(protocol.c_str());
                    esp_err_t err = climate_.set_protocol(proto);
                    if (err != ESP_OK) {
                        char resp[160];
                        snprintf(resp, sizeof(resp), "{\"success\": false, \"message\": \"protocol not supported: %s\"}", protocol.c_str());
                        return std::string(resp);
                    }
                    ac_protocol_name_ = heatpump_ir_tx::protocol_to_string(proto);
                    ac_protocol_configured_ = true;
                    SaveACProtocolToNVS(ac_protocol_name_);
                } else if (!ac_protocol_configured_) {
                    // 从未明确设置过空调品牌 → 拒绝盲发, 引导先设置品牌
                    return "{\"success\": false, \"message\": \"尚未设置空调品牌，请先调用 self.ac.set_protocol 告知空调品牌(如 gree/panasonic_lke/midea/daikin)\"}";
                } else {
                    protocol = ac_protocol_name_;
                }

                std::string mode = properties["mode"].value<std::string>();
                auto& state = climate_.state();

                if (mode == "off") {
                    state.mode = heatpump_ir_tx::ClimateMode::OFF;
                } else if (mode == "cool") {
                    state.mode = heatpump_ir_tx::ClimateMode::COOL;
                } else if (mode == "heat") {
                    state.mode = heatpump_ir_tx::ClimateMode::HEAT;
                } else if (mode == "auto") {
                    state.mode = heatpump_ir_tx::ClimateMode::HEAT_COOL;
                } else if (mode == "fan") {
                    state.mode = heatpump_ir_tx::ClimateMode::FAN_ONLY;
                } else if (mode == "dry") {
                    state.mode = heatpump_ir_tx::ClimateMode::DRY;
                } else {
                    return "{\"success\": false, \"message\": \"invalid mode, use off/cool/heat/auto/fan/dry\"}";
                }

                state.target_temperature = static_cast<float>(properties["temperature"].value<int>());

                std::string fan = properties["fan"].value<std::string>();
                if (fan == "low") {
                    state.fan = heatpump_ir_tx::ClimateFanMode::LOW;
                } else if (fan == "medium") {
                    state.fan = heatpump_ir_tx::ClimateFanMode::MEDIUM;
                } else if (fan == "high") {
                    state.fan = heatpump_ir_tx::ClimateFanMode::HIGH;
                } else {
                    state.fan = heatpump_ir_tx::ClimateFanMode::AUTO;
                }

                std::string swing = properties["swing"].value<std::string>();
                if (swing == "horizontal") {
                    state.swing = heatpump_ir_tx::ClimateSwingMode::HORIZONTAL;
                } else if (swing == "vertical") {
                    state.swing = heatpump_ir_tx::ClimateSwingMode::VERTICAL;
                } else if (swing == "both") {
                    state.swing = heatpump_ir_tx::ClimateSwingMode::BOTH;
                } else {
                    state.swing = heatpump_ir_tx::ClimateSwingMode::OFF;
                }

                esp_err_t err = climate_.transmit_state();
                ESP_LOGI("WIFI-REMOTE-COMPANION", "self.ac.set => protocol=%s mode=%s temp=%d fan=%s swing=%s, transmit=%s",
                         protocol.c_str(), mode.c_str(), properties["temperature"].value<int>(),
                         fan.c_str(), swing.c_str(), esp_err_to_name(err));
                if (err != ESP_OK) {
                    char resp[128];
                    snprintf(resp, sizeof(resp), "{\"success\": false, \"message\": \"transmit failed: %s\"}", esp_err_to_name(err));
                    return std::string(resp);
                }
                char resp[320];
                snprintf(resp, sizeof(resp), "{\"success\": true, \"message\": \"AC sent: protocol=%s mode=%s temp=%d fan=%s swing=%s\"}",
                         protocol.c_str(), mode.c_str(), properties["temperature"].value<int>(), fan.c_str(), swing.c_str());
                return std::string(resp);
            });

        mcp_server.AddTool("self.ac.get",
            "获取当前空调状态(品牌协议/温度/模式)",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                auto& s = climate_.state();
                const char* mode = "off";
                switch (s.mode) {
                    case heatpump_ir_tx::ClimateMode::COOL: mode = "cool"; break;
                    case heatpump_ir_tx::ClimateMode::HEAT: mode = "heat"; break;
                    case heatpump_ir_tx::ClimateMode::HEAT_COOL: mode = "auto"; break;
                    case heatpump_ir_tx::ClimateMode::FAN_ONLY: mode = "fan"; break;
                    case heatpump_ir_tx::ClimateMode::DRY: mode = "dry"; break;
                    default: mode = "off"; break;
                }
                char resp[320];
                snprintf(resp, sizeof(resp), "{\"success\": true, \"protocol\": \"%s\", \"mode\": \"%s\", \"temperature\": %d}",
                         ac_protocol_name_.c_str(), mode, (int)s.target_temperature);
                return std::string(resp);
            });

        mcp_server.AddTool("self.ac.reset",
            "清除已保存的空调品牌协议和当前空调状态(清空 NVS 中的品牌记录, 释放协议对象)。"
            "调用后 self.ac.set 将拒绝发送直到再次通过 self.ac.set_protocol 设置品牌。"
            "当用户说'清除空调数据/重新设置空调'时应调用本工具",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                ClearACProtocolFromNVS();
                climate_.reset();                     // 释放当前协议对象
                ac_protocol_name_ = "";               // 清空当前品牌
                ac_protocol_configured_ = false;      // 标记未配置
                return "{\"success\": true, \"message\": \"空调品牌协议和状态已清除\"}";
            });

        // ========== 红外学习/回放 ==========
        mcp_server.AddTool("self.ir.learn_start",
            "开始红外学习。preset 可选 air_conditioner(空调)/tv(电视)/custom(自定义, 默认); custom 时用 keys 指定逗号分隔的按键名列表, 如 \"电源,模式,温度+,温度-\"。学习过程中请按提示依次用遥控器对准接收头按键",
            PropertyList({
                Property("preset", kPropertyTypeString, std::string("custom")),
                Property("keys", kPropertyTypeString, std::string(""))
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (ir_learner_ == nullptr) {
                    return "{\"success\": false, \"message\": \"IR learner not initialized\"}";
                }
                std::string preset = properties["preset"].value<std::string>();
                if (preset == "air_conditioner") {
                    // EnsureIRLoopTask();  // 学习需要 ir_loop 任务驱动捕获
                    ir_learner_->learnAirConditioner();  // 内部会 reset + 添加空调按键 + startLearning
                    return "{\"success\": true, \"message\": \"开始学习空调按键: 电源/模式/温度+/温度-/风速/上下扫风/左右扫风/定时\"}";
                }
                if (preset == "tv") {
                    // EnsureIRLoopTask();  // 学习需要 ir_loop 任务驱动捕获
                    ir_learner_->learnTV();  // 内部会 reset + 添加电视按键 + startLearning
                    return "{\"success\": true, \"message\": \"开始学习电视按键: 电源/信号源/音量+/音量-/频道+/频道-/静音/菜单\"}";
                }

                // custom: 解析 keys (兼容中英文逗号与空格)
                ir_learner_->reset();
                std::string keys = properties["keys"].value<std::string>();
                for (auto& ch : keys) {
                    if (ch == '，' || ch == ',') ch = ',';
                }
                size_t pos = 0;
                while ((pos = keys.find(',')) != std::string::npos) {
                    std::string key = keys.substr(0, pos);
                    key.erase(0, key.find_first_not_of(" \t\r\n"));
                    key.erase(key.find_last_not_of(" \t\r\n") + 1);
                    if (!key.empty()) ir_learner_->addTargetKey(key);
                    keys.erase(0, pos + 1);
                }
                keys.erase(0, keys.find_first_not_of(" \t\r\n"));
                keys.erase(keys.find_last_not_of(" \t\r\n") + 1);
                if (!keys.empty()) ir_learner_->addTargetKey(keys);

                if (ir_learner_->getKeys().empty()) {
                    return "{\"success\": false, \"message\": \"no keys specified\"}";
                }
                // EnsureIRLoopTask();  // 学习需要 ir_loop 任务驱动捕获
                ir_learner_->startLearning();
                return "{\"success\": true, \"message\": \"开始学习, 请按提示依次按键\"}";
            });

        mcp_server.AddTool("self.ir.learn_status",
            "查询红外学习状态、当前按键清单与提示(应轮询此工具获知下一个要按的键)",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (ir_learner_ == nullptr) {
                    return "{\"success\": false, \"message\": \"IR learner not initialized\"}";
                }
                std::string msg = ir_learner_->getStatusMessage().c_str();
                auto& keys = ir_learner_->getKeys();
                std::string keylist = "[";
                for (size_t i = 0; i < keys.size(); i++) {
                    char b[64];
                    snprintf(b, sizeof(b), "%s\"%s\"", i ? "," : "", keys[i].name.c_str());
                    keylist += b;
                }
                keylist += "]";
                char resp[512];
                snprintf(resp, sizeof(resp), "{\"success\": true, \"learning\": %s, \"keys\": %s, \"message\": \"%s\"}",
                         ir_learner_->isLearning() ? "true" : "false", keylist.c_str(), msg.c_str());
                return std::string(resp);
            });

        mcp_server.AddTool("self.ir.play",
            "回放已学习的按键。index 为按键序号, 可通过 self.ir.keys 查询",
            PropertyList({
                Property("index", kPropertyTypeInteger, 0, 0, 100)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int index = properties["index"].value<int>();
                auto& keys = ir_learner_->getKeys();
                if (index < 0 || index >= (int)keys.size()) {
                    return "{\"success\": false, \"message\": \"index out of range\"}";
                }
                ir_learner_->playKey(index);
                // EnsureIRLoopTask();  // 回放需要 ir_loop 任务执行实际发送
                char resp[256];
                snprintf(resp, sizeof(resp), "{\"success\": true, \"message\": \"playing [%s]\", \"index\": %d}",
                         keys[index].name.c_str(), index);
                return std::string(resp);
            });

        mcp_server.AddTool("self.ir.skip",
            "跳过当前正在学习的按键",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                ir_learner_->skipCurrentKey();
                return "{\"success\": true, \"message\": \"skipped\"}";
            });

        mcp_server.AddTool("self.ir.reset",
            "清空所有已学习按键并停止学习",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                ir_learner_->reset();
                return "{\"success\": true, \"message\": \"reset\"}";
            });

        mcp_server.AddTool("self.ir.keys",
            "列出所有已学习/待学习的按键",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                auto& keys = ir_learner_->getKeys();
                std::string resp = "{\"success\": true, \"keys\": [";
                for (size_t i = 0; i < keys.size(); i++) {
                    char line[256];
                    snprintf(line, sizeof(line), "{\"index\": %d, \"name\": \"%s\", \"learned\": %s}",
                             (int)i, keys[i].name.c_str(), keys[i].isLearned ? "true" : "false");
                    resp += line;
                    if (i < keys.size() - 1) resp += ",";
                }
                resp += "]}";
                return resp;
            });

        // ========== 闹钟 ==========
        mcp_server.AddTool("self.alarm.add",
            "Add an alarm clock with specified time and optional label.",
            PropertyList({
                Property("hour", kPropertyTypeInteger, 0, 23),// "Hour of the alarm (0-23)"),
                Property("minute", kPropertyTypeInteger, 0, 59), // "Minute of the alarm (0-59)"),
                Property("label", kPropertyTypeString, std::string("")) // "Optional label for the alarm", "")
            }),
            [](const PropertyList& properties) -> ReturnValue {
                int hour = properties["hour"].value<int>();
                int minute = properties["minute"].value<int>();
                std::string label = properties["label"].value<std::string>();

                auto& alarm_manager = AlarmManager::GetInstance();
                int alarm_id = alarm_manager.AddAlarm(hour, minute, label);

                char response[150];
                snprintf(response, sizeof(response), "{\"success\": true, \"message\": \"Alarm set for %02d:%02d\", \"alarm_id\": %d}", hour, minute, alarm_id);
                return std::string(response);
            });

        mcp_server.AddTool("self.alarm.remove",
            "Remove an alarm by ID.",
            PropertyList({
                Property("alarm_id", kPropertyTypeInteger, 1, 100) // "Alarm ID to remove")
            }),
            [](const PropertyList& properties) -> ReturnValue {
                int alarm_id = properties["alarm_id"].value<int>();

                auto& alarm_manager = AlarmManager::GetInstance();
                bool success = alarm_manager.RemoveAlarm(alarm_id);

                if (success) {
                    return "{\"success\": true, \"message\": \"Alarm removed successfully\"}";
                } else {
                    return "{\"success\": false, \"message\": \"Failed to remove alarm\"}";
                }
            });

        mcp_server.AddTool("self.alarm.list",
            "List all configured alarms.",
            PropertyList(),
            [](const PropertyList&) -> ReturnValue {
                auto& alarm_manager = AlarmManager::GetInstance();
                const auto& alarms = alarm_manager.GetAlarms();

                if (alarms.empty()) {
                    return "{\"success\": true, \"message\": \"No alarms set\", \"alarms\": []}";
                }

                std::string response = "{\"success\": true, \"alarms\": [";
                for (size_t i = 0; i < alarms.size(); i++) {
                    const auto& alarm = alarms[i];
                    char line[100];
                    snprintf(line, sizeof(line), "{\"id\": %d, \"hour\": %d, \"minute\": %d, \"label\": \"%s\"}",
                            i + 1, alarm.GetHour(), alarm.GetMinute(), alarm.GetLabel().c_str());
                    response += line;
                    if (i < alarms.size() - 1) {
                        response += ",";
                    }
                }
                response += "]}";

                return response;
            });
    }

public:
    WifiRemoteCompanion() : boot_button_(BOOT_BUTTON_GPIO),
        audio_wake_button_(AUDIO_WAKE_BUTTON_GPIO),
        move_wake_button_(MOVE_WAKE_BUTTON_GPIO)
    {
        InitializeButtons();
        InitializeIR();
        InitializeTools();
    }

    virtual AudioCodec* GetAudioCodec() override
    {
        // if (mix_audio_codec_ == nullptr) {
            static AdcPdmAudioCodec audio_codec(
                AUDIO_INPUT_SAMPLE_RATE,
                AUDIO_OUTPUT_SAMPLE_RATE,
                AUDIO_ADC_MIC_CHANNEL,
                AUDIO_PDM_SPEAK_P_GPIO,
                AUDIO_PDM_SPEAK_N_GPIO,
                AUDIO_PA_CTL_GPIO);
            return &audio_codec;
        //     mix_audio_codec_ = new MixAudioCodec(&audio_codec);
        // }
        // return mix_audio_codec_;
    }

    // 无屏幕板卡：不重写 GetDisplay()，使用基类默认的 NoDisplay
};

DECLARE_BOARD(WifiRemoteCompanion);
