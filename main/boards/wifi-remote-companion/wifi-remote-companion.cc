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
#include "assets/lang_config.h"

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
        // 捕获到按键时播放提示音, 让用户知道信号收到、可以松开遥控器并说"下一个"
        ir_learner_->setOnKeyCaptured([]() {
            Application::GetInstance().PlaySound(Lang::Sounds::OGG_SUCCESS);
        });

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
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();

        // ========== 空调控制 (全协议) ==========
        mcp_server.AddTool("self.ac.set_protocol",
            "设置并记住空调品牌协议(保存到 NVS, 之后 self.ac.set 无需再传 protocol)。重要约束: 只有在用户明确说出空调品牌/型号时才能调用本工具; 学习的时候不用调这个工具设置品牌；禁止猜测、推断或默认空调品牌(如用户只说'打开空调'而未告知品牌, 绝不能调用本工具, 必须先向用户询问'你的空调是什么品牌')。protocol 可选: aux/ballu/carrier_mca/carrier_nqv/daikin_arc417/daikin_arc480/daikin/electroluxyal/fuego/fujitsu/gree/greeyaa/greeyan/greeyac/greeyt/greeyap/hisense_aud/hitachi/hyundai/ivt/midea/mitsubishi_fa/mitsubishi_fd/mitsubishi_fe/mitsubishi_heavy_fdtc/mitsubishi_heavy_zj/mitsubishi_heavy_zm/mitsubishi_heavy_zmp/mitsubishi_kj/mitsubishi_msc/mitsubishi_msy/mitsubishi_sez/panasonic_ckp/panasonic_dke/panasonic_eke/panasonic_jke/panasonic_lke/panasonic_nke/samsung_aqv/samsung_fjm/sharp/toshiba_daiseikai/toshiba/zhlt01/nibe/qlima_1/qlima_2/samsung_aqv12msan/zhjg01/airway/bgh_aud/panasonic_altdke/philco_phs32/vaillantvai8/r51m",
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
            "设置空调状态并通过红外发送控制信号。protocol 可空: 未传 protocol 时优先使用本地学习到的红外码发送(开机/关机→[电源]键, 切换模式→[模式]键, 调高/调低温度→[温度+]/[温度-]键); 本地没有学习码时, 再使用已设置的品牌协议, 若品牌也未设置则提示先调用 self.ac.set_protocol。protocol 也可直接指定品牌。mode: off/cool/heat/auto/fan/dry; 重要: 用户要求'打开/开启空调'时, 必须传开机模式(如 cool/heat/auto/fan), 绝不能因为 self.ac.get 返回的当前状态是 off 就发送 off(off 只在用户明确说'关闭/关机'时才传); temperature: 16-30; fan: auto/low/medium/high; swing: off/horizontal/vertical/both",
            PropertyList({
                Property("protocol", kPropertyTypeString, std::string("")),
                Property("mode", kPropertyTypeString, std::string("cool")),
                Property("temperature", kPropertyTypeInteger, 24, 16, 30),
                Property("fan", kPropertyTypeString, std::string("auto")),
                Property("swing", kPropertyTypeString, std::string("off"))
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string mode = properties["mode"].value<std::string>();
                auto& state = climate_.state();
                int target_temp = properties["temperature"].value<int>();

                // 解析 mode → 协议枚举
                heatpump_ir_tx::ClimateMode want_mode;
                if (mode == "off") {
                    want_mode = heatpump_ir_tx::ClimateMode::OFF;
                } else if (mode == "cool") {
                    want_mode = heatpump_ir_tx::ClimateMode::COOL;
                } else if (mode == "heat") {
                    want_mode = heatpump_ir_tx::ClimateMode::HEAT;
                } else if (mode == "auto") {
                    want_mode = heatpump_ir_tx::ClimateMode::HEAT_COOL;
                } else if (mode == "fan") {
                    want_mode = heatpump_ir_tx::ClimateMode::FAN_ONLY;
                } else if (mode == "dry") {
                    want_mode = heatpump_ir_tx::ClimateMode::DRY;
                } else {
                    return "{\"success\": false, \"message\": \"invalid mode, use off/cool/heat/auto/fan/dry\"}";
                }

                std::string protocol = properties["protocol"].value<std::string>();
                if (!protocol.empty()) {
                    // 指定了品牌 → 切换并记住
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
                } else if (ir_learner_) {
                    // 没传 protocol → 优先使用本地学习码(用户学习过的红外按键)
                    // 指令 → 学习键 映射(按优先级)
                    std::vector<std::string> candidates;
                    if (mode == "off") {
                        candidates.push_back("电源");  // 关机 → 电源键
                    } else {
                        candidates.push_back("电源");  // 开机 → 电源键
                        if (state.mode != heatpump_ir_tx::ClimateMode::OFF &&
                            state.mode != want_mode) {
                            candidates.push_back("模式");  // 切换模式 → 模式键
                        }
                        if ((int)state.target_temperature != target_temp) {
                            candidates.push_back(target_temp > (int)state.target_temperature ? "温度+" : "温度-");
                        }
                    }
                    // 按优先级在本地学习键中查找已学习的键
                    int play_index = -1;
                    std::string play_name;
                    auto& keys = ir_learner_->getKeys();
                    for (const auto& c : candidates) {
                        for (size_t i = 0; i < keys.size(); i++) {
                            if (keys[i].isLearned && !keys[i].rawData.empty() &&
                                keys[i].name == c) {
                                play_index = (int)i;
                                play_name = c;
                                break;
                            }
                        }
                        if (play_index >= 0) break;
                    }
                    if (play_index >= 0) {
                        EnsureIRLoopTask();  // 唤醒 ir_loop 任务执行实际发送(空闲后会自挂起)
                        ir_learner_->playKey(play_index);
                        // 记录状态, 供后续温度/模式升降判断
                        state.mode = want_mode;
                        state.target_temperature = static_cast<float>(target_temp);
                        char resp[192];
                        snprintf(resp, sizeof(resp), "{\"success\": true, \"message\": \"使用本地学习码发送[%s]键\"}", play_name.c_str());
                        return std::string(resp);
                    }
                    // 本地没有学习码 → 品牌已设置则用品牌, 否则提示设置品牌
                    if (!ac_protocol_configured_) {
                        return "{\"success\": false, \"message\": \"尚未设置空调品牌，请先调用 self.ac.set_protocol 告知空调品牌(如 gree/panasonic_lke/midea/daikin)\"}";
                    }
                    protocol = ac_protocol_name_;
                } else {
                    // 学习者未初始化 → 品牌已设置则用品牌, 否则提示设置品牌
                    if (!ac_protocol_configured_) {
                        return "{\"success\": false, \"message\": \"尚未设置空调品牌，请先调用 self.ac.set_protocol 告知空调品牌(如 gree/panasonic_lke/midea/daikin)\"}";
                    }
                    protocol = ac_protocol_name_;
                }

                state.mode = want_mode;
                state.target_temperature = static_cast<float>(target_temp);

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
            "获取当前空调状态和控制来源。返回: protocol(品牌协议名, 未设置时可能为空), mode, temperature, source(控制来源: learned=使用本地学习码, protocol=使用品牌协议), learned_keys(已学习的红外按键列表, 逗号分隔)。"
            "当用户询问'按键是本地学习的还是内置协议的/当前用哪种方式控制'时, 根据 source 和 learned_keys 如实回答",
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
                // 统计本地学习到的按键
                std::string learned_keys;
                int learned_count = 0;
                const char* source = "protocol";  // 控制来源: 本地学习码 or 品牌协议
                if (ir_learner_) {
                    auto& keys = ir_learner_->getKeys();
                    for (auto& k : keys) {
                        if (k.isLearned && !k.rawData.empty()) {
                            if (learned_count > 0) learned_keys += ",";
                            learned_keys += k.name;
                            learned_count++;
                            // 本地学过空调相关键(电源/模式/温度±) → 控制来源为学习码
                            if (k.name == "电源" || k.name == "模式" ||
                                k.name == "温度+" || k.name == "温度-") {
                                source = "learned";
                            }
                        }
                    }
                }
                char resp[512];
                snprintf(resp, sizeof(resp), "{\"success\": true, \"protocol\": \"%s\", \"mode\": \"%s\", \"temperature\": %d, \"source\": \"%s\", \"learned_keys\": \"%s\"}",
                         ac_protocol_name_.c_str(), mode, (int)s.target_temperature, source, learned_keys.c_str());
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
            "开始红外学习。重要: 用户说'学习空调/电视/遥控器按键'时, 必须直接调用本工具开始学习, 不要询问品牌、不要询问要学哪些键, 也不要跳过工具自行播报。"
            "type 默认 air_conditioner(空调, 固定顺序: 电源/模式/温度+/温度-, 用户无需选择按键); tv(电视, 顺序: 电源/信号源/音量+/音量-/频道+/频道-/静音/菜单); custom(自定义, 用 keys 指定逗号分隔的按键名列表)。"
            "调用后工具返回的 message 就是你要播报的内容。学习流程: 每按完一个键设备会播放提示音表示收到, 然后用户说'下一个', 此时调用 self.ir.learn_status 获取下一键提示并播报(如'电源已学习, 请按[模式]键'); 全部学完后 learn_status 返回 learning=false, 播报'按键学习完成'",
            PropertyList({
                Property("type", kPropertyTypeString, std::string("air_conditioner")),
                Property("keys", kPropertyTypeString, std::string(""))
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (ir_learner_ == nullptr) {
                    return "{\"success\": false, \"message\": \"IR learner not initialized\"}";
                }
                std::string type = properties["type"].value<std::string>();
                EnsureIRLoopTask();  // 学习需要 ir_loop 任务驱动捕获
                if (type == "tv") {
                    ir_learner_->learnTV();  // 固定顺序: 电源/信号源/音量+/音量-/频道+/频道-/静音/菜单
                    return "{\"success\": true, \"message\": \"开始学习电视按键, 请先按[电源]键, 每按完一个键告诉我'下一个'\"}";
                }
                if (type == "custom") {
                    // 解析 keys (兼容中英文逗号与空格)
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
                        return "{\"success\": false, \"message\": \"custom 模式需要提供 keys 按键列表\"}";
                    }
                    ir_learner_->startLearning();
                    return "{\"success\": true, \"message\": \"开始学习自定义按键, 请先按[第一个键], 每按完一个键告诉我'下一个'\"}";
                }
                // 默认: 空调
                ir_learner_->learnAirConditioner();  // 固定顺序: 电源/模式/温度+/温度-
                return "{\"success\": true, \"message\": \"开始学习空调按键, 请先按[电源]键, 每按完一个键告诉我'下一个'\"}";
            });

        mcp_server.AddTool("self.ir.learn_status",
            "查询红外学习状态与当前提示。用户每按完一个键并说'下一个'后, 必须调用本工具: 它会确认当前键已学习并推进到下一键。根据返回结果向用户播报: "
            "若 learning=true, 播报 message 中的提示(如'电源已学习, 请按[模式]键'); 若 learning=false, 播报'按键学习完成'",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (ir_learner_ == nullptr) {
                    return "{\"success\": false, \"message\": \"IR learner not initialized\"}";
                }
                // 用户说"下一个" → 确认当前键并推进到下一键
                ir_learner_->advanceToNextKey();
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
                EnsureIRLoopTask();  // 回放需要 ir_loop 任务执行实际发送
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
