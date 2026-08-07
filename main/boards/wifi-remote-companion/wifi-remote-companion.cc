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

// ===== IR 遥控伴侣模块 (纯学习模式) =====
#include <remote_transmitter.h>
#include <IRLearner.h>

// HeatpumpIRCompat.h 定义的 LOW/HIGH 宏与代码中其它枚举值同名, 取消定义
#undef LOW
#undef HIGH

#ifdef CONFIG_ESP_HI_WEB_CONTROL_ENABLED
#include "esp_hi_web_control.h"
#endif //CONFIG_ESP_HI_WEB_CONTROL_ENABLED

#define TAG "WIFI-REMOTE-COMPANION"

using heatpump_ir_tx::RemoteTransmitter;
using heatpump_ir_tx::RemoteTransmitData;

class WifiRemoteCompanion : public WifiBoard {
private:
    Button boot_button_;
    Button audio_wake_button_;
    Button move_wake_button_;
    MixAudioCodec* mix_audio_codec_ = nullptr;

    // 红外遥控硬件
    RemoteTransmitter ir_tx_;
    IRLearner* ir_learner_ = nullptr;
    TaskHandle_t ir_task_ = nullptr;

    static void IRLoopTask(void* arg) {
        auto* self = static_cast<WifiRemoteCompanion*>(arg);
        while (true) {
            if (self->ir_learner_) {
                self->ir_learner_->loop();
            }
            vTaskDelay(pdMS_TO_TICKS(20));
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

        // 红外学习: 接收用 GPIO 中断, 回放共用 RMT 发射器
        ir_learner_ = new IRLearner(IR_RX_GPIO, IR_TX_GPIO);
        ir_learner_->setup();
        ir_learner_->setIRSender(new IRSenderRMT(&ir_tx_));

        // 开机预置默认学习按键(空调), 保证按键列表始终已初始化;
        // 之后可通过 self.ir.learn_start 的 preset/keys 参数覆盖
        ir_learner_->addTargetKey("电源");
        ir_learner_->addTargetKey("模式");
        ir_learner_->addTargetKey("温度+");
        ir_learner_->addTargetKey("温度-");
        ir_learner_->addTargetKey("风速");
        ir_learner_->addTargetKey("上下扫风");
        ir_learner_->addTargetKey("左右扫风");
        ir_learner_->addTargetKey("定时");
        ESP_LOGI(TAG, "预置学习按键: 电源/模式/温度+/温度-/风速/上下扫风/左右扫风/定时");

        xTaskCreate(IRLoopTask, "ir_loop", 6144, this, 1, &ir_task_);
        ESP_LOGI(TAG, "IR learner ready");
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();

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
                    ir_learner_->learnAirConditioner();  // 内部会 reset + 添加空调按键 + startLearning
                    return "{\"success\": true, \"message\": \"开始学习空调按键: 电源/模式/温度+/温度-/风速/上下扫风/左右扫风/定时\"}";
                }
                if (preset == "tv") {
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
        if (mix_audio_codec_ == nullptr) {
            static AdcPdmAudioCodec audio_codec(
                AUDIO_INPUT_SAMPLE_RATE,
                AUDIO_OUTPUT_SAMPLE_RATE,
                AUDIO_ADC_MIC_CHANNEL,
                AUDIO_PDM_SPEAK_P_GPIO,
                AUDIO_PDM_SPEAK_N_GPIO,
                AUDIO_PA_CTL_GPIO);
            mix_audio_codec_ = new MixAudioCodec(&audio_codec);
        }
        return mix_audio_codec_;
    }

    // 无屏幕板卡：不重写 GetDisplay()，使用基类默认的 NoDisplay
};

DECLARE_BOARD(WifiRemoteCompanion);
