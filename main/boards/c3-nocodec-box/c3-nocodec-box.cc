#include "wifi_board.h"
#include "adc_pdm_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_wifi.h>
#include <esp_event.h>

#include "display/lcd_display.h"
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

#include "assets/lang_config.h"
#include "servo_dog_ctrl.h"
#include "led_strip.h"
#include "driver/rmt_tx.h"
#include "device_state.h"
#include "system_reset.h"
#include "alarm_manager.h"
#include "sdkconfig.h"
#include "codecs/mix_audio_codec.h"

#ifdef CONFIG_ESP_HI_WEB_CONTROL_ENABLED
#include "esp_hi_web_control.h"
#endif //CONFIG_ESP_HI_WEB_CONTROL_ENABLED

#define TAG "C3-NOCODEC-BOX"


class C3NoCodecBox : public WifiBoard {
private:
    Button boot_button_;
    Button audio_wake_button_;
    Button move_wake_button_;
    LcdDisplay* display_ = nullptr;
    bool web_server_initialized_ = false;
    MixAudioCodec* mix_audio_codec_ = nullptr;

    void InitializeButtons() {
        static int64_t last_trigger_time = 0;
        static int gesture_state = 0;  // 0: init, 1: wait second long interval, 2: wait oscillation

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
    
    void InitializeLed() {

    }


    void InitializeIot() {
        ESP_LOGI(TAG, "Initialize Iot");
        InitializeLed();
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * 10 * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay()
    {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        ESP_LOGI(TAG, "LCD panel create success, %p", panel);

        // 使用 LVGL 显示（SpiLcdDisplay 内部会主动画白清屏并处理偏移）
        ESP_LOGI(TAG, "Create LVGL display, panel: %p, panel_io: %p", panel, panel_io);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);

#if CONFIG_ESP_CONSOLE_NONE
        servo_dog_ctrl_config_t config = {
            .fl_gpio_num = FL_GPIO_NUM,
            .fr_gpio_num = FR_GPIO_NUM,
            .bl_gpio_num = BL_GPIO_NUM,
            .br_gpio_num = BR_GPIO_NUM,
        };

        servo_dog_ctrl_init(&config);
#endif
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();         

        // mcp_server.AddTool("self.system.reset_nvs", "重置NVS数据，清除所有配置信息", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
        //     auto& app = Application::GetInstance();
        //     app.Schedule([&app]() {
        //         ESP_LOGW(TAG, "User requested NVS reset");
        //         SystemReset system_reset(RESET_NVS_BUTTON_GPIO, RESET_FACTORY_BUTTON_GPIO);
        //         system_reset.ResetNvsFlash();
        //     });
        //     return true;
        // });

        // mcp_server.AddTool("self.system.factory_reset", "恢复出厂设置，清除所有数据并重启进入配置模式", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
        //     auto& app = Application::GetInstance();
        //     app.Schedule([&app]() {
        //         ESP_LOGW(TAG, "User requested factory reset");
        //         SystemReset system_reset(RESET_NVS_BUTTON_GPIO, RESET_FACTORY_BUTTON_GPIO);
        //         system_reset.ResetToFactory();
        //     });
        //     return true;
        // });
        // Alarm tools
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
    C3NoCodecBox() : boot_button_(BOOT_BUTTON_GPIO),
        audio_wake_button_(AUDIO_WAKE_BUTTON_GPIO),
        move_wake_button_(MOVE_WAKE_BUTTON_GPIO)
    {
        InitializeButtons();
        InitializeIot();
        InitializeSpi();
        InitializeLcdDisplay();
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
            // mix_audio_codec_ = new MixAudioCodec(&audio_codec);
        // }
        return &audio_codec;
    }

    virtual Display* GetDisplay() override
    {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(C3NoCodecBox);
