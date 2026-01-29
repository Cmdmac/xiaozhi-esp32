#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "esp32_camera.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_station.h>
#include <esp_lcd_touch_ft5x06.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include "audio/codecs/no_audio_codec.h"
#include "led/single_led.h"

#define TAG "StarkBoard"

class TCA6408 : public I2cDevice {
public:
    enum DeviceAddress
    {
        DEVICE_ADDRESS_0 = 0x20,
        DEVICE_ADDRESS_1 = 0x21,
    };

    enum RegisterAddress
    {
        INPUT_PORT = 0,
        OUTPUT_PORT = 1,
        POLARITY_INVERSION = 2,
        CONFIGURATION = 3
    };

    enum PinMode
    {
        INPUT = 0x01,
        OUTPUT = 0x03
    };

    enum Pin
    {
        P0,
        P1,
        P2,
        P3,
        P4,
        P5,
        P6,
        P7
    };

    TCA6408(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        // WriteReg(0x01, 0x03);
        // WriteReg(0x03, 0xf8);
        // 更新指定引脚的模式
        // #define INPUT 0x01
        // Changed OUTPUT from 0x02 to behave the same as Arduino pinMode(pin,OUTPUT)
        // where you can read the state of pin even when it is set as OUTPUT
        // #define OUTPUT            0x03
    }

    // #define LOW  0x0
    // #define HIGH 0x1
    bool digitalWrite(Pin pin, int value) {
        // 读取当前输出端口状态
        
        uint8_t currentState = ReadReg(INPUT_PORT);
    
        // 更新指定引脚的状态
        if (value == 0x1) {
            currentState |= (1 << pin);
        } else if (value == 0x0) {
            currentState &= ~(1 << pin);
        } else {
            return false;
        }
        ESP_LOGE(TAG, "TCA6408 digitalWrite pin=%d value=%d currentState=0x%02X", pin, value, currentState);
        WriteReg(OUTPUT_PORT, currentState);
        return true;
    }

    int digitalRead(Pin pin) {
        // 读取输入端口状态
        uint8_t inputState = ReadReg(INPUT_PORT);
    
        // 返回指定引脚的状态
        return (inputState & (1 << pin)) != 0;
    }

    bool pinMode(Pin pin, int mode) {

        uint8_t v = ReadReg(CONFIGURATION);
        // 更新指定引脚的模式
        if (mode == INPUT) {
            v |= (1 << pin);
            WriteReg(CONFIGURATION, v);
        } else if (mode == OUTPUT) {
            v &= ~(1 << pin);    
            WriteReg(CONFIGURATION, v);
        } else {
            return false;
        }
        return true;
    }

};


class StarkBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    Display* display_;
    Esp32Camera* camera_;
    TCA6408* tca6408;

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = I2C_SDA,
            .scl_io_num = I2C_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // Initialize Tca6408
        tca6408 = new TCA6408(i2c_bus_, TCA6408::DEVICE_ADDRESS_0);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        // pca9557_->SetOutputState(0, 0);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }
 
    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    void InitializeCamera() {
        // Open camera power
        tca6408->pinMode(TCA6408::P3, TCA6408::OUTPUT);
        if (tca6408->digitalWrite(TCA6408::P3, 0)) {
            ESP_LOGI("Main", "TCA6408 P3 set to LOW");
        } 
        static esp_cam_ctlr_dvp_pin_config_t dvp_pin_config = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                [0] = CAMERA_PIN_D0,
                [1] = CAMERA_PIN_D1,
                [2] = CAMERA_PIN_D2,
                [3] = CAMERA_PIN_D3,
                [4] = CAMERA_PIN_D4,
                [5] = CAMERA_PIN_D5,
                [6] = CAMERA_PIN_D6,
                [7] = CAMERA_PIN_D7,
            },
            .vsync_io = CAMERA_PIN_VSYNC,
            .de_io = CAMERA_PIN_HREF,
            .pclk_io = CAMERA_PIN_PCLK,
            .xclk_io = CAMERA_PIN_XCLK,
        };

        esp_video_init_sccb_config_t sccb_config = {
            .init_sccb = false,
            .i2c_handle = i2c_bus_,
            .freq = 100000,
        };

        esp_video_init_dvp_config_t dvp_config = {
            .sccb_config = sccb_config,
            .reset_pin = CAMERA_PIN_RESET,
            .pwdn_pin = CAMERA_PIN_PWDN,
            .dvp_pin = dvp_pin_config,
            .xclk_freq = XCLK_FREQ_HZ,
        };

        esp_video_init_config_t video_config = {
            .dvp = &dvp_config,
        };

        camera_ = new Esp32Camera(video_config);
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeTools() {
        // static LampController lamp(LAMP_GPIO);
        // ESP_LOGI(TAG, "Adding MusicControl controller to MCP server");
        // new MusicControl();
    }

public:
    StarkBoard() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        // display_ = new NoDisplay();
        InitializeButtons();
        InitializeTools();
        InitializeCamera();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
        
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    // virtual Display* GetDisplay() override {
    //     return display_;
    // }

    // virtual Camera* GetCamera() override {
    //     ESP_LOGE(TAG, "GetCamera called");
    //     return camera_;
    // }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(StarkBoard);

