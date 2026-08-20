#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// 配置PDM上采样fs参数（取值范围<=480）。部分设备在441时表现更稳定
#define AUDIO_PDM_UPSAMPLE_FS    441                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
 
#define AUDIO_ADC_MIC_CHANNEL       ADC_CHANNEL_1
#define AUDIO_PDM_SPEAK_P_GPIO      GPIO_NUM_3
#define AUDIO_PDM_SPEAK_N_GPIO      GPIO_NUM_5
#define AUDIO_PA_CTL_GPIO           GPIO_NUM_4

#define BUILTIN_LED_GPIO            GPIO_NUM_NC
#define BOOT_BUTTON_GPIO            GPIO_NUM_9
#define MOVE_WAKE_BUTTON_GPIO       GPIO_NUM_NC
#define AUDIO_WAKE_BUTTON_GPIO      GPIO_NUM_NC

#define DISPLAY_MOSI_PIN            GPIO_NUM_7
#define DISPLAY_CLK_PIN             GPIO_NUM_8
#define DISPLAY_DC_PIN              GPIO_NUM_6
#define DISPLAY_RST_PIN             GPIO_NUM_2
#define DISPLAY_CS_PIN              GPIO_NUM_10
#define DISPLAY_BACKLIGHT_PIN       GPIO_NUM_0

// #define RESET_NVS_BUTTON_GPIO        GPIO_NUM_12

#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH           128
#define DISPLAY_HEIGHT          128
#define DISPLAY_MIRROR_X        true
#define DISPLAY_MIRROR_Y        true
#define DISPLAY_SWAP_XY         false

#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER       LCD_RGB_ELEMENT_ORDER_BGR
#define DISPLAY_OFFSET_X        2
#define DISPLAY_OFFSET_Y        3
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT true
#define DISPLAY_SPI_MODE        0

#endif // _BOARD_CONFIG_H_
