#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// 配置PDM上采样fs参数（取值范围<=480）。部分设备在441时表现更稳定
#define AUDIO_PDM_UPSAMPLE_FS    441

#define AUDIO_ADC_MIC_CHANNEL       ADC_CHANNEL_1
#define AUDIO_PDM_SPEAK_P_GPIO      GPIO_NUM_4
#define AUDIO_PDM_SPEAK_N_GPIO      GPIO_NUM_0
#define AUDIO_PA_CTL_GPIO           GPIO_NUM_5

#define BUILTIN_LED_GPIO            GPIO_NUM_NC
#define BOOT_BUTTON_GPIO            GPIO_NUM_9
#define MOVE_WAKE_BUTTON_GPIO       GPIO_NUM_NC
#define AUDIO_WAKE_BUTTON_GPIO      GPIO_NUM_NC

// 本板卡无屏幕，不定义任何 DISPLAY_* 引脚

// #define RESET_NVS_BUTTON_GPIO        GPIO_NUM_12

#endif // _BOARD_CONFIG_H_
