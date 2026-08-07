#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// 配置PDM上采样fs参数（取值范围<=480）。部分设备在441时表现更稳定
#define AUDIO_PDM_UPSAMPLE_FS    441

#define AUDIO_ADC_MIC_CHANNEL       ADC_CHANNEL_0
#define AUDIO_PDM_SPEAK_P_GPIO      GPIO_NUM_4
#define AUDIO_PDM_SPEAK_N_GPIO      GPIO_NUM_1
#define AUDIO_PA_CTL_GPIO           GPIO_NUM_10

#define BUILTIN_LED_GPIO            GPIO_NUM_NC
#define BOOT_BUTTON_GPIO            GPIO_NUM_9
#define MOVE_WAKE_BUTTON_GPIO       GPIO_NUM_NC
#define AUDIO_WAKE_BUTTON_GPIO      GPIO_NUM_NC

// 本板卡无屏幕，不定义任何 DISPLAY_* 引脚

// ===== 红外遥控 =====
// IR 发射（红外 LED）: GPIO3
// IR 接收（一体化接收头，信号脚输出低电平表示有载波）: GPIO6
// 如需改接，请按实际硬件调整这两个引脚（避免使用 GPIO2/GPIO8/GPIO9 等 strapping 引脚）
#define IR_TX_GPIO                  GPIO_NUM_3
#define IR_RX_GPIO                  GPIO_NUM_5

// #define RESET_NVS_BUTTON_GPIO        GPIO_NUM_12

#endif // _BOARD_CONFIG_H_
