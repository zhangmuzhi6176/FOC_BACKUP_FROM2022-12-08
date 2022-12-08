#ifndef _ZMZ_ADC_DRV_STM32F103_H
#define _ZMZ_ADC_DRV_STM32F103_H
#include "zmz_uart_hal.h"
#include "zmz_gpio_drv_STM32F103.h"
#include "zmz_system_hardware.h"

#define ADC_DMA_BUFFER_DEPTH 6
#define ADC_14_BIT_MID_VAL 2048

#define ZSS_ADC_LOGD(KEY, format, ...) ZSS_LOGD("ADC", KEY, format, ##__VA_ARGS__)
#define ZSS_ADC_LOGI(format, ...) ZSS_LOGI("ADC", format, ##__VA_ARGS__)
#define ZSS_ADC_LOGW(format, ...) ZSS_LOGW("ADC", format, ##__VA_ARGS__)
#define ZSS_ADC_LOGE(format, ...) ZSS_LOGE("ADC", format, ##__VA_ARGS__)
#define ZSS_ADC_LOGF(format, ...) ZSS_LOGF("ADC", format, ##__VA_ARGS__)

#define ADC_CASE_GPIO_INIT_LOG(CASE, PIN, FUMCTION)                                                  \
    {                                                                                                \
        CASE_GPIO_INIT_LOG(CASE, PIN, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, FUMCTION) \
    }

typedef enum adc_idx_type {
    ADC_I = 0,
    ADC_NUM,
} adc_idx_type_e;

typedef enum adc_pin_type {
    ADC_PIN_I = 0,
    ADC_PIN_II,
    ADC_PIN_III,
    ADC_PIN_NUM,
} adc_pin_type_e;

void ADC_Init_Drv(void);
u16 ADC_Get_Val_Poll_Drv(u8 adc_index, u8 adc_channel);
u16 ADC_Get_Val_Drv_Average(u8 adc_index, u8 adc_channel, u8 times);
u16 ADC_Get_Val_From_DMA_Drv(u8 adc_index, u8 adc_channel);

#endif
