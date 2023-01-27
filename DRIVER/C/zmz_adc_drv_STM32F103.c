#include "zmz_adc_drv_STM32F103.h"
#include "zmz_delay.h"
#include "zmz_gpio_drv_STM32F103.h"
#include "stdlib.h"

typedef struct adc_channel {
    gpio_spec_t adc_io;
    ADC_ChannelConfTypeDef channel_conf;
} adc_channel_t;

typedef struct adc_dev {
    adc_channel_t channels[ADC_PIN_NUM];

    RCC_PeriphCLKInitTypeDef adc_clk_select;
    ADC_HandleTypeDef adc_handler;

    DMA_HandleTypeDef hdma_adc_handler;
    u16 adc_dma_buffer[ADC_DMA_BUFFER_DEPTH][ADC_PIN_NUM];
    u8 adc_dma_buffer_depth;

    u8 adc_number;
} adc_dev_t;

static adc_dev_t adc_dev_g[] = {
    [ADC_I] = {
        .channels = {
            [ADC_PIN_I] = {
                .adc_io = {
                    .gpio_grp = A,
                    .gpio_num = 1,
                },
                .channel_conf = {
                    .Channel = ADC_CHANNEL_3,
                    .Rank = ADC_REGULAR_RANK_1,
                    .SamplingTime = ADC_SAMPLETIME_55CYCLES_5,
                },
            },
            [ADC_PIN_II] = {
                .adc_io = {
                    .gpio_grp = A,
                    .gpio_num = 2,
                },
                .channel_conf = {
                    .Channel = ADC_CHANNEL_1,
                    .Rank = ADC_REGULAR_RANK_2,
                    .SamplingTime = ADC_SAMPLETIME_55CYCLES_5,
                },
            },
            [ADC_PIN_III] = {
                .adc_io = {
                    .gpio_grp = A,
                    .gpio_num = 3,
                },
                .channel_conf = {
                    .Channel = ADC_CHANNEL_2,
                    .Rank = ADC_REGULAR_RANK_3,
                    .SamplingTime = ADC_SAMPLETIME_55CYCLES_5,
                },
            },
        },
        .adc_clk_select = {
            .PeriphClockSelection = RCC_PERIPHCLK_ADC,
            .AdcClockSelection = RCC_ADCPCLK2_DIV4,
        },
        .adc_handler = {
            .Instance = ADC1,
            .Init = {
                .DataAlign = ADC_DATAALIGN_RIGHT,
                .ScanConvMode = ADC_SCAN_ENABLE,
                .ContinuousConvMode = ENABLE,
                .NbrOfConversion = 3,
                .DiscontinuousConvMode = DISABLE,
                .ExternalTrigConv = ADC_SOFTWARE_START,
            },
        },
        .hdma_adc_handler = {
            .Instance = DMA1_Channel1,
            .Init = {
                .Direction = DMA_PERIPH_TO_MEMORY,
                .PeriphInc = DMA_PINC_DISABLE,
                .MemInc = DMA_MINC_ENABLE,
                .PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD,
                .MemDataAlignment = DMA_MDATAALIGN_HALFWORD,
                .Mode = DMA_CIRCULAR,
                .Priority = DMA_PRIORITY_HIGH,
            },
        },
        .adc_dma_buffer_depth = ADC_DMA_BUFFER_DEPTH,
        .adc_number = 1,
    },
};

static void Adc_Dma_Config(u8 adc_index)
{
    __DMA1_CLK_ENABLE();
    HAL_DMA_Init(&(adc_dev_g[adc_index].hdma_adc_handler));
    __HAL_LINKDMA(&(adc_dev_g[adc_index].adc_handler), DMA_Handle, adc_dev_g[adc_index].hdma_adc_handler);
}

void ADC_Init_Drv(void)
{
    for (u8 i = 0; i < ZSS_ARRAY_SIZE(adc_dev_g); i++) {
        Adc_Dma_Config(i);
        HAL_RCCEx_PeriphCLKConfig(&(adc_dev_g[i].adc_clk_select));
        HAL_ADC_Init(&(adc_dev_g[i].adc_handler));
        HAL_ADCEx_Calibration_Start(&(adc_dev_g[i].adc_handler));

        for (u8 j = 0; j < ZSS_ARRAY_SIZE(adc_dev_g[i].channels); j++) {
            HAL_ADC_ConfigChannel(&(adc_dev_g[i].adc_handler), &(adc_dev_g[i].channels[j].channel_conf));
        }

        HAL_ADC_Start_DMA(&(adc_dev_g[i].adc_handler), (uint32_t *)(adc_dev_g[i].adc_dma_buffer),
                          (ZSS_ARRAY_SIZE(adc_dev_g[i].channels) * adc_dev_g[i].adc_dma_buffer_depth));
    }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    for (u32 i = 0; i < ZSS_ARRAY_SIZE(adc_dev_g); i++) {
        if (hadc->Instance == adc_dev_g[i].adc_handler.Instance) {
            switch (adc_dev_g[i].adc_number) {
            case 1:
                __HAL_RCC_ADC1_CLK_ENABLE();
                break;
            case 2:
                __HAL_RCC_ADC2_CLK_ENABLE();
                break;
            default:
                ZSS_ASSERT_WITH_LOG("__HAL_RCC_ADC[%d]_CLK_ENABLE Unsupported", adc_dev_g[i].adc_number);
            }

            for (u32 j = 0; j < ZSS_ARRAY_SIZE(adc_dev_g[i].channels); j++) {
                switch (adc_dev_g[i].channels[j].adc_io.gpio_grp) {
                    ADC_CASE_GPIO_INIT_LOG(A, adc_dev_g[i].channels[j].adc_io.gpio_num, "ANALOG_IN_FOR_ADC");
                    ADC_CASE_GPIO_INIT_LOG(B, adc_dev_g[i].channels[j].adc_io.gpio_num, "ANALOG_IN_FOR_ADC");
                    ADC_CASE_GPIO_INIT_LOG(C, adc_dev_g[i].channels[j].adc_io.gpio_num, "ANALOG_IN_FOR_ADC");
                    ADC_CASE_GPIO_INIT_LOG(D, adc_dev_g[i].channels[j].adc_io.gpio_num, "ANALOG_IN_FOR_ADC");
                    ADC_CASE_GPIO_INIT_LOG(E, adc_dev_g[i].channels[j].adc_io.gpio_num, "ANALOG_IN_FOR_ADC");
                }
                delay_ms(50);
            }
        }
    }
}

u16 ADC_Get_Val_Poll_Drv(u8 adc_index, u8 adc_channel)
{
    ADC_ChannelConfTypeDef ADC1_ChanConf;
    ADC1_ChanConf.Channel = adc_channel;
    ADC1_ChanConf.Rank = 1;
    ADC1_ChanConf.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

    HAL_ADC_ConfigChannel(&(adc_dev_g[adc_index].adc_handler), &ADC1_ChanConf);
    HAL_ADC_Start(&(adc_dev_g[adc_index].adc_handler));
    HAL_ADC_PollForConversion(&(adc_dev_g[adc_index].adc_handler), 10);
    return (u16)HAL_ADC_GetValue(&(adc_dev_g[adc_index].adc_handler));
}

u16 ADC_Get_Val_Drv_Average_Poll(u8 adc_index, u8 adc_channel, u8 times)
{
    u32 temp_val = 0;
    for (u8 i = 0; i < times; i++) {
        temp_val += ADC_Get_Val_Poll_Drv(adc_index, adc_channel);
        delay_us(500);
    }
    return temp_val / times;
}

u16 ADC_Get_Val_From_DMA_Drv(u8 adc_index, u8 adc_channel)
{
    u16 val_tmp = 0;
    for (u8 i = 0; i < adc_dev_g[adc_index].adc_dma_buffer_depth; i++) {
        val_tmp += adc_dev_g[adc_index].adc_dma_buffer[i][adc_channel];
    }
    return (val_tmp / adc_dev_g[adc_index].adc_dma_buffer_depth);
}
