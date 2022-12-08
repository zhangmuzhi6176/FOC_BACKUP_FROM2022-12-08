#include "zmz_system_hardware.h"

/*
 * @param: PLLMUL:  indicates multiplication factor for PLL VCO input clock,
 *                  shoule be RCC_PLL_MUL2~RCC_PLL_MUL16.
 * @return:     0, successful
 *              1, failed
 */
void Stm32_Clock_Init(u32 PLLMUL)
{
    RCC_OscInitTypeDef RCC_OscInit;
    RCC_ClkInitTypeDef RCC_ClkInit;

    RCC_OscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInit.HSEState = RCC_HSE_ON;
    RCC_OscInit.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInit.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInit.PLL.PLLMUL = PLLMUL;
    while (HAL_OK != HAL_RCC_OscConfig(&RCC_OscInit)) {
    }

    RCC_ClkInit.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInit.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInit.APB2CLKDivider = RCC_HCLK_DIV1;
    while (HAL_OK != HAL_RCC_ClockConfig(&RCC_ClkInit, FLASH_LATENCY_2)) {
    }
}

u16 Stm32_Get_Clock_MHz(void)
{
    return MAIN_FREQ;
}
