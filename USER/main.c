#include "zmz_led.h"
#include "zmz_foc.h"
#include "zmz_mt6701.h"
#include "zmz_delay.h"

#include "zmz_iic_soft_hal.h"
#include "zmz_uart_hal.h"
#include "zmz_timer_hal.h"

#include "zmz_can_drv_STM32F103.h"
#include "zmz_adc_drv_STM32F103.h"

#include "zmz_system_hardware.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <math.h>

#define ZSS_MAIN_LOGD(KEY, format, ...) ZSS_LOGD("FOC_MAIN", KEY, format, ##__VA_ARGS__)
#define ZSS_MAIN_LOGI(format, ...) ZSS_LOGI("FOC_MAIN", format, ##__VA_ARGS__)
#define ZSS_MAIN_LOGW(format, ...) ZSS_LOGW("FOC_MAIN", format, ##__VA_ARGS__)
#define ZSS_MAIN_LOGE(format, ...) ZSS_LOGE("FOC_MAIN", format, ##__VA_ARGS__)
#define ZSS_MAIN_LOGF(format, ...) ZSS_LOGF("FOC_MAIN", format, ##__VA_ARGS__)

//任务优先级
#define START_TASK_PRIO 1
//任务堆栈大小
#define START_STK_SIZE 128
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define TASK1_TASK_PRIO 3
//任务堆栈大小
#define TASK1_STK_SIZE 128
//任务句柄
TaskHandle_t Task1Task_Handler;
//任务函数
void task1_task(void *pvParameters);

//任务优先级
#define TASK2_TASK_PRIO 3
//任务堆栈大小
#define TASK2_STK_SIZE 128
//任务句柄
TaskHandle_t Task2Task_Handler;
//任务函数
void task2_task(void *pvParameters);

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //进入临界区
    //创建TASK1任务
    xTaskCreate((TaskFunction_t)task1_task,
                (const char *)"task1_task",
                (uint16_t)TASK1_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)TASK1_TASK_PRIO,
                (TaskHandle_t *)&Task1Task_Handler);
    //创建TASK2任务
    xTaskCreate((TaskFunction_t)task2_task,
                (const char *)"task2_task",
                (uint16_t)TASK2_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)TASK2_TASK_PRIO,
                (TaskHandle_t *)&Task2Task_Handler);
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

double angle_deg_1 = 0;
double angle_deg_2 = 0;
int i = 0;
double last_angle, cur_angle, next_angle = 0;
// task1任务函数
void task1_task(void *pvParameters)
{
    while (1) {
        taskENTER_CRITICAL(); //进入临界区

        /* angle_deg_2 = MT_Get_ANGLE(ENC_NO_2); */
        /* printf("%.3f, ", angle_deg_2/10); */

        /* FOC_Keep_Torque(FOC_I, (MT_Get_ANGLE(ENC_NO_2)/180)-1); */

        /* printf("%.3f, %.3f\r\n", angle_deg_1/10, MT_Get_ANGLE(ENC_NO_2)/10); */

        /* SVPWM_Generate_Mech_Ang(FOC_I, angle_deg_1, 80); */
        /* SVPWM_Generate_Mech_Ang(FOC_I, 0, 50); */

        /* angle_deg_1 = MT_Get_ANGLE(ENC_NO_2);
        SVPWM_Generate_Mech_Ang(FOC_I, angle_deg_1, 40);
        printf("%.3f, %.3f\r\n", angle_deg_1 / 10, MT_Get_ANGLE(ENC_NO_1) / 10); */

        /* angle_deg_1 -= 90;
        if (angle_deg_1 >= 360*11) angle_deg_1 = 0; */

        /* printf("%d, %d, %d, %d, %d\r\n", Get_Uart_DBG_INT(0, 0), Get_Uart_DBG_INT(0, 1), Get_Uart_DBG_INT(0, 2), Get_Uart_DBG_INT(0, 3), Get_Uart_DBG_INT(0, 4)); */

        /* if(_FOC_Get_Elec_angle(FOC_I) < (double)Get_Uart_DBG_INT(0, 0)*11) {
            cur_angle = _FOC_Get_Elec_angle(FOC_I);
            next_angle = cur_angle + (double)Get_Uart_DBG_INT(0, 1);
            if (next_angle > 360*11) next_angle -= 360*11;
            if (next_angle < 0) next_angle += 360*11;
            SVPWM_Generate_Elec_Ang(FOC_I, next_angle, Get_Uart_DBG_INT(0, 2));
            printf("%4d, %-*.3f, %-*.3f, %-*.3f\r\n", i++, 8, cur_angle, 8, next_angle, 8, cur_angle - last_angle);
            last_angle = cur_angle;
        } else {
            SVPWM_Generate_Elec_Ang(FOC_I, _FOC_Get_Elec_angle(FOC_I) + 90, 0);
            i = 0;
        } */

        /* if(Get_Uart_DBG_INT(0, 0)) {
            cur_angle = _FOC_Get_Elec_angle(FOC_I);
            angle_deg_1 += Get_Uart_DBG_INT(0, 1);
            if (angle_deg_1 < 0) angle_deg_1 += 360*11;
            if (i >= (Get_Uart_DBG_INT(0, 2) / (abs(Get_Uart_DBG_INT(0, 1))))*11) Set_Uart_DBG_INT(0, 0, 0);
            SVPWM_Generate_Elec_Ang(FOC_I, angle_deg_1, Get_Uart_DBG_INT(0, 3));
            printf("%4d, %-*.3f, %-*.3f, %-*.3f\r\n", i++, 8, (angle_deg_1-Get_Uart_DBG_INT(0, 1)), 8, cur_angle, 8, (angle_deg_1-Get_Uart_DBG_INT(0, 1)) - cur_angle);
        } else {
            SVPWM_Generate_Elec_Ang(FOC_I, _FOC_Get_Elec_angle(FOC_I) + 90, 0);
            angle_deg_1 = 0;
            i = 0;
        } */

        /* SVPWM_Generate_Alpha_Beta(FOC_I, 0.7*cos(DEG_2_RAD(angle_deg_1)), 0.7*sin(DEG_2_RAD(angle_deg_1))); */

        angle_deg_1 += 1;
        if (angle_deg_1 >= 360) angle_deg_1 = 0;
        SVPWM_Generate_Mech_Ang(FOC_I, angle_deg_1, MT_Get_ANGLE(ENC_NO_2)/3.6);
        printf("%-*.3f\r\n", 8, MT_Get_ANGLE(ENC_NO_2)/3.6);
        

        /* ZSS_MAIN_LOGD("ENC_NO_1", "angle_deg_1: [%.3f]\r\n", angle_deg_1);
        ZSS_MAIN_LOGD("ENC_NO_2", "angle_deg_1: [%.3f]\r\n", angle_deg_1); */

        taskEXIT_CRITICAL(); //退出临界区
        /* vTaskDelay(Get_Uart_DBG_INT(0, 4)); */
    }
}

// task2任务函数
void task2_task(void *pvParameters)
{
    /* double alpha = 0;
    double beta = 0; */

    /* double count = 0;
    double Ia, Ib, Ic, I_alpha, I_beta, Id, Iq;
    double AMP = 10; */

    while (1) {
        taskENTER_CRITICAL(); //进入临界区

        /* alpha = cos(DEG_2_RAD(angle_deg_1));
        beta = sin(DEG_2_RAD(angle_deg_1));

        printf("%.3f, %.3f, %.3f, ", angle_deg_1/10, 10*alpha, 10*beta);

        angle_deg_2 = RAD_2_DEG(PI - atan2(beta , -1 * alpha));

        printf("%.3f, ", angle_deg_2/15);

        printf("\r\n");

        angle_deg_1 += 1.5;
        if (angle_deg_1 >= 360) angle_deg_1 = 0; */

        /* Ia = sin(count * PI / 180.0);
        Ib = sin((240 + count) * PI / 180.0);
        Ic = sin((120 + count) * PI / 180.0);
        count+=0.5;

        printf("%.3f, %.3f, %.3f, ", Ia * AMP, Ib * AMP, Ic * AMP);
        printf("%f, ", sqrt(Ia*Ia + Ib*Ib + Ic*Ic) * AMP);

        clark_transform(Ia, Ib, Ic, &I_alpha, &I_beta);
        //printf("%f, %f, ", I_alpha*2, I_beta*2);
        printf("%f, ", sqrt(I_alpha*I_alpha + I_beta*I_beta) * AMP);

        park_transform(count, I_alpha, I_beta, &Id, &Iq);
        //printf("%f, %f, ", Id * 3, Iq * 3);
        printf("%f, ", sqrt(Id*Id + Iq*Iq) * AMP);

        //reverse_park_transform(count, Id, Iq, &I_alpha, &I_beta);
        //printf("%f, %f", I_alpha * 2.5, I_beta * 2.5);

        if (count >= 360)
          count = 0;

        printf("\r\n"); */

        /* Timer_Set_Duty_Hal(0, MT_Get_ANGLE(ENC_NO_2)/3.6);
        Timer_Set_Duty_Hal(1, MT_Get_ANGLE(ENC_NO_2)/3.6);
        Timer_Set_Duty_Hal(2, MT_Get_ANGLE(ENC_NO_2)/3.6); */

        taskEXIT_CRITICAL(); //退出临界区
        /* vTaskDelay(10); */
    }
}

int main(void)
{
    ZSS_SHUT_DOWN_IT;
    HAL_Init();                     //初始化HAL库
    Stm32_Clock_Init(RCC_PLL_MUL9); //设置时钟,72M
    Uart_Init();
    Timer_Init_Hal();
    delay_init();
    RGB_Led_Init();
    MT_Init();
    IIC_Init_Hal();
    ADC_Init_Drv();
    MX_CAN_Init();
    Can_Config();
    FOC_Init();
    delay_ms(500);

    Timer_Set_Duty_Hal(4, 100);
    Timer_Set_Duty_Hal(5, 100);
    Timer_Set_Duty_Hal(6, 100);

    ZSS_MAIN_LOGI("10-27----001\r\n");
    ZSS_MAIN_LOGI("+++++++++++++++++++Initialization done+++++++++++++++++++\r\n");
    ZSS_TURN_UP_IT;

    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
    /* vTaskStartScheduler(); */

    while (1) {
        /* FOC_Keep_Torque(FOC_I, (MT_Get_ANGLE(ENC_NO_2) / 180) - 1); */
        /* FOC_Keep_Speed(FOC_I, (MT_Get_ANGLE(ENC_NO_2) / 180) - 1); */
        FOC_Keep_Position(FOC_I, MT_Get_ANGLE(ENC_NO_2), 1);

        /* delay_ms(50); */


        /* RGB_Led_Set_Color(RGB_LED_I, RGB_LED_LAKE_BLUE, (fabs(100 - MT_Get_ANGLE(ENC_NO_2)/1.8)) / 100); */
        /* printf("%-*.3f\r\n", 8, MT_Get_ANGLE(ENC_NO_2)/3.6); */


        /* SVPWM_Generate_Mech_Ang(FOC_I, MT_Get_ANGLE(ENC_NO_2), 50); */



        /* printf("%5d, %5d, %5d,\r\n", 
        ADC_Get_Val_From_DMA_Drv(ADC_I, 0), 
        ADC_Get_Val_From_DMA_Drv(ADC_I, 1), 
        ADC_Get_Val_From_DMA_Drv(ADC_I, 2));

        delay_ms(50); */

        /* Timer_Set_Duty_Hal(0, MT_Get_ANGLE(ENC_NO_2)/3.6);
        Timer_Set_Duty_Hal(1, MT_Get_ANGLE(ENC_NO_2)/3.6);
        Timer_Set_Duty_Hal(2, MT_Get_ANGLE(ENC_NO_2)/3.6); */



        /* if (Get_Uart_DBG_INT(0, 0)) {
            Timer_Set_Duty_Hal(0, MT_Get_ANGLE(ENC_NO_2)/3.6);
        } else {
            Timer_Set_Duty_Hal(0, 0);
        }
        if (Get_Uart_DBG_INT(0, 1)) {
            Timer_Set_Duty_Hal(1, MT_Get_ANGLE(ENC_NO_2)/3.6);
        } else {
            Timer_Set_Duty_Hal(1, 0);
        }
        if (Get_Uart_DBG_INT(0, 2)) {
            Timer_Set_Duty_Hal(2, MT_Get_ANGLE(ENC_NO_2)/3.6);
        } else {
            Timer_Set_Duty_Hal(2, 0);
        }
        if (Get_Uart_DBG_INT(0, 3)) {
            Timer_Set_Duty_Hal(4, MT_Get_ANGLE(ENC_NO_2)/3.6);
        } else {
            Timer_Set_Duty_Hal(4, 100);
        }
        if (Get_Uart_DBG_INT(0, 4)) {
            Timer_Set_Duty_Hal(5, MT_Get_ANGLE(ENC_NO_2)/3.6);
        } else {
            Timer_Set_Duty_Hal(5, 100);
        }
        if (Get_Uart_DBG_INT(0, 5)) {
            Timer_Set_Duty_Hal(6, MT_Get_ANGLE(ENC_NO_2)/3.6);
        } else {
            Timer_Set_Duty_Hal(6, 100);
        }

        printf("%-*.3f\r\n", 8, MT_Get_ANGLE(ENC_NO_2)/3.6); */


        /* for(int i = 0; i < RGB_LED_COLOR_NUM; i++) {
            RGB_Led_Set_Color(RGB_LED_I, (rgb_led_color_spec_e)i, 1);
            delay_ms(500);
        } */

        /* RGB_Led_Set(RGB_LED_I, (double)Get_Uart_DBG_INT(0, 3), (double)Get_Uart_DBG_INT(0, 4), (double)Get_Uart_DBG_INT(0, 5)); */


        /* printf("Timer_Get_System_Time_Second_Drv [%f]\r\n", Timer_Get_System_Time_Second_Drv());
        delay_ms(1); */

    }
}
