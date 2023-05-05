#ifndef __BSP_BDRV_H__
#define __BSP_BDRV_H__

#include "gd32e23x.h"
#include <stdint.h>

#define BSP_MDrv_M1_PORT GPIOA
#define BSP_MDrv_M2_PORT GPIOA
#define BSP_MDrv_M3_PORT GPIOA
#define BSP_MDrv_M4_PORT GPIOB

#define BSP_MDrv_M1_Pin1 GPIO_PIN_8
#define BSP_MDrv_M1_Pin2 GPIO_PIN_9
#define BSP_MDrv_M2_Pin1 GPIO_PIN_10
#define BSP_MDrv_M2_Pin2 GPIO_PIN_11
#define BSP_MDrv_M3_Pin1 GPIO_PIN_6
#define BSP_MDrv_M3_Pin2 GPIO_PIN_7
#define BSP_MDrv_M4_Pin1 GPIO_PIN_0
#define BSP_MDrv_M4_Pin2 GPIO_PIN_1

#define BSP_MDrv_M1M2_TIM TIMER0
#define BSP_MDrv_M3M4_TIM TIMER2

#define BSP_MDrv_M1_TCH1 TIMER_CH_0
#define BSP_MDrv_M1_TCH2 TIMER_CH_1
#define BSP_MDrv_M2_TCH1 TIMER_CH_2
#define BSP_MDrv_M2_TCH2 TIMER_CH_3
#define BSP_MDrv_M3_TCH1 TIMER_CH_0
#define BSP_MDrv_M3_TCH2 TIMER_CH_1
#define BSP_MDrv_M4_TCH1 TIMER_CH_2
#define BSP_MDrv_M4_TCH2 TIMER_CH_3

const static uint32_t _BSP_MDrv_MotorTIM[4] = {TIMER0, TIMER0, TIMER2, TIMER2};
const static uint16_t _BSP_MDrv_MotorCH1[4] = {TIMER_CH_0, TIMER_CH_2, TIMER_CH_0, TIMER_CH_2};
const static uint16_t _BSP_MDrv_MotorCH2[4] = {TIMER_CH_1, TIMER_CH_3, TIMER_CH_1, TIMER_CH_3};

typedef enum
{
    BSP_MDrv_M1 = 0,
    BSP_MDrv_M2,
    BSP_MDrv_M3,
    BSP_MDrv_M4
} BSP_MDrv_Motor;

typedef enum
{
    BSP_MDrv_Forward = 0,
    BSP_MDrv_Backward
} BSP_MDrv_Direction;

void BSP_MDrv_Init();
void BSP_MDrv_SetSpeed(BSP_MDrv_Motor motorId, uint16_t speed, BSP_MDrv_Direction direction);
uint8_t BSP_MDrv_GetMovStatus();
void BSP_MDrv_AllBrake();
void BSP_MDrv_AllCoast();

#endif