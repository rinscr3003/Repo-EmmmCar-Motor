#ifndef __BSP_MSPD_H__
#define __BSP_MSPD_H__

/*
 * BSP - Motor Speed Measure
 * Version = v1.0.0.1
 * Author = 9223020209
 * Comment = Supports motors speed measuring.
 */

#include "gd32e23x.h"
#include <stdint.h>

#include "bsp_mdrv.h"

#define BSP_MSpd_S1_PORT GPIOB
#define BSP_MSpd_S2_PORT GPIOB
#define BSP_MSpd_S3_PORT GPIOB
#define BSP_MSpd_S4_PORT GPIOB

#define BSP_MSpd_S1_Pin GPIO_PIN_4
#define BSP_MSpd_S2_Pin GPIO_PIN_5
#define BSP_MSpd_S3_Pin GPIO_PIN_8
#define BSP_MSpd_S4_Pin GPIO_PIN_9

#define BSP_MSpd_S1_EXTIPORT EXTI_SOURCE_GPIOB
#define BSP_MSpd_S2_EXTIPORT EXTI_SOURCE_GPIOB
#define BSP_MSpd_S3_EXTIPORT EXTI_SOURCE_GPIOB
#define BSP_MSpd_S4_EXTIPORT EXTI_SOURCE_GPIOB

#define BSP_MSpd_S1_EXTIPin EXTI_SOURCE_PIN4
#define BSP_MSpd_S2_EXTIPin EXTI_SOURCE_PIN5
#define BSP_MSpd_S3_EXTIPin EXTI_SOURCE_PIN8
#define BSP_MSpd_S4_EXTIPin EXTI_SOURCE_PIN9

#define BSP_MSpd_S1_EXTINo EXTI_4
#define BSP_MSpd_S2_EXTINo EXTI_5
#define BSP_MSpd_S3_EXTINo EXTI_8
#define BSP_MSpd_S4_EXTINo EXTI_9

#define BSP_MSpd_PulseTIM TIMER5

#define _BSP_MSpd_SigPerCycle 40

#define _BSP_MSpd_PID_P 58.0f
#define _BSP_MSpd_PID_I 90.0f
#define _BSP_MSpd_PID_D 0.0f

#define _BSP_MSpd_PID_LIM_MIN 150.0f
#define _BSP_MSpd_PID_LIM_MAX 1023.0f

typedef enum
{
    BSP_MSpd_S1 = 0,
    BSP_MSpd_S2,
    BSP_MSpd_S3,
    BSP_MSpd_S4
} BSP_MSpd_Sensor;

void _BSP_MSpd_SigIRQ(BSP_MSpd_Sensor sensorId);
void _BSP_MSpd_PulseIRQ();

void BSP_MSpd_Init();
float BSP_MSpd_GetSpeed(BSP_MSpd_Sensor sensorId);
void BSP_MSpd_GetSpeeds(float *speeds);
uint8_t BSP_MSpd_GetPIDOn();
void BSP_MSpd_SetPIDOn(uint8_t pidOn);

void BSP_MSpd_SetGivenSpeed(BSP_MDrv_Motor motorId, float givenSpeed);

#endif