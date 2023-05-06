#include "bsp_mspd.h"
#include "gd32e23x.h"
#include <string.h>

/*
 * BSP - Motor Speed Measure
 * Version = v1.0.0.0
 * Author = 9223020209
 * Comment = Supports motors speed measuring.
 */

#define _BSP_MSpd_HPulseFreq 100000UL
#define _BSP_MSpd_CalcPeriod 25000U // loop period: 250ms

volatile static uint16_t _BSP_MSpd_SigCnt[4] = {0, 0, 0, 0};
volatile static float _BSP_MSpd_MeasuredSpeed[4] = {0, 0, 0, 0};
volatile static uint16_t _BSP_MSpd_PulseCnt = 0;

void _BSP_MSpd_SigIRQ(BSP_MSpd_Sensor sensorId)
{
    _BSP_MSpd_SigCnt[sensorId]++;
}

void _BSP_MSpd_PulseIRQ()
{
    _BSP_MSpd_PulseCnt++;
    // process
    if (_BSP_MSpd_PulseCnt % _BSP_MSpd_CalcPeriod == 0)
    {
			timer_interrupt_disable(BSP_MSpd_PulseTIM, TIMER_INT_UP);
        // calculate speed
        for (uint8_t i = 0; i < 4; i++)
        {
            _BSP_MSpd_MeasuredSpeed[i] = ((float)_BSP_MSpd_SigCnt[i] * (float)_BSP_MSpd_HPulseFreq) / ((float)_BSP_MSpd_PulseCnt * (float)_BSP_MSpd_SigPerCycle);
            _BSP_MSpd_SigCnt[i] = 0;
        }
        _BSP_MSpd_PulseCnt = 0;
				timer_interrupt_enable(BSP_MSpd_PulseTIM, TIMER_INT_UP);
    }
}

void BSP_MSpd_Init()
{
    // =======================
    // Init GPIOs.
    // =======================

    // Enable GPIO Clock
    rcu_periph_clock_enable(RCU_GPIOC);
    // Init GPIO for sensors
    gpio_mode_set(BSP_MSpd_S1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BSP_MSpd_S1_Pin);
    gpio_mode_set(BSP_MSpd_S2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BSP_MSpd_S2_Pin);
    gpio_mode_set(BSP_MSpd_S3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BSP_MSpd_S3_Pin);
    gpio_mode_set(BSP_MSpd_S4_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BSP_MSpd_S4_Pin);

    // =======================
    // Init EXTIs.
    // =======================

    // Init EXTI(Clock,IRQ)
    rcu_periph_clock_enable(RCU_CFGCMP);
    nvic_irq_enable(EXTI4_15_IRQn, 1U);

    // Init EXTI lines
    syscfg_exti_line_config(BSP_MSpd_S1_EXTIPORT, BSP_MSpd_S1_EXTIPin);
    syscfg_exti_line_config(BSP_MSpd_S2_EXTIPORT, BSP_MSpd_S2_EXTIPin);
    syscfg_exti_line_config(BSP_MSpd_S3_EXTIPORT, BSP_MSpd_S3_EXTIPin);
    syscfg_exti_line_config(BSP_MSpd_S4_EXTIPORT, BSP_MSpd_S4_EXTIPin);
    exti_init(BSP_MSpd_S1_EXTINo, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_init(BSP_MSpd_S2_EXTINo, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_init(BSP_MSpd_S3_EXTINo, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_init(BSP_MSpd_S4_EXTINo, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_interrupt_flag_clear(BSP_MSpd_S1_EXTINo);
    exti_interrupt_flag_clear(BSP_MSpd_S2_EXTINo);
    exti_interrupt_flag_clear(BSP_MSpd_S3_EXTINo);
    exti_interrupt_flag_clear(BSP_MSpd_S4_EXTINo);

    // =======================
    // Init TIMERs.
    // =======================

    // Enable TIMER-SPD Clock
    rcu_periph_clock_enable(RCU_TIMER5);
    // Reset TIMER-SPD
    timer_deinit(BSP_MSpd_PulseTIM);
    // Prepare structs
    timer_parameter_struct timer_initpara;
    timer_struct_para_init(&timer_initpara);
    // TIMER-SPD Basic Config
    timer_initpara.prescaler = 71; // fCnt = 1MHz
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = 9; // fPulse = 100kHz
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(BSP_MSpd_PulseTIM, &timer_initpara);

    // Enable TIMER-SPD
    timer_auto_reload_shadow_enable(BSP_MSpd_PulseTIM);
    timer_interrupt_flag_clear(BSP_MSpd_PulseTIM, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(BSP_MSpd_PulseTIM, TIMER_INT_UP);
    timer_enable(BSP_MSpd_PulseTIM);
    nvic_irq_enable(TIMER5_IRQn, 1U);
}

float BSP_MSpd_GetSpeed(BSP_MSpd_Sensor sensorId)
{
    return _BSP_MSpd_MeasuredSpeed[sensorId];
}
void BSP_MSpd_GetSpeeds(float *speeds)
{
    memcpy(speeds, (const float *)_BSP_MSpd_MeasuredSpeed, 4 * sizeof(float));
}