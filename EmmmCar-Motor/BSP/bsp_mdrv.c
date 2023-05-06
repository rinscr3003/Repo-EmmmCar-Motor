#include "bsp_mdrv.h"
#include "gd32e23x.h"

/*
 * BSP - Motor Driver
 * Version = v1.0.0.0
 * Author = 9223020209
 * Comment = Supports motors running controlling.
 */

void BSP_MDrv_Init()
{
    // =======================
    // Init GPIOs.
    // =======================

    // Enable GPIO Clock
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    // Init GPIO M1/M2
    gpio_mode_set(BSP_MDrv_M1_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BSP_MDrv_M1_Pin1);
    gpio_output_options_set(BSP_MDrv_M1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_MDrv_M1_Pin1);
    gpio_mode_set(BSP_MDrv_M1_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BSP_MDrv_M1_Pin2);
    gpio_output_options_set(BSP_MDrv_M1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_MDrv_M1_Pin2);
    gpio_mode_set(BSP_MDrv_M2_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BSP_MDrv_M2_Pin1);
    gpio_output_options_set(BSP_MDrv_M2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_MDrv_M2_Pin1);
    gpio_mode_set(BSP_MDrv_M2_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BSP_MDrv_M2_Pin2);
    gpio_output_options_set(BSP_MDrv_M2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_MDrv_M2_Pin2);
    // TODO: Auto replace AF here to correct AF
    gpio_af_set(BSP_MDrv_M1_PORT, GPIO_AF_2, BSP_MDrv_M1_Pin1); // All linked to TIMER0
    gpio_af_set(BSP_MDrv_M1_PORT, GPIO_AF_2, BSP_MDrv_M1_Pin2);
    gpio_af_set(BSP_MDrv_M2_PORT, GPIO_AF_2, BSP_MDrv_M2_Pin1);
    gpio_af_set(BSP_MDrv_M2_PORT, GPIO_AF_2, BSP_MDrv_M2_Pin2);
    // Init GPIO M3/M4
    gpio_mode_set(BSP_MDrv_M3_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BSP_MDrv_M3_Pin1);
    gpio_output_options_set(BSP_MDrv_M3_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_MDrv_M3_Pin1);
    gpio_mode_set(BSP_MDrv_M3_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BSP_MDrv_M3_Pin2);
    gpio_output_options_set(BSP_MDrv_M3_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_MDrv_M3_Pin2);
    gpio_mode_set(BSP_MDrv_M4_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BSP_MDrv_M4_Pin1);
    gpio_output_options_set(BSP_MDrv_M4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_MDrv_M4_Pin1);
    gpio_mode_set(BSP_MDrv_M4_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BSP_MDrv_M4_Pin2);
    gpio_output_options_set(BSP_MDrv_M4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_MDrv_M4_Pin2);
    // TODO: Auto replace AF here to correct AF
    gpio_af_set(BSP_MDrv_M3_PORT, GPIO_AF_1, BSP_MDrv_M3_Pin1); // All linked to TIMER2
    gpio_af_set(BSP_MDrv_M3_PORT, GPIO_AF_1, BSP_MDrv_M3_Pin2);
    gpio_af_set(BSP_MDrv_M4_PORT, GPIO_AF_1, BSP_MDrv_M4_Pin1);
    gpio_af_set(BSP_MDrv_M4_PORT, GPIO_AF_1, BSP_MDrv_M4_Pin2);

    // =======================
    // Init TIMERs.
    // =======================

    // Enable TIMER-M1M2 Clock
    rcu_periph_clock_enable(RCU_TIMER0);
    // Reset TIMER-M1M2
    timer_deinit(BSP_MDrv_M1M2_TIM);
    // Prepare structs
    timer_parameter_struct timer_initpara;
    timer_oc_parameter_struct timer_ocinitpara;
    timer_struct_para_init(&timer_initpara);
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    // TIMER-M1M2 Basic Config
    timer_initpara.prescaler = 125; // fCnt = 571.429kHz
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = 1023; // fPWM = 558Hz
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(BSP_MDrv_M1M2_TIM, &timer_initpara);

    // TIMER-M1M2 Channel Config
    timer_ocinitpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M1_TCH1, &timer_ocinitpara);
    timer_channel_output_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M1_TCH2, &timer_ocinitpara);
    timer_channel_output_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M2_TCH1, &timer_ocinitpara);
    timer_channel_output_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M2_TCH2, &timer_ocinitpara);

    timer_channel_output_pulse_value_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M1_TCH1, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M1_TCH2, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M2_TCH1, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M2_TCH2, 0);

    timer_channel_output_mode_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M1_TCH1, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M1_TCH2, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M2_TCH1, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M2_TCH2, TIMER_OC_MODE_PWM0);

    //   timer_channel_output_shadow_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M1_TCH1, TIMER_OC_SHADOW_DISABLE);
    //   timer_channel_output_shadow_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M1_TCH2, TIMER_OC_SHADOW_DISABLE);
    //   timer_channel_output_shadow_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M2_TCH1, TIMER_OC_SHADOW_DISABLE);
    //   timer_channel_output_shadow_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M2_TCH2, TIMER_OC_SHADOW_DISABLE);

    // Enable TIMER-M1M2
    timer_primary_output_config(BSP_MDrv_M1M2_TIM, ENABLE);
    timer_auto_reload_shadow_enable(BSP_MDrv_M1M2_TIM);
    timer_enable(BSP_MDrv_M1M2_TIM);

    // Enable TIMER-M3M4 Clock
    rcu_periph_clock_enable(RCU_TIMER2);
    // Reset TIMER-M3M4
    timer_deinit(BSP_MDrv_M3M4_TIM);
    // TIMER-M3M4 Basic Config
    timer_init(BSP_MDrv_M3M4_TIM, &timer_initpara);

    // TIMER-M3M4 Channel Config
    timer_channel_output_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M3_TCH1, &timer_ocinitpara);
    timer_channel_output_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M3_TCH2, &timer_ocinitpara);
    timer_channel_output_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M4_TCH1, &timer_ocinitpara);
    timer_channel_output_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M4_TCH2, &timer_ocinitpara);

    timer_channel_output_pulse_value_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M3_TCH1, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M3_TCH2, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M4_TCH1, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M4_TCH2, 0);

    timer_channel_output_mode_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M3_TCH1, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M3_TCH2, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M4_TCH1, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M4_TCH2, TIMER_OC_MODE_PWM0);

    //   timer_channel_output_shadow_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M3_TCH1, TIMER_OC_SHADOW_DISABLE);
    //   timer_channel_output_shadow_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M3_TCH2, TIMER_OC_SHADOW_DISABLE);
    //   timer_channel_output_shadow_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M4_TCH1, TIMER_OC_SHADOW_DISABLE);
    //   timer_channel_output_shadow_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M4_TCH2, TIMER_OC_SHADOW_DISABLE);

    // Enable TIMER-M3M4
    timer_auto_reload_shadow_enable(BSP_MDrv_M3M4_TIM);
    timer_enable(BSP_MDrv_M3M4_TIM);
}

void BSP_MDrv_SetSpeed(BSP_MDrv_Motor motorId, uint16_t speed, BSP_MDrv_Direction direction)
{
    if (direction == BSP_MDrv_Forward)
    {
        timer_channel_output_pulse_value_config(_BSP_MDrv_MotorTIM[motorId], _BSP_MDrv_MotorCH1[motorId], speed);
        timer_channel_output_pulse_value_config(_BSP_MDrv_MotorTIM[motorId], _BSP_MDrv_MotorCH2[motorId], 0);
    }
    else
    {
        timer_channel_output_pulse_value_config(_BSP_MDrv_MotorTIM[motorId], _BSP_MDrv_MotorCH1[motorId], 0);
        timer_channel_output_pulse_value_config(_BSP_MDrv_MotorTIM[motorId], _BSP_MDrv_MotorCH2[motorId], speed);
    }
}

uint32_t _BSP_MDrv_GetPulseVal(uint32_t timer, uint16_t channel)
{
    switch (channel)
    {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        return TIMER_CH0CV(timer);
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        return TIMER_CH1CV(timer);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        return TIMER_CH2CV(timer);
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        return TIMER_CH3CV(timer);
        break;
    default:
        return 0xFFFFFFFFU;
        break;
    }
}

uint8_t BSP_MDrv_GetMovStatus()
{
    // B7-0 M4N M3N M2N M1N M4 M3 M2 M1
    uint8_t res = 0x00;
    for (uint8_t i = 0; i < 4; i++)
    {
        uint16_t ch1 = _BSP_MDrv_GetPulseVal(_BSP_MDrv_MotorTIM[i], _BSP_MDrv_MotorCH1[i]);
        uint16_t ch2 = _BSP_MDrv_GetPulseVal(_BSP_MDrv_MotorTIM[i], _BSP_MDrv_MotorCH2[i]);
        if (ch1 != ch2)
        {
            if (ch1 > ch2)
            {
                res |= (0x01 << (i + 0));
            }
            else
            {
                res |= (0x01 << (i + 4));
            }
        }
    }
    return res;
}

void BSP_MDrv_AllBrake()
{
    timer_channel_output_pulse_value_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M1_TCH1, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M1_TCH2, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M2_TCH1, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M2_TCH2, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M3_TCH1, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M3_TCH2, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M4_TCH1, 0);
    timer_channel_output_pulse_value_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M4_TCH2, 0);
}

void BSP_MDrv_AllCoast()
{
    timer_channel_output_pulse_value_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M1_TCH1, 255);
    timer_channel_output_pulse_value_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M1_TCH2, 255);
    timer_channel_output_pulse_value_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M2_TCH1, 255);
    timer_channel_output_pulse_value_config(BSP_MDrv_M1M2_TIM, BSP_MDrv_M2_TCH2, 255);
    timer_channel_output_pulse_value_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M3_TCH1, 255);
    timer_channel_output_pulse_value_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M3_TCH2, 255);
    timer_channel_output_pulse_value_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M4_TCH1, 255);
    timer_channel_output_pulse_value_config(BSP_MDrv_M3M4_TIM, BSP_MDrv_M4_TCH2, 255);
}
