/*!
    \file    main.c
    \brief   led spark with systick, USART print and key example

    \version 2019-02-19, V1.0.0, firmware for GD32E23x
    \version 2020-12-12, V1.1.0, firmware for GD32E23x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32e23x.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"

#include "bsp_mdrv.h"
#include "bsp_mspd.h"
#include "spi_protocol.h"

void gd_log_com_init()
{
    /* enable COM GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOB);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_ */
    gpio_af_set(GPIOB, GPIO_AF_0, GPIO_PIN_6);
    gpio_af_set(GPIOB, GPIO_AF_0, GPIO_PIN_7);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_6);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_7);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

    usart_enable(USART0);
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
    SystemInit();
    /* configure systick */
    systick_config();
    /* initilize the peripherals */
    gd_log_com_init();

    /* print out the clock frequency of system, AHB, APB1 and APB2 */
    printf("CK_SYS is %d\r\n", rcu_clock_freq_get(CK_SYS));
    printf("CK_AHB is %d\r\n", rcu_clock_freq_get(CK_AHB));
    printf("CK_APB1 is %d\r\n", rcu_clock_freq_get(CK_APB1));
    printf("CK_APB2 is %d\r\n", rcu_clock_freq_get(CK_APB2));

    // start board init
    // Init Motors
    BSP_MDrv_Init();
    BSP_MDrv_AllBrake();
    BSP_MSpd_Init();
    BSP_MSpd_SetPIDOn(1);
    // Init SPI Comm
    SPI_Protocol_Init();
    // Init Board LEDs
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
    gpio_bit_reset(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    delay_1ms(500);
    gpio_bit_set(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

    // BSP_MDrv_SetSpeed(BSP_MDrv_M1,1023,BSP_MDrv_Forward);
    // BSP_MDrv_SetSpeed(BSP_MDrv_M2,1023,BSP_MDrv_Backward);
    // BSP_MDrv_SetSpeed(BSP_MDrv_M3,1023,BSP_MDrv_Forward);
    // BSP_MDrv_SetSpeed(BSP_MDrv_M4,1023,BSP_MDrv_Backward);

    uint32_t lastPeriod = getSysPeriod();
    while (1)
    {
        /*if (getSysPeriod() > 3500)
        {
            BSP_MSpd_SetGivenSpeed(BSP_MDrv_M1, 0.0f);
            BSP_MSpd_SetGivenSpeed(BSP_MDrv_M2, 0.0f);
            BSP_MSpd_SetGivenSpeed(BSP_MDrv_M3, 0.0f);
            BSP_MSpd_SetGivenSpeed(BSP_MDrv_M4, 0.0f);
        }*/

        // float speeds[4] = {0, 0, 0, 0};
        // BSP_MSpd_GetSpeeds(speeds);
        // printf("speed: %3.2f %3.2f %3.2f %3.2f\n", speeds[0], speeds[1], speeds[2], speeds[3]);

        uint8_t runstate = BSP_MDrv_GetMovStatus();
        if (runstate)
        {
            gpio_bit_reset(GPIOC, GPIO_PIN_14);
        }
        else
        {
            gpio_bit_set(GPIOC, GPIO_PIN_14);
        }

        if (getSysPeriod() - lastPeriod >= 500)
        {
            gpio_bit_toggle(GPIOC, GPIO_PIN_13);
            lastPeriod = getSysPeriod();
        }
    }
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE))
        ;

    return ch;
}
