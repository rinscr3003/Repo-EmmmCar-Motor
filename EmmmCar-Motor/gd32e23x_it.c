/*!
    \file    gd32e23x_it.c
    \brief   interrupt service routines

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

#include "gd32e23x_it.h"
#include "main.h"
#include "systick.h"

#include "bsp_mspd.h"
#include "spi_protocol.h"

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_decrement();
}

void EXTI4_15_IRQHandler(void)
{
    if (RESET != exti_interrupt_flag_get(BSP_MSpd_S1_EXTINo))
    {
        _BSP_MSpd_SigIRQ(BSP_MSpd_S1);
        exti_interrupt_flag_clear(BSP_MSpd_S1_EXTINo);
    }
    if (RESET != exti_interrupt_flag_get(BSP_MSpd_S2_EXTINo))
    {
        _BSP_MSpd_SigIRQ(BSP_MSpd_S2);
        exti_interrupt_flag_clear(BSP_MSpd_S2_EXTINo);
    }
    if (RESET != exti_interrupt_flag_get(BSP_MSpd_S3_EXTINo))
    {
        _BSP_MSpd_SigIRQ(BSP_MSpd_S3);
        exti_interrupt_flag_clear(BSP_MSpd_S3_EXTINo);
    }
    if (RESET != exti_interrupt_flag_get(BSP_MSpd_S4_EXTINo))
    {
        _BSP_MSpd_SigIRQ(BSP_MSpd_S4);
        exti_interrupt_flag_clear(BSP_MSpd_S4_EXTINo);
    }
    if (RESET != exti_interrupt_flag_get(SPIPROC_NSS_EXTINo))
    {
        _SPIPROC_SigIRQ(gpio_input_bit_get(SPIPROC_GPIO_PORT,SPIPROC_NSS_PIN)==SET);
        exti_interrupt_flag_clear(SPIPROC_NSS_EXTINo);
    }
}

void TIMER5_IRQHandler()
{
    if (SET == timer_interrupt_flag_get(TIMER5, TIMER_INT_FLAG_UP))
    {
        timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
        _BSP_MSpd_PulseIRQ();
    }
}

extern uint8_t spi1_recvbuf[];
extern uint8_t spi1_sendbuf[];
extern uint8_t spi1_recvptr;
extern uint8_t spi1_sendptr;

void SPI1_IRQHandler()
{
     if (spi_i2s_interrupt_flag_get(SPI1, SPI_I2S_INT_FLAG_RBNE) != RESET)
    {
        spi1_recvbuf[spi1_recvptr++] = spi_i2s_data_receive(SPI1);
        _SPIPROC_Handler();
    }
}
