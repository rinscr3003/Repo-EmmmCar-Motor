#include "spi_protocol.h"
#include "gd32e23x.h"
#include <string.h>
#include <stdio.h>

/*
 * SPI Protocol
 * Version = v1.0.0.0
 * Author = 9223020209
 * Comment = Used for communications from main controller.
 */

uint8_t spi1_recvbuf[32];
uint8_t spi1_sendbuf[32];
volatile uint8_t spi1_recvptr = 0;
volatile uint8_t spi1_sendptr = 0;

void _SPIPROC_SigIRQ(uint8_t nssState)
{
    if (nssState)
    {
        spi_nss_internal_high(SPIPROC_DEV);
    }
    else
    {
        spi1_recvptr = 0;
        spi_nss_internal_low(SPIPROC_DEV);
    }
}

void SPI_Protocol_Init()
{
    // Interrupt on
    nvic_irq_enable(SPI1_IRQn, 1U);
    // Clock on
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI1);
    // GPIO config
    gpio_af_set(SPIPROC_GPIO_PORT, GPIO_AF_0, SPIPROC_SCK_PIN | SPIPROC_MISO_PIN | SPIPROC_MOSI_PIN);
    gpio_mode_set(SPIPROC_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPIPROC_SCK_PIN | SPIPROC_MISO_PIN | SPIPROC_MOSI_PIN);
    gpio_output_options_set(SPIPROC_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SPIPROC_MISO_PIN);
    // NSS Interrupt config
    gpio_mode_set(SPIPROC_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, SPIPROC_NSS_PIN);
    // rcu_periph_clock_enable(RCU_CFGCMP);
    // nvic_irq_enable(EXTI4_15_IRQn, 0U);
    syscfg_exti_line_config(SPIPROC_NSS_EXTIPORT, SPIPROC_NSS_EXTIPin);
    exti_init(SPIPROC_NSS_EXTINo, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_interrupt_flag_clear(SPIPROC_NSS_EXTINo);

    // Deinit SPI dev
    spi_i2s_deinit(SPIPROC_DEV);
    // Prepare struct
    spi_parameter_struct spi_init_struct;
    spi_struct_para_init(&spi_init_struct);
    // Config SPI dev
    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode = SPI_SLAVE;
    spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss = SPI_NSS_SOFT;
    spi_init_struct.endian = SPI_ENDIAN_MSB;
    spi_init(SPIPROC_DEV, &spi_init_struct);
    spi_fifo_access_size_config(SPIPROC_DEV, SPI_BYTE_ACCESS);
    // Enable SPI dev
    spi_i2s_interrupt_enable(SPIPROC_DEV, SPI_I2S_INT_RBNE);
    spi_enable(SPIPROC_DEV);
    memset(spi1_recvbuf, 0x00, 32);
    memset(spi1_sendbuf, 0x00, 32);
    spi1_recvptr = 0;
    printf("SPI READY.\n");
}

typedef enum
{
    SPICMD_SETMOTORPWM = 1,
    SPICMD_SETGIVENSPEED,
    SPICMD_SETPWMS,
    SPICMD_SETSPEEDS,
    SPICMD_GETSPEED,
    SPICMD_GETSPEEDS,
    SPICMD_GETPIDSTATE,
    SPICMD_SETPIDSTATE,
    SPICMD_MAXCMDINDEX
} _SPIPROC_CMDCODE;

void _SPIPROC_Handler()
{
    uint8_t len = spi1_recvptr;
    if (len == 0) // No Data
        return;
    printf("SPI[%d]=%2X\n", len - 1, spi1_recvbuf[len - 1]);
    if (spi1_recvbuf[0] >= SPICMD_MAXCMDINDEX || spi1_recvbuf[0] == 0) // Invalid CMDCode
        return;
    switch (spi1_recvbuf[0])
    {
    case SPICMD_SETMOTORPWM: // 1byte motorId, 2bytes PWM(rAlign10bit)
        if (len >= 2)
        {
            if (spi1_recvbuf[1] >= 4)
                break;
            if (len == 4)
            {
                uint16_t uPwm = (spi1_recvbuf[2] << 8) + spi1_recvbuf[3];
                uint8_t uDir = (spi1_recvbuf[2] & 0x80) == 0x80;
                uPwm = uPwm & 0x7fff;
                if (uPwm > 1023)
                    break;
                BSP_MDrv_SetSpeed(spi1_recvbuf[1], uPwm, uDir);
            }
        }
        break;

    case SPICMD_SETGIVENSPEED: // 1byte motorId, 1byte 10xSpeed(rAlign7bit)
        if (len >= 2)
        {
            if (spi1_recvbuf[1] >= 4)
                break;
            if (len == 3)
            {
                float uSpd = (int8_t)(spi1_recvbuf[2] & 0x7f);
                uint8_t uDir = (spi1_recvbuf[2] & 0x80) == 0x80;
                uSpd /= 10.0f;
                BSP_MSpd_SetGivenSpeed(spi1_recvbuf[1], uSpd * ((uDir) ? 1.0f : -1.0f));
            }
        }
        break;

    case SPICMD_SETPWMS: // 2bytes PWM(rAlign10bit) *4
        if (len == 9)
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                uint16_t uPwm = (spi1_recvbuf[2 * i + 1] << 8) + spi1_recvbuf[2 * i + 2];
                uint8_t uDir = (spi1_recvbuf[2 * i + 1] & 0x80) == 0x80;
                uPwm = uPwm & 0x7fff;
                if (uPwm > 1023)
                    break;
                BSP_MDrv_SetSpeed(i, uPwm, uDir);
            }
        }
        break;

    case SPICMD_SETSPEEDS: // 1byte 10xSpeed(rAlign7bit) *4
        if (len == 5)
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                float uSpd = (int8_t)(spi1_recvbuf[i + 1] & 0x7f);
                uint8_t uDir = (spi1_recvbuf[i + 1] & 0x80) == 0x80;
                uSpd /= 10.0f;
                BSP_MSpd_SetGivenSpeed(i, uSpd * ((uDir) ? -1.0f : 1.0f));
            }
        }
        break;

    case SPICMD_GETSPEED:;
        break;

    case SPICMD_GETSPEEDS:
        /* code */
        break;

    case SPICMD_GETPIDSTATE:
        /* code */
        break;

    case SPICMD_SETPIDSTATE:
        if (len == 1)
        {
            BSP_MSpd_SetPIDOn(spi1_recvbuf[1] == 0x01);
        }
        break;

    default:
        break;
    }
}