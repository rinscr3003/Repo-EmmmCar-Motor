#ifndef __SPI_PROC_H__
#define __SPI_PROC_H__

/*
 * SPI Protocol
 * Version = v1.0.0.0
 * Author = 9223020209
 * Comment = Used for communications from main controller.
 */

#include "gd32e23x.h"
#include <stdint.h>

#include "bsp_mdrv.h"
#include "bsp_mspd.h"

#define SPIPROC_DEV SPI1
#define SPIPROC_GPIO_PORT GPIOB
#define SPIPROC_NSS_PIN GPIO_PIN_12
#define SPIPROC_SCK_PIN GPIO_PIN_13
#define SPIPROC_MISO_PIN GPIO_PIN_14
#define SPIPROC_MOSI_PIN GPIO_PIN_15

#define SPIPROC_NSS_EXTIPORT EXTI_SOURCE_GPIOB
#define SPIPROC_NSS_EXTIPin EXTI_SOURCE_PIN12
#define SPIPROC_NSS_EXTINo EXTI_12

void _SPIPROC_SigIRQ(uint8_t nssState);
void _SPIPROC_Handler();

void SPI_Protocol_Init();

#endif