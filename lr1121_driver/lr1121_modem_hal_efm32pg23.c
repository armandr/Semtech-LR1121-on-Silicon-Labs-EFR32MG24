/*!
 * @file      lr1121_modem_hal_efm32pg23.c
 *
 * @brief     Hardware Abstraction Layer (HAL) implementation for LR1121 modem,
 *            ported from STM32 to Silicon Labs EFM32PG23 using EMLIB.
 *
 * @note      This port assumes that the LR1121 context (of type lr1121_t) has been
 *            updated to use a gpio_t type for NSS, BUSY, and RESET lines, and that
 *            the SPI instance is a pointer to a USART peripheral configured in synchronous mode.
 */

/* -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_core.h"
//#include "em_systick.h"
#include <stdint.h>
#include <stdbool.h>
#include "lr1121_hal.h"
#include "lr1121_modem_hal.h"
#include "lr1121_modem_system.h"
#include "lr1121_modem_board.h"  // (This header should now define the lr1121_t and gpio_t types)


