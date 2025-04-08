/*
 * lr1121_hal.h
 *
 *  Created on: Feb 27, 2025
 *      Author: raz_b
 */

#ifndef LR1121_MODEM_BOARD_H_
#define LR1121_MODEM_BOARD_H_

#include "gpiointerrupt.h"
#include "spidrv.h"

void lr1121_init(void);

/// Structure for GPIO Port and Pin.
/// This was copied from sl_device_gpio.h which is not present in MG24's component selections
typedef struct {
  uint8_t port;
  uint8_t pin;
} sl_gpio_t;

typedef struct hal_gpio_s
{
  sl_gpio_t pin;
} hal_gpio_t;

typedef struct lr1121_s
{
    hal_gpio_t     reset;
    hal_gpio_t     busy;
    //GPIOINT_IrqCallbackPtr_t event;
    hal_gpio_t     nss;
   // SPIDRV_Handle_t spi;
    EUSART_TypeDef *spi_id;
} lr1121_t;

extern lr1121_t lr1121_context;

/**
 * @brief notify the user is the modem is ready
 *
 * @returns Modem ready state.
 */
bool lr1121_modem_board_is_ready( void );

/**
 * @brief set the modem is ready flag
 *
 * @param [in] ready ready state
 */
void lr1121_modem_board_set_ready( bool ready );


void               HAL_Delay(uint32_t Delay);
uint32_t           HAL_GetTick(void);

/**
 * @brief Sets MCU pin to given value
 *
 * @param [in] pin   MCU pin to be set
 * @param [in] value MCU pin state to be set
 */
void hal_gpio_set_value( const sl_gpio_t pin, const uint32_t value );
/**
 * @brief Gets MCU pin state value
 *
 * @param [in] pin   MCU pin to be read
 *
 * @returns value Current MCU pin state
 */
uint32_t hal_gpio_get_value( const sl_gpio_t pin );

/**
 * @brief Returns the current RTC time in seconds
 *
 * @remark Used for scheduling autonomous retransmissions (i.e: NbTrans),
 *         transmitting MAC answers, basically any delay without accurate time
 *         constraints. It is also used to measure the time spent inside the
 *         LoRaWAN process for the integrated failsafe.
 *
 * @returns rtc_time_s Current RTC time in seconds
 */
uint32_t hal_rtc_get_time_s( void );

/**
 * @brief Returns the current RTC time in milliseconds
 *
 * @remark Used to timestamp radio events (i.e: end of TX), will also be used
 * for ClassB
 *
 * @returns rtc_time_ms Current RTC time in milliseconds wraps every 49 days
 */
uint32_t hal_rtc_get_time_ms( void );


#endif /* LR1121_MODEM_BOARD_H_ */
