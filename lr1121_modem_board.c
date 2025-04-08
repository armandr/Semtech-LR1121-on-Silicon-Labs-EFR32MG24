/*
 * lr1121_hal.c
 *
 *  Created on: Feb 27, 2025
 *      Author: raz_armand
 */
#include <lr1121_modem_board.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <stdio.h>
#include "event_groups.h"


#include "spidrv.h"

#include "sl_spidrv_instances.h"
#include "sl_simple_led.h"
#include "sl_simple_led_instances.h"

//#include "sl_hal_gpio.h"

#include "gpiointerrupt.h"

#include "em_gpio.h"
#include "em_eusart.h"

#include "sl_emlib_gpio_init_RDO_BUSY_config.h"
#include "sl_emlib_gpio_init_RDO_INT_config.h"
#include "sl_emlib_gpio_init_RDO_NSS_config.h"
#include "sl_emlib_gpio_init_RDO_RES_config.h"
#include "sl_spidrv_eusart_RDO_config.h"

#include "lr1121_modem_modem_types.h"
#include "lr1121_modem_helper.h"
#include "lr1121_modem_system.h"
#include "lr1121_modem_modem.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
void print_version( lr1121_modem_version_t modem_version );
void init_lr1121_spi();
void init_lr1121_goio();
#define hal_spi_in_out(EUSART, DATA) EUSART_Spi_TxRx(EUSART, DATA)

/* -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */
#define LR1121_MODEM_RESET_TIMEOUT 3000

/* -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief lr1121 modem-e reset timeout flag
 */
//static bool lr1121_modem_reset_timeout = false;

/*!
 * @brief modem ready flag
 */
static bool modem_is_ready = false;

bool lr1121_modem_board_is_ready( void ) { return modem_is_ready; }

void lr1121_modem_board_set_ready( bool ready ) { modem_is_ready = ready; }


#ifndef TOOGLE_DELAY_MS
#define TOOGLE_DELAY_MS            (1000)
#endif

#ifndef BLINK_TASK_STACK_SIZE
#define BLINK_TASK_STACK_SIZE      configMINIMAL_STACK_SIZE
#endif

#ifndef BLINK_TASK_PRIO
#define BLINK_TASK_PRIO            20
#endif

#define BIT_0 ( 1 << 0 )
#define BIT_1 ( 1 << 1 )
#define BIT_TX_COMPLETE ( 1 << 2 )

 TaskHandle_t xLR1121_Handle = NULL;
 TaskHandle_t xLR1121_Handle2 = NULL;
 /* Declare a variable to hold the data associated with the created
     event group. */
 StaticEventGroup_t xCreatedEventGroup;
 /* Declare a variable to hold the created event group. */
 EventGroupHandle_t xTxRxCreatedEventGroup;

 lr1121_t lr1121;


static void lr1121_task(void *arg);
static void lr1121_task2(void *arg);

#include "lr1121_modem_hal.h"

static void lr1121_task2(void *arg)
{
  (void)&arg;

    printf("\r\n[%ld] Task2 Start", xTaskGetTickCount());


     init_lr1121_goio();
     lr1121.busy.pin.pin = SL_EMLIB_GPIO_INIT_RDO_BUSY_PIN;
     lr1121.busy.pin.port = SL_EMLIB_GPIO_INIT_RDO_BUSY_PORT;

     lr1121.nss.pin.pin = SL_EMLIB_GPIO_INIT_RDO_NSS_PIN;
     lr1121.nss.pin.port = SL_EMLIB_GPIO_INIT_RDO_NSS_PORT;

     lr1121.reset.pin.pin = SL_EMLIB_GPIO_INIT_RDO_RES_PIN;
     lr1121.reset.pin.port = SL_EMLIB_GPIO_INIT_RDO_RES_PORT;

     lr1121.spi_id = EUSART1;


    //Use the provided calculation macro to convert milliseconds to OS ticks
    const TickType_t xDelay = pdMS_TO_TICKS(TOOGLE_DELAY_MS);
    EventBits_t uxBits;

    while (1) {
        //hal_gpio_set_value(( &lr1121 )->nss.pin, 1 );
        vTaskDelay(xDelay);
        //hal_gpio_set_value(( &lr1121 )->nss.pin, 0 );
        //vTaskDelay(xDelay);

#if 0
      hal_gpio_set_value(( &lr1121 )->nss.pin, 1 );
      for (int i=0; i < 1100; i++)
        EUSART_Spi_TxRx(EUSART1, 0x55);

      continue;
#endif

      int s = lr1121_modem_hal_wakeup( &lr1121);
      printf("\r\n LR STATUS [%x] ", s);
      lr1121_modem_response_code_t r = lr1121_modem_system_reboot( &lr1121, false );
      printf("\r\n LR Reboot [%x] ", r);
      lr1121_modem_version_t modem_version;
      r = lr1121_modem_get_modem_version(&lr1121, &modem_version);
      printf("[%ld] Task2 LR: %x", hal_rtc_get_time_ms(), r);
      print_version(modem_version);



    }

}

#define hal_spi_in_out(EUSART, DATA) EUSART_Spi_TxRx(EUSART, DATA)

static void lr1121_task(void *arg){
  (void)&arg;

  printf("\r\n[%ld] Task Start", xTaskGetTickCount());
  //Use the provided calculation macro to convert milliseconds to OS ticks
  const TickType_t xDelay = pdMS_TO_TICKS(TOOGLE_DELAY_MS);
  EventBits_t uxBits;
  while (1) {

#if 0
      //Wait for specified delay
      /* NSS low */
              hal_gpio_set_value(( &lr1121 )->nss.pin, 1 );

              /* Send CMD */
              for( uint16_t i = 0; i < 100; i++ )
              {
                  hal_spi_in_out(( &lr1121)->spi_id, 0x55 );
              }
              vTaskDelay(xDelay);
              hal_gpio_set_value(( &lr1121 )->nss.pin, 0 );
              //vTaskDelay(xDelay);
              //hal_gpio_set_value(( &lr1121 )->nss.pin, 1);
     //vTaskDelay(xDelay);
    //uint32_t t = ulTaskNotifyTake( pdFALSE, xDelay );

      /* Wait a maximum of 100ms for either bit 0 or bit 4 to be set within
        the event group.  Clear the bits before exiting. */
          uxBits = xEventGroupWaitBits(
            xTxRxCreatedEventGroup,   /* The event group being tested. */
                  BIT_0 | BIT_1 | BIT_TX_COMPLETE, /* The bits within the event group to wait for. */
                  pdTRUE,        /* BIT_0 & BIT_4 should be cleared before returning. */
                  pdFALSE,       /* Don't wait for both bits, either bit will do. */
                  xDelay );/* Wait a maximum of 100ms for either bit to be set. */

          if( ( uxBits & ( BIT_0 | BIT_1 ) ) == ( BIT_0 | BIT_1 ) ){
              /* xEventGroupWaitBits() returned because both bits were set. */
              xEventGroupClearBits(xTxRxCreatedEventGroup,BIT_TX_COMPLETE);
          }
#endif
          vTaskDelay(xDelay);
          printf("\r\n[%ld] Hello, World! Task: %ld PIN: %d", xTaskGetTickCount(),hal_rtc_get_time_ms(), GPIO_PinInGet(SL_EMLIB_GPIO_INIT_RDO_INT_PORT,SL_EMLIB_GPIO_INIT_RDO_INT_PIN));

    // Toggle led
    sl_led_toggle(&sl_led_led0);
  }
}


void lr1121_int_Callback(uint8_t intNo)
{
  (void) intNo;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Add your interrupt handling code here

  // This creates many gives if there is bouncing.
  // instead you probably want to use
  xEventGroupSetBitsFromISR(xTxRxCreatedEventGroup,   /* The event group being updated. */ BIT_TX_COMPLETE, /* The bits being set. */ &xHigherPriorityTaskWoken );
  //vTaskNotifyGiveFromISR(xLR1121_Handle,&xHigherPriorityTaskWoken);
}

void init_lr1121_goio() {

    GPIOINT_Init();

    /* Attempt to create the event group. */
    xTxRxCreatedEventGroup = xEventGroupCreateStatic(&xCreatedEventGroup);


    // Register callback functions and enable interrupts
    GPIOINT_CallbackRegister(SL_EMLIB_GPIO_INIT_RDO_INT_PIN, lr1121_int_Callback);
    GPIO_ExtIntConfig(SL_EMLIB_GPIO_INIT_RDO_INT_PORT, SL_EMLIB_GPIO_INIT_RDO_INT_PIN, SL_EMLIB_GPIO_INIT_RDO_INT_PIN, true, false, true);
    //GPIOINT_CallbackRegister(3, gpioCallback3);
    GPIO_IntEnable(SL_EMLIB_GPIO_INIT_RDO_INT_PIN | SL_EMLIB_GPIO_INIT_RDO_BUSY_PIN);

    CMU_ClockEnable(cmuClock_GPIO, true);

    // Configure MOSI (TX) pin as an output
    GPIO_PinModeSet(SL_SPIDRV_EUSART_RDO_TX_PORT, SL_SPIDRV_EUSART_RDO_TX_PIN, gpioModePushPull, 0);

    // Configure MISO (RX) pin as an input
    GPIO_PinModeSet(SL_SPIDRV_EUSART_RDO_RX_PORT, SL_SPIDRV_EUSART_RDO_RX_PIN, gpioModeInput, 0);

    // Configure SCLK pin as an output low (CPOL = 0)
    GPIO_PinModeSet(SL_SPIDRV_EUSART_RDO_SCLK_PORT, SL_SPIDRV_EUSART_RDO_SCLK_PIN, gpioModePushPull, 0);



}

#define HAL_DBG_TRACE_INFO printf
#define HAL_DBG_TRACE_PRINTF printf


void print_version( lr1121_modem_version_t modem_version )
{
    HAL_DBG_TRACE_INFO( "###### ===== lr1121 MODEM-E VERSION ==== ######\n\n\n" );
    HAL_DBG_TRACE_PRINTF( "USE CASE   : %02X\n", modem_version.use_case );
    HAL_DBG_TRACE_PRINTF( "MODEM      : %02X.%02X.%02X\n", modem_version.modem_major, modem_version.modem_minor,
                          modem_version.modem_patch );
    HAL_DBG_TRACE_PRINTF( "LBM        : %02X.%02X.%02X\n\n", modem_version.lbm_major, modem_version.lbm_minor,
                          modem_version.lbm_patch );
}


void lr1121_init(void) {


  printf("\r\n[%ld] Task Init", xTaskGetTickCount());


  // Flush events before enabling irq
  //lr1121_modem_board_event_flush( &lr1121 );

  // Init done: enable interruption
  //    hal_mcu_enable_irq( );


  #if 0
  //GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_RDO_BUSY_PORT, SL_EMLIB_GPIO_INIT_RDO_BUSY_PIN, gpioModePushPull, SL_EMLIB_GPIO_INIT_RDO_BUSY_DOUT);
  //#define EUSART_Tx(EUSART_TypeDef *eusart, uint8_t data)
  //uint16_t EUSART_Spi_TxRx(EUSART_TypeDef *eusart, uint16_t data) &SL_SPIDRV_EUSART_RDO_PERIPHERAL

  xLedTimer = xTimerCreate( /* Just a text name, not used by the RTOS kernel. */
      "LedTimer",
      /* The timer period in ticks, must be greater than 0. */
      pdMS_TO_TICKS(1000),
      /* The timers will auto-reload themselves when they expire. */
      pdTRUE,
      /*The ID is used to store a count of the number of times the
        timer has expired, which is initialised to 0. */
      (void*)0,
        /* Each timer calls the same callback when it expires. */
      vLedTimerCallback);
  #endif

  static StaticTask_t xTaskBuffer;
  static StackType_t  xStack[BLINK_TASK_STACK_SIZE];

  // Create Blink Task without using any dynamic memory allocation
  xLR1121_Handle = xTaskCreateStatic(lr1121_task,
                              "blink task",
                              BLINK_TASK_STACK_SIZE,
                              ( void * ) NULL,
                              tskIDLE_PRIORITY + 1,
                              xStack,
                              &xTaskBuffer);

  // Since puxStackBuffer and pxTaskBuffer parameters are not NULL,
  // it is impossible for xLR1121_Handle to be null. This check is for
  // rigorous example demonstration.
  EFM_ASSERT(xLR1121_Handle != NULL);

  //return;

  static StaticTask_t xTaskBuffer2;
  static StackType_t  xStack2[BLINK_TASK_STACK_SIZE];

  // Create Blink Task without using any dynamic memory allocation
  xLR1121_Handle2 = xTaskCreateStatic(lr1121_task2,
                              "lr task",
                              BLINK_TASK_STACK_SIZE,
                              ( void * ) NULL,
                              tskIDLE_PRIORITY + 1,
                              xStack2,
                              &xTaskBuffer2);

  // Since puxStackBuffer and pxTaskBuffer parameters are not NULL,
  // it is impossible for xLR1121_Handle to be null. This check is for
  // rigorous example demonstration.
  EFM_ASSERT(xLR1121_Handle2 != NULL);




  /* Board is initialized */
  //leds_blink( LED_TX_MASK, 100, 20, true );
  //return;

}


#if 0  // DMA based transfers
void TransferComplete(SPIDRV_Handle_t handle,
                      Ecode_t transferStatus,
                      int itemsTransferred)
{
  if (transferStatus == ECODE_EMDRV_SPIDRV_OK) {
   // Success !
  }
}


void init_lr1121_spi()
{
  // SPIDRV_Handle_t handle = &handleData;

  uint8_t buffer[10];
 // SPIDRV_Init_t initData = SPIDRV_MASTER_EUSART1;

  // Initialize an SPI driver instance.
  //SPIDRV_Init(handle, &initData);
  //;


  // Transmit data using a blocking transmit function.
  SPIDRV_MTransmitB(sl_spidrv_eusart_RDO_handle, buffer, 10);

  // Transmit data using a callback to catch transfer completion.
  SPIDRV_MTransmit(sl_spidrv_eusart_RDO_handle, buffer, 10, TransferComplete);
}
#endif

uint32_t HAL_GetTick(void) { return xTaskGetTickCount();}

#define MIN_OS_DELAY_MS (30)
void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = xTaskGetTickCount();

  // RTOS may not be able to delay less than a context slice, we busy loop anything below that.
  if (Delay < MIN_OS_DELAY_MS){
     while((xTaskGetTickCount() - tickstart) < Delay){
     }
  }else {
      vTaskDelay(pdMS_TO_TICKS(Delay));
  }
}
/**
 * @brief Sets MCU pin to given value
 *
 * @param [in] pin   MCU pin to be set
 * @param [in] value MCU pin state to be set
 */
void hal_gpio_set_value( const sl_gpio_t pin, const uint32_t value ) {
  if (value)
    GPIO_PinOutSet(pin.port, pin.pin); //sl_gpio_set_pin(&pin);
  else
    GPIO_PinOutClear(pin.port, pin.pin);// sl_gpio_clear_pin(&pin);
}

/**
 * @brief Gets MCU pin state value
 *
 * @param [in] pin   MCU pin to be read
 *
 * @returns value Current MCU pin state
 */
uint32_t hal_gpio_get_value( const sl_gpio_t pin ){
  return GPIO_PinInGet(pin.port, pin.pin);// sl_gpio_clear_pin(&pin); //sl_hal_gpio_get_pin_input(&pin);
}

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
uint32_t hal_rtc_get_time_s( void ) { return (xTaskGetTickCount()) / configTICK_RATE_HZ;}

/**
 * @brief Returns the current RTC time in milliseconds
 *
 * @remark Used to timestamp radio events (i.e: end of TX), will also be used
 * for ClassB
 *
 * @returns rtc_time_ms Current RTC time in milliseconds wraps every 49 days
 */
uint32_t hal_rtc_get_time_ms( void ) { return (xTaskGetTickCount()); }


