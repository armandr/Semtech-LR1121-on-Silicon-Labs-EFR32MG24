#include "sl_emlib_gpio_simple_init.h"
#include "sl_emlib_gpio_init_RDO_BUSY_config.h"
#include "sl_emlib_gpio_init_RDO_INT_config.h"
#include "sl_emlib_gpio_init_RDO_NSS_config.h"
#include "sl_emlib_gpio_init_RDO_RES_config.h"
#include "em_gpio.h"
#include "em_cmu.h"

void sl_emlib_gpio_simple_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_RDO_BUSY_PORT,
                  SL_EMLIB_GPIO_INIT_RDO_BUSY_PIN,
                  SL_EMLIB_GPIO_INIT_RDO_BUSY_MODE,
                  SL_EMLIB_GPIO_INIT_RDO_BUSY_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_RDO_INT_PORT,
                  SL_EMLIB_GPIO_INIT_RDO_INT_PIN,
                  SL_EMLIB_GPIO_INIT_RDO_INT_MODE,
                  SL_EMLIB_GPIO_INIT_RDO_INT_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_RDO_NSS_PORT,
                  SL_EMLIB_GPIO_INIT_RDO_NSS_PIN,
                  SL_EMLIB_GPIO_INIT_RDO_NSS_MODE,
                  SL_EMLIB_GPIO_INIT_RDO_NSS_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_RDO_RES_PORT,
                  SL_EMLIB_GPIO_INIT_RDO_RES_PIN,
                  SL_EMLIB_GPIO_INIT_RDO_RES_MODE,
                  SL_EMLIB_GPIO_INIT_RDO_RES_DOUT);
}
