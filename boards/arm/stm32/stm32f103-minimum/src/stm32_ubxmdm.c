/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_ubxmdm.c
 *
 *   Copyright (C) 2016 Vladimir Komendantskiy. All rights reserved.
 *   Author: Vladimir Komendantskiy <vladimir@moixaenergy.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/modem/u-blox.h>

#include <arch/board/board.h>
#include "stm32_gpio.h"
#include "stm32f103_minimum.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing the modem driver */

#ifdef CONFIG_MODEM_U_BLOX_DEBUG
#  define m_err     _err
#  define m_info    _info
#else
#  define m_err(x...)
#  define m_info(x...)
#endif

#define UBXMDM_REGISTER_COUNT                           \
  (sizeof(stm32_ubxmdm_name_pins) /                     \
   sizeof(struct stm32_name_pin))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Core interface pin connections.  These do not include UART or USB pins. */

struct stm32_ubxmdm_pins
{
  uint32_t ldo_enable;
  uint32_t power_on_n;
  uint32_t reset_n;
  uint32_t shifter_en_n;
  uint32_t usb_detect;
};

/* This structure type provides the private representation of the
 * "lower-half" driver state.  This type must be coercible to type
 * 'ubxmdm_lower'.
 */

struct stm32_ubxmdm_lower
{
  const struct ubxmdm_ops * ops;  /* Lower half operations */

  /* Private, architecture-specific information. */

  const struct stm32_ubxmdm_pins * pins;
  bool usb_used;
};

/* Pair type for associating a register name to a pin. */

struct stm32_name_pin
{
  const char name[3];
  const uint32_t pin;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* "Lower half" driver methods **********************************************/

static int stm32_poweron  (struct ubxmdm_lower * lower);
static int stm32_poweroff (struct ubxmdm_lower * lower);
static int stm32_reset    (struct ubxmdm_lower * lower);
static int stm32_getstatus(struct ubxmdm_lower * lower,
                              struct ubxmdm_status * status);
static int stm32_ioctl    (struct ubxmdm_lower * lower,
                              int cmd,
                              unsigned long arg);

/* "Lower half" driver state */

static struct stm32_ubxmdm_lower stm32_ubxmdm_lower;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct stm32_ubxmdm_pins stm32_ubxmdm_pins =
{
  .ldo_enable   = GPIO_UBXMDM_LDO,
  .power_on_n   = GPIO_UBXMDM_PWR,
  .reset_n      = 0,
  .shifter_en_n = 0,
  .usb_detect   = 0,
};

static const struct ubxmdm_ops stm32_ubxmdm_ops =
{
  .poweron      = stm32_poweron,
  .poweroff     = stm32_poweroff,
  .reset        = stm32_reset,
  .getstatus    = stm32_getstatus,
  .ioctl        = stm32_ioctl,
};

static const struct stm32_name_pin stm32_ubxmdm_name_pins[] =
{
  {
    {
      'T', 'X', 'D'
    },
    GPIO_USART1_TX
  },
  {
    {
      'R', 'X', 'D'
    },
    GPIO_USART1_RX
  },
#if 0
  {
    {
      'C', 'T', 'S'
    },
    0
  },
  {
    {
      'R', 'T', 'S'
    },
    0
  },
  {
    {
      'D', 'C', 'D'
    },
    0
  },
  {
    {
      'D', 'S', 'R'
    },
    0
  },
  {
    {
      'D', 'T', 'R'
    },
    0
  },
  {
    {
      'R', 'I', ' '
    },
    0
  },
  {
    {
      'I', 'O', '1'
    },
    0
  },
  {
    {
      'R', 'S', 'T'
    },
    0
  },
#endif
  {
    {
      'P', 'W', 'R'
    },
    GPIO_UBXMDM_PWR
  },
#if 0
  {
    {
      'D', 'E', 'T'
    },
    0
  },
#endif
  {
    {
      'L', 'D', 'O'
    },
    GPIO_UBXMDM_LDO
  },
#if 0
  {
    {
      'L', 'V', 'L'
    },
    0
  },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_poweron(struct ubxmdm_lower * lower)
{
  struct stm32_ubxmdm_lower * priv =
    (struct stm32_ubxmdm_lower *) lower;

  stm32_configgpio(priv->pins->ldo_enable);
  stm32_configgpio(priv->pins->power_on_n);

  stm32_gpiowrite(priv->pins->ldo_enable, 1); // ldo:1
  stm32_gpiowrite(priv->pins->power_on_n, 0); // pwr:1

  nxsig_usleep(300*1000);

  stm32_gpiowrite(priv->pins->ldo_enable, 1); // ldo:1
  stm32_gpiowrite(priv->pins->power_on_n, 1); // pwr:0

  nxsig_sleep(2);

  stm32_gpiowrite(priv->pins->ldo_enable, 1); // ldo:1
  stm32_gpiowrite(priv->pins->power_on_n, 0); // pwr:1

  return OK;
}

static int stm32_poweroff(struct ubxmdm_lower * lower)
{
  struct stm32_ubxmdm_lower * priv =
    (struct stm32_ubxmdm_lower *) lower;

  stm32_configgpio(priv->pins->ldo_enable);
  stm32_configgpio(priv->pins->power_on_n);

  stm32_gpiowrite(priv->pins->ldo_enable, 0); // ldo:0
  stm32_gpiowrite(priv->pins->power_on_n, 1); // pwr:0

  return OK;
}

static int stm32_reset(struct ubxmdm_lower * lower)
{
#if 0
  struct stm32_ubxmdm_lower * priv =
    (struct stm32_ubxmdm_lower *) lower;

  /* Modem in reset */

  stm32_configgpio(priv->pins->reset_n | GPIO_VALUE_ZERO);

  /* The minimum reset_n low time is 50 ms */

  nxsig_usleep(75 * 1000);

  /* Modem not in reset */

  stm32_configgpio(priv->pins->reset_n | GPIO_VALUE_ONE);
#endif
  return OK;
}

static int stm32_getstatus(struct ubxmdm_lower * lower,
                              struct ubxmdm_status * status)
{
  struct stm32_ubxmdm_lower * priv =
    (struct stm32_ubxmdm_lower *) lower;
  int i;

  status->on =
    stm32_gpioread(priv->pins->ldo_enable) &&
    !stm32_gpioread(priv->pins->power_on_n);

  DEBUGASSERT(status->register_values_size >= UBXMDM_REGISTER_COUNT);
  status->register_values_size = UBXMDM_REGISTER_COUNT;

  for (i = 0; i < UBXMDM_REGISTER_COUNT; i++)
    {
      strncpy(status->register_values[i].name,
              stm32_ubxmdm_name_pins[i].name,
              3);
      status->register_values[i].val =
        stm32_gpioread(stm32_ubxmdm_name_pins[i].pin);
    }

  return OK;
}

static int stm32_ioctl(struct ubxmdm_lower * lower,
                          int cmd, unsigned long arg)
{
  /* No platform-specific IOCTL at the moment. */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ubxmdm_init
 *
 * Description:
 *   Initialize the modem.  The modem is initialized and
 *   registered at '/dev/ubxmdm'.
 *
 * Input Parameters:
 *   usb_used - enables the USB sense pin if 'true'
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_ubxmdm_init(bool usb_used)
{
  struct stm32_ubxmdm_lower * priv = &stm32_ubxmdm_lower;

  DEBUGASSERT(priv->ops == NULL && priv->pins == NULL);

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops      = &stm32_ubxmdm_ops;
  priv->pins     = &stm32_ubxmdm_pins;
  priv->usb_used = usb_used;

  stm32_configgpio(priv->pins->ldo_enable);
  stm32_configgpio(priv->pins->power_on_n);

  stm32_poweroff((struct ubxmdm_lower *) priv);

  ubxmdm_register("/dev/ubxmdm", (struct ubxmdm_lower *) priv);
}
