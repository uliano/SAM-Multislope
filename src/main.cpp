/*
 * Copyright (c) 2016, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samc21.h"
#include "pin.hpp"
#include "globals.hpp"

//-----------------------------------------------------------------------------
#define PERIOD_FAST     100
#define PERIOD_SLOW     500

using LedPin = sam::gpio::Pin<sam::gpio::Bank::A, 15>;
using ButtonPin = sam::gpio::Pin<sam::gpio::Bank::A, 28>;

//-----------------------------------------------------------------------------
static void timer_set_period(uint16_t i)
{
  TC3->COUNT16.CC[0].reg = (F_CPU / 1000ul / 1024) * i;
  TC3->COUNT16.COUNT.reg = 0;
}

//-----------------------------------------------------------------------------
extern "C" void irq_handler_tc3(void)
{
  if (TC3->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1))
  {
    LedPin::toggle();
    TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
  }
}

//-----------------------------------------------------------------------------
extern "C" void irq_handler_sercom4(void)
{
  g_uart.irq_handler();
}

//-----------------------------------------------------------------------------
static void timer_init(void)
{
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC3;

  GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN(0) | GCLK_PCHCTRL_CHEN;
  while (0 == (GCLK->PCHCTRL[TC3_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));

  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1024 |
      TC_CTRLA_PRESCSYNC_RESYNC;

  TC3->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;

  TC3->COUNT16.COUNT.reg = 0;

  timer_set_period(PERIOD_SLOW);

  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

  TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);

  NVIC_EnableIRQ(TC3_IRQn);
}

//-----------------------------------------------------------------------------
static void uart_init(uint32_t baud)
{
  g_uart.init(baud);
}

//-----------------------------------------------------------------------------
static void uart_putc(char c)
{
  g_uart.write_blocking((uint8_t)c);
}

//-----------------------------------------------------------------------------
static void uart_puts(const char *s)
{
  g_uart.write_blocking(s);
}

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  // Set flash wait states to maximum for 48 MHz operation
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS(2) | NVMCTRL_CTRLB_MANW;

  // Switch to 48MHz clock (disable prescaler)
  OSCCTRL->OSC48MDIV.reg = OSCCTRL_OSC48MDIV_DIV(0);
  while (OSCCTRL->OSC48MSYNCBUSY.reg);
}

//-----------------------------------------------------------------------------
int main(void)
{
  uint32_t cnt = 0;
  bool fast = false;

  sys_init();
  timer_init();
  uart_init(115200);

  uart_puts("\r\nHello, world!\r\n");

  LedPin::as_output();
  LedPin::clear();

  ButtonPin::as_input(sam::gpio::Pull::Up);

  while (1)
  {
    if (ButtonPin::read())
      cnt = 0;
    else if (cnt < 5001)
      cnt++;

    if (5000 == cnt)
    {
      fast = !fast;
      timer_set_period(fast ? PERIOD_FAST : PERIOD_SLOW);
      uart_putc('.');
    }
  }

  return 0;
}
