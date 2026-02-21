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
#include "line_reader.hpp"
#include "pin.hpp"
#include "serial.hpp"
#include "timebase.hpp"

//-----------------------------------------------------------------------------
#define PERIOD_FAST     100
#define PERIOD_SLOW     500
#define BUTTON_DEBOUNCE_MS  30

using LedPin = sam::gpio::Pin<sam::gpio::Bank::A, 15>;
using ButtonPin = sam::gpio::Pin<sam::gpio::Bank::A, 28>;

// SW0 on Xplained Pro is wired active-low (pressed = 0) with pull-up.
static inline bool button_pressed(void)
{
  return !ButtonPin::read();
}

static inline bool streq(const char *a, const char *b)
{
  return 0 == strcmp(a, b);
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
  bool fast = false;
  bool button_raw = false;
  bool button_debounced = false;
  char cli_buffer[64];
  LineReader cli(cli_buffer, sizeof(cli_buffer));
  uint32_t blink_period = PERIOD_SLOW;
  uint32_t last_blink_ms = 0;
  uint32_t button_change_ms = 0;

  sys_init();
  Timebase::init();
  Serial::init(115200);

  Serial::newline();
  Serial::print("Hello, world!");
  Serial::newline();
  Serial::print("Type 'help' then ENTER.");
  Serial::newline();
  Serial::print("> ");

  LedPin::as_output();
  LedPin::clear();

  ButtonPin::as_input(sam::gpio::Pull::Up);
  button_raw = button_pressed();
  button_debounced = button_raw;
  last_blink_ms = Timebase::millis();
  button_change_ms = last_blink_ms;

  while (1)
  {
    uint32_t now = Timebase::millis();
    bool raw_now = button_pressed();
    LineReader::PollResult cli_result = cli.poll();

    if (LineReader::Status::Ready == cli_result.status)
    {
      const char *cmd = cli.c_str();

      if (streq(cmd, "help"))
      {
        Serial::print("Commands: help, status, fast, slow");
        Serial::newline();
      }
      else if (streq(cmd, "status"))
      {
        Serial::print("mode=");
        Serial::print(fast ? "fast" : "slow");
        Serial::print(" period_ms=");
        Serial::print((uint32_t)blink_period);
        Serial::print(" uptime_ms=");
        Serial::print(Timebase::millis());
        Serial::newline();
      }
      else if (streq(cmd, "fast"))
      {
        fast = true;
        blink_period = PERIOD_FAST;
        Serial::print("Blink mode: FAST");
        Serial::newline();
      }
      else if (streq(cmd, "slow"))
      {
        fast = false;
        blink_period = PERIOD_SLOW;
        Serial::print("Blink mode: SLOW");
        Serial::newline();
      }
      else if (cmd[0] != '\0')
      {
        Serial::print("Unknown command: ");
        Serial::print(cmd);
        Serial::newline();
      }

      cli.consume_line();
      Serial::print("> ");
    }
    else if (LineReader::Status::Overflow == cli_result.status)
    {
      Serial::print("ERR: line too long");
      Serial::newline();
      cli.consume_line();
      Serial::print("> ");
    }
    else if (LineReader::Status::Timeout == cli_result.status)
    {
      Serial::newline();
      Serial::print("ERR: input timeout");
      Serial::newline();
      cli.consume_line();
      Serial::print("> ");
    }

    if (raw_now != button_raw)
    {
      button_raw = raw_now;
      button_change_ms = now;
    }

    if ((button_debounced != button_raw) &&
        ((uint32_t)(now - button_change_ms) >= BUTTON_DEBOUNCE_MS))
    {
      button_debounced = button_raw;

      if (button_debounced)
      {
        fast = !fast;
        blink_period = fast ? PERIOD_FAST : PERIOD_SLOW;
        Serial::print(".");
      }
    }

    if ((uint32_t)(now - last_blink_ms) >= blink_period)
    {
      last_blink_ms += blink_period;
      LedPin::toggle();
    }
  }

  return 0;
}
