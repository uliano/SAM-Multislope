#ifndef _HEARTBEAT_HPP_
#define _HEARTBEAT_HPP_

#include <stdint.h>
#include "samc21.h"
#include "pins.hpp"

// TCC0 in NPWM mode (single-slope, count 0..PER).
// f = 48 MHz / (PER+1) = 48 MHz / 128 = 375 kHz
//
//   WO[0] PA08  MUX E  CC[0]=64   50%   duty
//   WO[1] PA09  MUX E  CC[1]=16  12.5%  duty  (1/8)
//   WO[2] PA10  MUX F  CC[2]=112  87.5% duty  (7/8)

struct Heartbeat
{
  static constexpr uint32_t PER  = 127;   // 48 MHz / 128 = 375 kHz
  static constexpr uint32_t CC0  =  64;   // 50%
  static constexpr uint32_t CC1  =  16;   // 1/8
  static constexpr uint32_t CC2  = 112;   // 7/8

  static void init(void)
  {
    // 1. APB bus clock
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC0;

    // 2. GCLK0 (48 MHz, generator 0) → TCC0
    GCLK->PCHCTRL[TCC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN(0) | GCLK_PCHCTRL_CHEN;
    while (0 == (GCLK->PCHCTRL[TCC0_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));

    // 3. Software reset; wait for both clock domains to settle
    TCC0->CTRLA.reg = TCC_CTRLA_SWRST;
    while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_SWRST);

    // 4. Waveform: Normal PWM (single-slope)
    TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
    while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_WAVE);

    // 5. Period
    TCC0->PER.reg = PER;
    while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_PER);

    // 6. Compare / duty cycles
    TCC0->CC[0].reg = CC0;
    while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_CC0);

    TCC0->CC[1].reg = CC1;
    while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_CC1);

    TCC0->CC[2].reg = CC2;
    while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_CC2);

    // 7. Mux pins to TCC0 outputs (output-only, no pull, no input buffer)
    Hb0Pin::set_peripheral(sam::gpio::Peripheral::E, false);  // WO[0]
    Hb1Pin::set_peripheral(sam::gpio::Peripheral::E, false);  // WO[1]
    Hb2Pin::set_peripheral(sam::gpio::Peripheral::F, false);  // WO[2]

    // 8. Enable (prescaler DIV1 — reset default)
    TCC0->CTRLA.reg = TCC_CTRLA_ENABLE;
    while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_ENABLE);
  }
};

#endif // _HEARTBEAT_HPP_
