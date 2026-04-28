#ifndef _INIT_HPP_
#define _INIT_HPP_

#include "samc21.h"

// System clock and oscillator initialization.
// Main clock: OSC48M @ 48 MHz on GCLK0 (unchanged).
// GCLK1:  XOSC   @ 24 MHz       (PA14/PA15 — for timers)
// GCLK3:  unused
inline void sys_init(void)
{
  // 2 wait states required at 48 MHz
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS(2) | NVMCTRL_CTRLB_MANW;

  // OSC48M: remove reset prescaler /2, run at full 48 MHz
  OSCCTRL->OSC48MDIV.reg = OSCCTRL_OSC48MDIV_DIV(0);
  while (OSCCTRL->OSC48MSYNCBUSY.reg);

  // XOSC: 24 MHz crystal (PA14/PA15), automatic gain control
  OSCCTRL->XOSCCTRL.reg =
    OSCCTRL_XOSCCTRL_STARTUP(8) |    // 16384 XOSC cycles ≈ 700 µs
    OSCCTRL_XOSCCTRL_AMPGC |         // automatic gain control
    OSCCTRL_XOSCCTRL_GAIN(4) |       // initial gain for ≤ 30 MHz crystals
    OSCCTRL_XOSCCTRL_XTALEN |        // crystal mode
    OSCCTRL_XOSCCTRL_ENABLE;
  while (!OSCCTRL->STATUS.bit.XOSCRDY);

  // GCLK1 = XOSC @ 24 MHz
  GCLK->GENCTRL[1].reg =
    GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_XOSC_Val) |
    GCLK_GENCTRL_GENEN;
  while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL_GCLK1);
}

#endif // _INIT_HPP_
