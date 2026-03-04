#ifndef _AC_HPP_
#define _AC_HPP_

#include <stdint.h>
#include "samc21.h"
#include "pins.hpp"

// Analog Comparator 2 (COMPCTRL[2]):
//   + input  PB06  mux B  → AC/AIN7   (MUXPOS = PIN3)
//   - input  Vdd/2        → internal  (MUXNEG = VSCALE, SCALER[2] = 31)
//            formula: V_neg = (SCALER+1)/64 × VDD  → 32/64 = VDD/2
//   output  PB30  mux H  → AC/CMP2
//
// COMPCTRL[0]/[1] use AIN0–3 (PA04–PA07) — all occupied on the Xplained Pro.
// COMPCTRL[2]/[3] use AIN4–7 (PA02, PA03, PB05, PB06):
//   PA02=ADC, PA03=VREFA, PB05=Switch → PB06 (AIN7) is the only free pin.
//
// OUT=ASYNC: the comparator output drives the pin immediately (~ns delay).
// OUT=SYNC:  output is sampled by the AC GCLK; requires FLEN > 0.
//            With FLEN=OFF the sync latch is never updated (pin stays low).
//
// GCLK: generator 1 is configured at 48 MHz / 128 = 375 kHz (same as TCC0).

struct ACMP
{
  static constexpr uint8_t SCALER_VDD_HALF = 31;   // (31+1)/64 × VDD = VDD/2

  static void init(void)
  {
    // 1. Configure GCLK generator 1: 48 MHz / 128 = 375 kHz
    GCLK->GENCTRL[1].reg = GCLK_GENCTRL_SRC_OSC48M
                         | GCLK_GENCTRL_DIV(128)
                         | GCLK_GENCTRL_GENEN;
    while (GCLK->SYNCBUSY.reg & (1u << (GCLK_SYNCBUSY_GENCTRL_Pos + 1)));

    // 2. Route GCLK generator 1 (375 kHz) to AC
    GCLK->PCHCTRL[AC_GCLK_ID].reg = GCLK_PCHCTRL_GEN(1) | GCLK_PCHCTRL_CHEN;
    while (0 == (GCLK->PCHCTRL[AC_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));

    // 3. APB clock
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_AC;

    // 4. Software reset
    AC->CTRLA.reg = AC_CTRLA_SWRST;
    while (AC->SYNCBUSY.reg & AC_SYNCBUSY_SWRST);

    // 5. VDD/2 reference for comparator 2 (APB-domain only, no sync needed)
    AC->SCALER[2].reg = SCALER_VDD_HALF;

    // 6. Pin mux — before enabling
    AcPosPin::set_peripheral(sam::gpio::Peripheral::B, false);  // PB06 → AIN7 (analog)
    AcOutPin::set_peripheral(sam::gpio::Peripheral::H, false);  // PB30 → CMP2

    // 7. Enable AC module
    AC->CTRLA.reg = AC_CTRLA_ENABLE;
    while (AC->SYNCBUSY.reg & AC_SYNCBUSY_ENABLE);

    // 8. Configure and enable comparator 2 (must be written after CTRLA.ENABLE):
    //    MUXPOS = PIN3   → AIN7 = PB06
    //    MUXNEG = VSCALE → VDD/2 from SCALER[2]
    //    SPEED  = HIGH   → required for fast response
    //    FLEN   = OFF    → no filter (NOTE: SYNC output requires FLEN > 0)
    //    OUT    = ASYNC  → immediate output (~ns delay)
    AC->COMPCTRL[2].reg = AC_COMPCTRL_MUXPOS_PIN3
                        | AC_COMPCTRL_MUXNEG_VSCALE
                        | AC_COMPCTRL_SPEED_HIGH
                        | AC_COMPCTRL_FLEN_OFF
                        | AC_COMPCTRL_OUT_ASYNC
                        | AC_COMPCTRL_ENABLE;
    while (AC->SYNCBUSY.reg & AC_SYNCBUSY_COMPCTRL2);
  }
};

#endif // _AC_HPP_
