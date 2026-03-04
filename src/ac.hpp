#ifndef _AC_HPP_
#define _AC_HPP_

#include <stdint.h>
#include "samc21.h"
#include "pins.hpp"

// Analog Comparator 0:
//   + input  PA04  mux B  → AC/AIN0   (MUXPOS = PIN0)
//   - input  Vdd/2        → internal  (MUXNEG = VSCALE, SCALER[0] = 31)
//            formula: V_neg = (SCALER+1)/64 × VDD  → 32/64 = VDD/2
//   output  PA12  mux H  → AC/CMP0   (asynchronous — see note)
//
// OUT=ASYNC: the comparator output drives the pin immediately (propagation delay
// only, ~ns). Synchronization to the 375 kHz integration clock is done in
// firmware by reading AC->STATUSA.STATE0 or the pin at each TCC0 edge.
//
// OUT=SYNC with FLEN=OFF does not work on the SAMC21 AC: the synchronous
// output path requires the digital filter to be active (FLEN > 0), otherwise
// the output latch is never updated and the pin stays permanently low.
//
// Clock: GCLK generator 0 (48 MHz, already running) is used for register-write
// synchronization. No new GCLK generator is needed for async output.

struct ACMP
{
  static constexpr uint8_t SCALER_VDD_HALF = 31;   // (31+1)/64 × VDD = VDD/2

  static void init(void)
  {
    // 1. Route GCLK generator 0 (48 MHz) to AC — needed for COMPCTRL sync
    GCLK->PCHCTRL[AC_GCLK_ID].reg = GCLK_PCHCTRL_GEN(0) | GCLK_PCHCTRL_CHEN;
    while (0 == (GCLK->PCHCTRL[AC_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));

    // 2. APB clock
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_AC;

    // 3. Software reset
    AC->CTRLA.reg = AC_CTRLA_SWRST;
    while (AC->SYNCBUSY.reg & AC_SYNCBUSY_SWRST);

    // 4. VDD/2 reference (SCALER is APB-domain only, no sync needed)
    AC->SCALER[0].reg = SCALER_VDD_HALF;

    // 5. Pin mux — before enabling
    AcPosPin::set_peripheral(sam::gpio::Peripheral::B, false);  // PA04 → AIN0 (analog)
    AcOutPin::set_peripheral(sam::gpio::Peripheral::H, false);  // PA12 → CMP0

    // 6. Enable AC module
    AC->CTRLA.reg = AC_CTRLA_ENABLE;
    while (AC->SYNCBUSY.reg & AC_SYNCBUSY_ENABLE);

    // 7. Configure and enable comparator 0 (must be written after CTRLA.ENABLE):
    //    MUXPOS = PIN0   → AIN0 = PA04
    //    MUXNEG = VSCALE → VDD/2 from SCALER[0]
    //    SPEED  = HIGH   → required for fast response
    //    FLEN   = OFF    → no filter
    //    OUT    = ASYNC  → direct asynchronous output to PA12
    AC->COMPCTRL[0].reg = AC_COMPCTRL_MUXPOS_PIN0
                        | AC_COMPCTRL_MUXNEG_VSCALE
                        | AC_COMPCTRL_SPEED_HIGH
                        | AC_COMPCTRL_FLEN_OFF
                        | AC_COMPCTRL_OUT_ASYNC
                        | AC_COMPCTRL_ENABLE;
    while (AC->SYNCBUSY.reg & AC_SYNCBUSY_COMPCTRL0);
  }
};

#endif // _AC_HPP_
