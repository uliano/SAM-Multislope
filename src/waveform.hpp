#ifndef _WAVEFORM_HPP_
#define _WAVEFORM_HPP_

#include "samc21.h"
#include "pin.hpp"

// TCC0 — NPWM single-slope, clocked from GCLK1 (24 MHz XOSC)
// PER = 63  →  24 MHz / 64 = 375 kHz
//
//  WO0  PA08  mux E   CC[0] = 56   56/64 duty  (87.5%)
//  WO2  PA10  mux F   CC[2] =  8    8/64 duty  (12.5%)
//  WO3  PA11  mux F   CC[3] = 32   32/64 duty  (50%)

struct Waveform
{
    static void init(void)
    {
        using namespace sam::gpio;

        MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC0;

        // GCLK1 (24 MHz XOSC) → TCC0
        GCLK->PCHCTRL[TCC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN(1) | GCLK_PCHCTRL_CHEN;
        while (!(GCLK->PCHCTRL[TCC0_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));

        TCC0->CTRLA.reg = TCC_CTRLA_SWRST;
        while (TCC0->SYNCBUSY.reg);

        TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
        while (TCC0->SYNCBUSY.reg);

        TCC0->PER.reg  = 63;
        TCC0->CC[0].reg = 56;   // PA08  87.5%
        TCC0->CC[2].reg =  8;   // PA10  12.5%
        TCC0->CC[3].reg = 32;   // PA11  50%
        while (TCC0->SYNCBUSY.reg);

        // Pin mux — after timer is configured, before enable
        Pin<Bank::A,  8>::set_peripheral(Peripheral::E, false);  // WO0
        Pin<Bank::A, 10>::set_peripheral(Peripheral::F, false);  // WO2
        Pin<Bank::A, 11>::set_peripheral(Peripheral::F, false);  // WO3

        TCC0->CTRLA.reg = TCC_CTRLA_ENABLE;
        while (TCC0->SYNCBUSY.reg);
    }
};

#endif // _WAVEFORM_HPP_
