#ifndef _PINS_HPP_
#define _PINS_HPP_

#include "pin.hpp"

using UartTxPin = sam::gpio::Pin<sam::gpio::Bank::B, 10>;
using UartRxPin = sam::gpio::Pin<sam::gpio::Bank::B, 11>;

using Hb0Pin = sam::gpio::Pin<sam::gpio::Bank::A,  8>;  // TCC0 WO[0] 50%   mux E
using Hb1Pin = sam::gpio::Pin<sam::gpio::Bank::A,  9>;  // TCC0 WO[1] 1/8   mux E
using Hb2Pin = sam::gpio::Pin<sam::gpio::Bank::A, 10>;  // TCC0 WO[2] 7/8   mux F

using AcPosPin = sam::gpio::Pin<sam::gpio::Bank::A,  4>;  // AC AIN0   mux B
using AcOutPin = sam::gpio::Pin<sam::gpio::Bank::A, 12>;  // AC CMP0   mux H

#endif // _PINS_HPP_

