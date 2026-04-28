#ifndef _PINS_HPP_
#define _PINS_HPP_

#include "pin.hpp"

// UART→USB bridge (CH340 or similar) → COM5
using UartTxPin = sam::gpio::Pin<sam::gpio::Bank::B, 30>;
using UartRxPin = sam::gpio::Pin<sam::gpio::Bank::B, 31>;

// Current bench jumper: active-high LED.
using Led0Pin = sam::gpio::Pin<sam::gpio::Bank::B, 23>;

// Current bench jumper: active-high button with external pull-down.
using Btn0Pin = sam::gpio::Pin<sam::gpio::Bank::B, 22>;


#endif // _PINS_HPP_

