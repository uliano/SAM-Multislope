#ifndef _PINS_HPP_
#define _PINS_HPP_

#include "pin.hpp"

using UartTxPin = sam::gpio::Pin<sam::gpio::Bank::B, 10>;
using UartRxPin = sam::gpio::Pin<sam::gpio::Bank::B, 11>;

#endif // _PINS_HPP_

