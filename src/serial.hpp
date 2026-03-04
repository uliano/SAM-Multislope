#ifndef _SERIAL_HPP_
#define _SERIAL_HPP_

#include "serial_port.hpp"
#include "uart.hpp"
#include "pins.hpp"

using SerialType = SerialPort<
  UartINT<SercomTraits<4>,
          UartPinout<UartTxPin, UartRxPin, sam::gpio::Peripheral::D, 1, 3>,
          8, 8>>;

extern SerialType Serial;

#endif // _SERIAL_HPP_
