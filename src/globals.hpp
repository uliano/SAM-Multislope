#ifndef _GLOBALS_HPP_
#define _GLOBALS_HPP_

#include "pins.hpp"
#include "uart.hpp"
#include "uart_bytestream.hpp"

using DebugUartTraits = SercomTraits<4>;
using DebugUartPinout = UartPinout<UartTxPin, UartRxPin, sam::gpio::Peripheral::D, 1, 3>;
using DebugUart = UartINT<DebugUartTraits, DebugUartPinout, 8, 8>;
using DebugUartStream = UartByteStream<DebugUart>;

extern DebugUart g_uart;
extern DebugUartStream g_uart_stream;

static_assert(ByteStream<DebugUartStream>, "DebugUartStream must satisfy ByteStream concept");

#endif // _GLOBALS_HPP_
