#ifndef _SERIAL_HPP_
#define _SERIAL_HPP_

#include <stddef.h>
#include <stdint.h>

#include "pins.hpp"
#include "uart.hpp"
#include "uart_bytestream.hpp"

class Serial
{
public:
  static void init(uint32_t baud, uint8_t gclk_generator = 0);
  static bool ready(void);

  static size_t write(const uint8_t *data, size_t size);
  static size_t read(uint8_t *data, size_t size);
  static size_t available(void);
  static void flush(void);
  static bool read_char(char& out);

  static void irq_handler(void);
  static void dma_irq_handler(void);

  static size_t print(const char *text);
  static size_t print(uint8_t value, PrintBase base = PrintBase::Dec);
  static size_t print(uint16_t value, PrintBase base = PrintBase::Dec);
  static size_t print(uint32_t value, PrintBase base = PrintBase::Dec);
  static size_t print(uint64_t value, PrintBase base = PrintBase::Dec);
  static size_t print(int8_t value, PrintBase base = PrintBase::Dec);
  static size_t print(int16_t value, PrintBase base = PrintBase::Dec);
  static size_t print(int32_t value, PrintBase base = PrintBase::Dec);
  static size_t print(int64_t value, PrintBase base = PrintBase::Dec);
  static size_t newline(void);

private:
  using UartTraits = SercomTraits<4>;
  using UartPinoutType = ::UartPinout<UartTxPin, UartRxPin, sam::gpio::Peripheral::D, 1, 3>;
  using UartType = UartDMA<UartTraits, UartPinoutType, 8, 8>;
  using StreamType = UartByteStream<UartType>;

  static UartType uart_;
  static StreamType stream_;

  static UartType& uart_instance(void);
  static StreamType& stream_instance(void);

  static inline bool initialized_ = false;
};

#endif // _SERIAL_HPP_
