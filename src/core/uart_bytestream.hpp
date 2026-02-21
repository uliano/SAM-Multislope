#ifndef _UART_BYTESTREAM_HPP_
#define _UART_BYTESTREAM_HPP_

#include <stddef.h>
#include <stdint.h>

#include "bytestream.hpp"

template <typename UartT>
class UartByteStream : public ByteStreamPrintMixin<UartByteStream<UartT>>
{
public:
  explicit UartByteStream(UartT& uart) : uart_(uart)
  {
  }

  size_t write(const uint8_t *data, size_t size)
  {
    return uart_.write(data, size);
  }

  size_t read(uint8_t *data, size_t size)
  {
    size_t count = 0;
    uint8_t value = 0;

    while (count < size)
    {
      if (!uart_.read(value))
        break;

      data[count++] = value;
    }

    return count;
  }

  size_t available(void) const
  {
    return uart_.available();
  }

  bool ready(void) const
  {
    return uart_.is_initialized();
  }

  void flush(void) const
  {
    while (!uart_.tx_idle());
  }

  UartT& uart(void)
  {
    return uart_;
  }

  const UartT& uart(void) const
  {
    return uart_;
  }

private:
  UartT& uart_;
};

#endif // _UART_BYTESTREAM_HPP_
