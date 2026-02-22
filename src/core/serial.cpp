#include "serial.hpp"

Serial::UartType Serial::uart_;
Serial::StreamType Serial::stream_(Serial::uart_);

Serial::UartType& Serial::uart_instance(void)
{
  return uart_;
}

Serial::StreamType& Serial::stream_instance(void)
{
  return stream_;
}

void Serial::init(uint32_t baud, uint8_t gclk_generator)
{
  uart_instance().init(baud, gclk_generator);
  initialized_ = true;
}

bool Serial::ready(void)
{
  return initialized_ && uart_instance().is_initialized();
}

size_t Serial::write(const uint8_t *data, size_t size)
{
  if (!ready())
    return 0;

  return stream_instance().write(data, size);
}

size_t Serial::read(uint8_t *data, size_t size)
{
  if (!ready())
    return 0;

  return stream_instance().read(data, size);
}

size_t Serial::available(void)
{
  if (!ready())
    return 0;

  return stream_instance().available();
}

void Serial::flush(void)
{
  if (ready())
    stream_instance().flush();
}

bool Serial::read_char(char& out)
{
  if (!ready())
    return false;

  uint8_t value = 0;
  if (1 != read(&value, 1))
    return false;

  out = (char)value;
  return true;
}

void Serial::irq_handler(void)
{
  if (ready())
    uart_instance().irq_handler();
}

void Serial::dma_irq_handler(void)
{
  if (ready())
    uart_instance().dma_irq_handler();
}

size_t Serial::print(const char *text)
{
  if (!ready())
    return 0;

  return stream_instance().print(text);
}

size_t Serial::print(uint8_t value, PrintBase base)
{
  if (!ready())
    return 0;

  return stream_instance().print(value, base);
}

size_t Serial::print(uint16_t value, PrintBase base)
{
  if (!ready())
    return 0;

  return stream_instance().print(value, base);
}

size_t Serial::print(uint32_t value, PrintBase base)
{
  if (!ready())
    return 0;

  return stream_instance().print(value, base);
}

size_t Serial::print(uint64_t value, PrintBase base)
{
  if (!ready())
    return 0;

  return stream_instance().print(value, base);
}

size_t Serial::print(int8_t value, PrintBase base)
{
  if (!ready())
    return 0;

  return stream_instance().print(value, base);
}

size_t Serial::print(int16_t value, PrintBase base)
{
  if (!ready())
    return 0;

  return stream_instance().print(value, base);
}

size_t Serial::print(int32_t value, PrintBase base)
{
  if (!ready())
    return 0;

  return stream_instance().print(value, base);
}

size_t Serial::print(int64_t value, PrintBase base)
{
  if (!ready())
    return 0;

  return stream_instance().print(value, base);
}

size_t Serial::newline(void)
{
  if (!ready())
    return 0;

  return stream_instance().newline();
}
