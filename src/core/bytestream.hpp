#ifndef _BYTESTREAM_HPP_
#define _BYTESTREAM_HPP_

#include <concepts>
#include <stddef.h>
#include <stdint.h>

template <typename StreamT>
concept ByteStream =
  requires(StreamT stream, const uint8_t *tx_data, uint8_t *rx_data, size_t size)
{
  { stream.write(tx_data, size) } -> std::same_as<size_t>;
  { stream.read(rx_data, size) } -> std::same_as<size_t>;
  { stream.available() } -> std::same_as<size_t>;
};

template <ByteStream StreamT>
inline size_t bytestream_write_cstr(StreamT& stream, const char *text)
{
  size_t count = 0;

  while (*text)
  {
    const uint8_t c = (uint8_t)*text++;
    if (1 != stream.write(&c, 1))
      break;
    ++count;
  }

  return count;
}

#endif // _BYTESTREAM_HPP_

