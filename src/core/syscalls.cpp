#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>

#include "globals.hpp"

extern "C" int _write(int file, const void *ptr, size_t len)
{
  if ((file != 1) && (file != 2))
  {
    errno = EBADF;
    return -1;
  }

  if (!g_uart.is_initialized())
    return (int)len;

  const uint8_t *data = (const uint8_t *)ptr;

  for (size_t i = 0; i < len; i++)
    g_uart.write_blocking(data[i]);

  return (int)len;
}

extern "C" int _read(int file, void *ptr, size_t len)
{
  if (file != 0)
  {
    errno = EBADF;
    return -1;
  }

  if ((0 == len) || !g_uart.is_initialized())
    return 0;

  uint8_t *data = (uint8_t *)ptr;
  size_t count = 0;
  uint8_t value = 0;

  while (count < len)
  {
    if (!g_uart.read(value))
      break;

    data[count++] = value;
  }

  return (int)count;
}

extern "C" int _close(int file)
{
  if ((file >= 0) && (file <= 2))
    return 0;

  errno = EBADF;
  return -1;
}

extern "C" int _isatty(int file)
{
  if ((file >= 0) && (file <= 2))
    return 1;

  errno = EBADF;
  return 0;
}

extern "C" int _fstat(int file, struct stat *st)
{
  if ((file < 0) || (file > 2))
  {
    errno = EBADF;
    return -1;
  }

  st->st_mode = S_IFCHR;
  return 0;
}

extern "C" int _lseek(int file, int ptr, int dir)
{
  (void)ptr;
  (void)dir;

  if ((file < 0) || (file > 2))
  {
    errno = EBADF;
    return -1;
  }

  errno = ESPIPE;
  return -1;
}
