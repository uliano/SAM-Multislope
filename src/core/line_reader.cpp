#include "line_reader.hpp"

#include "serial.hpp"
#include "timebase.hpp"

LineReader::LineReader(char *buffer, size_t capacity, bool echo) :
  buffer_(buffer),
  capacity_(capacity),
  length_(0),
  echo_(echo),
  started_(false),
  skip_lf_once_(false),
  line_ready_(false),
  overflow_(false),
  timeout_ms_(0),
  start_ms_(0)
{
  if (has_valid_storage())
    buffer_[0] = '\0';
}

void LineReader::reset(void)
{
  length_ = 0;
  started_ = false;
  line_ready_ = false;
  overflow_ = false;
  start_ms_ = 0;

  if (has_valid_storage())
    buffer_[0] = '\0';
}

void LineReader::consume_line(void)
{
  reset();
}

void LineReader::set_echo(bool enabled)
{
  echo_ = enabled;
}

void LineReader::set_timeout_ms(uint32_t timeout_ms)
{
  timeout_ms_ = timeout_ms;
}

LineReader::PollResult LineReader::poll(void)
{
  if (!has_valid_storage())
    return { Status::InvalidArgs, 0 };

  if (!Serial::ready())
    return { Status::Pending, 0 };

  if (line_ready_)
    return current_status();

  if (started_ && timeout_enabled() && timeout_expired(Timebase::millis()))
  {
    const size_t current_length = length_;
    reset();
    return { Status::Timeout, current_length };
  }

  char c = 0;

  while (Serial::read_char(c))
  {
    if (!started_)
      mark_started(Timebase::millis());

    if (skip_lf_once_ && ('\n' == c))
    {
      skip_lf_once_ = false;
      continue;
    }
    skip_lf_once_ = false;

    if (('\r' == c) || ('\n' == c))
    {
      if ('\r' == c)
        skip_lf_once_ = true;

      if (echo_)
        Serial::newline();

      finish_line();
      return current_status();
    }

    if (('\b' == c) || (0x7f == (uint8_t)c))
    {
      handle_backspace();
      continue;
    }

    if (!is_accepted_char(c))
      continue;

    if (length_ < (capacity_ - 1))
    {
      buffer_[length_++] = c;
      buffer_[length_] = '\0';

      if (echo_)
        Serial::write((const uint8_t *)&c, 1);
    }
    else
    {
      overflow_ = true;
    }
  }

  if (started_ && timeout_enabled() && timeout_expired(Timebase::millis()))
  {
    const size_t current_length = length_;
    reset();
    return { Status::Timeout, current_length };
  }

  return { Status::Pending, length_ };
}

bool LineReader::line_ready(void) const
{
  return line_ready_;
}

const char *LineReader::c_str(void) const
{
  if (!has_valid_storage())
    return "";

  return buffer_;
}

size_t LineReader::length(void) const
{
  return length_;
}

bool LineReader::has_valid_storage(void) const
{
  return (0 != buffer_) && (capacity_ >= 2);
}

bool LineReader::timeout_enabled(void) const
{
  return timeout_ms_ > 0;
}

bool LineReader::timeout_expired(uint32_t now_ms) const
{
  return (uint32_t)(now_ms - start_ms_) >= timeout_ms_;
}

void LineReader::mark_started(uint32_t now_ms)
{
  started_ = true;
  start_ms_ = now_ms;
}

void LineReader::handle_backspace(void)
{
  if (0 == length_)
    return;

  --length_;
  buffer_[length_] = '\0';

  if (echo_)
  {
    static const uint8_t backspace_seq[] = { '\b', ' ', '\b' };
    Serial::write(backspace_seq, sizeof(backspace_seq));
  }
}

void LineReader::finish_line(void)
{
  line_ready_ = true;
  started_ = false;
}

LineReader::PollResult LineReader::current_status(void) const
{
  if (!line_ready_)
    return { Status::Pending, length_ };

  if (overflow_)
    return { Status::Overflow, length_ };

  return { Status::Ready, length_ };
}

bool LineReader::is_accepted_char(char c)
{
  return ((c >= 0x20) && (c <= 0x7e)) || ('\t' == c);
}
