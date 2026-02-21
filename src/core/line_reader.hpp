#ifndef _LINE_READER_HPP_
#define _LINE_READER_HPP_

#include <stddef.h>
#include <stdint.h>

class LineReader
{
public:
  enum class Status : uint8_t
  {
    Pending = 0,
    Ready,
    Overflow,
    Timeout,
    InvalidArgs,
  };

  struct PollResult
  {
    Status status;
    size_t length;
  };

  LineReader(char *buffer, size_t capacity, bool echo = true);

  void reset(void);
  void consume_line(void);

  void set_echo(bool enabled);
  void set_timeout_ms(uint32_t timeout_ms);

  PollResult poll(void);

  bool line_ready(void) const;
  const char *c_str(void) const;
  size_t length(void) const;

private:
  bool has_valid_storage(void) const;
  bool timeout_enabled(void) const;
  bool timeout_expired(uint32_t now_ms) const;
  void mark_started(uint32_t now_ms);
  void handle_backspace(void);
  void finish_line(void);
  PollResult current_status(void) const;

  static bool is_accepted_char(char c);

  char *buffer_;
  size_t capacity_;
  size_t length_;
  bool echo_;
  bool started_;
  bool skip_lf_once_;
  bool line_ready_;
  bool overflow_;
  uint32_t timeout_ms_;
  uint32_t start_ms_;
};

#endif // _LINE_READER_HPP_
