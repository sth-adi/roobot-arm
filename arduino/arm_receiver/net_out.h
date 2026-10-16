#pragma once
// A fixed-size output queue for the WiFi mirror link. Everything the sketch prints is copied here and sent when the socket has room,
// so a slow or dead link can never block the control loop. When it is full new bytes are dropped (and counted), never waited for.
// Plain C++ with no hardware, so a PC can test it (sim/test_net_out.py).
#include <stddef.h>
#include <stdint.h>

template <size_t N>
struct ByteQueue {
  uint8_t buf[N];
  size_t head, count;          // head = where the next byte goes; the oldest byte is `count` places behind it
  unsigned long dropped;

  ByteQueue() : head(0), count(0), dropped(0) {}

  void clear() { head = 0; count = 0; }

  void push(uint8_t c) {
    if (count == N) { dropped++; return; }
    buf[head] = c;
    head = (head + 1) % N;
    count++;
  }

  // Copies up to `max` of the oldest bytes into dst and removes them. Returns how many.
  size_t take(uint8_t *dst, size_t max) {
    size_t n = count < max ? count : max;
    size_t tail = (head + N - count) % N;
    for (size_t i = 0; i < n; i++) dst[i] = buf[(tail + i) % N];
    count -= n;
    return n;
  }
};
