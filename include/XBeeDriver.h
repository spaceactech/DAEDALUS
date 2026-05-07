#ifndef XBEE_DRIVER_H
#define XBEE_DRIVER_H

#pragma once
#include <Arduino.h>

extern HardwareSerial XbeeSerial;
extern int8_t         xbee_rssi_dbm;

// ── AP=2 (escaped API mode) frame parser ─────────────────────────────────────

enum class XBeeState : uint8_t { SYNC, LEN_H, LEN_L, DATA, CHKSUM };

inline XBeeState xb_state = XBeeState::SYNC;
inline uint8_t   xb_buf[256]{};
inline uint16_t  xb_len   = 0;
inline uint16_t  xb_idx   = 0;
inline uint8_t   xb_sum   = 0;
inline bool      xb_esc   = false;

inline void xbWriteEsc(uint8_t b) {
  if (b == 0x7E || b == 0x7D || b == 0x11 || b == 0x13) {
    XbeeSerial.write(0x7D);
    XbeeSerial.write(b ^ 0x20);
  } else {
    XbeeSerial.write(b);
  }
}

inline void xbSendAtDB() {
  static const uint8_t frame[] = { 0x08, 0x01, 0x44, 0x42 };
  uint8_t sum = 0;
  for (auto x : frame) sum += x;
  XbeeSerial.write(0x7E);
  xbWriteEsc(0x00);
  xbWriteEsc(sizeof(frame));
  for (auto x : frame) xbWriteEsc(x);
  xbWriteEsc(0xFF - sum);
}

// Returns true when a complete, checksum-valid frame is ready in xb_buf[0..xb_len-1]
inline bool xbFeedByte(uint8_t raw) {
  if (raw == 0x7E) {
    xb_state = XBeeState::LEN_H;
    xb_len = 0; xb_idx = 0; xb_sum = 0; xb_esc = false;
    return false;
  }
  if (xb_state == XBeeState::SYNC) return false;
  if (raw == 0x7D) { xb_esc = true; return false; }
  uint8_t b = xb_esc ? (raw ^ 0x20) : raw;
  xb_esc = false;

  switch (xb_state) {
    case XBeeState::LEN_H:
      xb_len = (uint16_t)b << 8;
      xb_state = XBeeState::LEN_L;
      break;
    case XBeeState::LEN_L:
      xb_len |= b;
      xb_state = (xb_len == 0 || xb_len > sizeof(xb_buf)) ? XBeeState::SYNC : XBeeState::DATA;
      break;
    case XBeeState::DATA:
      xb_buf[xb_idx++] = b;
      xb_sum += b;
      if (xb_idx >= xb_len) xb_state = XBeeState::CHKSUM;
      break;
    case XBeeState::CHKSUM:
      xb_state = XBeeState::SYNC;
      return ((xb_sum + b) & 0xFF) == 0xFF;
    default:
      xb_state = XBeeState::SYNC;
      break;
  }
  return false;
}

#endif