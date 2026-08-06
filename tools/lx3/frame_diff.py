#!/usr/bin/env python3
"""Capture the complete 0x10b frame idle vs during a real button press, and diff them.

ICBM builds its press by replaying the car's last frame and setting one bit in byte 10. That
assumes a press is only that bit. If a real press also changes some other field, our frame is
not a valid press and the car is right to ignore it -- which would explain the set speed never
following even though the frames go out.

Run in offroad mode so nothing openpilot does is in the way. Press + a few times, then -.

Prints:
  IDLE     the most common frame with no button held
  PRESS    each distinct frame seen while a button bit is set
  DIFF     which bytes differ, ignoring checksum and counter
"""
import sys
import time
from collections import Counter

from openpilot.cereal import messaging

ADDR = 0x10B
IGNORE = (0, 1, 2)   # bytes 0-1 checksum, byte 2 counter -- expected to differ every frame
BTN_BYTE = 10
BTN_MASK = 0x87      # bit 0 +, bit 1 -, bit 2 resume, bit 7 LFA
SECONDS = 30


def hexs(b):
  return " ".join(f"{x:02x}" for x in b)


def main():
  sock = messaging.sub_sock("can", conflate=False, timeout=1000)
  idle = Counter()
  pressed = {}

  print(f"capturing {SECONDS}s -- press + a few times, then -, with pauses between",
        file=sys.stderr)
  end = time.monotonic() + SECONDS
  while time.monotonic() < end:
    for msg in messaging.drain_sock(sock):
      for c in msg.can:
        if c.address != ADDR or c.src >= 128 or len(c.dat) < 16:
          continue
        dat = bytes(c.dat)
        key = tuple(v for i, v in enumerate(dat) if i not in IGNORE)
        if dat[BTN_BYTE] & BTN_MASK:
          pressed.setdefault(dat[BTN_BYTE] & BTN_MASK, []).append(dat)
        else:
          idle[key] += 1
    time.sleep(0.002)

  if not idle:
    print("saw no 0x10b frames -- is the car on?")
    return

  # Rebuild a representative idle frame from the most common non-volatile content
  common = idle.most_common(1)[0][0]
  idle_full = bytearray(16)
  j = 0
  for i in range(16):
    if i in IGNORE:
      continue
    idle_full[i] = common[j]
    j += 1

  print(f"\nIDLE   ({sum(idle.values())} frames, {len(idle)} distinct)")
  print(f"       {hexs(idle_full)}")
  if len(idle) > 1:
    print(f"       note: {len(idle)} distinct idle frames -- other fields move on their own")

  if not pressed:
    print("\nNO PRESS CAPTURED -- byte 10 never had a button bit set")
    return

  for btn, frames in sorted(pressed.items()):
    name = {1: "+", 2: "-", 4: "resume", 128: "LFA"}.get(btn, hex(btn))
    f = frames[len(frames) // 2]
    print(f"\nPRESS {name}  ({len(frames)} frames)")
    print(f"       {hexs(f)}")
    diff = [i for i in range(16) if i not in IGNORE and f[i] != idle_full[i]]
    print(f"DIFF   bytes {diff}")
    for i in diff:
      extra = "  <- the button byte" if i == BTN_BYTE else "  <-- UNEXPECTED"
      print(f"       byte{i:<2} idle=0x{idle_full[i]:02x} press=0x{f[i]:02x}{extra}")


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
