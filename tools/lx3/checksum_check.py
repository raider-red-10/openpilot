#!/usr/bin/env python3
"""Does 0x10b carry the standard HKG CAN FD checksum in bytes 0-1?

To make ICBM press the buttons this car actually reads, we have to transmit a modified 0x10b
frame, which means computing its checksum. Declaring CHECKSUM in the DBC is what makes the
packer compute it -- but it also makes the parser *validate* it on receive. If the assumption
is wrong, every 0x10b frame gets dropped and we lose the cruise buttons and the LFA button
that are finally working.

So check it against frames the car is already sending, using the same function the parser
would use. Read-only, works parked.
"""
import sys
import time
from collections import Counter

from openpilot.cereal import messaging
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum

TARGETS = (0x10B, 0x1AA)  # 0x1aa is a known-good control: it declares CHECKSUM today
SECONDS = 15


def main():
  sock = messaging.sub_sock("can", conflate=False, timeout=1000)
  match = Counter()
  total = Counter()
  sizes = {}

  print(f"sampling for {SECONDS}s...", file=sys.stderr)
  end = time.monotonic() + SECONDS
  while time.monotonic() < end:
    for msg in messaging.drain_sock(sock):
      for c in msg.can:
        if c.address not in TARGETS or c.src >= 128:
          continue
        dat = bytearray(c.dat)
        key = (c.src, c.address)
        sizes[key] = len(dat)
        total[key] += 1
        embedded = dat[0] | (dat[1] << 8)          # CHECKSUM is bits 0..15, little endian
        expected = hkg_can_fd_checksum(c.address, None, dat)
        if embedded == expected:
          match[key] += 1
    time.sleep(0.002)

  print()
  for key in sorted(total):
    bus, addr = key
    n, ok = total[key], match[key]
    pct = 100.0 * ok / n if n else 0.0
    verdict = "MATCHES" if pct > 99 else ("NO MATCH" if pct < 1 else "PARTIAL -- suspicious")
    print(f"bus{bus} 0x{addr:03x}  {sizes[key]} bytes  {n} frames  {ok} checksum matches "
          f"({pct:.1f}%)  {verdict}")

  if not total:
    print("saw neither 0x10b nor 0x1aa -- is the car on?")


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
