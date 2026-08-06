#!/usr/bin/env python3
"""Record everything the car and openpilot say, once, so analysis needs no further drives.

Every question so far has cost a separate trip because each probe only captured the one thing
it was written to look at. This records the raw streams instead -- all CAN, plus everything
openpilot asks the panda to send -- so any question can be answered afterwards from the file.

  ./capture.py            record until ctrl-c
  ./capture.py 300        record 300 seconds then stop

Writes /data/lx3_capture.bin.gz. Roughly 1 MB per minute compressed.
"""
import gzip
import signal
import struct
import sys
import time

from openpilot.cereal import messaging

OUT = "/data/lx3_capture.bin.gz"
# t(float32) src(uint8) addr(uint32) len(uint8) data
REC = struct.Struct("<fBIB")


def main():
  # pkill sends SIGTERM, which would kill us mid-stream and leave the gzip without its footer.
  # Turn it into KeyboardInterrupt so the file closes cleanly.
  signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

  limit = float(sys.argv[1]) if len(sys.argv) > 1 else None
  can = messaging.sub_sock("can", conflate=False, timeout=1000)
  send = messaging.sub_sock("sendcan", conflate=False, timeout=1000)

  n = 0
  start = time.monotonic()
  last = 0.0
  print(f"recording to {OUT} -- drive normally, use the cruise buttons. ctrl-c to stop.",
        file=sys.stderr)
  with gzip.open(OUT, "wb", compresslevel=6) as f:
    try:
      while limit is None or time.monotonic() - start < limit:
        now = time.monotonic() - start
        for msg in messaging.drain_sock(can):
          for c in msg.can:
            d = bytes(c.dat)
            f.write(REC.pack(now, c.src, c.address, len(d)) + d)
            n += 1
        # openpilot's own requests get src 200+bus so they are distinguishable
        for msg in messaging.drain_sock(send):
          for c in msg.sendcan:
            d = bytes(c.dat)
            f.write(REC.pack(now, 200 + c.src, c.address, len(d)) + d)
            n += 1
        if now - last > 15:
          last = now
          print(f"  {now:6.0f}s  {n} frames", flush=True)
        time.sleep(0.002)
    except KeyboardInterrupt:
      pass
  print(f"\ndone: {n} frames over {time.monotonic() - start:.0f}s -> {OUT}", file=sys.stderr)


if __name__ == "__main__":
  main()
