#!/usr/bin/env python3
"""Does our cruise button actually reach the bus, and with what value?

ICBM queues CRUISE_BUTTONS_ALT (0x1aa) frames and the car's set speed does not follow. Three
different problems look identical from inside openpilot:

  1. panda rejects the frame  -> it appears on sendcan but is never transmitted
  2. we send the wrong value  -> it is transmitted, but not the button we intended
  3. the car ignores us       -> transmitted correctly, no effect

openpilot echoes what the panda actually transmitted back on the `can` socket with
src = bus + 128, so all three are directly measurable.

  sendcan   what openpilot asked to send
  src=130   what panda actually put on bus 2
  src=2     the car's own button module

Buttons: 1=RES/+  2=SET/-  4=CANCEL
"""
import sys
import time
from collections import Counter

from openpilot.cereal import messaging

TARGETS = (0x10B, 0x1AA)
CAM_BUS = 2
NAMES = {0: "none", 1: "RES/+", 2: "SET/-", 3: "?3", 4: "CANCEL"}


def button_of(addr, dat):
  if addr == 0x10B:
    # byte 10: bit 0 +, bit 1 -, bit 2 resume, bit 7 LFA
    b = dat[10] if len(dat) > 10 else 0
    return {1: "+", 2: "-", 4: "resume", 128: "LFA"}.get(b & 0x87, "none" if not (b & 0x87) else hex(b))
  return (dat[4] >> 4) & 0x7 if len(dat) > 4 else -1


def main():
  sock_can = messaging.sub_sock("can", conflate=False, timeout=1000)
  sock_send = messaging.sub_sock("sendcan", conflate=False, timeout=1000)

  asked = Counter()     # what openpilot queued
  went_out = Counter()  # what panda actually transmitted
  from_car = Counter()  # the car's own presses
  last_report = 0.0
  start = time.monotonic()

  print("watching 0x10b and 0x1aa -- drive with cruise on and let ICBM try. ctrl-c to stop.", file=sys.stderr)
  while True:
    for msg in messaging.drain_sock(sock_send):
      for c in msg.sendcan:
        if c.address in TARGETS:
          asked[(c.address, button_of(c.address, c.dat))] += 1

    for msg in messaging.drain_sock(sock_can):
      for c in msg.can:
        if c.address not in TARGETS:
          continue
        if c.src >= 128:
          went_out[(c.address, button_of(c.address, c.dat))] += 1
        else:
          from_car[(c.address, button_of(c.address, c.dat))] += 1

    now = time.monotonic()
    if now - last_report > 5.0:
      last_report = now
      def fmt(counter):
        live = {f"0x{a:03x}:{b}": v for (a, b), v in counter.items() if b not in (0, "none")}
        return live if live else "-"
      print(f"{now - start:6.0f}s  openpilot asked: {fmt(asked)}   "
            f"panda transmitted: {fmt(went_out)}   car's own: {fmt(from_car)}", flush=True)
    time.sleep(0.002)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
