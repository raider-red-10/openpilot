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

BUTTONS_ALT = 0x1AA
CAM_BUS = 2
NAMES = {0: "none", 1: "RES/+", 2: "SET/-", 3: "?3", 4: "CANCEL"}


def button_of(dat):
  # CRUISE_BUTTONS is bits 36..38 -- byte 4, bits 4..6. Same position panda reads.
  return (dat[4] >> 4) & 0x7 if len(dat) > 4 else -1


def main():
  sock_can = messaging.sub_sock("can", conflate=False, timeout=1000)
  sock_send = messaging.sub_sock("sendcan", conflate=False, timeout=1000)

  asked = Counter()     # what openpilot queued
  went_out = Counter()  # what panda actually transmitted
  from_car = Counter()  # the car's own presses
  last_report = 0.0
  start = time.monotonic()

  print("watching 0x1aa -- drive with cruise on and let ICBM try. ctrl-c to stop.", file=sys.stderr)
  while True:
    for msg in messaging.drain_sock(sock_send):
      for c in msg.sendcan:
        if c.address == BUTTONS_ALT:
          asked[button_of(c.dat)] += 1

    for msg in messaging.drain_sock(sock_can):
      for c in msg.can:
        if c.address != BUTTONS_ALT:
          continue
        if c.src == CAM_BUS + 128:
          went_out[button_of(c.dat)] += 1
        elif c.src < 128:
          from_car[button_of(c.dat)] += 1

    now = time.monotonic()
    if now - last_report > 5.0:
      last_report = now
      def fmt(counter):
        live = {NAMES.get(k, k): v for k, v in counter.items() if k != 0}
        return live if live else "-"
      print(f"{now - start:6.0f}s  openpilot asked: {fmt(asked)}   "
            f"panda transmitted: {fmt(went_out)}   car's own: {fmt(from_car)}", flush=True)
    time.sleep(0.002)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
