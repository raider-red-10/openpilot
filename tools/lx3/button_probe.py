#!/usr/bin/env python3
"""Watch a cruise button from the wire all the way to carState.buttonEvents.

Speed Limit Assist waits for a button release and never sees one -- plusHold stayed 0.0 for a
whole drive and no buttonEvent was ever logged. This shows the two layers side by side so the
break is visible:

  RAW   the CRUISE_BUTTONS field in the message the car actually sends
  EVENT what openpilot turned that into (carState.buttonEvents)

Works parked -- the buttons transmit whether or not cruise is engaged.

  1=RES/+  2=SET/-  4=CANCEL
"""
import sys
import time

from openpilot.cereal import messaging

ALT, STD = 0x1AA, 0x1CF
NAMES = {0: "none", 1: "RES/+", 2: "SET/-", 3: "?3", 4: "CANCEL", 5: "?5", 6: "?6", 7: "?7"}


def raw_button(addr, dat):
  # ALT: bits 36..38 (byte 4, bits 4..6).  STD: bits 16..18 (byte 2, bits 0..2)
  if addr == ALT:
    return (dat[4] >> 4) & 0x7 if len(dat) > 4 else -1
  return dat[2] & 0x7 if len(dat) > 2 else -1


def main():
  can = messaging.sub_sock("can", conflate=False, timeout=1000)
  sm = messaging.SubMaster(["carState"])

  print("press RES/+ and SET/- a few times each. ctrl-c to stop.\n", file=sys.stderr)
  start = time.monotonic()
  last_raw = {}
  seen_addr = set()

  while True:
    for msg in messaging.drain_sock(can):
      for c in msg.can:
        if c.address not in (ALT, STD) or c.src >= 128:
          continue
        seen_addr.add((c.src, c.address))
        b = raw_button(c.address, c.dat)
        key = (c.src, c.address)
        if last_raw.get(key) != b:
          last_raw[key] = b
          if b != 0:
            print(f"{time.monotonic() - start:7.2f}  RAW    bus={c.src} 0x{c.address:03x} "
                  f"button={b} ({NAMES.get(b, b)})", flush=True)

    sm.update(0)
    if sm.updated["carState"]:
      for be in sm["carState"].buttonEvents:
        print(f"{time.monotonic() - start:7.2f}  EVENT  type={be.type} pressed={be.pressed}", flush=True)

    time.sleep(0.002)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
