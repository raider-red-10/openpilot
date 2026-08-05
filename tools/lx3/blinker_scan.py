#!/usr/bin/env python3
"""Find the Palisade's turn signal bits.

BLINKERS (0x413) is absent on this car, so leftBlinker/rightBlinker are always False and
openpilot never starts a lane change. The state is on the bus somewhere; this walks through
phases and reports bits that are set only while one stalk is held.

Run it parked with the car on. Follow the prompts.
"""
import sys
import time
from collections import defaultdict

from openpilot.cereal import messaging

OUT = "/data/blinker_scan.txt"

PHASES = [
  ("baseline", "BOTH OFF -- do not touch anything", 12),
  ("left", "LEFT blinker ON -- push the stalk so it LOCKS on", 12),
  ("off1", "blinkers OFF", 8),
  ("right", "RIGHT blinker ON -- push the stalk so it LOCKS on", 12),
  ("off2", "blinkers OFF", 8),
]

# Phases where no turn signal is active. A real blinker bit must never assert here.
QUIET = ("baseline", "off1", "off2")


def collect(sock, seconds):
  """Bits seen high and low during this window, keyed (bus, addr, bit)."""
  high, low = set(), set()
  seen = defaultdict(int)
  end = time.monotonic() + seconds
  while time.monotonic() < end:
    for msg in messaging.drain_sock(sock, wait_for_one=True):
      for c in msg.can:
        if c.src >= 128:  # relay-forwarded echo of a frame we already counted
          continue
        seen[(c.src, c.address)] += 1
        for byte_i, byte in enumerate(c.dat):
          for bit_i in range(8):
            key = (c.src, c.address, byte_i * 8 + bit_i)
            (high if (byte >> bit_i) & 1 else low).add(key)
  return high, low, seen


def main():
  sock = messaging.sub_sock("can", conflate=False, timeout=1000)
  print("waiting for CAN...")
  for _ in range(50):
    if messaging.drain_sock(sock, wait_for_one=True):
      break
  else:
    sys.exit("no CAN data -- is the car on and openpilot running?")

  results = {}
  totals = defaultdict(int)
  for name, prompt, secs in PHASES:
    print(f"\n=== {prompt} ===")
    for n in (3, 2, 1):
      print(f"  starting in {n}...", flush=True)
      time.sleep(1)
    print(f"  recording {secs}s", flush=True)
    high, low, seen = collect(sock, secs)
    results[name] = (high, low)
    for k, v in seen.items():
      totals[k] += v
    print(f"  {len(seen)} distinct messages, {len(high)} bits high")

  quiet_high = set().union(*(results[p][0] for p in QUIET))
  quiet_low = set().union(*(results[p][1] for p in QUIET))

  lines = []
  for side in ("left", "right"):
    high, low = results[side]
    # active-high: asserted while this stalk is held, never asserted with both off
    rising = sorted(high - quiet_high)
    # active-low: cleared while held, never cleared with both off
    falling = sorted(low - quiet_low)
    other = "right" if side == "left" else "left"
    other_high, other_low = results[other]

    for label, cands, other_set in (("active-high", rising, other_high),
                                    ("active-low", falling, other_low)):
      for bus, addr, bit in cands:
        both = "BOTH" if (bus, addr, bit) in other_set else f"{side.upper()} only"
        lines.append(f"{side:5s} {label:11s} bus={bus} addr=0x{addr:03x} "
                     f"byte={bit // 8:2d} bit={bit % 8} -> {both}  ({totals[(bus, addr)]} frames)")

  out = "\n".join(lines) if lines else "NO CANDIDATES -- see notes at the bottom of the script"
  print("\n" + "=" * 70)
  print(out)
  print("=" * 70)
  with open(OUT, "w") as f:
    f.write(out + "\n")
  print(f"\nwritten to {OUT}")


if __name__ == "__main__":
  main()
