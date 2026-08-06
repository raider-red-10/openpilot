#!/usr/bin/env python3
"""Find the Palisade's cruise button bits.

BLINKERS (0x413) is absent on this car, so leftBlinker/rightBlinker are always False and
openpilot never starts a lane change. The state is on the bus somewhere.

The discriminator is that a blinker FLASHES. A lamp bit toggles continuously while the
stalk is held and is perfectly static otherwise, so we look for bits that change during
one side's phase and never change with both stalks off. Counters, checksums and rolling
sensor values toggle in every phase, which is what makes them fall out -- "was this bit
ever set" does not separate them and produces hundreds of false hits.

Frames are accumulated as integer bitmasks rather than bit by bit. A CAN FD frame is up
to 512 bits and per-bit Python work cannot keep up with the bus; falling behind is not
just slow, it means a phase records the previous phase's backlog and the comparison is
meaningless.

  scan:  ./button_scan.py                   (parked, car on, follow the prompts)
  watch: ./button_scan.py 0 0x208 12 1      (live value of bus 0, 0x208, byte 12, bit 1)
"""
import sys
import time
from collections import defaultdict

from openpilot.cereal import messaging

OUT = "/data/button_scan.txt"

PHASES = [
  ("baseline", "HANDS OFF -- touch nothing", 12),
  ("plus", "press RES/+ over and over, about once a second", 12),
  ("off1", "HANDS OFF -- touch nothing", 8),
  ("minus", "press SET/- over and over, about once a second", 12),
  ("off2", "HANDS OFF -- touch nothing", 8),
]

# Phases with no button pressed. A button bit must be static in all of them.
QUIET = ("baseline", "off1", "off2")

WIDTH: dict[tuple[int, int], int] = {}


def collect(sock, seconds):
  """Per (bus, addr): bits that changed, bits ever high, and frame/transition counts."""
  changed: dict[tuple[int, int], int] = defaultdict(int)
  ones: dict[tuple[int, int], int] = defaultdict(int)
  prev: dict[tuple[int, int], int] = {}
  seen: dict[tuple[int, int], int] = defaultdict(int)
  edges: dict[tuple[int, int], int] = defaultdict(int)

  end = time.monotonic() + seconds
  while time.monotonic() < end:
    for msg in messaging.drain_sock(sock):
      for c in msg.can:
        if c.src >= 128:  # relay-forwarded echo of a frame we already counted
          continue
        k = (c.src, c.address)
        v = int.from_bytes(c.dat, "little")
        seen[k] += 1
        ones[k] |= v
        if k in prev:
          diff = prev[k] ^ v
          if diff:
            changed[k] |= diff
            edges[k] += 1
        prev[k] = v
        if len(c.dat) > WIDTH.get(k, 0):
          WIDTH[k] = len(c.dat)
    time.sleep(0.002)

  return {"changed": changed, "ones": ones, "seen": seen, "edges": edges, "secs": seconds}


def flush(sock):
  """Drop whatever queued during the prompt so a phase only sees its own traffic."""
  for _ in range(500):
    if not messaging.drain_sock(sock):
      return
  print("  warning: could not drain the backlog")


def bits(mask, width):
  for bit in range(8 * width):
    if (mask >> bit) & 1:
      yield bit


def find_candidates(results):
  """Bits that flash during one side's phase and never change with both stalks off."""
  quiet_changed: dict[tuple[int, int], int] = defaultdict(int)
  quiet_seen: dict[tuple[int, int], int] = defaultdict(int)
  for p in QUIET:
    for k, v in results[p]["changed"].items():
      quiet_changed[k] |= v
    for k, v in results[p]["seen"].items():
      quiet_seen[k] += v

  cands = {}
  for side in ("plus", "minus"):
    found = {}
    for k, ch in results[side]["changed"].items():
      # Without quiet observations we cannot claim the bit is normally static
      if quiet_seen.get(k, 0) < 5:
        continue
      flashing = ch & ~quiet_changed[k]
      if flashing:
        found[k] = flashing
    cands[side] = found
  return cands


def watch(bus, addr, byte_i, bit_i):
  """Print the live value of one bit. Flip the stalk and watch it track."""
  sock = messaging.sub_sock("can", conflate=False, timeout=1000)
  print(f"watching bus={bus} addr=0x{addr:03x} byte={byte_i} bit={bit_i} -- ctrl-c to stop")
  last, changes, start = None, 0, time.monotonic()
  while True:
    for msg in messaging.drain_sock(sock):
      for c in msg.can:
        if c.src != bus or c.address != addr or len(c.dat) <= byte_i:
          continue
        val = (c.dat[byte_i] >> bit_i) & 1
        if val != last:
          changes += 1
          hz = changes / max(0.001, time.monotonic() - start) / 2
          print(f"  {time.monotonic() - start:6.2f}s  value={val}   (~{hz:.2f} Hz)", flush=True)
          last = val
    time.sleep(0.002)


def main():
  if len(sys.argv) == 5:
    watch(int(sys.argv[1], 0), int(sys.argv[2], 0), int(sys.argv[3], 0), int(sys.argv[4], 0))
    return

  sock = messaging.sub_sock("can", conflate=False, timeout=1000)
  print("waiting for CAN...")
  for _ in range(50):
    if messaging.drain_sock(sock, wait_for_one=True):
      break
  else:
    sys.exit("no CAN data -- is the car on and openpilot running?")

  results = {}
  for name, prompt, secs in PHASES:
    print(f"\n=== {prompt} ===")
    for n in (3, 2, 1):
      print(f"  starting in {n}...", flush=True)
      time.sleep(1)
    flush(sock)
    print(f"  recording {secs}s", flush=True)
    results[name] = collect(sock, secs)
    r = results[name]
    print(f"  {len(r['seen'])} messages, {sum(r['seen'].values())} frames", flush=True)

  cands = find_candidates(results)

  lines = []
  paired = sorted(set(cands["left"]) & set(cands["right"]))
  lines.append(f"PAIRED -- changes for both buttons, static when idle ({len(paired)} messages)")
  for k in paired:
    bus, addr = k
    w = WIDTH.get(k, 8)
    lb = [f"byte={b // 8} bit={b % 8}" for b in bits(cands["left"][k], w)]
    rb = [f"byte={b // 8} bit={b % 8}" for b in bits(cands["right"][k], w)]
    rate = results["left"]["edges"][k] / results["left"]["secs"]
    lines.append(f"  bus={bus} addr=0x{addr:03x}  PLUS[{', '.join(lb)}]  MINUS[{', '.join(rb)}]"
                 f"   ~{rate:.1f} changes/s")

  for side, other in (("plus", "minus"), ("minus", "plus")):
    only = sorted(set(cands[side]) - set(cands[other]))
    lines.append(f"\n{side.upper()} ONLY ({len(only)} messages)")
    for k in only:
      bus, addr = k
      w = WIDTH.get(k, 8)
      bl = [f"byte={b // 8} bit={b % 8}" for b in bits(cands[side][k], w)]
      lines.append(f"  bus={bus} addr=0x{addr:03x}  [{', '.join(bl)}]")

  out = "\n".join(lines)
  print("\n" + "=" * 74)
  print(out)
  print("=" * 74)
  with open(OUT, "w") as f:
    f.write(out + "\n")
  print(f"\nwritten to {OUT}")
  print("confirm a candidate live, e.g.:  ./button_scan.py 0 0x208 12 1")


if __name__ == "__main__":
  main()
