#!/usr/bin/env python3
"""Answer questions from a capture.py recording, without driving again.

  ./replay.py summary            what was on the bus, and what openpilot tried to send
  ./replay.py msg 0x10b          every distinct payload for one address, per bus
  ./replay.py diff 0x10b 10      idle vs each value of a byte -- shows what a press changes
  ./replay.py tx                 did openpilot's frames reach the wire
"""
import gzip
import struct
import zlib
import sys
from collections import Counter, defaultdict

IN = "/data/lx3_capture.bin.gz"
REC = struct.Struct("<fBIB")


def read(path=None):
  # resolved at call time so IN can be overridden.
  # Decompress incrementally and keep whatever survives: a capture killed with SIGTERM never
  # gets its gzip footer, and gzip.read() throws away the entire file over a missing trailer.
  blob = b""
  try:
    with gzip.open(path or IN, "rb") as f:
      blob = f.read()
  except (EOFError, OSError):
    d = zlib.decompressobj(16 + zlib.MAX_WBITS)
    chunks = []
    with open(path or IN, "rb") as raw:
      while True:
        piece = raw.read(1 << 20)
        if not piece:
          break
        try:
          chunks.append(d.decompress(piece))
        except zlib.error:
          break
    blob = b"".join(chunks)
    print(f"note: capture was truncated, recovered {len(blob) / 1e6:.1f} MB", file=sys.stderr)
  i, n = 0, len(blob)
  while i + REC.size <= n:
    t, src, addr, ln = REC.unpack_from(blob, i)
    i += REC.size
    yield t, src, addr, blob[i:i + ln]
    i += ln


def summary():
  per = Counter()
  sent = Counter()
  tmax = 0.0
  for t, src, addr, _ in read():
    tmax = max(tmax, t)
    (sent if src >= 200 else per)[(src % 200 if src >= 200 else src, addr)] += 1
  print(f"{tmax:.0f}s recorded, {sum(per.values())} bus frames, {sum(sent.values())} openpilot requests\n")
  print("openpilot asked the panda to send:")
  for (bus, addr), n in sorted(sent.items(), key=lambda kv: -kv[1]):
    print(f"  bus{bus} 0x{addr:03x}  {n}")
  print(f"\n{len(per)} distinct (bus, addr) on the bus")


def msg(addr_hex):
  want = int(addr_hex, 0)
  seen = defaultdict(Counter)
  for _, src, addr, dat in read():
    if addr == want:
      seen[src][dat] += 1
  for src in sorted(seen):
    label = f"openpilot->bus{src - 200}" if src >= 200 else f"bus{src}"
    print(f"\n{label}: {sum(seen[src].values())} frames, {len(seen[src])} distinct payloads")
    for dat, n in seen[src].most_common(8):
      print(f"  {n:6d}  {' '.join(f'{b:02x}' for b in dat)}")


def diff(addr_hex, byte_i):
  want, bi = int(addr_hex, 0), int(byte_i)
  by_val = defaultdict(Counter)
  for _, src, addr, dat in read():
    if addr == want and src < 200 and len(dat) > bi:
      by_val[dat[bi]][tuple(dat)] += 1
  if not by_val:
    print("no frames")
    return
  base_val = max(by_val, key=lambda v: sum(by_val[v].values()))
  base = list(by_val[base_val].most_common(1)[0][0])
  print(f"most common byte{bi} = 0x{base_val:02x}  ({sum(by_val[base_val].values())} frames)")
  print(f"  {' '.join(f'{b:02x}' for b in base)}\n")
  for val in sorted(by_val):
    if val == base_val:
      continue
    frames = by_val[val]
    f = list(frames.most_common(1)[0][0])
    d = [i for i in range(len(f)) if i not in (0, 1, 2) and i < len(base) and f[i] != base[i]]
    print(f"byte{bi} = 0x{val:02x}  ({sum(frames.values())} frames)  differs at bytes {d}")
    print(f"  {' '.join(f'{b:02x}' for b in f)}")


def press(addr_hex="0x10b", byte_i=10):
  """Every real press: how long the bit is held, how many frames, and the counter across it.

  Knowing a press is only byte 10 is not enough to replicate one. What matters is the shape --
  consecutive frames, duration, and whether the counter keeps its natural sequence while the
  button is held.
  """
  want, bi = int(addr_hex, 0), int(byte_i)
  frames = [(t, dat) for t, src, addr, dat in read()
            if addr == want and src < 128 and len(dat) > bi]
  if not frames:
    print("no frames")
    return

  runs, cur = [], []
  for t, dat in frames:
    if dat[bi]:
      cur.append((t, dat))
    elif cur:
      runs.append(cur)
      cur = []
  if cur:
    runs.append(cur)

  names = {1: "+", 2: "-", 4: "resume", 8: "?0x08", 128: "LFA"}
  print(f"{len(frames)} frames of 0x{want:03x}, {len(runs)} presses\n")
  gaps = [b[0][0] - a[-1][0] for a, b in zip(runs, runs[1:])]
  for r in runs[:14]:
    val = r[0][1][bi]
    dur = r[-1][0] - r[0][0]
    ctrs = [d[2] for _, d in r]
    steps = {b - a for a, b in zip(ctrs, ctrs[1:])}
    print(f"  t={r[0][0]:7.2f}s  {names.get(val, hex(val)):>6}  {len(r):2d} frames  "
          f"{dur * 1000:5.0f}ms  counter {ctrs[0]}..{ctrs[-1]} steps={sorted(steps) or ['-']}")
  if len(runs) > 14:
    print(f"  ... {len(runs) - 14} more")

  lens = [len(r) for r in runs]
  durs = [(r[-1][0] - r[0][0]) * 1000 for r in runs]
  print(f"\nheld for {min(lens)}-{max(lens)} frames ({min(durs):.0f}-{max(durs):.0f}ms)")
  if gaps:
    print(f"gap between presses: {min(gaps) * 1000:.0f}-{max(gaps) * 1000:.0f}ms")

  # what the counter does when nothing is pressed, for comparison
  idle_ctrs = [dat[2] for _, dat in frames[:200] if not dat[bi]]
  idle_steps = {(b - a) % 256 for a, b in zip(idle_ctrs, idle_ctrs[1:])}
  print(f"idle counter steps: {sorted(idle_steps)}")


def tx():
  asked = Counter()
  went = Counter()
  for _, src, addr, dat in read():
    if src >= 200:
      asked[addr] += 1
    elif src >= 128:
      went[addr] += 1
  print("address   openpilot asked   panda transmitted")
  for addr in sorted(set(asked) | set(went)):
    print(f"0x{addr:03x}       {asked.get(addr, 0):8d}          {went.get(addr, 0):8d}")
  if not went:
    print("\nno transmit echoes recorded at all -- panda may echo differently on this build")


if __name__ == "__main__":
  args = sys.argv[1:]
  if args and args[0].endswith((".gz", ".bin")):
    IN = args.pop(0)
  cmd = args[0] if args else "summary"
  sys.argv = ["replay"] + args
  if cmd == "summary":
    summary()
  elif cmd == "msg":
    msg(args[1])
  elif cmd == "diff":
    diff(args[1], args[2])
  elif cmd == "tx":
    tx()
  elif cmd == "press":
    press(*args[1:])
  else:
    print(__doc__)
