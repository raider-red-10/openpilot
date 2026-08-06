"""Temporary instrumentation for the Speed Limit Assist / ICBM chain.

The chain spans four processes -- card (set speed), plannerd (assist state machine), and
carcontroller (button send, in opendbc) -- and each step was verified in isolation while the
feature still failed on the vehicle. This logs what each step actually computes, in the real
running code, so no step has to be inferred.

Off unless /data/lx3_debug is present, so it costs nothing when not in use:

    touch /data/lx3_debug      # enable, then restart openpilot
    rm /data/lx3_debug         # disable

Writes to /data/lx3_debug.txt. Every call is wrapped -- instrumentation must never be able
to take down a driving process.
"""
import os
import time

_ENABLE_FLAG = "/data/lx3_debug"
_OUT_PATH = "/data/lx3_debug.txt"

_enabled = os.path.exists(_ENABLE_FLAG)
_fh = None
_last = {}
_checked = 0.0
_t0 = time.monotonic()


def _handle():
  """Reopen if the file was deleted underneath us.

  Deleting the log mid-run used to leave every process writing to a dead inode, so the file
  silently never came back and a drive's worth of data went nowhere.
  """
  global _fh, _checked
  now = time.monotonic()
  if _fh is not None and now - _checked > 2.0:
    _checked = now
    if not os.path.exists(_OUT_PATH):
      try:
        _fh.close()
      except Exception:
        pass
      _fh = None
  if _fh is None:
    # Each process opens append; lines are single writes so they interleave without tearing
    _fh = open(_OUT_PATH, "a", buffering=1)
    _checked = now
  return _fh


def dlog(tag: str, dedupe: bool = True, **fields) -> None:
  """Log one step's values. Repeats are suppressed unless the values change."""
  if not _enabled:
    return
  try:
    body = " ".join(f"{k}={v}" for k, v in fields.items())
    if dedupe:
      if _last.get(tag) == body:
        return
      _last[tag] = body
    _handle().write(f"{time.monotonic() - _t0:8.2f} {tag:<18} {body}\n")
  except Exception:
    pass


def enabled() -> bool:
  return _enabled
