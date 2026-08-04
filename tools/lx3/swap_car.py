#!/usr/bin/env python3
"""Clear per-vehicle state before moving the comma device between cars.

This branch runs both the 2026 Kia Sorento Hybrid (HDA2) and the 2026 Hyundai
Palisade Hybrid (HDA1/LX3) from one build. HDA1 vs HDA2 is re-detected from live
CAN on every boot, so architecture detection is self-correcting -- but cached and
learned state is not, and it is per-vehicle.

Two classes of state:

  IDENTITY  -- caches which car this is. If not cleared, the device can resolve
               the wrong CarParams for the car it is actually plugged into.
               This is a correctness problem. Always cleared.

  LEARNED   -- calibration and vehicle dynamics learned by paramsd/torqued.
               If not cleared, one car drives on the other's learned values.
               This is a ride-quality problem, not a misidentification one.
               Cleared by default; keep with --keep-learned.

Usage on the device:
    python3 /data/openpilot/tools/lx3/swap_car.py
    sudo reboot
"""
import argparse
import sys

IDENTITY_PARAMS = [
  "CarParamsCache",
  "CarParamsPersistent",
  "CarParamsPrevRoute",
  "FirmwareQueryDone",
]

LEARNED_PARAMS = [
  "CalibrationParams",
  "LiveParameters",
  "LiveTorqueParameters",
  "LiveDelay",
  "CarBatteryCapacity",
]


def current_car(params) -> str:
  """Best-effort read of the car the device currently believes it is in."""
  try:
    from openpilot.cereal import messaging
    from opendbc.car.structs import car
    raw = params.get("CarParamsPersistent")
    if raw is None:
      return "unknown (no cached CarParams)"
    CP = messaging.log_from_bytes(raw, car.CarParams)
    return str(CP.carFingerprint) or "unknown (empty fingerprint)"
  except Exception as e:
    return f"unknown ({type(e).__name__})"


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__,
                               formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--keep-learned", action="store_true",
                  help="clear identity params only; keep calibration and learned dynamics")
  ap.add_argument("--dry-run", action="store_true",
                  help="show what would be cleared without changing anything")
  args = ap.parse_args()

  try:
    from openpilot.common.params import Params
  except ImportError:
    print("error: run this on the comma device, from the openpilot directory", file=sys.stderr)
    return 1

  params = Params()

  print(f"device currently configured for: {current_car(params)}\n")

  targets = IDENTITY_PARAMS + ([] if args.keep_learned else LEARNED_PARAMS)
  cleared = 0

  for key in targets:
    kind = "identity" if key in IDENTITY_PARAMS else "learned"
    present = params.check_key(key) and params.get(key) is not None
    if args.dry_run:
      print(f"  would clear  {key:24} ({kind}, {'set' if present else 'already empty'})")
      continue
    params.remove(key)
    cleared += 1
    print(f"  cleared      {key:24} ({kind})")

  if args.dry_run:
    print("\ndry run -- nothing changed")
    return 0

  if args.keep_learned:
    print(f"\n{cleared} params cleared. Calibration and learned dynamics KEPT "
          "(--keep-learned); the car will reuse the previous vehicle's values.")
  else:
    print(f"\n{cleared} params cleared. The car will re-fingerprint and re-learn "
          "calibration on the next drive (5-10 min highway above 25 mph).")

  print("Reboot the device before driving.")
  return 0


if __name__ == "__main__":
  sys.exit(main())
