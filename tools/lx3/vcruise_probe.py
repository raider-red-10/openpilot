#!/usr/bin/env python3
"""Replay live carState through VCruiseHelper and print every branch decision.

The full chain works in simulation but not on the vehicle, so something in the real
CarState differs from what the bench feeds it. This runs the identical logic against real
data and reports which branch ran, whether the seed fired, and why not.

Read-only: builds its own VCruiseHelper, touches nothing openpilot is using.

  ./vcruise_probe.py        prints on every change, plus a 5s heartbeat
"""
import sys
import time

from openpilot.cereal import custom, messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import set_speed_management_engaged
from opendbc.car.structs import car as car_structs  # noqa: F401  (capnp registration)

OUT = "/data/vcruise_probe.txt"
HEARTBEAT = 5.0


def load_params():
  p = Params()
  cp_raw, cp_sp_raw = p.get("CarParamsPersistent"), p.get("CarParamsSPPersistent")
  if cp_raw is None or cp_sp_raw is None:
    sys.exit("no CarParams yet -- start the car first")
  from opendbc.car.structs import car as C
  CP = C.CarParams.from_bytes(cp_raw)
  CP_SP = messaging.log_from_bytes(cp_sp_raw, custom.CarParamsSP)
  return CP, CP_SP


def main():
  CP, CP_SP = load_params()
  print(f"pcmCruise={CP.pcmCruise}  opLong={CP.openpilotLongitudinalControl}  "
        f"pcmCruiseSpeed={CP_SP.pcmCruiseSpeed}", file=sys.stderr)
  if CP_SP.pcmCruiseSpeed:
    print("WARNING: pcmCruiseSpeed is True -- ICBM is not managing the set speed", file=sys.stderr)

  helper = VCruiseHelper(CP, CP_SP)
  sm = messaging.SubMaster(["carState", "carControl", "longitudinalPlanSP"])
  out = open(OUT, "w", buffering=1)

  last, last_emit, start = None, 0.0, time.monotonic()
  while True:
    sm.update(100)
    if not sm.updated["carState"]:
      continue

    CS = sm["carState"]
    enabled = set_speed_management_engaged(CP, CP_SP, sm["carControl"].enabled, CS.cruiseState.enabled)

    # Which branch update_v_cruise will take, evaluated the same way it does
    _enabled = helper.update_enabled_state(CS, enabled)
    non_pcm = (not CP.pcmCruise) or (not CP_SP.pcmCruiseSpeed and _enabled)
    seeded_before = helper.v_cruise_seeded_from_car

    helper.update_speed_limit_assist(False, sm["longitudinalPlanSP"])
    helper.update_v_cruise(CS, enabled, False)

    why = ""
    if not CS.cruiseState.available:
      why = "cruise unavailable"
    elif not non_pcm:
      why = "mirror branch (else)"
    elif seeded_before:
      why = "already seeded"
    elif CS.cruiseState.speed <= 0:
      why = "car reports no set speed"
    else:
      why = "SEEDED" if helper.v_cruise_seeded_from_car else "seed skipped?!"

    line = (f"carCruise={int(CS.cruiseState.enabled)} avail={int(CS.cruiseState.available)} "
            f"ccEn={int(sm['carControl'].enabled)} enabled={int(enabled)} _enabled={int(_enabled)} "
            f"branch={'nonPCM' if non_pcm else 'mirror'} "
            f"carSpeed={CS.cruiseState.speed * CV.MS_TO_MPH:5.1f} "
            f"opSet={helper.v_cruise_kph * CV.KPH_TO_MS * CV.MS_TO_MPH:6.1f} "
            f"{'UNSET' if helper.v_cruise_kph == V_CRUISE_UNSET else '     '} "
            f"seeded={int(helper.v_cruise_seeded_from_car)}  {why}")

    key = (int(CS.cruiseState.enabled), int(CS.cruiseState.available), int(enabled), int(_enabled),
           non_pcm, round(CS.cruiseState.speed, 1), round(helper.v_cruise_kph, 1),
           helper.v_cruise_seeded_from_car, why)
    now = time.monotonic()
    if key != last or (now - last_emit) > HEARTBEAT:
      print(f"{now - start:7.1f}s  {line}", file=out, flush=True)
      last, last_emit = key, now


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    print(f"\nstopped -- see {OUT}", file=sys.stderr)
