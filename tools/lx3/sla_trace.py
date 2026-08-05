#!/usr/bin/env python3
"""Trace Speed Limit Assist end to end while driving.

The assist is a chain, and a break anywhere gives the same symptom -- nothing happens:

  camera sign -> carStateSP.speedLimit
              -> resolver (car vs map, offset applied)
              -> assist state machine (inactive/preActive/adapting/active)
              -> ICBM (which button to press)
              -> cruise set speed

This samples all of it at 2Hz and logs a line whenever anything changes, plus a heartbeat
so a static log still proves it was running. Drive past a speed limit sign, then read the
file: the last stage that shows a sensible value is the last one that works.

  ./sla_trace.py            log to /data/sla_trace.txt
  ./sla_trace.py -          log to stdout
"""
import sys
import time

from openpilot.cereal import messaging
from openpilot.common.params import Params

OUT = "/data/sla_trace.txt"
HEARTBEAT = 10.0  # secs

ASSIST_STATE = ["disabled", "inactive", "preActive", "pending", "adapting", "active"]
SOURCE = ["none", "car", "map"]
ICBM_STATE = ["inactive", "preActive", "increasing", "decreasing", "holding"]
SEND_BUTTON = ["none", "increase", "decrease"]


def name(table, idx):
  try:
    return table[int(idx)]
  except (ValueError, TypeError, IndexError):
    return str(idx)


def main():
  to_stdout = len(sys.argv) > 1 and sys.argv[1] == "-"
  out = sys.stdout if to_stdout else open(OUT, "w", buffering=1)

  sm = messaging.SubMaster(["carState", "carStateSP", "longitudinalPlanSP",
                            "carControl", "carControlSP", "liveMapDataSP"])
  params = Params()

  print("tracing -- drive past a speed limit sign. ctrl-c to stop.", file=sys.stderr)
  start = time.monotonic()
  last_line, last_emit = None, 0.0

  while True:
    sm.update(100)
    if not sm.updated["longitudinalPlanSP"]:
      continue

    cs = sm["carState"]
    cc = sm["carControl"]
    sl = sm["longitudinalPlanSP"].speedLimit
    icbm = sm["carControlSP"].intelligentCruiseButtonManagement
    mapd = sm["liveMapDataSP"]

    # The assist leaves `disabled` only when long_enabled and enabled are both true.
    # long_enabled is carControl.enabled -- openpilot's own engaged state, NOT the car's
    # cruise -- and enabled is the SpeedLimitMode param. Log both or the state is unreadable.
    mode = params.get("SpeedLimitMode", return_default=True)

    fields = [
      f"v={cs.vEgo * 2.237:5.1f}mph",
      f"set={cs.cruiseState.speedCluster * 2.237:5.1f}",
      f"carCruise={int(cs.cruiseState.enabled)}",
      f"ccEnabled={int(cc.enabled)}",
      f"lat={int(cc.latActive)}",
      f"mode={mode}",
      # stage 1: does the camera see a sign?
      f"car={sm['carStateSP'].speedLimit * 2.237:5.1f}",
      # stage 1b: does map data have anything?
      f"map={mapd.speedLimit * 2.237:5.1f}/{int(mapd.speedLimitValid)}",
      # stage 2: what did the resolver settle on?
      f"resolved={sl.resolver.speedLimit * 2.237:5.1f}",
      f"final={sl.resolver.speedLimitFinal * 2.237:5.1f}",
      f"valid={int(sl.resolver.speedLimitValid)}",
      f"src={name(SOURCE, sl.resolver.source)}",
      # stage 3: the assist state machine
      f"assist={name(ASSIST_STATE, sl.assist.state)}",
      f"en={int(sl.assist.enabled)}/act={int(sl.assist.active)}",
      f"vTarget={sl.assist.vTarget * 2.237:5.1f}",
      # stage 4: is ICBM asking for a button?
      f"icbm={name(ICBM_STATE, icbm.state)}",
      f"btn={name(SEND_BUTTON, icbm.sendButton)}",
    ]
    line = "  ".join(fields)

    # Change detection ignores speed and set point. vEgo dithers around 0.0/-0.0 when
    # parked and drifts constantly when moving, so including them means every sample
    # counts as a change and the interesting transitions scroll away.
    key = (round(sm["carStateSP"].speedLimit, 1), round(mapd.speedLimit, 1), int(mapd.speedLimitValid),
           round(sl.resolver.speedLimit, 1), round(sl.resolver.speedLimitFinal, 1),
           int(sl.resolver.speedLimitValid), str(sl.resolver.source),
           str(sl.assist.state), int(sl.assist.enabled), int(sl.assist.active),
           str(icbm.state), str(icbm.sendButton), int(cs.cruiseState.enabled),
           int(cc.enabled), int(cc.latActive), int(mode))

    now = time.monotonic()
    if key != last_line or (now - last_emit) > HEARTBEAT:
      print(f"{now - start:7.1f}s  {line}", file=out, flush=True)
      last_line, last_emit = key, now


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    print(f"\nstopped -- see {OUT}", file=sys.stderr)
