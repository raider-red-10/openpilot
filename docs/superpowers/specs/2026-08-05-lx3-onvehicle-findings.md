# LX3 on-vehicle findings

Measured on the owner's 2026 Palisade Hybrid. Everything here came from CAN
capture on the car, not from reading code.

## Fixed and verified

| Issue | Cause | Fix |
|---|---|---|
| "Unknown Vehicle Variant" | `canError` (mislabeled alert), not a fingerprint miss | below |
| `rxInvalid` | `0x105` counter steps by 2 (100Hz counter, 50Hz transmit) and is declared 100Hz | CCNC RX check: `50U`, `max_counter=0`, **and** `ignore_counter=true` |
| `canValid` false | `0x130 GEAR_SHIFTER` counter also steps by 2 | `ignore_counter` in the CAN parser, CCNC-gated |
| `canValid` false | `HOD_FD` (0x2af), `DOORS_SEATBELTS` (0x411), `BLINKERS` (0x413) absent; reading via `cp.vl[...]` **registers** them, and a registered message that never arrives invalidates the parser | LX3-gated guards |
| Cruise revving / "SCC conditions not met" | `cruiseControl.cancel` held true whenever lateral active + cruise on, because MADS lateral-only keeps `enabled` false. On `CANFD_ALT_BUTTONS` cars openpilot cancels via an SCC message, un-rate-limited | `mads_lateral_only` guard in `controlsd.py` |

`max_counter = 0` alone is **not** enough — it routes into the else branch of
`rx_msg_safety_check()` which pins `wrong_counters` to MAX unless
`ignore_counter` is also set.

## LFA button

Not in `CRUISE_BUTTONS_ALT` where openpilot expects it. Located by three-phase
differential scan (idle / hand resting on wheel / pressing), which subtracts
the steering-torque noise from reaching for the button:

- **`0x10b` byte 10 bit 7**, 16-byte message, not previously in the DBC
- Confirmed by uneven cadence (3 quick, 10s gap, 2 quick) — two other
  candidates hit 5-for-5 by coincidence and were eliminated this way
- SET-/RES+/gap produce zero activations of this bit
- Also found: **SET- is `0x208` byte 12**; `0x208` is a button message but
  does not carry LFA

Decoded in both layers (DBC + carstate, and panda `hyundai_canfd.h`). Button
events now appear as `ButtonType.lkas`.

## Open: engagement faults

Engaging lateral **from the button** makes the MDPS fault:
`MDPS_LkaFailSta` / `MDPS_ADAS_AciFltSig_Lv2` -> `steerFaultTemporary` ->
`latActive` drops -> re-engages -> flaps ~2Hz -> takeover alert.

Engaging **via main cruise** is stable (held 21s in one log).

Not a hard block — it is intermittent. Same conditions (`cruiseOn=True`)
worked at t=73.6 and t=83.6 but flapped at t=92.0. Lateral also stayed stable
for 2s after cruise disengaged at t=121.9.

**Why this car is unique:** it is the only angle-steering CCNC vehicle
anywhere. `sunnypilot/ccnc-port` has 12 non-HDA2 CCNC platforms but **zero**
`CANFD_ANGLE_STEERING` and **zero** `LFA_ALT` references — all torque steering.
A torque command is a nudge the MDPS blends; an angle command is an absolute
target it is stricter about honoring.

openpilot sends byte-identical `LFA_ALT` in both the working and failing case,
so message content is not the differentiator — the car's internal ADAS state
is. `ADAS_ActvACISta` is hardcoded to `INIT` in `create_steering_messages()`.

## Next step

Observe the car's **native** LFA rather than guessing the arming protocol:
turn MADS off in settings so openpilot ignores the button, drive, press LFA,
and log the camera's `LFA_ALT` and `CCNC_0x161` output on bus 2 — including a
case where the car refuses (no lane lines). Script at `/data/lx3_native.py`.

If the camera's `ADAS_ActvACISta` differs from openpilot's `INIT`, that is the
arming signal, measured rather than inferred.

## Known limitations (car does not broadcast)

Door/seatbelt state, hands-on-wheel detection, blinker detection. Not bugs.
