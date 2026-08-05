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

## Resolved: engagement is car state, not message content

Captured the camera's own `LFA_ALT` while it natively steered the car
(openpilot offroad, relay closed, so the camera reaches the MDPS):

```
ADAS_ActvACISta=0  ADAS_ActvACILvl2Sta=2  ADAS_StrAnglReqVal=-6.5
ADAS_ACIAnglTqRedcGainVal=0.1  FCA_ESA_ActvSta=0
```

`ADAS_ActvACISta` is **0 (INIT) even while actively steering** -- identical to
what openpilot hardcodes. Every other field matches what openpilot already
sends. There is no arming field in `LFA_ALT` we were failing to set.

So the ADAS is armed by ACC engagement through some path with no CAN
signature we could find, and button-engage is not reachable. Hypothesis ruled
out by measurement rather than abandoned.

Two mechanics worth remembering:

- `check_relay` messages are **statically blocked from forwarding**, always --
  not only while engaged. With openpilot running, the camera's `0xCB` never
  reaches the MDPS, so the car's native LFA cannot work at all. Offroad mode
  drops that blocking, which is how the native capture was possible.
- In offroad the panda reports only **bus 0 and 1**; bus 2 is absent. Read the
  camera's messages off bus 0 there (relay closed, so they forward).

**Landed:** LFA button is disable-only on CCNC angle-steering cars. Engage via
main cruise (arms the ADAS), disengage with the button (commands no steering,
so no fault). Gated to `HYUNDAI_PALISADE_HEV_LX3` alone.

## Known limitations (car does not broadcast)

Door/seatbelt state, hands-on-wheel detection, blinker detection. Not bugs.
