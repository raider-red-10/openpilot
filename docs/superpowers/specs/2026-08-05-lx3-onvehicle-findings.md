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

## Resolved: ICBM and Speed Limit Assist "Assist" mode

**Symptom:** "Assist" not selectable in Speed Limit Settings; on the Palisade the
ICBM toggle was absent entirely.

**Cause, traced end to end:**

```
Assist selectable <- sla_available = (has_long or has_icbm)     # has_long False, stock SCC
has_icbm          <- intelligentCruiseButtonManagementAvailable and the ICBM param
available         <- not (flags & CANFD_ALT_BUTTONS)
```

The Palisade has no `0x1cf` on E-CAN, so it gets `CANFD_ALT_BUTTONS` and its
buttons live in `CRUISE_BUTTONS_ALT` (`0x1aa`). ICBM presses SET+/SET- for the
driver, and the CAN FD path for alt-button cars was an empty `pass` with a TODO
-- the availability flag was guarding an unimplemented feature, not an unsafe
one. The UI additionally force-downgrades the mode to Warning whenever it is
unavailable, which is why it would not stay on Assist.

**Landed:** `create_buttons_alt()` replays the car's last `0x1aa` frame and
overrides only the button field; the packer already computes the HKG CAN FD
checksum for every message in this DBC, so no reverse engineering was needed.
Buttons present in the captured frame are cleared, or a frame sampled while the
driver held main-cruise or LDA would replay that press 20 times. `0x1aa` is in
the TX list for the CCNC camera-SCC config only, the tx hook also requires the
alt-buttons flag, and the availability rule in `interface.py` is scoped to match
-- all three must stay in sync.

Enabling the ICBM param also sets `CP_SP.pcmCruiseSpeed = False`, which is what
actually unblocks Assist, Dynamic Experimental Control, Custom ACC Increments and
Smart Cruise Control Vision/Map.

**Not changed:** `HYST_GAP = 0.0` in `icbm/controller.py` makes `apply_hysteresis`
a pass-through, which in principle lets `v_target` oscillate across a rounding
boundary and spam buttons. Left alone deliberately: it is shared tuning, and the
Sorento has run ICBM at this value without trouble. Watch for set-speed hunting
on the first Palisade drive.

**Untested on vehicle.** This is the first code in this branch that transmits to
the car rather than reading from it.

## Resolved: turn signals, and with them lane changes

**Symptom:** no automatic lane change. `desire_helper` needs
`leftBlinker != rightBlinker`, and with `BLINKERS` (`0x413`) absent both stayed
False forever.

**Found:** `0x3e3` byte 11, bit 2 left and bit 4 right -- the same two-bits-apart
layout `BLINKERS` uses for `LEFT_LAMP`/`RIGHT_LAMP`. 16 bytes at 5Hz on E-CAN.
Confirmed live: each bit toggles at 1.32Hz for its own stalk and is static for
the other.

**What made the scan work.** The first pass tested "was this bit ever high" and
returned ~500 candidates -- every counter and checksum on the bus passes that.
The discriminator that works is that a blinker *flashes*: require the bit to
change during one stalk's phase and never change with both stalks off. That drops
it to a handful. Two mechanical points mattered as much as the idea:

- Accumulate frames as integer bitmasks. Per-bit Python cannot keep up with CAN
  FD; the queue saturates, `drain_sock` stops returning, and each phase silently
  records the previous phase's backlog.
- Hyundai CAN FD puts `CHECKSUM` in bytes 0-1 and `COUNTER` in byte 2. A dozen
  messages reporting "byte 2 bit 7" is the counter rolling, not a signal.

The cross-check came free: `find_candidates` only drops a bit that also changed
while both stalks were off, so a bit appearing under LEFT and absent under RIGHT
was already proven static for the other stalk.

**Landed:** `BLINKERS_ALT` in the DBC with only the two lamp bits declared, and a
100 frame hold instead of the usual 50 -- at 5Hz, up to 588ms (59 frames) can pass
between two frames that catch the lamp lit, and 50 would drop the blinker between
flashes.

Tool: `tools/lx3/blinker_scan.py`, which also has a `watch` mode for confirming a
single bit live.

## Known limitations (car does not broadcast)

Door/seatbelt state, hands-on-wheel detection. Not bugs.
