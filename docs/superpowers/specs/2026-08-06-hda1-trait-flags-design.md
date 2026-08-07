# HDA1 trait flags — design

Generalize the LX3-only gates so other HDA1 / CCNC Hyundais can be added without touching
shared code: everything bus-detectable configures itself from the fingerprint, and the one
content-defined trait (where the buttons live) becomes a declared platform flag with a paired
panda safety bit. Companion to `2026-08-06-lx3-handoff.md`, which documents the quirks
themselves.

## Goal

Adding the next HDA1 car should require only:

1. a platform entry with `CarSpecs` + FW fingerprints (or sunnypilot's manual car selection),
2. declaring `BTN_CLUSTER_0X10B` **iff** the `tools/lx3/` scanners confirm the buttons live in
   `LFA_BUTTON_ALT` byte 10 with a vestigial `0x1aa`.

Nothing else: blinkers, missing messages, HDA1-vs-HDA2, and the angle-steering path all
configure themselves.

## The trait model

Two kinds of quirk, two mechanisms:

**Bus-detectable → auto-detected at `get_params` from the fingerprint.** Presence/absence of
an address is visible during identification, exactly like the existing `lka_steering` and
`SEND_LFA` probes. New auto-detected traits (stored as `HyundaiFlagsSP` bits set by
`interface.py`, never declared per platform):

| trait | predicate | consumer |
|---|---|---|
| `BLINKERS_ALT` | `0x413` absent **and** `0x3e3` present on E-CAN | carstate blinker source + 100-frame hold |
| `ABSENT_DOORS_MSG` | `0x411` absent on E-CAN | carstate skips registering `DOORS_SEATBELTS` |
| `ABSENT_HOD_MSG` | `0x2af` absent on E-CAN | carstate skips registering `HOD_FD_01_100ms` |

A car that transmits these messages gets the standard paths automatically; a registered
message that never arrives would fail `can_valid`, which is why absence must gate parser
registration, not just the read.

**Content-defined → declared per platform.** `HyundaiFlagsSP.BTN_CLUSTER_0X10B`: the whole
steering-wheel cluster is in `LFA_BUTTON_ALT` (`0x10b`) byte 10 and `0x1aa`'s button field is
permanently zero. Not detectable from presence (`0x1aa` *exists*; its emptiness is content),
and it drives panda safety behavior, so it must be static and measured. Declaring it gates:

- carstate: button decode, the interaction-deque feed (`_update_lx3_buttons`), button events,
  and the slow-`0x1cf` parser skip (`0x1cf` is vestigial on cluster-button cars)
- ICBM: the one-frame `0x10b` press path
- panda, via a paired `HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B = 16` on `safetyParamSP`
  (follows the existing `ESCC`/`HAS_LDA_BUTTON`/`NON_SCC` pattern)

## Panda changes — fixing the CCNC asymmetry

Today the `0x10b` handling is gated on the CCNC flag because panda cannot see
`carFingerprint`. That over-reaches: every CCNC car on this branch reads buttons only from
`0x10b` and carries a 25 Hz `0x10b` RX check — on the twelve existing torque CCNC cars this
would break the `controls_allowed` latch outright (missing RX check → `is_msg_valid` clears
it continuously) if any of them lack the message. With the new safety bit:

- **RX checks:** the `0x10b` entry moves from the CCNC alt-buttons macro to a
  `BTN_CLUSTER_0X10B` variant; CCNC cars without the bit keep the standard alt-buttons checks.
- **RX hook:** the "skip `0x1cf`/`0x1aa` buttons, read `0x10b` buttons + MADS bit" behavior
  gates on the bit, not on CCNC. CCNC cars without the bit regain stock button handling.
- **TX:** the `0x10b` allowance in the tx hook additionally requires the bit (`0x10b` stays in
  the CCNC TX list; the hook is the gate, as today with `alt_buttons`).
- Unchanged: `0x161`/`0x162` cluster retransmission and the `0x105` counter-step-2
  accommodation stay CCNC-gated (cluster traits, as today), `VSetDis ≥ 255 → 0` stays global.

openpilot and panda then gate the same behavior on the same declared trait — the asymmetry
called out in the handoff is gone.

## Resulting LX3 declaration

`HYUNDAI_PALISADE_HEV_LX3`: `flags = CANFD_ANGLE_STEERING | CCNC`, SP flags =
`BTN_CLUSTER_0X10B`. All six `carFingerprint == HYUNDAI_PALISADE_HEV_LX3` checks (doors,
hands-on-detection, blinkers, button deque/events, the `0x1cf` parser skip, and the ICBM press
path) are deleted; behavior is identical by construction.

## Invariants and tests

- **Sorento regression** (`test_lx3_regression.py`) stays byte-identical — it declares nothing
  new. This is the hard gate on every step.
- **All existing LX3 tests pass unmodified** under the flags (buttons, press shape, cadence,
  interaction deque, blinkers) — they construct the real `CarParams`, so they exercise the new
  gating for free.
- **New: auto-detection tests** — `get_params` with fingerprints that include/omit `0x413`,
  `0x411`, `0x2af` set/clear the right `HyundaiFlagsSP` bits; a fingerprint with both `0x413`
  and `0x3e3` uses the standard blinker path.
- **New: safety matrix** — CCNC **with** the bit: `0x10b` buttons latch `controls_allowed`
  (including the cancel/set toggle), tx allowed with `controls_allowed`; CCNC **without** the
  bit: buttons read from `0x1aa`, `0x10b` tx rejected, no `0x10b` RX check to fault.
- **New: param resolution test** — the LX3 resolves with `BTN_CLUSTER_0X10B` set and the
  safety bit present in `safetyParamSP`.

## Deployment

The LX3's `safetyParamSP` value changes, so the device needs the usual synchronized
openpilot + opendbc update (submodule!) with a panda-firmware rebuild, then a short sanity
drive: engage, confirm a limit change, verify the set speed still walks to limit+offset, and
re-engage once via the cancel/set toggle.

## Out of scope (deliberate)

- A generic `HYUNDAI_CANFD_HDA1` catch-all platform for cars with no FW entry — worthwhile
  follow-up, needs conservative-physics validation of its own.
- Learning button locations from driver presses — rejected: couples learned data into what the
  panda permits.
- Auto-detecting counter step sizes or `0x1aa` emptiness from content — same reason.
