# Unified sunnypilot branch for 2026 Sorento Hybrid (HDA2) + 2026 Palisade Hybrid (HDA1/LX3)

**Date:** 2026-08-04
**Author:** Nicholas Evans (`raider-red-10`)
**Status:** Design — awaiting review

---

## Goal

One sunnypilot branch that runs **both** vehicles from a **single comma device** moved between
them, with lateral (steering) control working on each.

Success means: flash once, move the device between cars, and each car is correctly detected and
steers — with no reflash and no regression on the Sorento, which works today.

---

## Background: what the investigation found

The initial framing was "merge the HDA1 and HDA2 sunnypilot branches." That framing is wrong, and
the reasons matter because they determine the whole shape of the work.

### 1. There is no HDA1-vs-HDA2 branch split to reconcile

`hkg-angle-steering-2025` (the so-called "HDA2 branch") **already supports non-HDA2 cars**. It
ships six platforms explicitly documented "without HDA II":

- Hyundai Ioniq 5 (without HDA II) 2022-24
- Hyundai Ioniq 6 (without HDA II) 2023-24
- Kia Niro EV (without HDA II) 2023-25
- Kia EV6 (without HDA II) 2022-24
- Genesis GV70 2.5T / 3.5T (without HDA II) 2022-24

### 2. HDA1 vs HDA2 is detected at runtime, not declared per-branch

`CANFD_HDA2` no longer exists as a flag. The modern codebase models the actual hardware
difference — whether the car has an ADAS DRV ECU between camera and MDPS — via
`CANFD_LKA_STEER_MSG`, detected from live CAN traffic:

```python
# opendbc/car/hyundai/interface.py
lka_steering = 0x50 in fingerprint[cam_can] or 0x110 in fingerprint[cam_can]
```

That single detection drives five downstream differences, all already implemented:

| Concern | HDA2 (`lka_steering=True`) | HDA1 (`lka_steering=False`) |
|---|---|---|
| Bus mapping | `ACAN=0, ECAN=1` | `ACAN=1, ECAN=0` |
| Steering msg | `LKAS`/`LKAS_ALT` → ADAS ECU → LFA → MDPS | LFA direct to MDPS |
| LFA suppression | Required (`CAM_0x2a4` / `CAM_0x362`) | Not needed |
| Auto-set flags | — | `CANFD_ALT_BUTTONS`, `CANFD_CAMERA_SCC` |
| Panda TX allowlist | `HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS` | LFA-steering TX set |

**Implication:** HDA1 support does not need to be written. It exists, it is mature, and other HDA1
cars run on it in production. The Palisade needs to be *recognized*, after which it flows down the
existing HDA1 path.

### 3. The `-hda1` branch is misnamed and is not about HDA1

`hkg-angle-steering-2025-hda1` is 4 commits of submodule bumps, last touched 2026-04-19. Its only
unique content is an `opendbc` pointer. That opendbc carries royjr's **CCNC** work (Connected Car
Navigation Cockpit — the new digital cluster architecture), which is *orthogonal* to HDA1/HDA2:
four CCNC-flagged platforms are documented **"with HDA II"** (`HYUNDAI_IONIQ_5_N`,
`HYUNDAI_KONA_EV_2ND_GEN`, `HYUNDAI_SANTA_FE_HEV_5TH_GEN`, `KIA_K4_2025`).

It went unmerged because the change rewrites shared safety-critical infrastructure far beyond
Hyundai — `lateral.h`, `mads.h`, `mads_declarations.h`, `safety.c`, 339 lines of `mads_common.py`,
plus `honda.h` and `tesla.h`. Large blast radius in safety code, author inactive since April.
**This branch is not a dependency for our work.**

### 4. The Palisade exists only in a third-party fork

No CAN-FD Palisade platform has ever existed in sunnypilot's opendbc. The port lives at
`kamdeva/freepalestine` (formerly `kamdeva/openpilot`), branch `lx3-hda1`, with opendbc at
`kamdeva/opendbc` branch `hkg-angle-steering-2025-hda1-lx3`.

### 5. The Sorento port is our own upstream work

`sunnypilot/opendbc` commit `115b68b5` — "Add 2026 Kia Sorento Hybrid SX Prestige HDA2 angle
steering support", authored by Nicholas Evans — is already upstream. `raider-red-10/opendbc` holds
the origin of that commit (`0814b0c8`) and nothing else unique. The fork is safe to keep and reuse;
it is not load-bearing for any code.

---

## Target vehicles

| | Kia Sorento Hybrid SX Prestige 2026 | Hyundai Palisade Hybrid 2026 |
|---|---|---|
| Platform | `KIA_SORENTO_HEV_4TH_GEN_LFA2` | `HYUNDAI_PALISADE_HEV_LX3` |
| Architecture | HDA2 (`lka_steering=True`) | HDA1 (`lka_steering=False`) |
| Flags | `CANFD_ANGLE_STEERING` | `CANFD_ANGLE_STEERING \| CCNC` |
| Harness | `hyundai_q` | `hyundai_n` (see note) |
| Specs | mass 1970, wb 2.814, sr 13.27 | mass 2175, wb 2.97, sr 13.72 |
| Status | Working, upstream | Ported in third-party fork only |
| Lateral | Working | Working (per fork author, on-vehicle) |
| Longitudinal | Unavailable | Unavailable |

Both cars are lateral-only, but **for different reasons** — this distinction matters when reasoning
about future work:

- **Sorento:** deliberately disabled in code. `interface.py` sets
  `alphaLongitudinalAvailable = False` for every `lka_steering and CANFD_ANGLE_STEERING` car.
  Not a defect.
- **Palisade:** unimplemented. `SCC_CONTROL` bytes 24-25 do not match the camera's format and have
  not been reverse-engineered. Enabling Alpha Longitudinal reportedly **silences the cruise
  buttons**.

> **Harness note.** kamdeva's port declares `CarHarness.hyundai_l`. The actual harness on our
> vehicle is **`hyundai_n`**, confirmed by the owner. We use `hyundai_n`. This is documentation
> metadata (`car_parts`) and does not affect CAN routing at runtime, so the discrepancy does not
> invalidate the fork's on-vehicle results — but our platform entry must reflect the hardware we
> actually run, and `hyundai_n` is consistent with other newer non-HDA2 Hyundais
> (`HYUNDAI_TUCSON_2025`, `HYUNDAI_TUCSON_HEV_2025`).

> **Fingerprint note.** The owner has previously run kamdeva's branch on this vehicle and it
> fingerprinted correctly. We adopt the fork's FW fingerprint block as-is rather than recapturing.

---

## Architecture

**Base:** current tip of `sunnypilot/sunnypilot` `hkg-angle-steering-2025` — the branch the Sorento
already runs. Working branch: `lx3-unified`.

The design principle throughout: **the Sorento's resolved behavior must be provably unchanged.**
Every LX3-specific change is gated so it cannot execute on an `lka_steering` car.

### Changes in `opendbc`

**A. LX3 platform + fingerprint.** Add `HYUNDAI_PALISADE_HEV_LX3` to `values.py` with the specs and
harness above, plus its FW fingerprint block in `fingerprints.py`, `car_list.json` entry, and
`torque_data` entries.

**B. Minimal CCNC subset.** The LX3 platform requires `HyundaiFlags.CCNC` because openpilot must
transmit the cluster messages that the ADAS ECU sends on HDA2 cars. Port only:

| Location | Change |
|---|---|
| `values.py` | `HyundaiFlags.CCNC = 2**28`, `HyundaiSafetyFlags.CCNC = 2048` |
| `carstate.py` | copy `CCNC_0x161`, `CCNC_0x162`, `FR_CMR_03_50ms` from cam bus |
| `hyundaicanfd.py` | build `CCNC_0x161` / `CCNC_0x162` on ECAN |
| `carcontroller.py` | `ccnc_non_hda2 = CCNC and not lka_steering` gate |
| `interface.py` | set safety param only when `CCNC and not CANFD_LKA_STEER_MSG` |
| `safety/modes/hyundai_canfd.h` | `HYUNDAI_PARAM_CCNC`, `0x161`/`0x162` in TX allowlist |
| `dbc/generator/hyundai/hyundai_canfd.dbc` | CCNC message definitions |

This subset is **already self-gating on `not lka_steering`**, so it is inert on the Sorento by
construction, not merely by our own added conditionals.

**Explicitly excluded:** the MADS/`lateral.h`/`honda.h`/`tesla.h`/`mads_common.py` rewrite from
royjr's `dba9b53d`. It is not required for the Palisade and carries the blast radius that stalled
the original branch.

**C. LX3 hardware quirks.** The fork's README describes "5 patches"; the actual opendbc side is
**8 LX3-specific commits**, including `revert HYBRID flag (bisect)` and `revert to drive 9
baseline`. That port went through real on-vehicle debugging, so part of the work is deciding which
commits represent final intent versus abandoned experiments — take the resolved end state, not the
commit sequence. Known deviations from the generic HDA1 path:

1. **`0x105` counter.** LX3 increments the `ACCELERATOR_ALT` counter by 2 per frame; panda's
   standard check rejects it. Fork sets `max_counter=0, ignore_counter=true`.
   **This is a relaxation of a safety check and MUST be gated to the LX3 platform.** Ungated, it
   weakens the check on the Sorento. This is the single highest-risk item in the project.
2. **Absent messages.** LX3 lacks `DOORS_SEATBELTS`, `BLINKERS`, `ADAS_CMD`, `HOD_FD`. Carstate
   needs pre-registered parser messages and guarded access. Write null-safe rather than
   LX3-conditional where that does not change Sorento parsing.

### Changes in the sunnypilot repo

**D. `msgq` `NUM_READERS` 15 → 64.** Generic fix, not car-specific: sunnypilot has 36+ subscribers
on high-frequency services (`carState` at 95 Hz), and the upstream cap of 15 caused subscribers past
the cap to read stale data, failing `sm.all_checks()` across daemons. Benefits both cars.
**Verify it is still needed on the current tip** — 743 commits have landed since the fork point.

**E. ICBM lateral-only behavior (3 files).** Undocumented in the fork's README but present in its
diff. All three teach Intelligent Cruise Button Management to operate when only lateral is engaged:

| File | Change |
|---|---|
| `selfdrive/controls/controlsd.py` | `cruiseControl.cancel` also requires `CP_SP.pcmCruiseSpeed` |
| `sunnypilot/.../longitudinal_planner.py` | `long_enabled` true on `latActive and cruiseState.enabled` |
| `sunnypilot/.../icbm/controller.py` | ICBM `ready` becomes `CC.enabled or CC.latActive` |

**Decision: adopt all three ungated.** They are unreachable on the Sorento while the
**Intelligent Cruise Button Management** toggle is off, which is its current state.

`CP_SP.pcmCruiseSpeed` is set `False` in exactly one place, and only on explicit opt-in:

```python
# openpilot/sunnypilot/selfdrive/car/interfaces.py
icbm_enabled = params.get_bool("IntelligentCruiseButtonManagement")
if icbm_enabled and CP_SP.intelligentCruiseButtonManagementAvailable and not CP.openpilotLongitudinalControl:
    CP_SP.pcmCruiseSpeed = False
```

With the toggle off, `pcmCruiseSpeed` remains `True` and each change collapses to nothing:

| Change | Why inert |
|---|---|
| `icbm/controller.py` | `update_readiness()` is only called from `run()`, which early-returns on `if self.CP_SP.pcmCruiseSpeed: return`. **Unreachable code.** |
| `controlsd.py` | The added conjunct is `True`, so the expression is unchanged. **Literal no-op.** |
| `longitudinal_planner.py` | `long_enabled` feeds `scc.update()` and `sla.update()`. In this exact configuration (`not openpilotLongitudinalControl and pcmCruiseSpeed`), `_cleanup_unsupported_params` force-removes `SmartCruiseControlVision`, `SmartCruiseControlMap`, and `DynamicExperimentalControl`. Nothing actuates the longitudinal plan. |

Gating them to LX3 would add conditionals guarding code that cannot execute, and would leave the
Sorento running the *broken* variant if ICBM were ever enabled on it later — since both cars are
lateral-only, these changes are the correct behavior for both. Keep them in an isolated commit so
they remain revertible independently of the LX3 port.

**Precondition:** confirm ICBM is off on the Sorento. If it is on, this analysis does not hold and
the changes must be evaluated on-vehicle before adoption.

### What ICBM does, and which car can actually use it

ICBM is sunnypilot's substitute for longitudinal control: it taps the stock cruise SET+/SET−
buttons to modulate the car's own ACC set speed. The planner takes the slowest of several
candidate targets (`cruise`, `sccVision`, `sccMap`, `speedLimitAssist`), so with ICBM on,
openpilot slows for upcoming curves and posted speed limits without ever commanding acceleration.

The three changes above are what make this work at all on a lateral-only car — without them
`CC.enabled` is `False` and ICBM never activates.

Availability is per-car:

```python
# opendbc/car/hyundai/interface.py
ret.intelligentCruiseButtonManagementAvailable = not (stock_cp.flags & HyundaiFlags.CANFD_ALT_BUTTONS)
```

| Car | ICBM available? | Why |
|---|---|---|
| Sorento (HDA2) | **Yes — confirmed** | `lka_steering=True`, so the `else:` branch that sets `CANFD_ALT_BUTTONS` is never reached, and the platform does not declare it |
| Palisade (HDA1) | **Undetermined** | Set at runtime if `0x1cf` is absent from ECAN. kamdeva added `CANFD_ALT_BUTTONS` to the LX3 platform (`f75dc291`) then reverted it by their tip (`d4f06c60`) |

**Consequence for validation:** the intuitive plan — try ICBM on the Palisade first to keep the
Sorento untouched — may not be available. If ICBM is unavailable on the Palisade, the Sorento is
the only car that can exercise the feature. Determine Palisade availability on-vehicle before
planning any ICBM testing.

**F. Drop the locationd/calibrationd workarounds.** `calibrationd.py` (`valid=True`) and
`locationd.py` (`all_alive()` vs `all_checks()`) are described by the fork author as superseded by
the `NUM_READERS` fix and "kept for redundancy". Drop them; re-add only if a failure is observed.

---

## Risks

| Risk | Severity | Mitigation |
|---|---|---|
| `0x105` counter relaxation reaches the Sorento | **High** — weakens a panda safety check on a working car | Gate to LX3 platform; assert in tests that the Sorento's safety param is unchanged |
| ICBM changes alter Sorento cruise behavior | **Low** — traced unreachable while the ICBM toggle is off | Confirm toggle is off; isolate in its own commit; re-evaluate on-vehicle if ever enabled |
| Learned params carry between cars on one device | Medium — one car running the other's `steerRatio`/calibration | Documented swap procedure (below) |
| 743-commit forward port | Medium — the port is small but the base moved a lot | Re-apply patches onto current tip by hand; do not rebase the fork branch wholesale |
| Repo restructured since fork point | Medium — every fork patch path is stale | The tree moved under `openpilot/` (e.g. `selfdrive/controls/controlsd.py` → `openpilot/selfdrive/controls/controlsd.py`). A wholesale rebase would conflict on every file; hand re-application is required, not optional |
| Wrong harness | Medium | Confirm `hyundai_l` before driving |
| No NNLC model for LX3 | Low — steering falls back to MOCK, less refined | Accepted; improves as `paramsd` learns |
| Fork dependency (`kamdeva`) | Low | We re-port the changes into our own fork; no runtime dependency on theirs |

---

## Single-device swap procedure

The device is moved between cars, so per-car state must not leak. `lka_steering` re-detects from
live CAN on every boot, so architecture detection is self-correcting — but learned and cached state
is not.

Params confirmed present in `openpilot/common/params_keys.h` that hold per-vehicle state:

| Param | Holds |
|---|---|
| `CarParamsCache` / `CarParamsPersistent` / `CarParamsPrevRoute` | cached resolved car + flags |
| `FirmwareQueryDone` | skips re-fingerprinting |
| `CalibrationParams` | per-vehicle camera mounting angles |
| `LiveParameters` | `paramsd`-learned `steerRatio`, stiffness, angle offset |
| `LiveTorqueParameters` | learned torque response |
| `LiveDelay` | learned actuator delay |
| `CarBatteryCapacity` | per-vehicle |

`FirmwareQueryDone` and the `CarParams*` trio are the ones that would actively mis-detect a swapped
car; the `Live*` and `Calibration` params degrade steering quality rather than identity.

Implementation delivers a small helper script that clears these keys, keyed on detected platform
change. A per-car params namespace (retaining each car's learned values across swaps) is a
desirable v2 and out of scope for v1.

---

## Validation plan

Staged so that **the Sorento is never the test bed**.

1. **Static equivalence.** Prove the Sorento's resolved `CarParams` — flags, `safetyParam`, TX
   allowlist — are byte-identical before and after the change. This is the primary regression gate.
2. **Test suites.** `opendbc` car tests and safety tests, including `test_hyundai_canfd.py`.
   Add LX3 cases mirroring the fork's additions.
3. **Sorento on-vehicle.** Flash `lx3-unified`, confirm no behavior change vs. today. If the ICBM
   changes are adopted, this is where their effect is evaluated.
4. **Palisade on-vehicle.** Confirm fingerprint resolves to `HYUNDAI_PALISADE_HEV_LX3`, `canValid`,
   calibration completes, lateral engages.
5. **Swap loop.** Move the device both directions, confirm correct re-detection and no param bleed.

**Rollback:** tag the current known-good Sorento commit before any change. Alpha Longitudinal stays
**OFF** on both cars throughout.

---

## Out of scope

- **Palisade longitudinal control.** Requires reverse-engineering `SCC_CONTROL` bytes 24-25.
  Separate project, after this lands. Use stock ACC.
- **Sorento longitudinal.** Deliberately disabled upstream for `lka_steering` angle-steering cars.
- **Merging royjr's CCNC branch wholesale.** We take the minimal subset only.
- **Upstreaming the LX3 port.** Possible follow-up once validated; not a goal for v1.

---

## Prerequisites

- **`git-lfs` is not installed** on this machine. The repo uses LFS and a real build requires it
  (`brew install git-lfs`). The current working tree was checked out with the LFS filter bypassed,
  so LFS files are pointers.
- Fork `sunnypilot/sunnypilot` to `raider-red-10`. `raider-red-10/opendbc` already exists and will
  be reused.
- `gh auth login` for `raider-red-10` (interactive; the `ceoairfarecoach` account is currently
  active). Repo-local git identity is already set to `Nicholas Evans
  <265895801+raider-red-10@users.noreply.github.com>`; global identity untouched.

---

## Resolved decisions

1. **Harness** — `hyundai_n`, confirmed against the physical vehicle. Overrides the fork's
   `hyundai_l`. See harness note above.
2. **Fingerprint** — adopt the fork's FW fingerprint block. The vehicle has previously fingerprinted
   correctly on that branch.
3. **ICBM** — adopt all three changes ungated, in an isolated commit. Traced unreachable while the
   ICBM toggle is off. See section E.

## Open questions

1. **Is ICBM currently off on the Sorento?** The entire section-E analysis depends on it. If it is
   on, those three changes must be evaluated on-vehicle before adoption.
2. **Does ICBM get enabled after this lands?** Out of scope for v1, but it is the main functional
   upside available to both cars — curve and speed-limit slowdown via cruise-button modulation,
   without openpilot longitudinal. If pursued, test on the Palisade first, not the Sorento.

---

## Definition of done

- `lx3-unified` builds and passes `opendbc` car + safety tests.
- Sorento resolved `CarParams` and `safetyParam` proven identical to the pre-change baseline.
- Sorento steers as before on-vehicle.
- Palisade fingerprints as `HYUNDAI_PALISADE_HEV_LX3` and steers on-vehicle.
- Device swaps both directions without manual reflash and without param bleed.
- Swap procedure documented in the repo.
