# Unified LX3 + Sorento Branch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `HYUNDAI_PALISADE_HEV_LX3` support to the current `hkg-angle-steering-2025` branch so one build runs both the 2026 Palisade Hybrid (HDA1) and the 2026 Sorento Hybrid (HDA2) from a single comma device, with the Sorento's resolved behavior provably unchanged.

**Architecture:** The HDA1/HDA2 *dispatch* is already automatic via runtime `lka_steering` detection, so no new detection logic is needed. But the base branch has **no HDA1 + angle-steering support at all** — all 10 of its angle-steering platforms are HDA2-only, its 6 non-HDA2 platforms are torque-steering, and `LFA_ALT` (0xCB) appears zero times. The Palisade is the first such car here. So the core of this work is porting the CCNC/`LFA_ALT` infrastructure (~400 lines, 9 files) that supplies the HDA1 angle-steering control path and the digital-cluster messages; the LX3 platform definition on top of it is small by comparison. Every ported change is either self-gating on `not lka_steering` or explicitly gated to the LX3 platform, so the Sorento's code path is untouched.

**Tech Stack:** Python 3, openpilot/sunnypilot, opendbc (car ports + panda safety in C), SCons, unittest, git submodules, git-lfs.

## Global Constraints

- **Base branch:** `sunnypilot/sunnypilot` `hkg-angle-steering-2025` at current tip. Local working branch `lx3-unified`.
- **Sorento is never the test bed.** Its resolved `CarParams.flags` and `safetyConfigs[-1].safetyParam` must be byte-identical before and after. This is a hard gate on every opendbc task.
- **ICBM changes are excluded from v1.** The three files in `selfdrive/controls/controlsd.py`, `sunnypilot/selfdrive/controls/lib/longitudinal_planner.py`, and `sunnypilot/selfdrive/car/intelligent_cruise_button_management/controller.py` are NOT to be merged. ICBM is believed ON on the Sorento, which makes them live on the working car.
- **Alpha Longitudinal stays OFF** on both vehicles throughout.
- **Port the resolved end state, not the commit sequence.** kamdeva's branch contains bisect reverts; take the tip state.
- **Harness for LX3 is `CarHarness.hyundai_n`**, overriding kamdeva's `hyundai_l`.
- **Commit identity** is already set repo-locally to `Nicholas Evans <265895801+raider-red-10@users.noreply.github.com>`.
- **All pushes in the sunnypilot repo must use `git push --no-verify`.** `.lfsconfig` points LFS at sunnypilot's GitLab with `pushurl = ssh://git@gitlab.com/...`, which requires their SSH access. git-lfs's pre-push hook tries to upload all 263 LFS objects and fails with `Permission denied (publickey)`. We modify **zero** LFS files, and the `.lfsconfig` read URL is public, so clones of the fork resolve LFS from GitLab regardless — the objects never need to reach the fork. `--no-verify` skips only that hook. **If a future task ever modifies an LFS-tracked file, stop** — this workaround silently drops the object and the change would not reach anyone cloning the fork.
- **`gh` account:** `raider-red-10` must be active (`gh auth status`). Switch back with `gh auth switch --user ceoairfarecoach` before running `gh` in unrelated projects — the active account is global to the CLI, not per-directory.

### Source references

| Ref | What |
|---|---|
| `origin/hkg-angle-steering-2025` | base branch (sunnypilot repo) |
| `85f53e05` | opendbc at base branch tip |
| `1c2d90df` | opendbc with CCNC/`LFA_ALT` work (royjr) |
| `kamdeva/hkg-angle-steering-2025-hda1-lx3` | opendbc with LX3 on top of CCNC |
| `cf3223a5` | opendbc merge-base of `85f53e05` and `1c2d90df` |

---

## File Structure

**opendbc (the bulk of the work):**

| File | Responsibility | Change |
|---|---|---|
| `opendbc/car/hyundai/values.py` | flags, platforms, enums | Add `CCNC` flags, `ActvACISta`/`ESA_ActvSta` enums, `HYUNDAI_PALISADE_HEV_LX3` platform |
| `opendbc/dbc/generator/hyundai/hyundai_canfd.dbc` | CAN message definitions | Add `LFA_ALT`, `CCNC_0x161`, `CCNC_0x162` |
| `opendbc/car/hyundai/hyundaicanfd.py` | CAN message construction | `LFA_ALT` steering path, `create_ccnc()` |
| `opendbc/car/hyundai/carstate.py` | CAN parsing | Capture CCNC messages; LX3 `CRUISE_BUTTONS` freq-check skip |
| `opendbc/car/hyundai/carcontroller.py` | TX orchestration | `ccnc_non_hda2` gate |
| `opendbc/car/hyundai/interface.py` | param resolution | Set `HyundaiSafetyFlags.CCNC` when `CCNC and not CANFD_LKA_STEER_MSG` |
| `opendbc/safety/modes/hyundai_canfd.h` | panda safety | `HYUNDAI_PARAM_CCNC`, `LFA_ALT` (0xCB) angle checks, CCNC TX allowlist |
| `opendbc/car/hyundai/fingerprints.py` | FW fingerprints | LX3 block |
| `opendbc/car/torque_data/override.toml` | torque tuning | LX3 entry |
| `opendbc/sunnypilot/car/car_list.json` | docs/car list | LX3 entry |
| `opendbc/car/hyundai/tests/test_lx3_regression.py` | **new** | Sorento equivalence gate |

**sunnypilot repo:**

| File | Change |
|---|---|
| `msgq_repo` submodule | unchanged — Task 10 verified the bump is unnecessary |
| `opendbc_repo` submodule | Point at our fork's `lx3-unified` |
| `tools/lx3/swap_car.py` | **new** — clears per-vehicle params on device swap |

---

## Task 1: Environment, forks, and the Sorento baseline

**Files:**
- Modify: repo-local git config
- Create: `docs/superpowers/baselines/sorento-baseline.json`

**Interfaces:**
- Produces: `sorento-baseline.json` containing `{"flags": int, "safetyParam": int, "carFingerprint": str}` — the frozen pre-change resolved values that Task 9 asserts against.

- [ ] **Step 1: Install git-lfs and undo the filter workaround**

The spec's LFS workaround must be reverted before any build.

```bash
brew install git-lfs
cd ~/sunnypilot
git config --local --unset-all filter.lfs.clean
git config --local --unset-all filter.lfs.smudge
git config --local --unset-all filter.lfs.process
git config --local --unset filter.lfs.required
git lfs install --local
git lfs pull
```

Expected: `git lfs pull` completes; `docs/assets/icon-star-empty.svg` is a real SVG, not a pointer.

- [ ] **Step 2: Authenticate the personal GitHub account**

This is interactive and must be run by the user — do not attempt to script credentials.

```bash
gh auth login
gh auth switch --user raider-red-10
```

Expected: `gh auth status` shows `raider-red-10` as active.

- [ ] **Step 3: Fork the sunnypilot repo and add remotes**

`raider-red-10/opendbc` already exists — reuse it.

```bash
gh repo fork sunnypilot/sunnypilot --remote=false --clone=false
cd ~/sunnypilot
git remote add mine https://github.com/raider-red-10/sunnypilot.git
git remote -v
```

Expected: `mine`, `origin`, `kamdeva` all listed.

- [ ] **Step 4: Initialize the opendbc submodule on the base commit**

```bash
cd ~/sunnypilot
git submodule update --init opendbc_repo
git -C opendbc_repo rev-parse HEAD
```

Expected: `85f53e05cb228d304e333a79dd53e914c327cfb7`

> Do **not** pass `--reference` pointing at a blobless clone (`--filter=blob:none`) — it advertises objects it does not have and the clone fails with `did not receive expected object`. Do not use `--depth=1` either; the submodule pins a specific SHA that a shallow fetch may not reach.

- [ ] **Step 4b: Set up the Python environment**

System `python3` on this machine is 3.9.6 with no `numpy`. Use `uv`, which the repo is configured for:

```bash
cd ~/sunnypilot/opendbc_repo
uv sync --extra testing
uv run python -c "import numpy, opendbc; print('imports ok')"
```

Expected: `imports ok`. All subsequent Python in opendbc runs via `uv run`.

> `--extra testing` is required — a plain `uv sync` installs runtime deps only and omits `hypothesis`, which the existing test module imports at load time.

**opendbc has no pytest.** It uses plain `unittest` plus `unittest-parallel`, and every test is a `unittest.TestCase` method. Commands in this plan use:
>
> | Intent | Command |
> |---|---|
> | one test | `uv run python -m unittest opendbc.car.hyundai.tests.test_hyundai.TestHyundaiFingerprint.test_name -v` |
> | one module | `uv run python -m unittest opendbc.car.hyundai.tests.test_hyundai` |
> | a directory | `uv run python -m unittest discover -s opendbc/car/hyundai/tests -t .` |
>
> New tests must therefore be **methods on a `unittest.TestCase`**, not bare `test_*` functions, and parametrization uses `self.subTest(...)` rather than `pytest.mark.parametrize`.

- [ ] **Step 4c: Set the git identity inside the submodule**

`opendbc_repo` is a separate git repository and does **not** inherit the parent's local identity. Without this, commits there are authored with the global `ceo@airfarecoach.com`:

```bash
cd ~/sunnypilot/opendbc_repo
git config --local user.name "Nicholas Evans"
git config --local user.email "265895801+raider-red-10@users.noreply.github.com"
git config --local --get-regexp '^user\.'
```

Expected: both values echoed back. Verify after the first commit with `git log -1 --format='%an <%ae>'`.

- [ ] **Step 5: Capture the Sorento baseline**

```bash
mkdir -p ~/sunnypilot/docs/superpowers/baselines
cd ~/sunnypilot/opendbc_repo
python3 - <<'PY' > ~/sunnypilot/docs/superpowers/baselines/sorento-baseline.json
import json
from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.structs import CarParams

candidate = CAR.KIA_SORENTO_HEV_4TH_GEN_LFA2
car_fw = [CarParams.CarFw(ecu=ecu, fwVersion=vers[0], address=addr, subAddress=sub or 0)
          for (ecu, addr, sub), vers in FW_VERSIONS[candidate].items()]

# LKA steering is detected from 0x50 on the camera bus; emulate an HDA2 fingerprint
fp = gen_empty_fingerprint()
fp[2][0x50] = 16

CP = CarInterface.get_params(candidate, fp, car_fw, False, True, False)
print(json.dumps({
    "carFingerprint": str(CP.carFingerprint),
    "flags": int(CP.flags),
    "safetyParam": int(CP.safetyConfigs[-1].safetyParam),
    "alphaLongitudinalAvailable": bool(CP.alphaLongitudinalAvailable),
}, indent=2))
PY
cat ~/sunnypilot/docs/superpowers/baselines/sorento-baseline.json
```

Expected: valid JSON with non-zero `flags` and `safetyParam`, `carFingerprint` of `KIA_SORENTO_HEV_4TH_GEN_LFA2`, and `alphaLongitudinalAvailable: false`.

> If `get_params` raises on the emulated fingerprint, adjust `fp[2][0x50]` to `fp[2][0x110]` and re-run — the detection accepts either. Record which one worked in the commit message; Task 9 must use the identical fingerprint.

- [ ] **Step 6: Commit the baseline**

```bash
cd ~/sunnypilot
git add docs/superpowers/baselines/sorento-baseline.json
git commit -m "test: freeze Sorento resolved CarParams baseline

Pre-change reference for the regression gate. Any change to flags or
safetyParam for KIA_SORENTO_HEV_4TH_GEN_LFA2 is a defect."
```

---

## Task 2: CCNC flags and enums in `values.py`

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/values.py`
- Test: `opendbc_repo/opendbc/car/hyundai/tests/test_hyundai.py`

**Interfaces:**
- Produces: `HyundaiFlags.CCNC` (`2 ** 28`), `HyundaiSafetyFlags.CCNC` (`2048`), `ActvACISta` and `ESA_ActvSta` enums. Tasks 4, 6, 7, 8 all consume these.

- [ ] **Step 1: Create the opendbc working branch**

```bash
cd ~/sunnypilot/opendbc_repo
git remote add mine https://github.com/raider-red-10/opendbc.git 2>/dev/null || true
git remote add kamdeva https://github.com/kamdeva/opendbc.git 2>/dev/null || true
git fetch kamdeva hkg-angle-steering-2025-hda1-lx3
git fetch origin
git checkout -b lx3-unified 85f53e05
```

Expected: `On branch lx3-unified`, HEAD at `85f53e05`.

- [ ] **Step 2: Write the failing test**

Append to `opendbc/car/hyundai/tests/test_hyundai.py`:

```python
def test_ccnc_flags_defined():
  # app-level and safety-level CCNC flags must exist and not collide
  assert HyundaiFlags.CCNC.value == 2 ** 28
  assert HyundaiSafetyFlags.CCNC.value == 2048
  # no other HyundaiFlags member may share the CCNC bit
  others = [f for f in HyundaiFlags if f is not HyundaiFlags.CCNC]
  assert all(f.value != HyundaiFlags.CCNC.value for f in others)
```

- [ ] **Step 3: Run it to make sure it fails**

```bash
cd ~/sunnypilot/opendbc_repo
uv run python -m unittest opendbc.car.hyundai.tests.test_hyundai.TestHyundaiFingerprint.test_ccnc_flags_defined -v
```

Expected: FAIL with `AttributeError: CCNC`

- [ ] **Step 4: Add the flags and enums**

In `opendbc/car/hyundai/values.py`, change the import:

```python
from enum import IntFlag, Enum
```

Add to `HyundaiSafetyFlags` after `CANFD_ANGLE_STEERING = 1024`:

```python
  CCNC = 2048
```

Add to `HyundaiFlags` after `CANFD_ANGLE_STEERING = 2 ** 27`:

```python
  CCNC = 2 ** 28
```

Append the enums at the end of the file:

```python
class ActvACISta(Enum):
  INIT = 0
  INACTIVE = 1
  ACTIVE35_ACTIVE = 2


class ESA_ActvSta(Enum):
  INACTIVE = 0
```

> Verify these enum member values against `git show 1c2d90df:opendbc/car/hyundai/values.py | sed -n '1050,1065p'` before committing — they must match the source exactly, since they are written into CAN frames.

- [ ] **Step 5: Run the test to verify it passes**

```bash
uv run python -m unittest opendbc.car.hyundai.tests.test_hyundai.TestHyundaiFingerprint.test_ccnc_flags_defined -v
```

Expected: PASS

- [ ] **Step 6: Run the full hyundai suite to check for regressions**

```bash
uv run python -m unittest opendbc.car.hyundai.tests.test_hyundai
```

Expected: all pass — adding an unused flag must not change any existing behavior.

- [ ] **Step 7: Commit**

```bash
git add opendbc/car/hyundai/values.py opendbc/car/hyundai/tests/test_hyundai.py
git commit -m "hyundai: add CCNC flags and LFA_ALT state enums"
```

---

## Task 3: DBC definitions — VERIFIED NO-OP, no changes required

**Status: complete, zero code changes.** The plan's premise was wrong. Every message and signal the port needs already exists in the base DBC; the base branch has the *definitions* and lacks only the *code that uses them* (Task 4).

**Verification performed:**

| Message | Address | In `85f53e05` |
|---|---|---|
| `LFA_ALT` | `BO_ 203` = 0xCB | present, all 6 signals (`ADAS_ActvACISta`, `ADAS_ActvACILvl2Sta`, `ADAS_StrAnglReqVal`, `ADAS_ACIAnglTqRedcGainVal`, `FCA_ESA_ActvSta`, `FCA_ESA_TqBstGainVal`) |
| `CCNC_0x161` | `BO_ 353` = 0x161 | present |
| `CCNC_0x162` | `BO_ 354` = 0x162 | present |
| `FR_CMR_03_50ms` | `BO_ 437` = 0x1B5 | present |

Every signal `create_ccnc()` writes was individually confirmed present (`ALERTS_*`, `SOUNDS_*`, `*_ICON`, `LANELINE_*`, `LCA_*`, `FAULT_*`, `SETSPEED*`, `DISTANCE*`, `LEAD*`, `CENTERLINE`, `VIBRATE`, `Info_*`, `Longitudinal_Distance`).

**Why the original Step 2 would have caused a regression.** It said to `git checkout 1c2d90df -- <dbc>` on the grounds the diff was additive. Both of its own guard conditions were in fact violated:

- the CCNC diff has **5 removals**, not purely additive
- our base independently changed the same file (**6 insertions, 4 deletions**) — the same BCW signal-width fixes, made on both sides

Taking the file wholesale would have silently reverted the base's work. Always diff *both* sides against the merge-base before a wholesale file take.

**The one genuine CCNC-side delta, deliberately not ported:**

```
-VAL_ 53 GEAR 0 "P" 5 "D" 6 "N" 7 "R";
+VAL_ 53 GEAR 0 "P" 4 "S" 5 "D" 6 "N" 7 "R";
```

Message 53 is `ACCELERATOR`, and `carstate.py:41` consumes its `GEAR` **only when `HyundaiFlags.EV` is set**. The Palisade is a hybrid without the EV flag, so it resolves to `GEAR_SHIFTER`. That Sport-mode label serves the EV platforms the CCNC branch added (Kona EV 2nd gen, Ioniq 5 N), not our car. Porting it would be unnecessary change in a file the Sorento also reads.

> If the Palisade ever reports an unmapped gear value on-vehicle, revisit this — but add the mapping to whichever message it actually uses, not blindly to 53.

---

## Task 4: `LFA_ALT` steering path and `create_ccnc()`

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/hyundaicanfd.py`

**Interfaces:**
- Consumes: `HyundaiFlags.CCNC`, `ActvACISta`, `ESA_ActvSta` (Task 2); DBC messages (Task 3).
- Produces: `create_ccnc(packer, CAN, openpilotLongitudinalControl, enabled, hud, leftBlinker, rightBlinker, msg_161, msg_162, msg_1b5, is_metric, out, main_cruise_enabled, lfa_icon) -> list` and an `LFA_ALT` branch inside `create_steering_messages()`. Task 6 calls `create_ccnc()`.

This is the largest single change (~120 lines). It provides the HDA1 angle-steering control path — without it an HDA1 LFA2 car cannot steer.

- [ ] **Step 1: Write the failing test**

Create `opendbc/car/hyundai/tests/test_lfa_alt.py`:

```python
from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.structs import CarParams


def _params(candidate, cam_msgs):
  car_fw = [CarParams.CarFw(ecu=ecu, fwVersion=vers[0], address=addr, subAddress=sub or 0)
            for (ecu, addr, sub), vers in FW_VERSIONS[candidate].items()]
  fp = gen_empty_fingerprint()
  for m in cam_msgs:
    fp[2][m] = 16
  return CarInterface.get_params(candidate, fp, car_fw, False, True, False)


def test_hda2_car_does_not_use_lfa_alt():
  # Sorento is LKA steering; it must take the LKAS branch, never LFA_ALT
  CP = _params(CAR.KIA_SORENTO_HEV_4TH_GEN_LFA2, [0x50])
  assert CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG
  assert not (CP.flags & HyundaiFlags.CCNC)
```

- [ ] **Step 2: Run it to verify it fails or passes for the right reason**

```bash
uv run python -m unittest opendbc.car.hyundai.tests.test_lfa_alt -v
```

Expected: PASS already (the Sorento has no CCNC flag). This test is a **guard** — it must keep passing through Tasks 4-8. If it ever fails, the Sorento has been contaminated.

- [ ] **Step 3: Port the hyundaicanfd.py changes**

```bash
cd ~/sunnypilot/opendbc_repo
git diff cf3223a5 1c2d90df -- opendbc/car/hyundai/hyundaicanfd.py > /tmp/ccnc_hyundaicanfd.patch
git apply --3way /tmp/ccnc_hyundaicanfd.patch
```

If the apply conflicts, resolve by hand. The three required changes are:

Imports:
```python
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.values import HyundaiFlags, ActvACISta, ESA_ActvSta
```

Inside `create_steering_messages()`, before `ret = []`:
```python
  LFA_ALT_values = {}
  if CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING and CP.flags & HyundaiFlags.SEND_LFA:
    ActvACILvl2Sta = ActvACISta.ACTIVE35_ACTIVE if lat_active else ActvACISta.INACTIVE if enabled else ActvACISta.INACTIVE
    LFA_ALT_values = {
      "ADAS_ActvACISta": ActvACISta.INIT.value,
      "ADAS_ActvACILvl2Sta": ActvACILvl2Sta.value,
      "ADAS_StrAnglReqVal": apply_angle,
      "ADAS_ACIAnglTqRedcGainVal": apply_torque if lat_active else 0,
      "FCA_ESA_ActvSta": ESA_ActvSta.INACTIVE.value,
      "FCA_ESA_TqBstGainVal": 0
    }
```

And the new dispatch branch:
```python
  ret = []
  if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
    lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT else "LKAS"
    if CP.openpilotLongitudinalControl:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))
    ret.append(packer.make_can_msg(lkas_msg, CAN.ACAN, values))
  elif CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING and CP.flags & HyundaiFlags.SEND_LFA:
    # For cars with an HDA1 and LFA2, we send LFA messages to the ADAS ECU.
    ret.append(packer.make_can_msg("LFA_ALT", CAN.ECAN, LFA_ALT_values))
  else:
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))
```

Plus the full `create_ccnc()` function. Take it verbatim:
```bash
git show 1c2d90df:opendbc/car/hyundai/hyundaicanfd.py | sed -n '/^def create_ccnc/,/^def /p'
```

**The `elif` ordering is load-bearing.** `CANFD_LKA_STEER_MSG` is checked first, so an HDA2 car can never reach the `LFA_ALT` branch regardless of its other flags.

- [ ] **Step 4: Verify the guard test still passes**

```bash
uv run python -m unittest opendbc.car.hyundai.tests.test_lfa_alt -v
uv run python -m unittest opendbc.car.hyundai.tests.test_hyundai
```

Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add opendbc/car/hyundai/hyundaicanfd.py opendbc/car/hyundai/tests/test_lfa_alt.py
git commit -m "hyundai: add LFA_ALT steering path and CCNC cluster messages

LFA_ALT is the HDA1 angle-steering control path. Gated behind
CANFD_ANGLE_STEERING and SEND_LFA, and unreachable on LKA steering
cars because the CANFD_LKA_STEER_MSG branch is checked first."
```

---

## Task 5: CCNC message capture in `carstate.py`

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/carstate.py`

**Interfaces:**
- Consumes: `HyundaiFlags.CCNC` (Task 2).
- Produces: `self.msg_161`, `self.msg_162`, `self.msg_1b5` on `CarState`. Task 6 passes these to `create_ccnc()`.

- [ ] **Step 1: Port the carstate CCNC capture**

```bash
cd ~/sunnypilot/opendbc_repo
git diff cf3223a5 1c2d90df -- opendbc/car/hyundai/carstate.py
```

Apply only the CCNC hunk — around line 273:

```python
    if self.CP.flags & HyundaiFlags.CCNC:
      if self.frame % 5 == 0:
        self.msg_161, self.msg_162, self.msg_1b5 = map(copy.copy, (cp_cam.vl["CCNC_0x161"], cp_cam.vl["CCNC_0x162"], cp_cam.vl["FR_CMR_03_50ms"]))
```

> Verify the exact hunk and surrounding context with the diff above — the frame gating and attribute initialisation in `__init__` must both be ported. Do **not** take the whole file; the CCNC branch also carries unrelated MADS churn.

- [ ] **Step 2: Verify the Sorento path is untouched**

```bash
uv run python -m unittest opendbc.car.hyundai.tests.test_lfa_alt -v
uv run python -m unittest opendbc.car.hyundai.tests.test_hyundai
```

Expected: all PASS. The new code is inside `if self.CP.flags & HyundaiFlags.CCNC`, which is false for the Sorento.

- [ ] **Step 3: Commit**

```bash
git add opendbc/car/hyundai/carstate.py
git commit -m "hyundai: capture CCNC cluster messages in carstate"
```

---

## Task 6: Wire CCNC in `carcontroller.py` and `interface.py`

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/carcontroller.py`
- Modify: `opendbc_repo/opendbc/car/hyundai/interface.py`

**Interfaces:**
- Consumes: `create_ccnc()` (Task 4), `msg_161/162/1b5` (Task 5), `HyundaiSafetyFlags.CCNC` (Task 2).
- Produces: the `CCNC` safety param on resolved `CarParams` for HDA1 CCNC cars only.

- [ ] **Step 1: Add the `ccnc_non_hda2` gate in carcontroller**

> **Dependency:** `create_acc_control()` gains a trailing parameter in the CCNC branch. Confirm Task 4 ported that signature change before starting this step:
> ```bash
> grep -n 'def create_acc_control' opendbc/car/hyundai/hyundaicanfd.py
> ```
> It must accept the extra `cruise_info` argument. If it does not, go back and complete Task 4.

In `create_canfd_msgs()`, alongside the existing `lka_steering` locals:

```python
    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl
    ccnc_non_hda2 = self.CP.flags & HyundaiFlags.CCNC and not lka_steering
```

Replace the LFA/HDA icon block:

```python
    # LFA and HDA icons
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long):
      if ccnc_non_hda2:
        can_sends.extend(hyundaicanfd.create_ccnc(self.packer, self.CAN, self.CP.openpilotLongitudinalControl, CC.enabled, CC.hudControl, CC.leftBlinker,
                                                  CC.rightBlinker, CS.msg_161, CS.msg_162, CS.msg_1b5, CS.is_metric, CS.out, CS.main_cruise_enabled,
                                                  self.lfa_icon))
      else:
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled, self.lfa_icon))
```

And the longitudinal block:

```python
    if self.CP.openpilotLongitudinalControl:
      if lka_steering:
        can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
      elif not ccnc_non_hda2:
        can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
      if self.frame % 2 == 0:
        can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                         set_speed_in_units, hud_control, self.lead_data, CS.main_cruise_enabled, self.tuning,
                                                         CS.cruise_info if ccnc_non_hda2 else None))
        self.accel_last = accel
```

**`and not lka_steering` is the safety property.** It makes CCNC transmission structurally impossible on an HDA2 car.

Note the whole `openpilotLongitudinalControl` block is dead code for both our vehicles (neither has openpilot longitudinal), but it must be ported consistently so the signatures line up.

- [ ] **Step 2: Set the safety param in interface.py**

After the existing `CANFD_LKA_STEER_MSG` safety-param block:

```python
      if ret.flags & HyundaiFlags.CCNC and not ret.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CCNC.value
```

This is self-gating: an HDA2 car never receives the CCNC safety param even if its platform declared the flag.

- [ ] **Step 3: Run the guard tests**

```bash
uv run python -m unittest opendbc.car.hyundai.tests.test_lfa_alt opendbc.car.hyundai.tests.test_hyundai
```

Expected: all PASS.

- [ ] **Step 4: Commit**

```bash
git add opendbc/car/hyundai/carcontroller.py opendbc/car/hyundai/interface.py
git commit -m "hyundai: wire CCNC transmission and safety param, gated to non-HDA2"
```

---

## Task 7: Panda safety — CCNC param and `LFA_ALT` angle checks

**Files:**
- Modify: `opendbc_repo/opendbc/safety/modes/hyundai_canfd.h`
- Test: `opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py`

**Interfaces:**
- Consumes: `HyundaiSafetyFlags.CCNC = 2048` (Task 2).
- Produces: `HYUNDAI_PARAM_CCNC`, `hyundai_ccnc` state, `0xCB` TX angle validation, `HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_CCNC_TX_MSGS` allowlist.

This is safety-critical C. Port precisely; do not paraphrase.

- [ ] **Step 1: Review the full safety diff**

```bash
cd ~/sunnypilot/opendbc_repo
git diff cf3223a5 1c2d90df -- opendbc/safety/modes/hyundai_canfd.h
```

Expected: ~50 changed lines.

- [ ] **Step 2: Port the changes**

Required pieces, all from the diff above:

Add `LFA_ALT` to the LFA steering TX macro:
```c
  {0xCB,  e_can, 24, .check_relay = (e_can) == 0},  /* LFA_ALT */ \
```

Add the CCNC state:
```c
static bool hyundai_ccnc = false;

static bool get_hyundai_ccnc(void) {
  return hyundai_ccnc;
}
```

Add the `0xCB` angle-command validation in `hyundai_canfd_tx_hook()`:
```c
  // HDA1 steering
  if ((msg->addr == 0xCBU) && hyundai_canfd_angle_steering) {
    const int lfa_angle_active = (msg->data[3] >> 4U);
    const bool steer_angle_req = lfa_angle_active == 2;

    int desired_angle = (((uint32_t)(msg->data[5] & 0x3FU)) << 8) | (uint32_t)msg->data[4];
    desired_angle = to_signed(desired_angle, 14);

    if (steer_angle_cmd_checks_vm(desired_angle, steer_angle_req, HYUNDAI_CANFD_ANGLE_STEERING_LIMITS, HYUNDAI_STEERING_PARAMS)) {
      tx = false;
    }
  }
```

Add the param and the CCNC TX allowlist:
```c
  const uint16_t HYUNDAI_PARAM_CCNC = 2048;
```
```c
#define HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_CCNC_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal)) \
    {0x161, 0, 32, .check_relay = true}, /* CCNC_0x161 */ \
    {0x162, 0, 32, .check_relay = true}, /* CCNC_0x162 */ \
    {0x7C4, 2, 8, .check_relay = true}, /* 0x7C4 */ \
    {0xEA, 2, 24, .check_relay = true}, /* MDPS */ \
```
```c
  hyundai_ccnc = GET_FLAG(param, HYUNDAI_PARAM_CCNC);
```

Plus the allowlist selection in `hyundai_canfd_init()` — take verbatim from the diff.

**Note:** kamdeva's README describes relaxing the `0x105` counter check (`max_counter=0`, `ignore_counter=true`). That change is **not present in the final tree** — it was reverted during their bisect. Do not port it. Verify with:
```bash
git show kamdeva/hkg-angle-steering-2025-hda1-lx3:opendbc/safety/modes/hyundai_canfd.h | grep -c ignore_counter
```
Expected: `0`.

- [ ] **Step 3: Build the safety code**

```bash
cd ~/sunnypilot/opendbc_repo
scons -j4 opendbc/safety/
```

Expected: compiles with no warnings.

- [ ] **Step 4: Run the panda safety tests**

```bash
uv run python -m unittest opendbc.safety.tests.test_hyundai_canfd
```

Expected: all PASS. Existing HDA2 tests must be unaffected — the new TX check is gated on `msg->addr == 0xCBU`, which HDA2 cars never send.

- [ ] **Step 5: Commit**

```bash
git add opendbc/safety/modes/hyundai_canfd.h
git commit -m "safety: add CCNC param and LFA_ALT (0xCB) angle command checks

Deliberately omits the 0x105 counter relaxation described in kamdeva's
README -- that change was reverted in their tree and is not needed."
```

---

## Task 8: The LX3 platform

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/values.py`
- Modify: `opendbc_repo/opendbc/car/hyundai/fingerprints.py`
- Modify: `opendbc_repo/opendbc/car/hyundai/carstate.py`
- Modify: `opendbc_repo/opendbc/car/torque_data/override.toml`
- Modify: `opendbc_repo/opendbc/sunnypilot/car/car_list.json`

**Interfaces:**
- Consumes: `HyundaiFlags.CCNC` (Task 2).
- Produces: `CAR.HYUNDAI_PALISADE_HEV_LX3`.

- [ ] **Step 1: Write the failing test**

Append to `opendbc/car/hyundai/tests/test_lfa_alt.py`:

```python
def test_lx3_platform_resolves_as_hda1_ccnc():
  # Palisade LX3: no LKAS on the camera bus => LFA steering (HDA1)
  CP = _params(CAR.HYUNDAI_PALISADE_HEV_LX3, [])
  assert CP.flags & HyundaiFlags.CCNC
  assert CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
  assert not (CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG)
```

- [ ] **Step 2: Run it to confirm it fails**

```bash
uv run python -m unittest opendbc.car.hyundai.tests.test_lfa_alt.TestLfaAlt.test_lx3_platform_resolves_as_hda1_ccnc -v
```

Expected: FAIL with `AttributeError: HYUNDAI_PALISADE_HEV_LX3`

- [ ] **Step 3: Add the platform**

In `values.py`, after `HYUNDAI_SANTA_FE_HEV_5TH_GEN`. **Note the harness differs from kamdeva's source — `hyundai_n`, confirmed on the vehicle:**

```python
  HYUNDAI_PALISADE_HEV_LX3 = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Palisade Hybrid (without HDA II, LFA2) 2026", "Lane Follow Assist 2",
                     car_parts=CarParts.common([CarHarness.hyundai_n])),
    ],
    CarSpecs(mass=2175, wheelbase=2.97, steerRatio=13.72),
    flags=HyundaiFlags.CANFD_ANGLE_STEERING | HyundaiFlags.CCNC,
  )
```

- [ ] **Step 4: Add the FW fingerprint**

In `fingerprints.py`, in `FW_VERSIONS`, keeping alphabetical placement:

```python
  CAR.HYUNDAI_PALISADE_HEV_LX3: {
    (Ecu.fwdCamera, 0x7c4, None): [
      b'\xf1\x00LX31.001.011.002551000HKP_LX325_50430099211P9020',
    ],
    (Ecu.fwdRadar, 0x7d0, None): [
      b'\xf1\x00LX3__               1.00 1.00 99110P9010          ',
    ],
  },
```

- [ ] **Step 5: Add the torque override**

In `opendbc/car/torque_data/override.toml`, in the sorted block:

```toml
"HYUNDAI_PALISADE_HEV_LX3" = [2.5, 2.5, 0.1]
```

- [ ] **Step 6: Add the LX3 `CRUISE_BUTTONS` frequency-check skip**

In `carstate.py`, `get_can_parsers_canfd()`:

```python
    if not (CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS):
      # LX3 transmits CRUISE_BUTTONS at <1Hz (gaps >1s); skip freq check for that car.
      # carstate still reads buttons via cruise_btns_msg_canfd = "CRUISE_BUTTONS".
      if CP.carFingerprint != CAR.HYUNDAI_PALISADE_HEV_LX3:
        # TODO: this can be removed once we add dynamic support to vl_all
        msgs += [
          # this message is 50Hz but the ECU frequently stops transmitting for ~0.5s
          ("CRUISE_BUTTONS", 1)
        ]
```

`CAR` is already imported in `carstate.py` — no import change needed.

- [ ] **Step 7: Add the car_list entry**

```bash
cd ~/sunnypilot/opendbc_repo
git show kamdeva/hkg-angle-steering-2025-hda1-lx3:opendbc/sunnypilot/car/car_list.json | \
  python3 -c "import json,sys; d=json.load(sys.stdin); print(json.dumps([c for c in d if 'LX3' in json.dumps(c)], indent=2))"
```

Add the resulting entry to `opendbc/sunnypilot/car/car_list.json`, changing the harness to match `hyundai_n`.

- [ ] **Step 8: Run all tests**

```bash
uv run python -m unittest discover -s opendbc/car/hyundai/tests -t .
uv run python -m unittest opendbc.safety.tests.test_hyundai_canfd
```

Expected: all PASS, including `test_hda2_car_does_not_use_lfa_alt`.

- [ ] **Step 9: Commit**

```bash
git add opendbc/car/hyundai/values.py opendbc/car/hyundai/fingerprints.py \
        opendbc/car/hyundai/carstate.py opendbc/car/torque_data/override.toml \
        opendbc/sunnypilot/car/car_list.json opendbc/car/hyundai/tests/test_lfa_alt.py
git commit -m "hyundai: add 2026 Palisade Hybrid LX3 (HDA1 + LFA2)

Harness is hyundai_n, confirmed on the vehicle, overriding the
hyundai_l in the source port."
```

---

## Task 9: Sorento regression gate

**Files:**
- Create: `opendbc_repo/opendbc/car/hyundai/tests/test_lx3_regression.py`

**Interfaces:**
- Consumes: `docs/superpowers/baselines/sorento-baseline.json` (Task 1).

This is the hard gate. It must pass before anything is flashed.

- [ ] **Step 1: Write the regression test**

```python
import json
import pathlib
import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.structs import CarParams

# Fixture lives alongside this test so the opendbc branch stays self-contained
# and its tests pass when opendbc is checked out on its own.
BASELINE = pathlib.Path(__file__).parent / "sorento_baseline.json"


class TestSorentoRegression(unittest.TestCase):
  """The Sorento's resolved CarParams must be identical to the pre-change baseline.

  Both camera-bus probes are checked because the real vehicle's LKAS address
  (0x50 vs 0x110) is unknown and they resolve to different flags -- 0x110 adds
  CANFD_LKA_STEER_MSG_ALT. Asserting both makes the gate valid either way.
  """

  def test_sorento_resolved_params_unchanged(self):
    expected_all = json.loads(BASELINE.read_text())
    candidate = CAR.KIA_SORENTO_HEV_4TH_GEN_LFA2
    car_fw = [CarParams.CarFw(ecu=ecu, fwVersion=vers[0], address=addr, subAddress=sub or 0)
              for (ecu, addr, sub), vers in FW_VERSIONS[candidate].items()]

    for probe, expected in sorted(expected_all["variants"].items()):
      with self.subTest(probe=probe):
        fp = gen_empty_fingerprint()
        fp[2][int(probe, 16)] = 16

        CP = CarInterface.get_params(candidate, fp, car_fw, False, True, False)

        assert str(CP.carFingerprint) == expected_all["carFingerprint"]
        assert int(CP.flags) == expected["flags"], (
          f"[{probe}] Sorento flags changed: {expected['flags']} -> {int(CP.flags)} "
          f"(baseline decoded as {expected['flagNames']})")
        assert int(CP.safetyConfigs[-1].safetyParam) == expected["safetyParam"], (
          f"[{probe}] Sorento safetyParam changed: {expected['safetyParam']} -> "
          f"{int(CP.safetyConfigs[-1].safetyParam)} (baseline decoded as {expected['safetyFlagNames']})")
        assert bool(CP.alphaLongitudinalAvailable) == expected["alphaLongitudinalAvailable"]
```

- [ ] **Step 1b: Copy the baseline fixture into opendbc**

The canonical baseline lives in the sunnypilot repo as project documentation, but the test needs it locally so the opendbc branch is self-contained:

```bash
cp ~/sunnypilot/docs/superpowers/baselines/sorento-baseline.json \
   ~/sunnypilot/opendbc_repo/opendbc/car/hyundai/tests/sorento_baseline.json
```

- [ ] **Step 2: Run it**

```bash
cd ~/sunnypilot/opendbc_repo
uv run python -m unittest opendbc.car.hyundai.tests.test_lx3_regression -v
```

Expected: **PASS.** If it fails, a change from Tasks 2-8 leaked into the Sorento's path. Stop and fix before continuing — do not proceed to any build or flash.

- [ ] **Step 3: Commit**

```bash
git add opendbc/car/hyundai/tests/test_lx3_regression.py
git commit -m "test: assert Sorento resolved CarParams match the frozen baseline"
```

- [ ] **Step 4: Push the opendbc branch**

```bash
git push mine lx3-unified
```

---

## Task 10: `msgq` `NUM_READERS` — VERIFIED UNNECESSARY, no changes

**Status: complete, zero changes. No msgq fork required.**

kamdeva's README justifies bumping `NUM_READERS` 15 -> 64 because "sunnypilot has 36+
subscribers per high-frequency service and the upstream limit of 15 caused subscribers beyond
the cap to receive stale data." That reasoning does not hold on the current tip.

**Measured:**

| | |
|---|---|
| `NUM_READERS` in `msgq_repo/msgq/msgq.h` on the base | **25** — already raised since kamdeva's April fork point |
| Reader sites for the busiest service (`carState`) | **18** |
| Of those, dev tools that do not run concurrently on-device | 5 (`joystickd`, `lateral_maneuversd`, `maneuversd`, `tools/replay/ui`, `mapd/live_map_data/debug`) |
| Realistic on-device peak | ~13 |
| Direct `sub_sock("carState")` call sites | 1 |
| C++ `SubMaster` sites including `carState` | 0 |

18 worst-case against a cap of 25, and ~13 in the configuration that actually runs on the
device. The condition kamdeva hit does not exist here.

**Why not bump anyway as insurance.** `NUM_READERS` sizes three arrays in the shared-memory
queue header (`read_pointers`, `read_valids`, `read_uids`), so raising it to 64 grows every
queue's header for no measured benefit, and it would mean forking and maintaining `msgq` —
a third submodule fork — to change a constant that already has 38% headroom.

**Revisit trigger.** If `commIssue` alerts appear during Task 13 on-vehicle validation, come
back here first. That is the exact symptom the cap produces, and it is also what kamdeva's
`calibrationd`/`locationd` workarounds were papering over. Re-measure before changing anything:
the count above is reproducible with a `SubMaster` scan over `openpilot/`.

---

## Task 11: Wire the submodules and build

**Files:**
- Modify: `~/sunnypilot/.gitmodules`
- Modify: `opendbc_repo`, possibly `msgq_repo` submodule pointers

- [ ] **Step 1: Point opendbc at our fork**

```bash
cd ~/sunnypilot
git config -f .gitmodules submodule.opendbc.url https://github.com/raider-red-10/opendbc.git
git config -f .gitmodules submodule.opendbc.branch lx3-unified
git submodule sync opendbc_repo
cd opendbc_repo && git rev-parse HEAD
```

Expected: the HEAD of the branch built in Tasks 2-9.

- [ ] **Step 2: msgq — nothing to do**

Task 10 verified no bump is needed (`NUM_READERS` is already 25 against a worst case of 18).
`msgq_repo` stays pointed at `sunnypilot/msgq` at the base commit. No fork, no submodule change.

- [ ] **Step 3: Full build**

```bash
cd ~/sunnypilot
scons -j8
```

Expected: builds clean. This is the first full build — expect it to take a while and to surface any LFS or submodule problems from Task 1.

- [ ] **Step 4: Run the full test suite**

```bash
cd ~/sunnypilot/opendbc_repo
uv run python -m unittest discover -s opendbc/car/hyundai/tests -t . && uv run python -m unittest opendbc.safety.tests.test_hyundai_canfd
```

Expected: all PASS, including the Task 9 regression gate.

- [ ] **Step 5: Commit and push**

```bash
cd ~/sunnypilot
git add .gitmodules opendbc_repo msgq_repo
git commit -m "build: point submodules at lx3-unified forks"
git push mine lx3-unified
```

---

## Task 12: Device swap helper

**Files:**
- Create: `~/sunnypilot/tools/lx3/swap_car.py`

**Interfaces:**
- Produces: a script that clears per-vehicle params so a device moved between cars re-detects cleanly.

- [ ] **Step 1: Write the script**

```python
#!/usr/bin/env python3
"""Clear per-vehicle state before moving the comma device between cars.

Identity params MUST be cleared or the device will mis-detect the swapped car.
Learned params are optional -- clearing them costs a relearn but avoids one
car running on the other's learned dynamics.
"""
import argparse

from openpilot.common.params import Params

# Clearing these is mandatory: they cache which car this is.
IDENTITY_PARAMS = [
  "CarParamsCache",
  "CarParamsPersistent",
  "CarParamsPrevRoute",
  "FirmwareQueryDone",
]

# Clearing these is recommended: they hold per-vehicle learned dynamics.
LEARNED_PARAMS = [
  "CalibrationParams",
  "LiveParameters",
  "LiveTorqueParameters",
  "LiveDelay",
  "CarBatteryCapacity",
]


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--keep-learned", action="store_true",
                  help="clear identity params only; keep calibration and learned dynamics")
  args = ap.parse_args()

  params = Params()
  targets = IDENTITY_PARAMS + ([] if args.keep_learned else LEARNED_PARAMS)

  for key in targets:
    params.remove(key)
    print(f"cleared {key}")

  print(f"\n{len(targets)} params cleared. Reboot the device before driving.")


if __name__ == "__main__":
  main()
```

- [ ] **Step 2: Verify it runs**

```bash
cd ~/sunnypilot
python3 tools/lx3/swap_car.py --help
```

Expected: help text, no import errors.

- [ ] **Step 3: Commit**

```bash
git add tools/lx3/swap_car.py
git commit -m "tools: add per-vehicle param clearing for device swaps"
```

---

## Task 13: Staged on-vehicle validation

**Files:** none — this is procedure. Record results in the commit message.

**Order is the safety property: the Sorento is validated as unchanged before the Palisade is ever driven.**

- [ ] **Step 1: Tag the current known-good state**

```bash
cd ~/sunnypilot
git tag sorento-known-good origin/hkg-angle-steering-2025
git push mine sorento-known-good
```

This is the rollback target. Record how to return to it: reflash from `install.sunnypilot.ai/hkg-angle-steering-2025`.

- [ ] **Step 2: Confirm ICBM's actual toggle state on the Sorento**

Before flashing, check sunnypilot settings on the device and record whether **Intelligent Cruise Button Management** is on or off. This does not gate v1 (the ICBM changes are excluded), but it determines whether they can be adopted later.

- [ ] **Step 2b: Run the ICBM oscillation confirming test on the OLD branch**

Do this **before** flashing `lx3-unified`, while kamdeva's branch is still installed — it is the only chance to confirm the diagnosis on the branch that exhibits the bug.

The owner observed that engaging lane centering made the Palisade's stock cruise hunt (slight engine revving). **Root cause is not established** — see the spec's open-investigation section. One hypothesis (`create_ccnc()` writing cruise signals) is eliminated; the ICBM oscillation hypothesis is unconfirmed because the toggle's state at the time is unknown.

This does not gate v1 — the ICBM changes are excluded and the CCNC port is clean — but the observation is worth capturing while the old branch is still installed.

Record, in this order of value:

1. **Pull the drive log.** If a route from an affected drive still exists on the device or in connect.comma.ai, download it. This is the only evidence that settles the question; everything below is weaker.
2. **Note the ICBM toggle's current state** before changing anything.
3. If time allows, drive once with ICBM **off** and once **on**, same road, and note whether the symptom tracks the toggle.

Do **not** form a third hypothesis from source reading alone — two have already been formed that way and one was wrong.

- [ ] **Step 3: Flash the Sorento and verify no change**

Install `lx3-unified` on the device, put it in the Sorento, and confirm:
- fingerprints as `KIA_SORENTO_HEV_4TH_GEN_LFA2`
- lateral engages and steers as before
- no new alerts
- cruise button behavior unchanged

**If anything differs, stop.** The Task 9 gate said the resolved params are identical, so a behavior difference means something outside `CarParams` changed — investigate before proceeding.

- [ ] **Step 4: Swap to the Palisade**

```bash
python3 /data/openpilot/tools/lx3/swap_car.py
sudo reboot
```

Then verify:
- fingerprints as `HYUNDAI_PALISADE_HEV_LX3`
- `canValid` is true
- calibration completes (5-10 min highway above 25 mph)
- lateral engages and steers

Record whether `CANFD_ALT_BUTTONS` got set — it determines ICBM availability on this car and answers an open question in the spec.

- [ ] **Step 5: Swap back and confirm**

Run the swap helper again, return the device to the Sorento, and confirm it still fingerprints and steers correctly. This validates the swap loop in both directions.

- [ ] **Step 6: Record results**

```bash
cd ~/sunnypilot
git commit --allow-empty -m "validation: on-vehicle results for lx3-unified

Sorento: <fingerprint / lateral / alerts>
Palisade: <fingerprint / canValid / calibration / lateral / ALT_BUTTONS>
Swap loop: <both directions>
ICBM toggle state on Sorento: <on|off>"
git push mine lx3-unified
```

---

## Spec items requiring no task

- **Spec change F — drop the `calibrationd.py` / `locationd.py` workarounds.** No task needed. This plan builds *forward* from `hkg-angle-steering-2025` rather than rebasing kamdeva's branch, so those two workarounds are never introduced in the first place. Nothing to remove. If `commIssue` alerts appear during Task 13 validation, revisit Task 10 (`NUM_READERS`) first — the workarounds were symptomatic patches for that root cause.
- **Spec change C — carstate missing-message guards.** kamdeva's README describes pre-registering parser messages and guarding absent fields (`DOORS_SEATBELTS`, `BLINKERS`, `ADAS_CMD`, `HOD_FD`). That work was **reverted** in their tree by `d4f06c60`. The only carstate change surviving to their tip is the LX3 `CRUISE_BUTTONS` frequency-check skip, which Task 8 Step 6 covers. If the Palisade throws parser errors during Task 13, revisit `cdbc9493` — but do not port it preemptively.

---

## Deferred (explicitly not in this plan)

| Item | Why deferred | Where it's scoped |
|---|---|---|
| ICBM lateral-only changes | ICBM believed ON, making them live on the working car | Spec section E |
| Sorento longitudinal | Blocked pending the reason for `143f9203` | Spec follow-up section |
| Palisade longitudinal | `SCC_CONTROL` bytes 24-25 not reverse-engineered | Spec out-of-scope |
| Upstreaming the LX3 port | Validate first | Spec out-of-scope |
