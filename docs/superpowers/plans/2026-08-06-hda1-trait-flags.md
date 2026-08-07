# HDA1 Trait Flags Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace every `carFingerprint == HYUNDAI_PALISADE_HEV_LX3` gate with declarable/auto-detected traits so other HDA1/CCNC Hyundais can be added by declaration, and align panda's gating with openpilot's via a paired safety bit.

**Architecture:** Two mechanisms per the spec (`docs/superpowers/specs/2026-08-06-hda1-trait-flags-design.md`): bus-detectable traits are auto-set in `_get_params_sp` from the fingerprint (blinkers-alt, absent doors msg, absent HOD msg); the content-defined button trait is declared per platform as `HyundaiFlagsSP.BTN_CLUSTER_0X10B` and mirrored to panda as `HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B = 16` on `safetyParamSP`.

**Tech Stack:** Python (opendbc car port), C (opendbc safety / panda firmware), pytest via `uv run pytest` from `/Users/nickevans10/sunnypilot`.

## Global Constraints

- The Sorento regression (`opendbc/car/hyundai/tests/test_lx3_regression.py`) must pass **byte-identical** after every task — run it at every verification step.
- LX3 on-road behavior must be identical by construction: all existing LX3 tests pass (with only the documented setUp change from fingerprint-gating to flag-gating).
- All work happens in `/Users/nickevans10/sunnypilot/opendbc_repo` (its own git repo, branch `lx3-unified`). Run commands from `/Users/nickevans10/sunnypilot`.
- New `HyundaiFlagsSP` bits: `BTN_CLUSTER_0X10B = 2 ** 11`, `BLINKERS_ALT = 2 ** 12`, `ABSENT_DOORS_MSG = 2 ** 13`, `ABSENT_HOD_MSG = 2 ** 14`. New `HyundaiSafetyFlagsSP` / `HYUNDAI_PARAM_SP` bit: `16`.
- Trait predicates (E-CAN bus, CAN FD cars only): `BLINKERS_ALT` ⇔ `0x413 not in fp and 0x3E3 in fp`; `ABSENT_DOORS_MSG` ⇔ `0x411 not in fp`; `ABSENT_HOD_MSG` ⇔ `0x2AF not in fp`.
- Comment style: repo uses `--` dashes and measured-fact comments; match it.

---

### Task 1: Flags and interface plumbing

**Files:**
- Modify: `opendbc_repo/opendbc/sunnypilot/car/hyundai/values.py` (both flag classes)
- Modify: `opendbc_repo/opendbc/car/hyundai/interface.py` (`_get_params_sp`)
- Create: `opendbc_repo/opendbc/car/hyundai/tests/test_hda1_trait_flags.py`

**Interfaces:**
- Consumes: existing `CarInterface.get_params` / `CarInterface.get_params_sp` classmethods; `HyundaiFlagsSP`, `HyundaiSafetyFlagsSP` in `opendbc/sunnypilot/car/hyundai/values.py`.
- Produces: `HyundaiFlagsSP.BTN_CLUSTER_0X10B/BLINKERS_ALT/ABSENT_DOORS_MSG/ABSENT_HOD_MSG` (IntFlag members), `HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B = 16`, and a `CP_SP` whose `.flags`/`.safetyParam` carry them — Tasks 2–4 gate on exactly these names.

- [ ] **Step 1: Write the failing test**

Create `opendbc_repo/opendbc/car/hyundai/tests/test_hda1_trait_flags.py`:

```python
import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP

ECAN = 0
BLINKERS, BLINKERS_ALT = 0x413, 0x3E3
DOORS, HOD = 0x411, 0x2AF


def resolve(candidate, fp):
  fw = [CarParams.CarFw(ecu=e, fwVersion=v[0], address=a, subAddress=s or 0)
        for (e, a, s), v in FW_VERSIONS[candidate].items()]
  CP = CarInterface.get_params(candidate, fp, fw, False, True, False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fp, fw, False, True, False)
  return CP, CP_SP


def lx3_fp(extra_ecan=()):
  fp = gen_empty_fingerprint()
  fp[2][0xCB] = 24  # LFA_ALT: selects the HDA1 angle path
  for addr in extra_ecan:
    fp[ECAN][addr] = 8
  return fp


class TestHda1TraitFlags(unittest.TestCase):
  """Bus-detectable traits configure themselves from the fingerprint; the content-defined
  button trait is declared per platform and mirrored onto safetyParamSP so panda gates on
  the same truth."""

  def test_lx3_declares_the_button_cluster(self):
    _, CP_SP = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp())
    self.assertTrue(CP_SP.flags & HyundaiFlagsSP.BTN_CLUSTER_0X10B)
    self.assertTrue(CP_SP.safetyParam & HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B)

  def test_sorento_declares_nothing(self):
    fp = gen_empty_fingerprint()
    fp[2][0x50] = 16  # LKAS: HDA2 probe
    _, CP_SP = resolve(CAR.KIA_SORENTO_HEV_4TH_GEN_LFA2, fp)
    self.assertFalse(CP_SP.flags & HyundaiFlagsSP.BTN_CLUSTER_0X10B)
    self.assertFalse(CP_SP.safetyParam & HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B)

  def test_blinkers_alt_detected_from_the_bus(self):
    # 0x413 absent AND 0x3e3 present -> alt; anything else -> standard
    _, with_alt = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp(extra_ecan=(BLINKERS_ALT,)))
    self.assertTrue(with_alt.flags & HyundaiFlagsSP.BLINKERS_ALT)

    _, both = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp(extra_ecan=(BLINKERS, BLINKERS_ALT)))
    self.assertFalse(both.flags & HyundaiFlagsSP.BLINKERS_ALT)

    _, neither = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp())
    self.assertFalse(neither.flags & HyundaiFlagsSP.BLINKERS_ALT)

  def test_message_absences_detected_from_the_bus(self):
    _, absent = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp())
    self.assertTrue(absent.flags & HyundaiFlagsSP.ABSENT_DOORS_MSG)
    self.assertTrue(absent.flags & HyundaiFlagsSP.ABSENT_HOD_MSG)

    _, present = resolve(CAR.HYUNDAI_PALISADE_HEV_LX3, lx3_fp(extra_ecan=(DOORS, HOD)))
    self.assertFalse(present.flags & HyundaiFlagsSP.ABSENT_DOORS_MSG)
    self.assertFalse(present.flags & HyundaiFlagsSP.ABSENT_HOD_MSG)


if __name__ == "__main__":
  unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest opendbc_repo/opendbc/car/hyundai/tests/test_hda1_trait_flags.py -q`
Expected: FAIL / AttributeError — `HyundaiFlagsSP` has no attribute `BTN_CLUSTER_0X10B`.

- [ ] **Step 3: Add the flag members**

In `opendbc_repo/opendbc/sunnypilot/car/hyundai/values.py`, extend `HyundaiFlagsSP` (after `HAS_LKAS12 = 2 ** 10`):

```python
  # Declared per platform: the whole steering-wheel cluster is in LFA_BUTTON_ALT (0x10b)
  # byte 10 and CRUISE_BUTTONS_ALT (0x1aa)'s button field is permanently zero. Content-defined
  # (0x1aa exists -- its emptiness is invisible to the fingerprint), so it cannot be
  # auto-detected. Mirrored onto safetyParamSP so panda gates on the same trait.
  BTN_CLUSTER_0X10B = 2 ** 11
  # Auto-detected from the fingerprint at _get_params_sp -- never declare these per platform.
  BLINKERS_ALT = 2 ** 12       # 0x413 absent, lamps in 0x3e3 byte 11 at 5Hz
  ABSENT_DOORS_MSG = 2 ** 13   # DOORS_SEATBELTS (0x411) not transmitted
  ABSENT_HOD_MSG = 2 ** 14     # HOD_FD_01_100ms (0x2af) not transmitted
```

And extend `HyundaiSafetyFlagsSP` (after `NON_SCC = 8`):

```python
  BTN_CLUSTER_0X10B = 16
```

- [ ] **Step 4: Wire `_get_params_sp`**

In `opendbc_repo/opendbc/car/hyundai/interface.py`, inside `_get_params_sp`, in the existing `if stock_cp.flags & HyundaiFlags.CANFD:` block (where `CAN` is already computed), add:

```python
      # Bus-detectable traits configure themselves; see the HDA1 trait flags design doc.
      if 0x413 not in fingerprint[CAN.ECAN] and 0x3E3 in fingerprint[CAN.ECAN]:
        ret.flags |= HyundaiFlagsSP.BLINKERS_ALT.value
      if 0x411 not in fingerprint[CAN.ECAN]:
        ret.flags |= HyundaiFlagsSP.ABSENT_DOORS_MSG.value
      if 0x2AF not in fingerprint[CAN.ECAN]:
        ret.flags |= HyundaiFlagsSP.ABSENT_HOD_MSG.value

    # Content-defined trait, declared per measured platform: buttons live in LFA_BUTTON_ALT
    # (0x10b) byte 10 and 0x1aa's button field is permanently zero.
    if candidate in (CAR.HYUNDAI_PALISADE_HEV_LX3,):
      ret.flags |= HyundaiFlagsSP.BTN_CLUSTER_0X10B.value
    if ret.flags & HyundaiFlagsSP.BTN_CLUSTER_0X10B:
      ret.safetyParam |= HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B
```

Note indentation: the three auto-detect `if`s go **inside** the CANFD block; the declaration block goes after it at function-body level. `CAR` is already imported in interface.py.

- [ ] **Step 5: Run tests to verify they pass**

Run: `uv run pytest opendbc_repo/opendbc/car/hyundai/tests/test_hda1_trait_flags.py opendbc_repo/opendbc/car/hyundai/tests/test_lx3_regression.py -q`
Expected: all PASS (regression byte-identical — CP_SP additions do not touch `CP.flags`/`safetyParam`).

- [ ] **Step 6: Commit**

```bash
cd /Users/nickevans10/sunnypilot/opendbc_repo && git add opendbc/sunnypilot/car/hyundai/values.py opendbc/car/hyundai/interface.py opendbc/car/hyundai/tests/test_hda1_trait_flags.py && git commit -m "hyundai: declarable HDA1 traits -- auto-detect the bus-visible ones"
```

---

### Task 2: carstate gates on traits, not the platform

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/carstate.py` (five gate sites + `get_can_parsers_canfd` signature)
- Modify: `opendbc_repo/opendbc/car/hyundai/tests/test_lx3_buttons.py` (setUp builds CP_SP via the interface; one new RED test)

**Interfaces:**
- Consumes: `HyundaiFlagsSP.BTN_CLUSTER_0X10B/BLINKERS_ALT/ABSENT_DOORS_MSG/ABSENT_HOD_MSG` from Task 1; `self.CP_SP.flags` (already stored by `CarStateBase`).
- Produces: `get_can_parsers_canfd(self, CP, CP_SP)` (new signature — update its caller `get_can_parsers`); `_update_lx3_buttons` unchanged in name and behavior, now reached via the flag.

- [ ] **Step 1: Write the failing test**

In `opendbc_repo/opendbc/car/hyundai/tests/test_lx3_buttons.py`, class `TestLx3ButtonInteraction`: replace the `setUp` body's CP_SP construction and add one test. New `setUp` (replacing `self.CS = CarState(CP, structs.CarParamsSP())`):

```python
  def setUp(self):
    c = CAR.HYUNDAI_PALISADE_HEV_LX3
    fw = [CarParams.CarFw(ecu=e, fwVersion=v[0], address=a, subAddress=s or 0)
          for (e, a, s), v in FW_VERSIONS[c].items()]
    fp = gen_empty_fingerprint()
    fp[2][0xCB] = 24
    CP = CarInterface.get_params(c, fp, fw, False, True, False)
    self.CP_SP = CarInterface.get_params_sp(CP, c, fp, fw, False, True, False)
    self.CS = CarState(CP, self.CP_SP)
    self.counter = 0
```

New test at the end of the class:

```python
  def test_trait_gates_on_the_flag_not_the_platform(self):
    # An LX3 CP with the trait cleared must behave like a stock alt-buttons car: the 0x10b
    # cluster is ignored. Proves the gate reads the declared flag, not carFingerprint.
    from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
    self.CP_SP.flags &= ~HyundaiFlagsSP.BTN_CLUSTER_0X10B.value
    self.CS = CarState(self.CS.CP, self.CP_SP)
    self.CS._update_lx3_buttons(self.btns(1 << MINUS))
    self.assertFalse(self.CS.recent_button_interaction())
```

Wait — `_update_lx3_buttons` itself is trait-internal; calling it directly bypasses the gate. The gate lives in `update_canfd`, which needs full parsers. Test the gate at its seam instead: the method that `update_canfd` uses to decide. Change the gate implementation (Step 3) to compute a helper property, and test that:

```python
  def test_trait_gates_on_the_flag_not_the_platform(self):
    from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
    self.assertTrue(self.CS.btn_cluster_0x10b)
    self.CP_SP.flags &= ~HyundaiFlagsSP.BTN_CLUSTER_0X10B.value
    self.assertFalse(CarState(self.CS.CP, self.CP_SP).btn_cluster_0x10b)
```

Use this second version. Also update the module imports:

```python
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.hyundai.interface import CarInterface
```
(`structs` stays — `ButtonType` uses it; `CarParams` import stays.)

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest opendbc_repo/opendbc/car/hyundai/tests/test_lx3_buttons.py -q`
Expected: FAIL — `CarState` has no attribute `btn_cluster_0x10b`.

- [ ] **Step 3: Implement the five carstate gates**

In `opendbc_repo/opendbc/car/hyundai/carstate.py`:

a. Import the SP flags (top of file, with the other sunnypilot imports):
```python
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP as HyundaiFlagsSPValues
```
Check first — carstate already imports `HyundaiFlagsSP` (it uses `NON_SCC`); if so, reuse that name and skip the new import.

b. In `__init__`, after `self.other_button = 0`:
```python
    # Declared/auto-detected HDA1 traits -- see the trait flags design doc
    self.btn_cluster_0x10b = bool(CP_SP.flags & HyundaiFlagsSP.BTN_CLUSTER_0X10B)
```

c. Doors gate — replace
```python
    if self.CP.carFingerprint != CAR.HYUNDAI_PALISADE_HEV_LX3:
      ret.doorOpen = cp.vl["DOORS_SEATBELTS"]["DRIVER_DOOR"] == 1
      ret.seatbeltUnlatched = cp.vl["DOORS_SEATBELTS"]["DRIVER_SEATBELT"] == 0
```
with
```python
    if not self.CP_SP.flags & HyundaiFlagsSP.ABSENT_DOORS_MSG:
      ret.doorOpen = cp.vl["DOORS_SEATBELTS"]["DRIVER_DOOR"] == 1
      ret.seatbeltUnlatched = cp.vl["DOORS_SEATBELTS"]["DRIVER_SEATBELT"] == 0
```
and reword the comment above it: the message-absence is auto-detected from the fingerprint (a registered message that never arrives fails `can_valid`).

d. HOD gate — replace `if self.CP.carFingerprint != CAR.HYUNDAI_PALISADE_HEV_LX3:` guarding `HOD_FD_01_100ms` with `if not self.CP_SP.flags & HyundaiFlagsSP.ABSENT_HOD_MSG:` (same comment treatment).

e. Blinker gate — replace `if self.CP.carFingerprint == CAR.HYUNDAI_PALISADE_HEV_LX3:` selecting `BLINKERS_ALT` with `if self.CP_SP.flags & HyundaiFlagsSP.BLINKERS_ALT:`.

f. Button gates — both `is_lx3 = self.CP.carFingerprint == CAR.HYUNDAI_PALISADE_HEV_LX3` and the earlier `if self.CP.carFingerprint == CAR.HYUNDAI_PALISADE_HEV_LX3:` / `lx3_button_events` block become `self.btn_cluster_0x10b` (rename the local `is_lx3` to `btn_cluster` for honesty; the comment about double events stays).

g. Parser gate — change the signature and the skip:
```python
  def get_can_parsers_canfd(self, CP, CP_SP):
    msgs = []
    if not (CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS):
      # Cluster-button cars transmit CRUISE_BUTTONS (0x1cf) at <1Hz (gaps >1s, measured), so
      # the frequency check would fault. Their buttons come from 0x10b anyway.
      if not CP_SP.flags & HyundaiFlagsSP.BTN_CLUSTER_0X10B:
        ...
```
and update the caller: `return self.get_can_parsers_canfd(CP, CP_SP)` in `get_can_parsers`.

h. Remove the now-unused `CAR.HYUNDAI_PALISADE_HEV_LX3` references; if `CAR` becomes unused in carstate, ruff will say so — keep the import only if still used elsewhere in the file.

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest opendbc_repo/opendbc/car/hyundai/tests/ -q`
Expected: all PASS (including regression, blinkers, buttons, shape, cadence).

- [ ] **Step 5: Commit**

```bash
cd /Users/nickevans10/sunnypilot/opendbc_repo && git add opendbc/car/hyundai/carstate.py opendbc/car/hyundai/tests/test_lx3_buttons.py && git commit -m "hyundai: carstate gates on HDA1 traits, not the LX3 platform"
```

---

### Task 3: ICBM gates on the trait

**Files:**
- Modify: `opendbc_repo/opendbc/sunnypilot/car/hyundai/icbm.py`
- Modify: `opendbc_repo/opendbc/car/hyundai/tests/test_lx3_press_cadence.py` (setUp builds CP_SP via the interface; one new RED test)

**Interfaces:**
- Consumes: `HyundaiFlagsSP.BTN_CLUSTER_0X10B`; `self.CP_SP` (stored by `IntelligentCruiseButtonManagementInterfaceBase.__init__`).
- Produces: nothing new — `update()` routes to `create_lx3_press_messages` via the flag.

- [ ] **Step 1: Write the failing test**

In `test_lx3_press_cadence.py`, change `setUp` to resolve CP_SP through the interface and pass it in:

```python
    CP = CarInterface.get_params(c, fp, fw, False, True, False)
    CP_SP = CarInterface.get_params_sp(CP, c, fp, fw, False, True, False)
    self.icbm = IntelligentCruiseButtonManagementInterface(CP, CP_SP)
```
(add `from opendbc.car.hyundai.interface import CarInterface` if not present — it is already imported there.)

Add one test:

```python
  def test_no_press_path_without_the_trait(self):
    from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
    self.icbm.CP_SP.flags &= ~HyundaiFlagsSP.BTN_CLUSTER_0X10B.value
    self.assertEqual(self.run_ticks(20), [])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest opendbc_repo/opendbc/car/hyundai/tests/test_lx3_press_cadence.py -q`
Expected: `test_no_press_path_without_the_trait` FAILS (frames still sent — the gate is still `carFingerprint`).

- [ ] **Step 3: Implement**

In `icbm.py` `update()`, replace

```python
    if self.CP.carFingerprint == CAR.HYUNDAI_PALISADE_HEV_LX3:
```
with
```python
    if self.CP_SP.flags & HyundaiFlagsSP.BTN_CLUSTER_0X10B:
```
adding `from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP` to the imports. If `CAR` is now unused in icbm.py, drop it from the import list (ruff confirms).

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest opendbc_repo/opendbc/car/hyundai/tests/test_lx3_press_cadence.py opendbc_repo/opendbc/car/hyundai/tests/test_lx3_button_press_shape.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
cd /Users/nickevans10/sunnypilot/opendbc_repo && git add opendbc/sunnypilot/car/hyundai/icbm.py opendbc/car/hyundai/tests/test_lx3_press_cadence.py && git commit -m "hyundai: ICBM presses 0x10b on the declared trait"
```

---

### Task 4: panda gates on the safety bit

**Files:**
- Modify: `opendbc_repo/opendbc/safety/modes/hyundai_common.h` (new SP param + bool)
- Modify: `opendbc_repo/opendbc/safety/modes/hyundai_canfd.h` (rx hook ×3, tx hook, rx-check variant)
- Modify: `opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py` (existing CCNC class setUp + new no-trait class)

**Interfaces:**
- Consumes: `current_safety_param_sp` (set by tests via `set_current_safety_param_sp`, by the car via `safetyParamSP` from Task 1); existing `hyundai_common_init`.
- Produces: `hyundai_btn_cluster_0x10b` (bool, file-scope in hyundai_common.h) used by hyundai_canfd.h; `HYUNDAI_PARAM_SP_BTN_CLUSTER_0X10B = 16`.

- [ ] **Step 1: Write the failing tests**

In `test_hyundai_canfd.py`:

a. In `TestHyundaiCanfdCcncAltButtonsTx.setUp`, before `set_safety_hooks`, add:
```python
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B)
```
with `from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP` at the top of the file (check whether it is already imported).

b. New class after it:

```python
class TestHyundaiCanfdCcncWithoutBtnCluster(unittest.TestCase):
  """CCNC describes the cluster, not the buttons. A CCNC car without the BTN_CLUSTER_0X10B
  safety bit must keep stock alt-buttons behavior: buttons (and the controls_allowed latch)
  come from 0x1aa, and 0x10b is neither read nor transmittable. On the LX3 branch this was
  CCNC-gated, which would have broken the latch on every torque CCNC car."""

  PT_BUS = 0
  SCC_BUS = 2
  CCNC_PARAM = (HyundaiSafetyFlags.CCNC | HyundaiSafetyFlags.CANFD_ALT_BUTTONS |
                HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CANFD_ANGLE_STEERING |
                HyundaiSafetyFlags.HYBRID_GAS)

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.CCNC_PARAM)
    self.safety.init_tests()

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def test_alt_buttons_latch_controls(self):
    # stock behavior restored: 0x1aa carries the buttons and arms the latch
    self.safety.set_controls_allowed(0)
    self._rx(self._pcm_status_msg(False))
    self._rx(self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS,
                                             {"CRUISE_BUTTONS": Buttons.SET}))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_10b_buttons_do_not_latch(self):
    self.safety.set_controls_allowed(0)
    self._rx(self._pcm_status_msg(False))
    self._rx(self.packer.make_can_msg_safety("LFA_BUTTON_ALT", self.PT_BUS, {"DECEL_BTN": 1}))
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_10b_tx_rejected_even_with_controls(self):
    self.safety.set_controls_allowed(1)
    msg = self.packer.make_can_msg_safety("LFA_BUTTON_ALT", 2, {"ACCEL_BTN": 1})
    self.assertFalse(self.safety.safety_tx_hook(msg))
```

Copy the import names (`CANPackerSafety`, `libsafety_py`, `CarParams`, `Buttons`, `HyundaiSafetyFlags`) from the top of the existing file — they are all already imported there.

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py -k "Ccnc" -q`
Expected: the three new `WithoutBtnCluster` tests FAIL (`0x1aa` ignored / `0x10b` read+allowed under plain CCNC); the existing class still passes (bit set but unread so far).

- [ ] **Step 3: Implement the panda gating**

a. `hyundai_common.h` — extend the SP enum:
```c
enum {
  HYUNDAI_PARAM_SP_ESCC = 1,
  HYUNDAI_PARAM_SP_LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE = 2,
  HYUNDAI_PARAM_SP_HAS_LDA_BUTTON = 4,
  HYUNDAI_PARAM_SP_NON_SCC = 8,
  HYUNDAI_PARAM_SP_BTN_CLUSTER_0X10B = 16,
};
```
add the state bool next to the others:
```c
extern bool hyundai_btn_cluster_0x10b;
bool hyundai_btn_cluster_0x10b = false;
```
and read it in `hyundai_common_init` with the other SP flags:
```c
  hyundai_btn_cluster_0x10b = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_BTN_CLUSTER_0X10B);
```

b. `hyundai_canfd.h` rx hook — three swaps:
- `if ((msg->addr == button_addr) && !get_hyundai_ccnc())` → `... && !hyundai_btn_cluster_0x10b)`
- the inner `if (!get_hyundai_ccnc())` guarding `mads_button_press` on `0x1aa` → `if (!hyundai_btn_cluster_0x10b)`
- `if (get_hyundai_ccnc() && (msg->addr == 0x10BU))` → `if (hyundai_btn_cluster_0x10b && (msg->addr == 0x10BU))`
Update the comments: the trait, not CCNC, says where the buttons are.

c. tx hook — the `0x10B` branch condition gains the bit:
```c
    if (!hyundai_canfd_alt_buttons || !hyundai_btn_cluster_0x10b || resume || lfa || !one_button || !controls_allowed) {
      tx = false;
    }
```

d. rx-check split — replace `HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS_CCNC` usage with two variants: keep the existing macro (CCNC common + `0x1aa`) **without** the `0x10b` entry, and add
```c
#define HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS_CCNC_BTN_CLUSTER(pt_bus)                                                                             \
  HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS_CCNC(pt_bus)                                                                                               \
  {.msg = {{0x10b, (pt_bus), 16, 25U, .ignore_checksum = true, .max_counter = 0U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
```
(i.e. move the `0x10b` line out of the base CCNC macro into the new one). In `hyundai_canfd_init`, where `hyundai_canfd_alt_buttons_ccnc_rx_checks` is selected, add a static array `hyundai_canfd_alt_buttons_ccnc_btn_cluster_rx_checks[]` built from the new macro and select it when `hyundai_btn_cluster_0x10b` is set. Apply the same treatment in the **longitudinal** branch only if it uses the CCNC rx-check macro (it currently does not — verify by reading the init function; do not add speculative variants).

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py -q`
Expected: all PASS, including the pre-existing CCNC latch/tx tests (now under the bit) and the new no-trait class.

- [ ] **Step 5: Run the full safety suite**

Run: `uv run pytest opendbc_repo/opendbc/safety/tests -q`
Expected: all PASS — the bit defaults to 0, so every non-Hyundai-CCNC config is untouched. If any other test class trips over a stale `current_safety_param_sp` (it is a global), add `self.safety.set_current_safety_param_sp(0)` to that class's setUp rather than relying on ordering.

- [ ] **Step 6: Commit**

```bash
cd /Users/nickevans10/sunnypilot/opendbc_repo && git add opendbc/safety/modes/hyundai_common.h opendbc/safety/modes/hyundai_canfd.h opendbc/safety/tests/test_hyundai_canfd.py && git commit -m "safety: gate the 0x10b button cluster on its own SP bit, not CCNC"
```

---

### Task 5: full verification, docs, and handoff

**Files:**
- Modify: `docs/superpowers/specs/2026-08-06-lx3-handoff.md` (the "Who can run this" section and the asymmetry paragraph)
- Modify: `/Users/nickevans10/sunnypilot` (openpilot repo): `opendbc_repo` submodule bump

**Interfaces:**
- Consumes: everything above, complete and committed in opendbc.
- Produces: a deployable branch pair and an accurate handoff doc.

- [ ] **Step 1: Full suites**

Run: `uv run pytest opendbc_repo/opendbc/car/hyundai/tests/ opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py openpilot/sunnypilot/selfdrive/car/tests/test_speed_limit_assist_chain.py openpilot/sunnypilot/selfdrive/car/tests/test_set_speed_management_engaged.py openpilot/sunnypilot/selfdrive/controls/lib/speed_limit/tests/test_speed_limit_assist_non_pcm.py -q`
Expected: all PASS.

Run: `uv run ruff check opendbc_repo/opendbc/car/hyundai/ opendbc_repo/opendbc/sunnypilot/car/hyundai/ opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py`
Expected: no new errors (two pre-existing hits in `test_lx3_regression.py` and one in `values.py` are known).

- [ ] **Step 2: Update the handoff doc**

In `docs/superpowers/specs/2026-08-06-lx3-handoff.md`:
- In *Who can run this*: replace the "six sites … widen the LX3 gates" sentence with the new reality — traits auto-detect from the bus; a new HDA1 car declares only `BTN_CLUSTER_0X10B` (if measured); point to the trait-flags design doc.
- Replace the *One asymmetry to reconcile before upstreaming* paragraph: resolved — panda now gates on `HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B`; CCNC cars without the bit keep stock button handling.

- [ ] **Step 3: Bump the submodule and commit both repos**

```bash
cd /Users/nickevans10/sunnypilot && git add opendbc_repo docs/superpowers/specs/2026-08-06-lx3-handoff.md docs/superpowers/plans/2026-08-06-hda1-trait-flags.md && git commit -m "build: bump opendbc for declarable HDA1 traits"
```

- [ ] **Step 4: Hand back to the user**

Present the push commands (both repos, `mine lx3-unified`), the device update command (fetch + reset + `git submodule update --init opendbc_repo` + `scons -j4` + reboot — panda firmware WILL rebuild: safety code changed), and the sanity-drive checklist from the spec: engage, confirm a limit change, watch the set speed land on limit+offset, re-engage once via the cancel/set toggle.

---

## Self-review notes

- Spec coverage: auto-detect traits (Task 1), declared trait + mirror bit (Task 1), carstate gates incl. parser signature (Task 2), ICBM (Task 3), panda rx/tx/rx-checks + CCNC-without-bit restoration (Task 4), invariants + deployment (every task's verification + Task 5). The spec's "param resolution test" is Task 1; "safety matrix" is Task 4; "auto-detection tests" are Task 1.
- Types: gate name `btn_cluster_0x10b` (carstate attr), flag `HyundaiFlagsSP.BTN_CLUSTER_0X10B`, safety `HyundaiSafetyFlagsSP.BTN_CLUSTER_0X10B` = C `HYUNDAI_PARAM_SP_BTN_CLUSTER_0X10B` = 16 — consistent across tasks.
- Known judgment calls an implementer must make in place (verify, don't assume): whether carstate already imports `HyundaiFlagsSP`; whether `CAR` remains used in carstate/icbm after the swaps; whether the longitudinal init branch uses the CCNC rx-check macro.
