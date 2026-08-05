import unittest

from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import set_speed_management_engaged


def params(op_long: bool, pcm_cruise_speed: bool):
  CP = structs.CarParams()
  CP.openpilotLongitudinalControl = op_long
  CP_SP = custom.CarParamsSP.new_message()
  CP_SP.pcmCruiseSpeed = pcm_cruise_speed
  return CP, CP_SP


class TestSetSpeedManagementEngaged(unittest.TestCase):
  """ICBM and Speed Limit Assist need the *car's* cruise running, not openpilot's own
  engagement. On a MADS car, openpilot's `enabled` never goes true while lateral is engaged
  -- that is what keeps steering independent of cruise -- so gating on it disabled both
  features outright."""

  def test_icbm_car_falls_back_to_car_cruise(self):
    # The Palisade: stock longitudinal, ICBM owning the set speed, MADS lateral-only
    CP, CP_SP = params(op_long=False, pcm_cruise_speed=False)
    self.assertTrue(set_speed_management_engaged(CP, CP_SP, op_enabled=False, car_cruise_enabled=True))

  def test_icbm_car_not_engaged_with_cruise_off(self):
    CP, CP_SP = params(op_long=False, pcm_cruise_speed=False)
    self.assertFalse(set_speed_management_engaged(CP, CP_SP, op_enabled=False, car_cruise_enabled=False))

  def test_icbm_car_still_engaged_when_openpilot_is(self):
    # Falling back must widen the condition, never narrow it
    CP, CP_SP = params(op_long=False, pcm_cruise_speed=False)
    self.assertTrue(set_speed_management_engaged(CP, CP_SP, op_enabled=True, car_cruise_enabled=False))

  def test_pcm_cruise_speed_car_is_unchanged(self):
    # ICBM not managing the set speed -> openpilot's engagement is the only thing that counts
    CP, CP_SP = params(op_long=False, pcm_cruise_speed=True)
    for op_enabled in (True, False):
      for car_cruise in (True, False):
        with self.subTest(op_enabled=op_enabled, car_cruise=car_cruise):
          self.assertEqual(op_enabled,
                           set_speed_management_engaged(CP, CP_SP, op_enabled, car_cruise))

  def test_openpilot_longitudinal_car_is_unchanged(self):
    # openpilot drives longitudinal itself; the car's cruise must not stand in for engagement
    CP, CP_SP = params(op_long=True, pcm_cruise_speed=False)
    self.assertFalse(set_speed_management_engaged(CP, CP_SP, op_enabled=False, car_cruise_enabled=True))
    self.assertTrue(set_speed_management_engaged(CP, CP_SP, op_enabled=True, car_cruise_enabled=False))

  def test_only_the_icbm_configuration_changes_behaviour(self):
    # Exhaustive: the fallback applies to exactly one of the four configurations
    changed = set()
    for op_long in (True, False):
      for pcm in (True, False):
        CP, CP_SP = params(op_long, pcm)
        if set_speed_management_engaged(CP, CP_SP, False, True):
          changed.add((op_long, pcm))
    self.assertEqual(changed, {(False, False)})


class TestVCruiseSeeding(unittest.TestCase):
  """openpilot's set speed has to start from the car's when ICBM owns it. Left unseeded it sits
  at V_CRUISE_UNSET, Speed Limit Assist compares its target against that and never confirms."""

  def helper(self, pcm_cruise: bool, pcm_cruise_speed: bool):
    from openpilot.selfdrive.car.cruise import VCruiseHelper
    CP = structs.CarParams()
    CP.pcmCruise = pcm_cruise
    CP.openpilotLongitudinalControl = False
    CP_SP = structs.CarParamsSP()
    CP_SP.pcmCruiseSpeed = pcm_cruise_speed
    h = VCruiseHelper(CP, CP_SP)
    h.get_minimum_set_speed(is_metric=False)
    return h

  def car_state(self, set_speed_mph):
    from openpilot.common.constants import CV
    CS = car.CarState.new_message()
    CS.cruiseState.speed = set_speed_mph * CV.MPH_TO_MS
    return CS

  def test_seeds_from_the_car_when_icbm_owns_the_set_speed(self):
    from openpilot.common.constants import CV
    h = self.helper(pcm_cruise=True, pcm_cruise_speed=False)
    h.initialize_v_cruise(self.car_state(45), False, False)
    self.assertAlmostEqual(h.v_cruise_kph * CV.KPH_TO_MS * CV.MS_TO_MPH, 45, delta=0.6)
    self.assertEqual(h.v_cruise_kph, h.v_cruise_cluster_kph)

  def test_pcm_car_without_icbm_is_untouched(self):
    # Stock behaviour: the PCM owns the set speed, openpilot must not invent one
    h = self.helper(pcm_cruise=True, pcm_cruise_speed=True)
    before = h.v_cruise_kph
    h.initialize_v_cruise(self.car_state(45), False, False)
    self.assertEqual(h.v_cruise_kph, before)

  def test_ignores_a_car_reporting_no_set_speed(self):
    # Cruise off reports 0 (or 255 unset) -- seeding from that would be worse than not seeding
    h = self.helper(pcm_cruise=True, pcm_cruise_speed=False)
    before = h.v_cruise_kph
    h.initialize_v_cruise(self.car_state(0), False, False)
    self.assertEqual(h.v_cruise_kph, before)


if __name__ == "__main__":
  unittest.main()
