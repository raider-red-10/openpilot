"""Full Speed Limit Assist chain, simulated end to end.

Wires the four pieces that have to agree for the set speed to follow a speed limit on a car
where openpilot does not drive longitudinal:

  VCruiseHelper  -- openpilot's set speed        (card.py)
  SpeedLimitAssist -- the state machine          (plannerd)
  planner target -- min() over cruise and SLA    (plannerd)
  ICBM           -- picks a button to press      (carcontroller)
  SimCar         -- the car, which owns the real set speed and responds to buttons

Each of those was verified in isolation and the feature still did not work, because the two
halves disagree about what "the set speed" is: SLA reads openpilot's (CS.vCruiseCluster)
while ICBM reads the car's (CS.cruiseState.speedCluster). On a car openpilot drives they are
the same number. Here they are not, so the loop has to be simulated as a loop.

Configuration is the 2026 Palisade (LX3): stock longitudinal, ICBM owns the set speed,
openpilot engaged laterally only so carControl.enabled stays False the whole time.
"""
import unittest

from openpilot.cereal import custom, messaging
from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import \
  IntelligentCruiseButtonManagement
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import set_speed_management_engaged
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

ButtonType = car.CarState.ButtonEvent.Type
# The controller emits custom.* (capnp) values, not the opendbc StrEnum of the same name.
# Comparing across the two silently never matches -- every button reads as 'not increase'.
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

PLANNER_EVERY = 5              # plannerd runs at 20Hz against card's 100Hz
BUTTON_PERIOD = int(0.25 / DT_CTRL)  # ICBM re-sends at ~4Hz; the car sees one step per burst

SPEED_LIMIT_MPH = 30
OFFSET_MPH = 4
TARGET_MPH = SPEED_LIMIT_MPH + OFFSET_MPH   # 34
START_SET_MPH = 20


class SimCar:
  """The car's own cruise: it owns the set speed and moves it one step per button burst."""

  def __init__(self, set_speed_mph):
    self.set_speed_mph = float(set_speed_mph)
    self.enabled = True
    self.frames_since_button = BUTTON_PERIOD

  def apply(self, send_button):
    self.frames_since_button += 1
    if send_button == SendButtonState.none or self.frames_since_button < BUTTON_PERIOD:
      return
    self.frames_since_button = 0
    self.set_speed_mph += 1 if send_button == SendButtonState.increase else -1

  def car_state(self, v_ego_mph, button_events=()):
    CS = car.CarState.new_message()
    CS.vEgo = v_ego_mph * CV.MPH_TO_MS
    CS.cruiseState.available = True
    CS.cruiseState.enabled = self.enabled
    CS.cruiseState.speed = self.set_speed_mph * CV.MPH_TO_MS
    CS.cruiseState.speedCluster = self.set_speed_mph * CV.MPH_TO_MS
    CS.buttonEvents = list(button_events)
    return CS


class Chain:
  def __init__(self, set_speed_mph=START_SET_MPH):
    params = Params()
    params.put("IsReleaseSpBranch", True, block=True)
    params.put("SpeedLimitMode", int(Mode.assist), block=True)
    params.put_bool("IsMetric", False, block=True)

    self.CP = structs.CarParams()
    self.CP.openpilotLongitudinalControl = False
    self.CP.pcmCruise = True
    self.CP_SP = structs.CarParamsSP()
    self.CP_SP.pcmCruiseSpeed = False   # ICBM owns the set speed

    self.car = SimCar(set_speed_mph)
    self.v_cruise = VCruiseHelper(self.CP, self.CP_SP)
    self.sla = SpeedLimitAssist(self.CP, self.CP_SP)
    self.icbm = IntelligentCruiseButtonManagement(self.CP, self.CP_SP)
    self.events_sp = EventsSP()

    self.has_limit = False
    self.frame = 0
    self.send_button = SendButtonState.none

    # openpilot is engaged laterally only -- this never becomes True on a MADS car
    self.CC = car.CarControl.new_message()
    self.CC.enabled = False
    self.CC.latActive = True

  def _plan_sp(self, v_target_ms):
    """The longitudinalPlanSP the planner would publish this tick."""
    msg = messaging.new_message('longitudinalPlanSP')
    lp = msg.longitudinalPlanSP
    lp.vTarget = float(v_target_ms)
    limit = SPEED_LIMIT_MPH * CV.MPH_TO_MS if self.has_limit else 0.0
    final = TARGET_MPH * CV.MPH_TO_MS if self.has_limit else 0.0
    r = lp.speedLimit.resolver
    r.speedLimit, r.speedLimitFinal = limit, final
    r.speedLimitLast, r.speedLimitFinalLast = limit, final
    r.speedLimitValid = r.speedLimitLastValid = self.has_limit
    lp.speedLimit.assist.state = self.sla.state
    return lp

  def step(self, v_ego_mph=25, button_events=(), n=1):
    for _ in range(n):
      self.frame += 1
      CS = self.car.car_state(v_ego_mph, button_events)
      button_events = ()  # a press is one frame, not a level

      lp = self._plan_sp(self.sla.output_v_target)

      # --- card.py ---
      self.v_cruise.update_speed_limit_assist(False, lp)
      enabled = set_speed_management_engaged(self.CP, self.CP_SP, self.CC.enabled, CS.cruiseState.enabled)
      self.v_cruise.update_v_cruise(CS, enabled, False)
      CS.vCruise = float(self.v_cruise.v_cruise_kph)
      CS.vCruiseCluster = float(self.v_cruise.v_cruise_cluster_kph)

      # --- plannerd ---
      # update_car_state runs every iteration (polled on carState, 100Hz) so transient button
      # releases land in the 0.5s hold window; the state machine only runs on modelV2 (20Hz).
      self.sla.update_car_state(CS)
      if self.frame % PLANNER_EVERY == 0:
        self.sla.update(enabled, False, CS.vEgo, 0.0,
                        min(CS.vCruiseCluster, 145) * CV.KPH_TO_MS,
                        SPEED_LIMIT_MPH * CV.MPH_TO_MS if self.has_limit else 0.0,
                        TARGET_MPH * CV.MPH_TO_MS if self.has_limit else 0.0,
                        self.has_limit, 0.0, self.events_sp)

      # planner picks the lowest target across sources
      v_target = min(self.v_cruise.v_cruise_kph * CV.KPH_TO_MS, self.sla.output_v_target)

      # --- ICBM ---
      self.icbm.run(CS, self.CC, self._plan_sp(v_target), False)
      self.send_button = self.icbm.cruise_button
      self.car.apply(self.send_button)

  @property
  def car_set_mph(self):
    return round(self.car.set_speed_mph)

  @property
  def op_set_mph(self):
    return round(self.v_cruise.v_cruise_cluster_kph * CV.KPH_TO_MS * CV.MS_TO_MPH)


def plus_press():
  return [car.CarState.ButtonEvent(pressed=False, type=ButtonType.accelCruise)]


class TestSpeedLimitAssistChain(unittest.TestCase):

  def test_openpilot_set_speed_tracks_the_car_before_any_limit(self):
    """Nothing should move while there is no speed limit to act on."""
    c = Chain()
    c.step(n=200)
    self.assertEqual(c.op_set_mph, START_SET_MPH,
                     f"openpilot invented a set speed: {c.op_set_mph} vs car {c.car_set_mph}")
    self.assertEqual(c.car_set_mph, START_SET_MPH, "ICBM moved the set speed with no limit present")
    self.assertEqual(c.send_button, SendButtonState.none)

  def test_assist_arms_when_a_limit_appears(self):
    c = Chain()
    c.step(n=100)
    c.has_limit = True
    c.step(n=200)
    self.assertEqual(c.sla.state, SpeedLimitAssistState.preActive,
                     f"assist did not arm, sat in {c.sla.state}")

  def test_confirm_press_walks_the_car_to_the_limit(self):
    """The whole feature: press + once and the car's set speed ends at limit + offset."""
    c = Chain()
    c.step(n=100)
    c.has_limit = True
    c.step(n=100)
    self.assertEqual(c.sla.state, SpeedLimitAssistState.preActive)

    c.step(button_events=plus_press())
    c.step(n=1500)  # ICBM needs time to walk 20 -> 34 one press at a time

    self.assertEqual(c.car_set_mph, TARGET_MPH,
                     f"car set speed ended at {c.car_set_mph}, expected {TARGET_MPH} "
                     f"(openpilot's was {c.op_set_mph}, assist {c.sla.state})")

  def test_does_not_overshoot(self):
    c = Chain()
    c.step(n=100)
    c.has_limit = True
    c.step(n=100)
    c.step(button_events=plus_press())
    c.step(n=1500)
    settled = c.car_set_mph
    c.step(n=500)
    self.assertEqual(c.car_set_mph, settled, "set speed kept moving after reaching the target")

  def test_never_targets_max_speed(self):
    """The measured failure: opSet pinned at 90 mph (V_CRUISE_MAX) with btn=increase held."""
    c = Chain()
    c.step(n=300)
    self.assertLess(c.op_set_mph, 60, f"openpilot's set speed ran away to {c.op_set_mph}")
    self.assertLess(c.car_set_mph, 60, f"ICBM dragged the car to {c.car_set_mph}")


if __name__ == "__main__":
  unittest.main()
