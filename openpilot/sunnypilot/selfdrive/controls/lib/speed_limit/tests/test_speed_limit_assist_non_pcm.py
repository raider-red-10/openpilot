"""End-to-end confirm flow for the non-PCM-longitudinal path.

The existing suite forces openpilotLongitudinalControl = True, so
update_state_machine_non_pcm_long -- the path a stock-longitudinal car with ICBM takes --
had no coverage. Every Speed Limit Assist bug hit on the 2026 Palisade (LX3) lived in it.

This drives the state machine the way plannerd does, so the confirm handshake can be
verified without a vehicle.
"""

from openpilot.cereal import custom
from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.selfdrived.button_state_tracker import ButtonStateTracker
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import (
  SpeedLimitAssist, ACTIVE_STATES, DISABLED_GUARD_PERIOD, PRE_ACTIVE_GUARD_PERIOD)
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
ButtonType = car.CarState.ButtonEvent.Type

SPEED_LIMIT = 30 * CV.MPH_TO_MS      # what the camera reads
OFFSET = 4 * CV.MPH_TO_MS            # the user's fixed offset
TARGET = SPEED_LIMIT + OFFSET        # 34 mph -- what the set speed should become
START_SET_SPEED = 20 * CV.MPH_TO_MS  # where the driver's cruise is


def release_toggle(button_type):
  """The release-toggle bitmask selfdrived publishes after one button is pressed and released.

  Built with the real ButtonStateTracker so this exercises the actual carState -> bitmask ->
  SLA path, not an assumed bit layout."""
  tracker = ButtonStateTracker()
  for pressed in (True, False):
    CS = car.CarState.new_message()
    CS.buttonEvents = [car.CarState.ButtonEvent(pressed=pressed, type=button_type)]
    tracker.update(CS)
  return tracker.release_toggle


class TestSpeedLimitAssistNonPcm:
  """The Palisade's configuration: openpilot does not drive longitudinal, ICBM owns the set
  speed, so pcm_op_long is False."""

  def setup_method(self, method):
    self.params = Params()
    self.params.put("IsReleaseSpBranch", True, block=True)
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    self.params.put_bool("IsMetric", False, block=True)
    self.events_sp = EventsSP()

    CP = structs.CarParams()
    CP.openpilotLongitudinalControl = False
    CP.pcmCruise = True
    CP_SP = structs.CarParamsSP()
    CP_SP.pcmCruiseSpeed = False  # ICBM owns the set speed

    self.sla = SpeedLimitAssist(CP, CP_SP)
    assert not self.sla.pcm_op_long, "this test is meaningless on the pcm_op_long path"
    assert self.sla.enabled, "SpeedLimitMode did not stick -- availability reset it"

  def step(self, v_cruise=START_SET_SPEED, has_limit=True, long_enabled=True, n=1):
    for _ in range(n):
      self.sla.update(long_enabled, False, 25 * CV.MPH_TO_MS, 0.0, v_cruise,
                      SPEED_LIMIT if has_limit else 0.0, TARGET if has_limit else 0.0,
                      has_limit, 0.0, self.events_sp)

  def reach_pre_active(self):
    """Drive the machine from disabled to preActive the way a real engagement does."""
    self.step(long_enabled=False)                                  # start disengaged
    self.step()                                                    # rising edge arms the guard
    self.step(n=int(DISABLED_GUARD_PERIOD / DT_MDL) + 2)           # let the guard expire
    return self.sla.state

  def test_reaches_pre_active_with_a_speed_limit(self):
    assert self.reach_pre_active() == SpeedLimitAssistState.preActive

  def test_confirm_press_activates(self):
    """The whole point: one press of + while preActive must confirm."""
    assert self.reach_pre_active() == SpeedLimitAssistState.preActive

    self.sla.update_buttons(release_toggle(ButtonType.accelCruise))
    self.step()

    assert self.sla.state in ACTIVE_STATES, \
      f"confirm press did not activate, stuck in {self.sla.state}"

  def test_wrong_direction_press_does_not_confirm(self):
    # Set speed is below the target, so only a + press is a valid confirmation
    assert self.reach_pre_active() == SpeedLimitAssistState.preActive
    self.sla.update_buttons(release_toggle(ButtonType.decelCruise))
    self.step()
    assert self.sla.state not in ACTIVE_STATES

  def test_confirms_without_a_press_when_set_speed_already_matches(self):
    # Nothing to confirm -- the set speed is already the target
    self.step(v_cruise=TARGET, long_enabled=False)
    self.step(v_cruise=TARGET)
    self.step(v_cruise=TARGET, n=int(DISABLED_GUARD_PERIOD / DT_MDL) + 2)
    assert self.sla.state in ACTIVE_STATES

  def test_times_out_without_a_press(self):
    assert self.reach_pre_active() == SpeedLimitAssistState.preActive
    self.step(n=int(PRE_ACTIVE_GUARD_PERIOD[False] / DT_MDL) + 2)
    assert self.sla.state == SpeedLimitAssistState.inactive

  def test_stale_press_does_not_confirm(self):
    """The release window is 0.5s; a press from long ago must not count."""
    import time
    assert self.reach_pre_active() == SpeedLimitAssistState.preActive
    self.sla.update_buttons(release_toggle(ButtonType.accelCruise))
    self.sla._plus_hold = time.monotonic() - 1.0  # expire it
    self.step()
    assert self.sla.state not in ACTIVE_STATES

