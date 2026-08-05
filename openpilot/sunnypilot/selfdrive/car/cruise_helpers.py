"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName

DISTANCE_LONG_PRESS = 50


def set_speed_management_engaged(CP: structs.CarParams, CP_SP: custom.CarParamsSP,
                                 op_enabled: bool, car_cruise_enabled: bool) -> bool:
  """Whether the set speed is ours to manage right now.

  ICBM moves the set speed by pressing cruise buttons, and Speed Limit Assist rides on top
  of it. Neither needs openpilot to hold longitudinal control -- they need the *car's*
  cruise to be running, so that changing the set speed does something.

  Gating them on openpilot's own `enabled` breaks that on a MADS car. MADS strips pcmEnable
  while lateral is already engaged, which is what keeps steering independent of cruise, so
  `enabled` never goes true and both features sit disabled with a valid speed limit next to
  them. Measured on a 2026 Palisade Hybrid: carCruise=1, ccEnabled=0, lat=1, a resolved
  30 mph limit, and the assist stuck in `disabled` for an entire drive.

  Only applies where ICBM owns the set speed -- pcmCruiseSpeed goes False exactly when ICBM
  is enabled and available on a car openpilot does not drive longitudinally. Everywhere
  else this is openpilot's `enabled`, unchanged.
  """
  if not CP.openpilotLongitudinalControl and not CP_SP.pcmCruiseSpeed:
    return op_enabled or car_cruise_enabled

  return op_enabled


class CruiseHelper:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.params = Params()

    self.button_frame_counts = {ButtonType.gapAdjustCruise: 0}
    self._experimental_mode = False
    self.experimental_mode_switched = False

  def update(self, CS, events, experimental_mode) -> None:
    if self.CP.openpilotLongitudinalControl:
      if CS.cruiseState.available:
        self.update_button_frame_counts(CS)

        # toggle experimental mode once on distance button hold
        self.update_experimental_mode(events, experimental_mode)

  def update_button_frame_counts(self, CS) -> None:
    for button in self.button_frame_counts:
      if self.button_frame_counts[button] > 0:
        self.button_frame_counts[button] += 1

    for button_event in CS.buttonEvents:
      button = button_event.type.raw
      if button in self.button_frame_counts:
        self.button_frame_counts[button] = int(button_event.pressed)

  def update_experimental_mode(self, events, experimental_mode) -> None:
    if self.button_frame_counts[ButtonType.gapAdjustCruise] >= DISTANCE_LONG_PRESS and not self.experimental_mode_switched:
      self._experimental_mode = not experimental_mode
      self.params.put_bool("ExperimentalMode", self._experimental_mode)
      events.add(EventNameSP.experimentalModeSwitched)
      self.experimental_mode_switched = True
