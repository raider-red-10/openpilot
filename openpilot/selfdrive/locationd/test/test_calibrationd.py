import random

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.selfdrive.locationd.calibrationd import Calibrator, INPUTS_NEEDED, INPUTS_WANTED, BLOCK_SIZE, MIN_SPEED_FILTER, \
                                                         MAX_YAW_RATE_FILTER, SMOOTH_CYCLES, HEIGHT_INIT, MAX_ALLOWED_PITCH_SPREAD, MAX_ALLOWED_YAW_SPREAD


def process_messages(c, cam_odo_calib, cycles,
                     cam_odo_speed=MIN_SPEED_FILTER + 1,
                     carstate_speed=MIN_SPEED_FILTER + 1,
                     cam_odo_yr=0.0,
                     cam_odo_speed_std=1e-3,
                     cam_odo_height_std=1e-3):
  old_rpy_weight_prev = 0.0
  for _ in range(cycles):
    assert (old_rpy_weight_prev - c.old_rpy_weight < 1/SMOOTH_CYCLES + 1e-3)
    old_rpy_weight_prev = c.old_rpy_weight
    c.handle_v_ego(carstate_speed)
    c.handle_cam_odom([cam_odo_speed,
                       np.sin(cam_odo_calib[2]) * cam_odo_speed,
                       -np.sin(cam_odo_calib[1]) * cam_odo_speed],
                        [0.0, 0.0, cam_odo_yr],
                        [0.0, 0.0, 0.0],
                        [cam_odo_speed_std, cam_odo_speed_std, cam_odo_speed_std],
                        [0.0, 0.0, HEIGHT_INIT.item()],
                        [cam_odo_height_std, cam_odo_height_std, cam_odo_height_std])

class TestCalibrationd:

  def test_read_saved_params(self):
    msg = messaging.new_message('liveCalibration')
    msg.liveCalibration.validBlocks = random.randint(1, 10)
    msg.liveCalibration.rpyCalib = [random.random() for _ in range(3)]
    msg.liveCalibration.height = [random.random() for _ in range(1)]
    Params().put("CalibrationParams", msg.to_bytes(), block=True)
    c = Calibrator(param_put=True)

    np.testing.assert_allclose(msg.liveCalibration.rpyCalib, c.rpy)
    np.testing.assert_allclose(msg.liveCalibration.height, c.height)
    assert msg.liveCalibration.validBlocks == c.valid_blocks


  def test_calibration_basics(self):
    c = Calibrator(param_put=False)
    process_messages(c, [0.0, 0.0, 0.0], BLOCK_SIZE * INPUTS_WANTED)
    assert c.valid_blocks == INPUTS_WANTED
    np.testing.assert_allclose(c.rpy, np.zeros(3))
    np.testing.assert_allclose(c.height, HEIGHT_INIT)
    c.reset()


  def test_calibration_low_speed_reject(self):
    c = Calibrator(param_put=False)
    process_messages(c, [0.0, 0.0, 0.0], BLOCK_SIZE * INPUTS_WANTED, cam_odo_speed=MIN_SPEED_FILTER - 1)
    process_messages(c, [0.0, 0.0, 0.0], BLOCK_SIZE * INPUTS_WANTED, carstate_speed=MIN_SPEED_FILTER - 1)
    assert c.valid_blocks == 0
    np.testing.assert_allclose(c.rpy, np.zeros(3))
    np.testing.assert_allclose(c.height, HEIGHT_INIT)


  def test_calibration_yaw_rate_reject(self):
    c = Calibrator(param_put=False)
    process_messages(c, [0.0, 0.0, 0.0], BLOCK_SIZE * INPUTS_WANTED, cam_odo_yr=MAX_YAW_RATE_FILTER)
    assert c.valid_blocks == 0
    np.testing.assert_allclose(c.rpy, np.zeros(3))
    np.testing.assert_allclose(c.height, HEIGHT_INIT)


  def test_calibration_speed_std_reject(self):
    c = Calibrator(param_put=False)
    process_messages(c, [0.0, 0.0, 0.0], BLOCK_SIZE * INPUTS_WANTED, cam_odo_speed_std=1e3)
    assert c.valid_blocks == INPUTS_NEEDED
    np.testing.assert_allclose(c.rpy, np.zeros(3))


  def test_calibration_speed_std_height_reject(self):
    c = Calibrator(param_put=False)
    process_messages(c, [0.0, 0.0, 0.0], BLOCK_SIZE * INPUTS_WANTED, cam_odo_height_std=1e3)
    assert c.valid_blocks == INPUTS_NEEDED
    np.testing.assert_allclose(c.rpy, np.zeros(3))


  def test_calibration_auto_reset(self):
    c = Calibrator(param_put=False)
    process_messages(c, [0.0, 0.0, 0.0], BLOCK_SIZE * INPUTS_NEEDED)
    assert c.valid_blocks == INPUTS_NEEDED
    np.testing.assert_allclose(c.rpy, [0.0, 0.0, 0.0], atol=1e-3)
    process_messages(c, [0.0, MAX_ALLOWED_PITCH_SPREAD*0.9, MAX_ALLOWED_YAW_SPREAD*0.9], BLOCK_SIZE + 10)
    assert c.valid_blocks == INPUTS_NEEDED + 1
    assert c.cal_status == log.LiveCalibrationData.Status.calibrated

    c = Calibrator(param_put=False)
    process_messages(c, [0.0, 0.0, 0.0], BLOCK_SIZE * INPUTS_NEEDED)
    assert c.valid_blocks == INPUTS_NEEDED
    np.testing.assert_allclose(c.rpy, [0.0, 0.0, 0.0])
    process_messages(c, [0.0, MAX_ALLOWED_PITCH_SPREAD*1.1, 0.0], BLOCK_SIZE + 10)
    assert c.valid_blocks == 1
    assert c.cal_status == log.LiveCalibrationData.Status.recalibrating
    np.testing.assert_allclose(c.rpy, [0.0, MAX_ALLOWED_PITCH_SPREAD*1.1, 0.0], atol=1e-2)

    c = Calibrator(param_put=False)
    process_messages(c, [0.0, 0.0, 0.0], BLOCK_SIZE * INPUTS_NEEDED)
    assert c.valid_blocks == INPUTS_NEEDED
    np.testing.assert_allclose(c.rpy, [0.0, 0.0, 0.0])
    process_messages(c, [0.0, 0.0, MAX_ALLOWED_YAW_SPREAD*1.1], BLOCK_SIZE + 10)
    assert c.valid_blocks == 1
    assert c.cal_status == log.LiveCalibrationData.Status.recalibrating
    np.testing.assert_allclose(c.rpy, [0.0, 0.0, MAX_ALLOWED_YAW_SPREAD*1.1], atol=1e-2)


class TestCalibrationCarChange:
  """Camera calibration is car-specific. On a car change, calibrationd archives the outgoing
  car's calibration under its fingerprint and restores the incoming car's if one exists -- a
  multi-car device relearns each car once, not on every swap. paramsd, torqued, and lagd
  already discard their learned state on a fingerprint change; calibration is archived rather
  than discarded because relearning it costs minutes of driving and gates engagement."""

  def _cal_msg(self, blocks):
    msg = messaging.new_message('liveCalibration')
    msg.liveCalibration.validBlocks = blocks
    return msg.to_bytes()

  def _set_state(self, prev_fingerprint, cal=None, archive=None):
    from opendbc.car.structs import car
    params = Params()
    if cal is None:
      params.remove("CalibrationParams")
    else:
      params.put("CalibrationParams", cal, block=True)
    if archive is None:
      params.remove("CalibrationParamsByCar")
    else:
      params.put("CalibrationParamsByCar", archive, block=True)
    if prev_fingerprint is None:
      params.remove("CarParamsPrevRoute")
    else:
      params.put("CarParamsPrevRoute", car.CarParams(carFingerprint=prev_fingerprint).to_bytes(), block=True)
    return params

  def test_reset_and_archive_when_no_archive_for_new_car(self):
    import base64
    from opendbc.car.structs import car
    from openpilot.selfdrive.locationd.calibrationd import swap_calibration_on_car_change
    cal_a = self._cal_msg(50)
    params = self._set_state("CAR_A", cal=cal_a)
    swap_calibration_on_car_change(params, car.CarParams(carFingerprint="CAR_B"))
    assert params.get("CalibrationParams") is None
    archive = params.get("CalibrationParamsByCar")
    assert base64.b64decode(archive["CAR_A"]) == cal_a

  def test_restore_from_archive(self):
    import base64
    from opendbc.car.structs import car
    from openpilot.selfdrive.locationd.calibrationd import swap_calibration_on_car_change
    cal_a, cal_b = self._cal_msg(15), self._cal_msg(50)
    params = self._set_state("CAR_A", cal=cal_a, archive={"CAR_B": base64.b64encode(cal_b).decode()})
    swap_calibration_on_car_change(params, car.CarParams(carFingerprint="CAR_B"))
    assert params.get("CalibrationParams") == cal_b
    archive = params.get("CalibrationParamsByCar")
    assert base64.b64decode(archive["CAR_A"]) == cal_a

  def test_round_trip_swap(self):
    from opendbc.car.structs import car
    from openpilot.selfdrive.locationd.calibrationd import swap_calibration_on_car_change
    cal_a = self._cal_msg(50)
    params = self._set_state("CAR_A", cal=cal_a)
    swap_calibration_on_car_change(params, car.CarParams(carFingerprint="CAR_B"))
    assert params.get("CalibrationParams") is None

    cal_b = self._cal_msg(30)
    params.put("CalibrationParams", cal_b, block=True)
    params.put("CarParamsPrevRoute", car.CarParams(carFingerprint="CAR_B").to_bytes(), block=True)
    swap_calibration_on_car_change(params, car.CarParams(carFingerprint="CAR_A"))
    assert params.get("CalibrationParams") == cal_a

    import base64
    archive = params.get("CalibrationParamsByCar")
    assert base64.b64decode(archive["CAR_B"]) == cal_b
    params.remove("CarParamsPrevRoute")
    params.remove("CalibrationParamsByCar")

  def test_kept_on_same_car(self):
    from opendbc.car.structs import car
    from openpilot.selfdrive.locationd.calibrationd import swap_calibration_on_car_change
    params = self._set_state("CAR_A", cal=self._cal_msg(50))
    swap_calibration_on_car_change(params, car.CarParams(carFingerprint="CAR_A"))
    assert params.get("CalibrationParams") is not None
    assert params.get("CalibrationParamsByCar") is None
    params.remove("CarParamsPrevRoute")

  def test_kept_on_first_boot(self):
    from opendbc.car.structs import car
    from openpilot.selfdrive.locationd.calibrationd import swap_calibration_on_car_change
    params = self._set_state(None, cal=self._cal_msg(50))
    swap_calibration_on_car_change(params, car.CarParams(carFingerprint="CAR_A"))
    assert params.get("CalibrationParams") is not None
