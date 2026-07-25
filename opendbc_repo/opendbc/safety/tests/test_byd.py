#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

MSG_MPC_LKAS_CMD = 790       # TX by OP, lateral actuation towards EPS
MSG_STEERING_TORQUE = 792    # RX from EPS / TX by OP, torque feedback towards camera
MSG_ACC_HUD_ADAS = 813       # RX from camera, ACC state
MSG_ACC_CMD = 814            # TX by OP, longitudinal actuation
MSG_PCM_BUTTONS = 944        # RX from PCM, steering wheel buttons


def byd_checksum(msg):
  addr, dat, bus = msg
  ret = bytearray(dat)

  if addr in (MSG_MPC_LKAS_CMD, MSG_STEERING_TORQUE, MSG_ACC_CMD):
    byte_key = 0xAF
    sum_first = sum(byte >> 4 for byte in ret)  # upper nibbles
    sum_second = sum(byte & 0xF for byte in ret)  # lower nibbles

    remainder = sum_second >> 4

    sum_first += (byte_key & 0xF)
    sum_second += (byte_key >> 4)

    inv_first = ((-sum_first + 0x9) & 0xF)
    inv_second = ((-sum_second + 0x9) & 0xF)

    ret[7] = (((inv_first + (5 - remainder)) << 4) + inv_second) & 0xFF

  return addr, ret, bus


class TestBydSafetyBase(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [[MSG_MPC_LKAS_CMD, 0], [MSG_STEERING_TORQUE, 2]]
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_MPC_LKAS_CMD,), 2: (MSG_STEERING_TORQUE,)}
  FWD_BLACKLISTED_ADDRS = {0: [MSG_STEERING_TORQUE], 2: [MSG_MPC_LKAS_CMD]}

  MAX_TORQUE_LOOKUP = ([0], [300])
  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 6
  MAX_RT_DELTA = 112
  MAX_TORQUE_ERROR = 46
  TORQUE_MEAS_TOLERANCE = 1

  def setUp(self):
    self.packer = CANPackerSafety("byd_common")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 0)
    self.safety.init_tests()

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_ACTIVE": steer_req}
    return self.packer.make_can_msg_safety("MPC_LKAS_CMD", 0, values, byd_checksum)

  def _torque_meas_msg(self, torque):
    values = {"MAIN_TORQUE": torque}
    return self.packer.make_can_msg_safety("STEERING_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"Steer_Torque_Sensor": torque}
    return self.packer.make_can_msg_safety("STEERING_TORQUE", 0, values)

  def _speed_msg(self, speed):
    values = {"VEHICLE_SPEED": speed}
    return self.packer.make_can_msg_safety("ESC", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL": brake}
    return self.packer.make_can_msg_safety("PEDAL", 0, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": gas}
    return self.packer.make_can_msg_safety("PEDAL", 0, values)

  def _pcm_status_msg(self, enable):
    # CRUISE_STATE: 3 == Active, 0 == Off
    values = {"CRUISE_STATE": 3 if enable else 0}
    return self.packer.make_can_msg_safety("ACC_HUD_ADAS", 2, values)

  def _lkas_button_msg(self, enabled):
    values = {"LKAS_ON_BTN": enabled}
    return self.packer.make_can_msg_safety("PCM_BUTTONS", 0, values)

  def _acc_state_msg(self, enable):
    # BYD has no separate ACC main switch state in panda; MADS is toggled
    # with the dedicated LKAS steering wheel button instead
    raise NotImplementedError


class TestBydStockSafety(TestBydSafetyBase):
  pass


class TestBydLongitudinalSafety(TestBydSafetyBase):
  TX_MSGS = [[MSG_MPC_LKAS_CMD, 0], [MSG_STEERING_TORQUE, 2], [MSG_ACC_CMD, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_MPC_LKAS_CMD, MSG_ACC_CMD), 2: (MSG_STEERING_TORQUE,)}
  FWD_BLACKLISTED_ADDRS = {0: [MSG_STEERING_TORQUE], 2: [MSG_MPC_LKAS_CMD, MSG_ACC_CMD]}

  MIN_ACCEL = 20    # -4.0 m/s^2, raw = (m/s^2 + 5) / 0.05
  MAX_ACCEL = 140   # 2.0 m/s^2
  INACTIVE_ACCEL = 100  # 0 m/s^2

  def setUp(self):
    self.packer = CANPackerSafety("byd_common")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 1)  # BydSafetyFlags.LONG_CONTROL
    self.safety.init_tests()

  def _accel_msg(self, accel: float):
    # accel in m/s^2, ACCEL_CMD raw = (m/s^2 + 5) / 0.05
    values = {"ACCEL_CMD": accel}
    return self.packer.make_can_msg_safety("ACC_CMD", 0, values, byd_checksum)

  def test_accel_actuation_limits(self):
    self.safety.set_controls_allowed(True)
    for accel in [self.MIN_ACCEL - 1, self.MIN_ACCEL, 100, self.MAX_ACCEL, self.MAX_ACCEL + 1]:
      send = self.MIN_ACCEL <= accel <= self.MAX_ACCEL
      self.assertEqual(send, self._tx(self._accel_raw_msg(accel)), f"{accel=}")

  def _accel_raw_msg(self, raw: int):
    msg = self._accel_msg(0)
    msg[0].data[0] = raw & 0xFF
    return msg

  def test_accel_when_not_allowed(self):
    # when controls are not allowed, only 0 m/s^2 (inactive) may be sent
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._accel_raw_msg(self.INACTIVE_ACCEL)))
    self.assertFalse(self._tx(self._accel_raw_msg(self.INACTIVE_ACCEL + 5)))
    self.assertFalse(self._tx(self._accel_raw_msg(self.INACTIVE_ACCEL - 5)))


if __name__ == "__main__":
  unittest.main()
