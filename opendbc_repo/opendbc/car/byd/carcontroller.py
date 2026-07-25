import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_meas_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd.bydcan import BydCAN
from opendbc.car.byd.values import CarControllerParams

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type
LongCtrlState = structs.CarControl.Actuators.LongControlState


class LKASState:
  INACTIVE = 0
  PREPARING = 1
  ACTIVE = 2


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.can = BydCAN(self.packer)
    self.apply_torque_last = 0

    self.lkas_state = LKASState.INACTIVE
    self.lkas_counter_updated = False

    self.apply_accel_last = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    # car control running in 100Hz
    actuators = CC.actuators
    can_sends = []

    if (self.frame % self.params.STEER_STEP) == 0:
      # steering control running in 50Hz
      if not self.lkas_counter_updated:
        self.lkas_counter_updated = True
        self.can.update_mpc_lkas_counter(
          int(CS.mpc_lkas_cmd_msg["COUNTER"] + 1) & 0xF)
        self.can.update_eps_steering_torque_counter(
          int(CS.eps_steering_torque_msg["COUNTER"] + 1) & 0xF)
        self.can.update_acc_cmd_counter(
          int(CS.acc_cmd_msg["COUNTER"] + 1) & 0xF)

      # update lkas state
      if self.lkas_state == LKASState.INACTIVE:
        if CC.latActive:
          self.lkas_state = LKASState.PREPARING
      elif self.lkas_state == LKASState.PREPARING:
        if CS.eps_prepared:
          self.lkas_state = LKASState.ACTIVE
      elif self.lkas_state == LKASState.ACTIVE:
        if not CC.latActive:
          self.lkas_state = LKASState.INACTIVE

      lkas_request_prepare = int(self.lkas_state == LKASState.PREPARING)
      lkas_active = int(self.lkas_state == LKASState.ACTIVE)
      if self.lkas_state in (LKASState.PREPARING, LKASState.ACTIVE):
        lkas_mode = BydCAN.LKAS_MODE_ACTIVE2
      else:
        lkas_mode = BydCAN.LKAS_MODE_PASSIVE

      # calc apply torque
      apply_torque = 0
      if self.lkas_state == LKASState.ACTIVE:
        apply_torque = int(round(actuators.torque * self.params.STEER_MAX))
        apply_torque = apply_meas_steer_torque_limits(apply_torque, self.apply_torque_last,
                                                      CS.out.steeringTorqueEps, self.params)

      pack = self.can.create_steering_control_torque(CS.mpc_lkas_cmd_msg, apply_torque,
                                                     lkas_request_prepare, lkas_active,
                                                     lkas_mode, CS.eps_activated)
      can_sends.append(pack)
      pack = self.can.create_steering_torque(CS.eps_steering_torque_msg,
                                             CS.mpc_lkas_output,
                                             CS.mpc_lkas_request_prepare,
                                             CS.mpc_lkas_active)
      can_sends.append(pack)

      self.apply_torque_last = apply_torque

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if (self.frame % self.params.ACC_SETP) == 0:
        accel = 0

        if CC.longActive:
          accel = float(np.clip(actuators.accel, self.params.ACCEL_MIN, self.params.ACCEL_MAX))

        pack = self.can.create_acc_cmd(CS.acc_cmd_msg, accel)
        can_sends.append(pack)
        self.apply_accel_last = accel

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = self.apply_accel_last

    self.frame += 1
    return new_actuators, can_sends
