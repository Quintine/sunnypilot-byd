from opendbc.car.byd.values import CANBUS


def byd_checksum(data: bytearray) -> int:
  byte_key = 0xAF
  sum_first = sum(byte >> 4 for byte in data) # Extract upper nibble.
  sum_second = sum(byte & 0xF for byte in data) # Extract lower nibble.

  remainder = sum_second >> 4

  sum_first += (byte_key & 0xF) # Low nibble of byte_key.
  sum_second += (byte_key >> 4) # High nibble of byte_key.

  # Inline inverse computation for each sum:
  # inv = (-sum + 0x9) & 0xF
  inv_first = ((-sum_first + 0x9) & 0xF)
  inv_second = ((-sum_second + 0x9) & 0xF)

  return (((inv_first + (5 - remainder)) << 4) + inv_second) & 0xFF


def byd_checksum_short(data: bytearray) -> int:
  # checksum for CHECKSUM_S 4byte
  pass


class BydCAN:
  LKAS_MODE_ACTIVE2 = 3
  LKAS_MODE_PASSIVE = 1

  def __init__(self, packer):
    self.packer = packer
    self.mpc_lkas_counter = 0
    self.eps_steering_torque_counter = 0
    self.mpc_acc_cmd_counter = 0

  def update_mpc_lkas_counter(self, val):
    self.mpc_lkas_counter = val

  def _generate_mpc_lkas_new_counter(self):
    counter = self.mpc_lkas_counter
    self.mpc_lkas_counter = int(self.mpc_lkas_counter + 1) & 0xF
    return counter

  def update_eps_steering_torque_counter(self, val):
    self.eps_steering_torque_counter = val

  def _generate_eps_steering_torque_new_counter(self):
    counter = self.eps_steering_torque_counter
    self.eps_steering_torque_counter = int(
      self.eps_steering_torque_counter + 1) & 0xF
    return counter

  def update_acc_cmd_counter(self, val):
    self.mpc_acc_cmd_counter = val

  def _generate_acc_cmd_new_counter(self):
    counter = self.mpc_acc_cmd_counter
    self.mpc_acc_cmd_counter = int(self.mpc_acc_cmd_counter + 1) & 0xF
    return counter

  # MPC -> Panda -> EPS
  def create_steering_control_torque(self, mpc_lkas_cmd_msg, torque,
                                     request_prepare, active, mode,
                                     eps_active):
    lkas_output = torque if active and eps_active else 0
    lkas_prepare = request_prepare
    lkas_active = active
    lkas_mode = mode
    left_lane = mpc_lkas_cmd_msg["LeftLane"]
    right_lane = mpc_lkas_cmd_msg["RightLane"]
    if active or request_prepare:
      left_lane = 1
      right_lane = 1

    values = {
      "SETME_0x1": mpc_lkas_cmd_msg["SETME_0x1"],
      "LeftLane": left_lane,
      "Config": mpc_lkas_cmd_msg["Config"],
      "SETME2_0x1": mpc_lkas_cmd_msg["SETME2_0x1"],
      "Keep_Hands_On_Wheel": 0,
      "MPCErr": mpc_lkas_cmd_msg["MPCErr"],
      "SETME3_0x1": mpc_lkas_cmd_msg["SETME3_0x1"],
      "LKAS_Output": lkas_output,
      "LKASPrepare": lkas_prepare,
      "LKAS_ACTIVE": lkas_active,
      "TSRStatus": mpc_lkas_cmd_msg["TSRStatus"],
      "SETME4_0x1": mpc_lkas_cmd_msg["SETME4_0x1"],
      "RightLane": right_lane,
      "LKAS_Mode": lkas_mode,
      "SETME_0x0": mpc_lkas_cmd_msg["SETME_0x0"],
      "TSRResult": mpc_lkas_cmd_msg["TSRResult"],
      "Unknow1": mpc_lkas_cmd_msg["Unknow1"],
      "COUNTER": self._generate_mpc_lkas_new_counter(),
    }
    data = self.packer.make_can_msg("MPC_LKAS_CMD", CANBUS.main_bus, values)[1]
    values["CHECKSUM"] = byd_checksum(data)
    return self.packer.make_can_msg("MPC_LKAS_CMD", CANBUS.main_bus, values)

  def create_steering_torque(self, eps_steering_torque_msg, mpc_lkas_output,
                             mpc_lkas_request_prepare, mpc_lkas_active):
    lks_prepared = eps_steering_torque_msg["LKSPrepare"]
    cruise_activated = eps_steering_torque_msg["Cruise_Activated"]
    main_torque = eps_steering_torque_msg["MAIN_TORQUE"]

    if mpc_lkas_active:
      lks_prepared = 0
      cruise_activated = 1
      main_torque = mpc_lkas_output
    elif mpc_lkas_request_prepare:
      lks_prepared = 1
      cruise_activated = 0
      main_torque = 0
    else:
      lks_prepared = 0
      cruise_activated = 0
      main_torque = 0

    values = {
      "LKSPrepare": lks_prepared,
      "Cruise_Activated": cruise_activated,
      "TORQUE_FAILED": eps_steering_torque_msg["TORQUE_FAILED"],
      "SET_ME_1_1": eps_steering_torque_msg["SET_ME_1_1"],
      "Steer_Warning": eps_steering_torque_msg["Steer_Warning"],
      "Steer_Error_1": eps_steering_torque_msg["Steer_Error_1"],
      "Steer_Error_2": eps_steering_torque_msg["Steer_Error_2"],
      "SET_ME_1": eps_steering_torque_msg["SET_ME_1"],
      "MAIN_TORQUE": main_torque,
      "SET_ME_1_2": eps_steering_torque_msg["SET_ME_1_2"],
      "Keep_Hands_On_Wheel": 0,
      "SET_ME_3": eps_steering_torque_msg["SET_ME_3"],
      "Steer_Torque_Sensor": eps_steering_torque_msg["Steer_Torque_Sensor"],
      "SET_ME_XF_1": eps_steering_torque_msg["SET_ME_XF_1"],
      "SET_ME_XFFF": eps_steering_torque_msg["SET_ME_XFFF"],
      "COUNTER": self._generate_eps_steering_torque_new_counter(),
    }
    data = self.packer.make_can_msg("STEERING_TORQUE", CANBUS.cam_bus, values)[1]
    values["CHECKSUM"] = byd_checksum(data)
    return self.packer.make_can_msg("STEERING_TORQUE", CANBUS.cam_bus, values)

  def create_acc_cmd(self, acc_cmd_msg, accel):
    values = {
      "ACCEL_CMD": accel,
      "ComfortBandUpper": acc_cmd_msg["ComfortBandUpper"],
      "ComfortBandLower": acc_cmd_msg["ComfortBandLower"],
      "JerkUpperLimit": 1,
      "SET_ME_1": acc_cmd_msg["SET_ME_1"],
      "JerkLowerLimit": -1,
      "STANDSTILL_RESUME": 0,
      "STANDSTILL_STATE": 0,
      "BRAKE_BEHAVIOR": acc_cmd_msg["BRAKE_BEHAVIOR"],
      "ACC_REQ_NOT_STANDSTILL": acc_cmd_msg["ACC_REQ_NOT_STANDSTILL"],
      "ACC_CONTROLLABLE_AND_ON": acc_cmd_msg["ACC_CONTROLLABLE_AND_ON"],
      "ACC_OVERRIDE_OR_STANDSTILL": acc_cmd_msg["ACC_OVERRIDE_OR_STANDSTILL"],
      "ESP_BEHAVIOR": acc_cmd_msg["ESP_BEHAVIOR"],
      "COUNTER": self._generate_acc_cmd_new_counter(),
      "SET_ME_XF": acc_cmd_msg["SET_ME_XF"],
    }
    data = self.packer.make_can_msg("ACC_CMD", CANBUS.main_bus, values)[1]
    values["CHECKSUM"] = byd_checksum(data)
    return self.packer.make_can_msg("ACC_CMD", CANBUS.main_bus, values)
