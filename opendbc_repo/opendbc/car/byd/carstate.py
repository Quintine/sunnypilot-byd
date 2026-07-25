import copy
from opendbc.car import Bus, create_button_events, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import CAR, DBC, CANBUS, STEER_THRESHOLD, GEAR_MAP
from opendbc.sunnypilot.car.byd.mads import MadsCarState

GearShifter = structs.CarState.GearShifter
ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase, MadsCarState):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)

    # EPS steering feedback message differs by platform:
    # HAN uses 792 STEERING_TORQUE, SEAL uses 508 STEERING_TORQUE_ANGLE
    if CP.carFingerprint == CAR.BYD_SEAL_PERFORMANCE_25:
      self.steer_msg = "STEERING_TORQUE_ANGLE"
    else:
      self.steer_msg = "STEERING_TORQUE"

    # ADAS cruise state (ACC_HUD_ADAS) and stock ACC echo (ACC_CMD) are on
    # the vehicle CAN (bus 0) on the SEAL, camera CAN (bus 2) on the HAN
    self.adas_on_pt_bus = CP.carFingerprint == CAR.BYD_SEAL_PERFORMANCE_25

    self.mpc_lkas_cmd_msg = None
    self.eps_steering_torque_msg = None
    self.eps_prepared = False
    self.eps_activated = False
    self.acc_cmd_msg = None

    self.mpc_lkas_output = 0
    self.mpc_lkas_active = False
    self.mpc_lkas_request_prepare = False
    self.mpc_lkas_angle_output = 0.0

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_adas = cp if self.adas_on_pt_bus else cp_cam
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # car Speed
    ret.vEgoRaw = cp.vl["ESC"]["VEHICLE_SPEED"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = cp.vl["BRAKE_APPLIED"]["STANDSTILL"] == 1

    # Gas pedal
    ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] > 0

    # Brake pedal
    ret.brakePressed = cp.vl["PEDAL"]["BRAKE_PEDAL"] > 0

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["STEER_MODULE"]["STEER_ANGLE"]
    ret.steeringRateDeg = cp.vl["STEER_MODULE"]["STEERING_RATE"]
    ret.steeringTorque = cp.vl[self.steer_msg]["Steer_Torque_Sensor"]
    ret.steeringTorqueEps = cp.vl[self.steer_msg]["MAIN_TORQUE"]
    # TODO: looks like STEER_THRESHOLD is too small
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)
    ret.steerFaultPermanent = cp.vl[self.steer_msg]["TORQUE_FAILED"] != 0
    ret.steerFaultTemporary = cp.vl[self.steer_msg]["TORQUE_FAILED"] != 0

    ret.stockAeb = cp_adas.vl["ACC_HUD_ADAS"]["AEB"] == 1
    ret.stockFcw = cp_adas.vl["ACC_HUD_ADAS"]["FCW"] == 1  # Forward Collision Warning

    # Cruise state
    ret.cruiseState.enabled = cp_adas.vl["ACC_HUD_ADAS"]["CRUISE_STATE"] in (3, 5)  # (Active, Override)
    ret.cruiseState.speed = cp_adas.vl["ACC_HUD_ADAS"]["SET_SPEED"] * 10 * CV.KPH_TO_MS
    ret.cruiseState.available = cp_adas.vl["ACC_HUD_ADAS"]["CRUISE_STATE"] not in (8, 9)  # (Failure, PermanentFailure)
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special

    # Gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["DRIVE_STATE"]["GEAR"]), GearShifter.unknown)

    # MADS: track the dedicated LKAS steering wheel button
    MadsCarState.update_mads(self, ret, can_parsers)

    # button presses
    ret.leftBlinker = cp.vl["STALKS"]["TURN_SIGNAL_SWITCH"] == 3  # LeftSteeringLongLift
    ret.rightBlinker = cp.vl["STALKS"]["TURN_SIGNAL_SWITCH"] == 5  # RightSteeringLongLift

    ret.buttonEvents = [*create_button_events(self.lkas_button, self.prev_lkas_button, {1: ButtonType.lkas})]

    # lock info
    ret.doorOpen = any([cp.vl["METER_CLUSTER"]["FRONT_LEFT_DOOR"],
                        cp.vl["METER_CLUSTER"]["FRONT_RIGHT_DOOR"],
                        cp.vl["METER_CLUSTER"]["BACK_LEFT_DOOR"],
                        cp.vl["METER_CLUSTER"]["BACK_RIGHT_DOOR"]])
    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"] != 1

    # blindspot sensors
    # ret.leftBlindspot = False
    # ret.rightBlindspot = False

    # Messages needed by carcontroller
    # for generate MPC_LKAS_CMD
    self.mpc_lkas_cmd_msg = copy.copy(cp_cam.vl["MPC_LKAS_CMD"])
    self.eps_prepared = cp.vl[self.steer_msg]["LKSPrepare"] != 0
    self.eps_activated = cp.vl[self.steer_msg]["Cruise_Activated"] != 0
    # for generate steering feedback message
    self.eps_steering_torque_msg = copy.copy(cp.vl[self.steer_msg])
    self.mpc_lkas_output = cp_cam.vl["MPC_LKAS_CMD"]["LKAS_Output"]
    self.mpc_lkas_active = cp_cam.vl["MPC_LKAS_CMD"]["LKAS_ACTIVE"] != 0
    self.mpc_lkas_request_prepare = cp_cam.vl["MPC_LKAS_CMD"]["LKASPrepare"] != 0
    # camera's angle-based LKAS request (SEAL), used to spoof the 508 TARGET_ANGLE echo
    self.mpc_lkas_angle_output = cp_cam.vl["MPC_LKAS_CMD_ANGLE"]["LKAS_Output"]
    # for generate ACC_CMD
    self.acc_cmd_msg = copy.copy(cp_adas.vl["ACC_CMD"])
    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    # Frequencies are deliberately conservative (10x timeout forgiveness):
    # only messages required for control are alive-checked, everything
    # optional/state-dependent is parsed but not alive-checked (nan).
    pt_messages = [
      # sig_address, frequency
      ("ESC", 10),
      ("BRAKE_APPLIED", 10),
      ("PEDAL", 10),
      ("STEER_MODULE", 10),
      ("DRIVE_STATE", 10),
      ("STALKS", float('nan')),
      ("METER_CLUSTER", float('nan')),
      # steering wheel buttons; bus assignment is harness dependent
      ("PCM_BUTTONS", float('nan')),
    ]

    cam_messages = [
      # stock camera command echoes, only present when stock ADAS is active
      ("MPC_LKAS_CMD", float('nan')),
      # SEAL camera also broadcasts an angle-based LKAS command (parsed for future use)
      ("MPC_LKAS_CMD_ANGLE", float('nan')),
      # PCM_BUTTONS may be forwarded on the camera bus on some harnesses
      ("PCM_BUTTONS", float('nan')),
    ]

    # ADAS cruise state and stock ACC echo: vehicle CAN (bus 0) on the SEAL,
    # camera CAN (bus 2) on the HAN. Alive-check ACC_HUD_ADAS only on the bus it lives on.
    adas_messages = [("ACC_HUD_ADAS", 10), ("ACC_CMD", float('nan'))]
    if CP.carFingerprint == CAR.BYD_SEAL_PERFORMANCE_25:
      pt_messages.extend(adas_messages)
    else:
      cam_messages.extend(adas_messages)

    # EPS steering feedback message differs by platform; alive-check only
    # the one this car has, parse the other opportunistically
    if CP.carFingerprint == CAR.BYD_SEAL_PERFORMANCE_25:
      pt_messages.append(("STEERING_TORQUE_ANGLE", 10))
      pt_messages.append(("STEERING_TORQUE", float('nan')))
    else:
      pt_messages.append(("STEERING_TORQUE", 10))
      pt_messages.append(("STEERING_TORQUE_ANGLE", float('nan')))

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.main_bus),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CANBUS.cam_bus),
    }
