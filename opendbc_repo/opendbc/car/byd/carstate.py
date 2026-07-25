import copy
from opendbc.car import Bus, create_button_events, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, CANBUS, STEER_THRESHOLD, GEAR_MAP
from opendbc.sunnypilot.car.byd.mads import MadsCarState

GearShifter = structs.CarState.GearShifter
ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase, MadsCarState):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)

    self.mpc_lkas_cmd_msg = None
    self.eps_steering_torque_msg = None
    self.eps_prepared = False
    self.eps_activated = False
    self.acc_cmd_msg = None

    self.mpc_lkas_output = 0
    self.mpc_lkas_active = False
    self.mpc_lkas_request_prepare = False

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
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
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]["Steer_Torque_Sensor"]
    ret.steeringTorqueEps = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"]
    # TODO: looks like STEER_THRESHOLD is too small
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)
    ret.steerFaultPermanent = cp.vl["STEERING_TORQUE"]["TORQUE_FAILED"] != 0
    ret.steerFaultTemporary = cp.vl["STEERING_TORQUE"]["TORQUE_FAILED"] != 0

    ret.stockAeb = cp_cam.vl["ACC_HUD_ADAS"]["AEB"] == 1
    ret.stockFcw = cp_cam.vl["ACC_HUD_ADAS"]["FCW"] == 1  # Forward Collision Warning

    # Cruise state
    ret.cruiseState.enabled = cp_cam.vl["ACC_HUD_ADAS"]["CRUISE_STATE"] in (3, 5)  # (Active, Override)
    ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SET_SPEED"] * 10 * CV.KPH_TO_MS
    ret.cruiseState.available = cp_cam.vl["ACC_HUD_ADAS"]["CRUISE_STATE"] not in (8, 9)  # (Failure, PermanentFailure)
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
    self.eps_prepared = cp.vl["STEERING_TORQUE"]["LKSPrepare"] != 0
    self.eps_activated = cp.vl["STEERING_TORQUE"]["Cruise_Activated"] != 0
    # for generate STEERING_TORQUE
    self.eps_steering_torque_msg = copy.copy(cp.vl["STEERING_TORQUE"])
    self.mpc_lkas_output = cp_cam.vl["MPC_LKAS_CMD"]["LKAS_Output"]
    self.mpc_lkas_active = cp_cam.vl["MPC_LKAS_CMD"]["LKAS_ACTIVE"] != 0
    self.mpc_lkas_request_prepare = cp_cam.vl["MPC_LKAS_CMD"]["LKASPrepare"] != 0
    # for generate ACC_CMD
    self.acc_cmd_msg = copy.copy(cp_cam.vl["ACC_CMD"])
    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
      # sig_address, frequency
      ("ESC", 50),
      ("BRAKE_APPLIED", 50),
      ("PEDAL", 50),
      ("STEER_MODULE", 100),
      ("STEERING_TORQUE", 50),
      ("DRIVE_STATE", 50),
      ("STALKS", 20),
      ("METER_CLUSTER", 20),
      ("PCM_BUTTONS", 20),
    ]

    cam_messages = [
      ("ACC_HUD_ADAS", 50),
      ("MPC_LKAS_CMD", 50),
      ("ACC_CMD", 25),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.main_bus),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CANBUS.cam_bus),
    }
