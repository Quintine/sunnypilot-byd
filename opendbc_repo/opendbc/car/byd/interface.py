from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.values import CAR, BydFlags, BydSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"
    ret.dashcamOnly = False

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]

    # lateral control parameters
    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.5

    if candidate in (CAR.BYD_HAN_EV_23, CAR.BYD_SEAL_PERFORMANCE_25):
      ret.steerControlType = structs.CarParams.SteerControlType.torque

    ret.minEnableSpeed = -1.
    ret.minSteerSpeed = 0.1 * CV.KPH_TO_MS
    ret.steerLimitTimer = 0.5

    if ret.steerControlType == structs.CarParams.SteerControlType.torque:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # longitudinal control parameters
    # HAN: openpilot long available (ACC_CMD replaced on the tapped camera bus).
    # SEAL: stock ACC_CMD is broadcast directly on the vehicle CAN (bus 0),
    # so it cannot be intercepted/replaced by the panda — stock ACC only (MADS).
    ret.alphaLongitudinalAvailable = candidate != CAR.BYD_SEAL_PERFORMANCE_25
    if alpha_long and ret.alphaLongitudinalAvailable:
      ret.openpilotLongitudinalControl = True
      ret.startingState = True
      ret.startAccel = 0.1
      ret.flags |= BydFlags.LONG_CONTROL.value
      ret.safetyConfigs[0].safetyParam |= BydSafetyFlags.LONG_CONTROL.value
      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

    # TODO: add radar support
    ret.radarUnavailable = True

    return ret
