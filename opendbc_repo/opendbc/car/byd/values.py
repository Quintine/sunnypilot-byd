from dataclasses import dataclass, field
from enum import IntFlag
from opendbc.car import Bus, DbcDict, PlatformConfig, Platforms, CarSpecs, STD_CARGO_KG
from opendbc.car.structs import CarParams, CarState
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts

Ecu = CarParams.Ecu


class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2


class BydSafetyFlags(IntFlag):
  LONG_CONTROL = 1


class BydFlags(IntFlag):
  LONG_CONTROL = 1


@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  #todo add docs and harness info


@dataclass
class BydPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "byd_common"})


class CAR(Platforms):
  BYD_HAN_EV_23 = BydPlatformConfig(
    [BydCarDocs("BYD HAN EV 2023")],
    CarSpecs(mass=1940 + STD_CARGO_KG, wheelbase=2.92, steerRatio=16.5,
             centerToFrontRatio=0.44, tireStiffnessFactor=1.0)
  )
  BYD_SEAL_PERFORMANCE_25 = BydPlatformConfig(
    [BydCarDocs("BYD SEAL PERFORMANCE 2025")],
    # TODO: verify steerRatio and centerToFrontRatio on a real car
    CarSpecs(mass=2100 + STD_CARGO_KG, wheelbase=2.92, steerRatio=16.0,
             centerToFrontRatio=0.44, tireStiffnessFactor=1.0)
  )


class CarControllerParams:
  def __init__(self, CP):
    self.STEER_STEP = 2  # 50Hz
    self.ACC_SETP = 4  # 25Hz

    self.STEER_ERROR_MAX = 46
    if CP.carFingerprint in (CAR.BYD_HAN_EV_23, CAR.BYD_SEAL_PERFORMANCE_25):
      self.STEER_MAX = 300
      self.STEER_DELTA_UP = 6
      self.STEER_DELTA_DOWN = 6

    self.ACCEL_MAX = 2.0  # m/s^2
    self.ACCEL_MIN = -4.0  # m/s^2
    self.JERK_LIMIT_MAX = 4.0  # m/s^3
    self.JERK_LIMIT_MIN = -4.0  # m/s^3


STEER_THRESHOLD = 1

GEAR_MAP = {
  4: CarState.GearShifter.drive,
  2: CarState.GearShifter.reverse,
  3: CarState.GearShifter.neutral,
  1: CarState.GearShifter.park,
}

# TODO: Get a BYD VDS to see how firmware could be queried. Until then BYD is
# identified by legacy CAN fingerprinting only.
DBC = CAR.create_dbc_map()
