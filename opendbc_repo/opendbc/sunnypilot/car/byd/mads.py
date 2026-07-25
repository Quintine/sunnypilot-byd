"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs

from opendbc.sunnypilot.mads_base import MadsCarStateBase
from opendbc.can.parser import CANParser


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    # PCM_BUTTONS bus assignment is harness dependent; read from whichever bus sees it
    lkas_button = cp.vl["PCM_BUTTONS"]["LKAS_ON_BTN"]
    if not cp.vl_all["PCM_BUTTONS"]["LKAS_ON_BTN"] and cp_cam.vl_all["PCM_BUTTONS"]["LKAS_ON_BTN"]:
      lkas_button = cp_cam.vl["PCM_BUTTONS"]["LKAS_ON_BTN"]

    self.prev_lkas_button = self.lkas_button
    self.lkas_button = lkas_button
