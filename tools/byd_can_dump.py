#!/usr/bin/env python3
"""BYD CAN bus dump — figures out which panda bus each BYD message is on.

Usage (on the comma device, SSH'd in):

  # Mode 1 (safest): analyze the most recent recorded route — no CAN hardware needed
  cd /data/openpilot
  python3 tools/byd_can_dump.py

  # Mode 2: live sniff via panda (manager must be stopped first!)
  tmux kill-server
  python3 tools/byd_can_dump.py --live 15
  # then reboot

Key BYD addresses it highlights:
  287 STEER_MODULE   289 ESC            482 MPC_LKAS_CMD_ANGLE
  508 STEER_TORQUE_ANGLE               544 BRAKE_APPLIED
  578 DRIVE_STATE    660 METER_CLUSTER  790 MPC_LKAS_CMD
  792 STEERING_TORQUE                  813 ACC_HUD_ADAS
  814 ACC_CMD        834 PEDAL          944 PCM_BUTTONS
"""

import argparse
import os
import sys
import time
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

KEY_ADDRS = {
  287: "STEER_MODULE",
  289: "ESC",
  482: "MPC_LKAS_CMD_ANGLE",
  508: "STEERING_TORQUE_ANGLE",
  544: "BRAKE_APPLIED",
  578: "DRIVE_STATE",
  660: "METER_CLUSTER",
  790: "MPC_LKAS_CMD",
  792: "STEERING_TORQUE",
  813: "ACC_HUD_ADAS",
  814: "ACC_CMD",
  834: "PEDAL",
  944: "PCM_BUTTONS",
}


def print_table(stats, started_mono=None):
  """stats: {(bus, addr): [len, count, first_t, last_t]}"""
  buses = defaultdict(dict)
  for (bus, addr), (length, count, first_t, last_t) in sorted(stats.items()):
    buses[bus][addr] = (length, count, first_t, last_t)

  for bus in sorted(buses):
    print(f"\n=== bus {bus} ({len(buses[bus])} addresses) ===")
    print(f"{'addr':>6}  {'len':>3}  {'count':>7}  {'rate_Hz':>7}  name")
    for addr, (length, count, first_t, last_t) in sorted(buses[bus].items()):
      dt = last_t - first_t
      rate = (count - 1) / dt if dt > 0 and count > 1 else 0.0
      name = KEY_ADDRS.get(addr, "")
      marker = " <<<" if addr in KEY_ADDRS else ""
      print(f"{addr:>6}  {length:>3}  {count:>7}  {rate:>7.1f}  {name}{marker}")


LOG_NAMES = ("rlog", "rlog.bz2", "rlog.zst", "qlog", "qlog.bz2", "qlog.zst")


def find_log_file(route_dir):
  for name in LOG_NAMES:
    p = os.path.join(route_dir, name)
    if os.path.exists(p):
      return p
  return None


def find_latest_log(base):
  """Newest dir (by name) that actually contains an rlog/qlog; skips 'boot'."""
  for d in sorted(os.listdir(base), reverse=True):
    p = os.path.join(base, d)
    if not os.path.isdir(p) or d == "boot":
      continue
    lp = find_log_file(p)
    if lp is not None:
      return lp
  return None


def dump_from_route(route_dir=None):
  from openpilot.tools.lib.logreader import LogReader

  base = "/data/media/0/realdata"
  if route_dir is None:
    log_path = find_latest_log(base)
    assert log_path is not None, f"no routes with rlog/qlog found in {base}"
  else:
    log_path = find_log_file(route_dir)
    assert log_path is not None, f"no rlog/qlog found in {route_dir}"
  print(f"reading {log_path} ...")

  stats = {}
  n_frames = 0
  for m in LogReader(log_path):
    if m.which() != "can":
      continue
    t = m.logMonoTime * 1e-9
    for f in m.can:
      if f.src >= 128:
        continue
      key = (f.src, f.address)
      if key not in stats:
        stats[key] = [len(f.dat), 0, t, t]
      s = stats[key]
      s[1] += 1
      s[3] = t
      n_frames += 1

  print(f"{n_frames} CAN frames")
  print_table(stats)


def dump_live(seconds):
  from panda import Panda
  from opendbc.car.structs import CarParams

  print("connecting to panda (manager must be stopped: tmux kill-server) ...")
  p = Panda()
  p.set_safety_mode(CarParams.SafetyModel.noOutput)
  print(f"sniffing for {seconds}s ...")

  stats = {}
  t_end = time.monotonic() + seconds
  t0 = time.monotonic()
  n_frames = 0
  while time.monotonic() < t_end:
    for addr, dat, bus in p.can_recv():
      if bus >= 128:
        continue
      t = time.monotonic() - t0
      key = (bus, addr)
      if key not in stats:
        stats[key] = [len(dat), 0, t, t]
      s = stats[key]
      s[1] += 1
      s[3] = t
      n_frames += 1

  print(f"{n_frames} CAN frames")
  print_table(stats)


if __name__ == "__main__":
  ap = argparse.ArgumentParser()
  ap.add_argument("--live", type=int, metavar="SECONDS", default=0,
                  help="sniff live via panda for N seconds (stop manager first!)")
  ap.add_argument("--route", type=str, default=None, help="route dir to analyze instead of latest")
  args = ap.parse_args()

  if args.live:
    dump_live(args.live)
  else:
    dump_from_route(args.route)
