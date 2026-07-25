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
  tx_stats = {}
  cruise_states = defaultdict(int)
  lkas_active_count = [0, 0]  # [active, total] from camera's 790 on bus 2
  n_frames = 0
  for m in LogReader(log_path):
    w = m.which()
    if w not in ("can", "sendcan"):
      continue
    t = m.logMonoTime * 1e-9
    frames = m.can if w == "can" else m.sendcan
    target = stats if w == "can" else tx_stats
    for f in frames:
      if w == "can" and f.src >= 128:
        continue
      key = (f.src, f.address)
      if key not in target:
        target[key] = [len(f.dat), 0, t, t]
      s = target[key]
      s[1] += 1
      s[3] = t
      n_frames += 1

      # decode ADAS cruise state (813 ACC_HUD_ADAS): CRUISE_STATE = byte5 >> 4 & 0xF
      if f.address == 813 and len(f.dat) >= 6:
        cruise_states[(f.dat[5] >> 4) & 0xF] += 1
      # decode camera LKAS active (790 MPC_LKAS_CMD): LKAS_ACTIVE = bit 28 = byte3 bit4
      if f.address == 790 and f.src == 2 and len(f.dat) >= 4:
        lkas_active_count[1] += 1
        if (f.dat[3] >> 4) & 0x1:
          lkas_active_count[0] += 1

  print(f"{n_frames} CAN frames")
  print("\n--- received (car -> panda) ---")
  print_table(stats)
  if tx_stats:
    print("\n--- openpilot TX (sendcan) ---")
    print_table(tx_stats)

  if cruise_states:
    names = {0: "Off", 1: "Standby", 3: "ACTIVE", 5: "Override", 8: "FAILURE", 9: "PERMANENT_FAILURE"}
    print("\n--- ACC_HUD_ADAS CRUISE_STATE histogram ---")
    for state, count in sorted(cruise_states.items()):
      print(f"  state {state:>2} ({names.get(state, '?')}): {count}")
    if cruise_states.get(8, 0) or cruise_states.get(9, 0):
      print("  !!! ADAS still in FAILURE state !!!")
    else:
      print("  OK: no failure states — ADAS ECU healthy")

  if lkas_active_count[1]:
    pct = 100.0 * lkas_active_count[0] / lkas_active_count[1]
    print(f"\n--- camera 790 LKAS_ACTIVE: {lkas_active_count[0]}/{lkas_active_count[1]} frames ({pct:.0f}%) ---")


# byd_checksum from opendbc.car.byd.bydcan (recomputed over 8 bytes with the
# CHECKSUM byte zeroed) — used to verify the algorithm against real frames
def byd_checksum(data):
  byte_key = 0xAF
  sum_first = sum(byte >> 4 for byte in data)
  sum_second = sum(byte & 0xF for byte in data)
  remainder = sum_second >> 4
  sum_first += (byte_key & 0xF)
  sum_second += (byte_key >> 4)
  inv_first = ((-sum_first + 0x9) & 0xF)
  inv_second = ((-sum_second + 0x9) & 0xF)
  return (((inv_first + (5 - remainder)) << 4) + inv_second) & 0xFF


def verify_checksums(route_dir=None, addrs=(508, 482, 790, 792, 814, 813)):
  """Recompute byd_checksum for each CHECKSUM'd message and report match rates."""
  from openpilot.tools.lib.logreader import LogReader

  base = "/data/media/0/realdata"
  if route_dir is None:
    log_path = find_latest_log(base)
    assert log_path is not None, f"no routes with rlog/qlog found in {base}"
  else:
    log_path = find_log_file(route_dir)
    assert log_path is not None, f"no rlog/qlog found in {route_dir}"
  print(f"reading {log_path} ...")

  results = defaultdict(lambda: [0, 0])  # addr -> [matches, total]
  samples = defaultdict(list)
  for m in LogReader(log_path):
    if m.which() != "can":
      continue
    for f in m.can:
      if f.src >= 128 or f.address not in addrs or len(f.dat) != 8:
        continue
      data = bytearray(f.dat)
      expected = data[7]
      data[7] = 0
      match = byd_checksum(data) == expected
      results[f.address][1] += 1
      if match:
        results[f.address][0] += 1
      elif len(samples[f.address]) < 3:
        samples[f.address].append((bytes(f.dat).hex(), expected, byd_checksum(data)))

  print("\n--- byd_checksum verification (CHECKSUM byte = last byte) ---")
  for addr in sorted(results):
    matches, total = results[addr]
    name = KEY_ADDRS.get(addr, "")
    status = "OK" if matches == total else "!!! MISMATCH !!!"
    print(f"  {addr:>4} {name:<24} {matches}/{total} ({100.0 * matches / max(total, 1):.1f}%)  {status}")
    for raw, exp, got in samples[addr]:
      print(f"       sample {raw}  expected={exp:#04x} computed={got:#04x}")


def decode_508_failures(route_dir=None):
  """Histogram TORQUE_FAILED / TORQUE_TEMP_FAILED / LKSPrepare / Cruise_Activated
  from stock 508 STEERING_TORQUE_ANGLE frames (bus 0)."""
  from openpilot.tools.lib.logreader import LogReader

  base = "/data/media/0/realdata"
  if route_dir is None:
    log_path = find_latest_log(base)
    assert log_path is not None, f"no routes with rlog/qlog found in {base}"
  else:
    log_path = find_log_file(route_dir)
    assert log_path is not None, f"no rlog/qlog found in {route_dir}"
  print(f"reading {log_path} ...")

  tf = defaultdict(int)
  ttf = defaultdict(int)
  prepare = defaultdict(int)
  activated = defaultdict(int)
  total = 0
  for m in LogReader(log_path):
    if m.which() != "can":
      continue
    for f in m.can:
      if f.src >= 128 or f.address != 508 or len(f.dat) < 6:
        continue
      total += 1
      tf[f.dat[0] & 0x4] += 1              # TORQUE_FAILED bit 2
      ttf[(f.dat[5] >> 6) & 0x3] += 1      # TORQUE_TEMP_FAILED bits 46-47 -> byte5 bits 6-7
      prepare[f.dat[0] & 0x1] += 1         # LKSPrepare bit 0
      activated[(f.dat[0] >> 1) & 0x1] += 1  # Cruise_Activated bit 1

  print(f"\n--- 508 STEERING_TORQUE_ANGLE decode ({total} frames) ---")
  print(f"  TORQUE_FAILED:      {dict(tf)}")
  print(f"  TORQUE_TEMP_FAILED: {dict(ttf)}")
  print(f"  LKSPrepare:         {dict(prepare)}")
  print(f"  Cruise_Activated:   {dict(activated)}")


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
  ap.add_argument("--verify-checksums", action="store_true",
                  help="verify byd_checksum algorithm against real 508/790/792/814 frames")
  ap.add_argument("--decode-508", action="store_true",
                  help="histogram TORQUE_FAILED / LKSPrepare / Cruise_Activated from stock 508 frames")
  args = ap.parse_args()

  if args.live:
    dump_live(args.live)
  elif args.verify_checksums:
    verify_checksums(args.route)
  elif args.decode_508:
    decode_508_failures(args.route)
  else:
    dump_from_route(args.route)
