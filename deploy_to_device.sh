#!/usr/bin/env bash
# Deploy this repo to a comma device over SSH, preserving symlinks.
#
# Usage:
#   ./deploy_to_device.sh <device-ip> [target-dir]
#
# Examples:
#   ./deploy_to_device.sh 192.168.1.50
#   ./deploy_to_device.sh comma.local /data/openpilot
#
# Notes:
# - Deploys EXACTLY what is committed in git (git archive), so commit first.
# - Symlinks are preserved (unlike scp -r / file managers).
# - The existing install is kept as <target>.old for rollback.
# - Only the last ~deploy is transferred (no .git, no .venv) — fast and small.

set -euo pipefail

DEVICE="${1:?usage: ./deploy_to_device.sh <device-ip> [target-dir]}"
TARGET="${2:-/data/openpilot}"
SSH_USER="${SSH_USER:-comma}"

cd "$(dirname "$0")"

# sanity: must be run from the repo root
if [ ! -d .git ]; then
  echo "error: no .git here — run from the repo root" >&2
  exit 1
fi

# sanity: working tree must be committed (deploy uses git archive HEAD)
if [ -n "$(git status --porcelain)" ]; then
  echo "warning: uncommitted changes exist — they will NOT be deployed."
  echo "         commit first, or press enter to deploy HEAD anyway."
  read -r _
fi

echo "==> deploying HEAD ($(git rev-parse --short HEAD)) to ${SSH_USER}@${DEVICE}:${TARGET}"

TMP="${TARGET}.deploy-new"
OLD="${TARGET}.old"

# 1) stream the archive to the device and extract into a staging dir
git archive HEAD | ssh "${SSH_USER}@${DEVICE}" "
  set -e
  rm -rf '$TMP'
  mkdir -p '$TMP'
  tar -xf - -C '$TMP'
"

# 2) verify symlinks landed correctly on the device
echo "==> verifying symlinks on device"
ssh "${SSH_USER}@${DEVICE}" "
  set -e
  for l in opendbc msgq rednose teleoprtc tinygrad \
           openpilot/sunnypilot/selfdrive/car/car_list.json; do
    if [ ! -L '$TMP/'\$l ]; then
      echo \"FATAL: \$l is not a symlink on device\" >&2
      exit 1
    fi
  done
  head -c 2 '$TMP/openpilot/sunnypilot/selfdrive/car/car_list.json' | grep -q '^{'
  echo '    symlinks OK'
"

# 3) swap into place (keep previous as .old)
ssh "${SSH_USER}@${DEVICE}" "
  set -e
  rm -rf '$OLD'
  if [ -d '$TARGET' ]; then mv '$TARGET' '$OLD'; fi
  mv '$TMP' '$TARGET'
  echo '==> swapped into place (previous kept as $OLD)'
"

echo "==> done. Reboot the device to start the new build."
