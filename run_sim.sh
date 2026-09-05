#!/bin/bash
set -e

# Path to your local ns-3 clone
NS3_PATH="${NS3_PATH:-../ns-3-dev}"

if [ ! -d "$NS3_PATH" ]; then
  echo "Error: ns-3-dev not found at $NS3_PATH" >&2
  echo "Set NS3_PATH to override, e.g. NS3_PATH=~/src/ns-3-dev ./run_sim.sh" >&2
  exit 1
fi

SIM="drone_wifi_simulation"
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

# Copy the simulation into ns-3's scratch/ directory
cp "$REPO_DIR/simulations/$SIM.cc" "$NS3_PATH/scratch/"

# Build and run, forwarding any arguments through to the simulation.
# e.g. ./run_sim.sh --maxRelays=5 --lossThreshold=15 --csv=results.csv
cd "$NS3_PATH"
./ns3 build "scratch_$SIM"

if [ "$#" -gt 0 ]; then
  ./ns3 run "scratch/$SIM" -- "$@"
else
  ./ns3 run "scratch/$SIM"
fi
