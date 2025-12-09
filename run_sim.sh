#!/bin/bash
set -e

# Path to your local ns-3 clone
NS3_PATH="../ns-3-dev"

FILE="multi_hop_drone_sim.cc"

if [ ! -d "$NS3_PATH" ]; then
  echo "Error: ns-3-dev not found at $NS3_PATH"
  exit 1
fi

# Copy your simulation into ns-3 scratch/
#cp simulations/drone_wifi_simulation.cc "$NS3_PATH/scratch/"
cp simulations/$FILE "$NS3_PATH/scratch/"

# Build and run the simulation
cd "$NS3_PATH"
./ns3 build
#./ns3 run scratch/drone_wifi_simulation
./ns3 run scratch/$FILE
