#!/bin/bash
set -e

# Path to your local ns-3 clone
NS3_PATH="../ns-3-dev"

if [ ! -d "$NS3_PATH" ]; then
  echo "Error: ns-3-dev not found at $NS3_PATH"
  exit 1
fi

# Copy your simulation into ns-3 scratch/
cp hybridbuildings_example/buildings.cc "$NS3_PATH/scratch/"

# Build and run the simulation
cd "$NS3_PATH"
./ns3 build
./ns3 run scratch/buildings
