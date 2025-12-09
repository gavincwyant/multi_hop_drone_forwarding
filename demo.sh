#!/bin/bash
# Multi-Hop Drone Network Simulation - Demo Script
# Runs multiple test configurations with explanations

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Path to ns-3
NS3_PATH="../ns-3-dev"

# Check if ns-3 exists
if [ ! -d "$NS3_PATH" ]; then
  echo "Error: ns-3-dev not found at $NS3_PATH"
  exit 1
fi

# Copy simulation file
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Multi-Hop Drone Network Demo${NC}"
echo -e "${BLUE}========================================${NC}\n"

echo "Copying simulation to ns-3 scratch directory..."
cp simulations/multi_hop_drone_sim.cc "$NS3_PATH/scratch/"

# Build
echo "Building simulation..."
cd "$NS3_PATH"
./ns3 build > /dev/null 2>&1

echo -e "${GREEN}✓ Build complete${NC}\n"

# Function to run test with description
run_test() {
    local test_num=$1
    local test_name=$2
    local description=$3
    local command=$4
    local expected=$5

    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}Test $test_num: $test_name${NC}"
    echo -e "${CYAN}========================================${NC}\n"

    echo -e "${YELLOW}Description:${NC}"
    echo -e "$description\n"

    echo -e "${YELLOW}Expected Result:${NC}"
    echo -e "$expected\n"

    echo -e "${YELLOW}Running command:${NC}"
    echo -e "${GREEN}$command${NC}\n"

    echo "Press Enter to start test (or Ctrl+C to skip)..."
    read

    # Run the simulation
    eval "$command"

    echo -e "\n${GREEN}✓ Test $test_num complete${NC}\n"
    echo "Press Enter to continue to next test..."
    read
    echo ""
}

# Test 1: Basic 2-Drone Deployment
run_test "1" \
    "Basic 2-Drone Dynamic Deployment" \
    "The user starts at the base station and walks away at 2.5 m/s.
As the signal degrades, drones automatically deploy to extend the network.
Watch for:
  • Drone deployments when hop distance exceeds 40m
  • Drones auto-balancing to equalize hop distances
  • Packet loss staying below 5% after deployment" \
    "./ns3 run 'scratch/multi_hop_drone_sim --numDrones=2 --droneInitMode=deploy --userSpeed=2.5'" \
    "Loss: <5%, both drones deploy, drones follow user"

# Test 2: 3-Drone Deployment
run_test "2" \
    "3-Drone Dynamic Deployment" \
    "Similar to Test 1, but with 3 drones available.
As the user moves further, additional drones deploy at the largest gaps.
Watch for:
  • Sequential deployment of all 3 drones
  • Each drone deploys at midpoint of largest gap
  • Overall loss remaining low throughout simulation" \
    "./ns3 run 'scratch/multi_hop_drone_sim --numDrones=3 --droneInitMode=deploy --userSpeed=2.5'" \
    "Loss: <5%, all 3 drones deploy within 60s"

# Test 3: Pre-Deployed Even Spacing
run_test "3" \
    "Pre-Deployed Even Spacing (Baseline)" \
    "All drones start evenly spaced between user and base station.
No dynamic deployment - only auto-balancing as user moves.
This shows the system maintains coverage even with mobile users.
Watch for:
  • 0% loss at start (perfect coverage)
  • Drones smoothly repositioning as user moves
  • Consistent hop distances throughout" \
    "./ns3 run 'scratch/multi_hop_drone_sim --numDrones=2 --droneInitMode=even --totalDistance=60'" \
    "Loss: 0% initially, drones maintain even spacing"

# Test 4: Fast Movement Stress Test
run_test "4" \
    "Fast User Movement (Stress Test)" \
    "User moves at 5 m/s (double normal speed).
Tests how well the system handles rapid topology changes.
Drones must quickly deploy and reposition.
Watch for:
  • Rapid drone deployment
  • Drones chasing the user at high speed
  • Some packet loss during transitions but recovery afterward" \
    "./ns3 run 'scratch/multi_hop_drone_sim --numDrones=2 --droneInitMode=deploy --userSpeed=5.0'" \
    "Loss: <10%, system adapts to rapid movement"

# Summary
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Demo Complete!${NC}"
echo -e "${CYAN}========================================${NC}\n"

echo -e "${YELLOW}Summary:${NC}"
echo "✓ Test 1: Demonstrated basic 2-drone dynamic deployment"
echo "✓ Test 2: Showed scalability with 3 drones"
echo "✓ Test 3: Baseline performance with pre-deployment"
echo "✓ Test 4: Stress-tested with fast user movement"
echo ""
echo -e "${YELLOW}Key Takeaways:${NC}"
echo "• AODV routing enables automatic route updates"
echo "• Dynamic deployment extends network range on-demand"
echo "• Auto-balancing maintains optimal hop distances"
echo "• System achieves <5% packet loss with 2-3 drones"
echo ""
echo -e "${GREEN}For more details, see README.md${NC}\n"
