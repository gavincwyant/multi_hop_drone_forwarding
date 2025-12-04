#!/bin/bash
# Quick Demo - Runs 2-drone test and shows key results
# Perfect for quick demonstrations

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

NS3_PATH="../ns-3-dev"

echo -e "${BLUE}╔════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Multi-Hop Drone Network - Quick Demo             ║${NC}"
echo -e "${BLUE}╔════════════════════════════════════════════════════╗${NC}\n"

echo -e "${YELLOW}Scenario:${NC}"
echo "• User walks away from base station at 2.5 m/s"
echo "• 2 drones staged, ready to deploy"
echo "• Drones deploy when network degrades"
echo "• 60-second simulation"
echo ""

# Copy and build
echo "Preparing simulation..."
cp simulations/multi_hop_drone_sim.cc "$NS3_PATH/scratch/"
cd "$NS3_PATH"
./ns3 build > /dev/null 2>&1
echo -e "${GREEN}✓ Ready${NC}\n"

echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}Starting Simulation...${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}\n"

# Run simulation and capture output
OUTPUT=$(./ns3 run 'scratch/multi_hop_drone_sim --numDrones=2 --droneInitMode=deploy --userSpeed=2.5' 2>&1)

# Extract key events
echo -e "${YELLOW}Key Events:${NC}"
echo "$OUTPUT" | grep -E "^\[Deploy\]" | head -3
echo ""

# Show final state
echo -e "${YELLOW}Final State (t=58s):${NC}"
echo "$OUTPUT" | grep -A 10 "58.00s" | grep -E "(UserX|Chain|Overall)"
echo ""

# Extract and display final statistics
FINAL_STATS=$(echo "$OUTPUT" | grep "Overall:" | tail -1)
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}✓ Simulation Complete${NC}\n"

echo -e "${YELLOW}Final Statistics:${NC}"
echo "$FINAL_STATS"
echo ""

# Parse loss percentage
LOSS=$(echo "$FINAL_STATS" | grep -oE "Loss=[0-9.]+%" | grep -oE "[0-9.]+")

if (( $(echo "$LOSS < 5" | bc -l) )); then
    echo -e "${GREEN}✓ SUCCESS: Packet loss ${LOSS}% < 5% (target met!)${NC}"
else
    echo -e "${YELLOW}⚠ Packet loss ${LOSS}% (target: <5%)${NC}"
fi

echo ""
echo -e "${BLUE}Run './demo.sh' for full interactive demo${NC}"
echo -e "${BLUE}See README.md for more test configurations${NC}\n"
