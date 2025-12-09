# Advanced Systems Software group project.

## CONTRIBUTERS: Luke Haidze, Vlad Bordia, Gavin Wyant
 
### The initial idea for this project is the following: 

We are simulating a wireless network with a single user who is traveling at a constant rate while sending and receiving packets to
an access point. When a certain amount of packets are lost (or some other metric is met), our simulation deploys another access
point to be equidistant between our user and the initial access point in order to forward packets and increase signal strength. 
This new access point is meant to simulate the deployment of a drone.

### Network Topology
```text
    User  <---- WiFi link ---->  Drone  <---- WiFi link ----> Base Station
```

The goals of this project are to determine the optimal metric for deciding when our drone will be deployed as well as where 
it should be placed between the user and the access point for highest throughput. We also hope to determine if this idea is feasible and scalable to any 
number of additional hops between initial access point and the user. 

### Scaled Network Topology
```text 
    User  <---- WiFi link ---->  Drone1  <---- WiFi link ----> ... <---- Wifi Link ----> DroneN <---- Wifi Link ---->  Base Station
```


## Build Information

This repo does not contain ns-3. In order to build this project, you must have ns-3 installed locally.
In order to use the current build script (run_sim.sh), the following directory structure is assumed:

```text
.
├── multi_hop_drone_forwarding
└── ns-3-dev

```

## Running the Simulation

### Quick Start

The simplest way to run the simulation is using the build script:

```bash
./run_sim.sh
```

This will:
1. Copy `multi_hop_drone_sim.cc` to ns-3's scratch directory
2. Build the simulation
3. Run with default parameters (2 drones, dynamic deployment)

### Test Configurations

#### Test 1: Basic 2-Drone Dynamic Deployment (Recommended)
```bash
cd ../ns-3-dev
./ns3 run "scratch/multi_hop_drone_sim --numDrones=2 --droneInitMode=deploy --userSpeed=2.5"
```

**What's happening:**
- User starts at X=0m and moves at 2.5 m/s to the right
- Base station is fixed at X=0m
- 2 drones are staged near the base station
- When network quality degrades (loss >20%, RTT >100ms, RSSI <-65dBm, or hop distance >40m), drones deploy
- Drones auto-balance to equalize hop distances
- **Expected result:** <5% packet loss, drones follow user as they move

#### Test 2: 3-Drone Dynamic Deployment
```bash
cd ../ns-3-dev
./ns3 run "scratch/multi_hop_drone_sim --numDrones=3 --droneInitMode=deploy --userSpeed=2.5"
```

**What's happening:**
- Same as Test 1, but with 3 drones available
- More drones deploy as user moves further from base station
- Each deployment places drone at midpoint of largest gap in chain
- **Expected result:** <5% packet loss, all 3 drones should deploy within 60s

#### Test 3: Pre-Deployed Even Spacing (No Dynamic Deployment)
```bash
cd ../ns-3-dev
./ns3 run "scratch/multi_hop_drone_sim --numDrones=3 --droneInitMode=even --totalDistance=120"
```

**What's happening:**
- All 3 drones start pre-deployed, evenly spaced between user (X=0) and base station (X=0)
- User starts at position X=0 and moves to X=120m over 60 seconds
- No dynamic deployment, only auto-balancing as user moves
- **Expected result:** 0% loss at start, drones reposition to maintain coverage

#### Test 4: Fast User Movement (Stress Test)
```bash
cd ../ns-3-dev
./ns3 run "scratch/multi_hop_drone_sim --numDrones=2 --droneInitMode=deploy --userSpeed=5.0"
```

**What's happening:**
- User moves at 5 m/s (double normal speed)
- Tests drone deployment and balancing under rapid topology changes
- Drones must quickly deploy and chase the user
- **Expected result:** Higher loss during transitions, but <10% overall

### Understanding the Output

The simulation prints detailed metrics every 2 seconds:

```
========== 58.00s ==========
UserX=145.00 m
Chain: U[145.00m] <-> D1[95.00m] <-> D2[46.00m] <-> A[0.00m]
Hop Metrics:
  Hop0 (Node0->Node1): dist=50.99m, RSSI=-23.01dBm
  Hop1 (Node1->Node2): dist=49.00m, RSSI=-24.02dBm
  Hop2 (Node2->Node3): dist=47.07m, RSSI=-22.50dBm
Overall: Tx=112, Rx=109, Loss=2.68%, AvgRTT=6.79ms
Window: Tx=56, Rx=54, Loss=3.57%, AvgRTT=9.46ms
```

**What each section means:**
- **UserX**: Current position of the mobile user (meters from origin)
- **Chain**: Active network topology showing node positions (U=User, D=Drone, A=Access Point)
- **Hop Metrics**: Per-hop distance and signal strength (RSSI) for each link
- **Overall**: Cumulative statistics since simulation start
- **Window**: Statistics since last drone deployment (used to trigger next deployment)

**Key indicators:**
- ✅ **Good**: Loss <5%, RSSI >-70dBm per hop, hop distances <50m
- ⚠️ **Degrading**: Loss 5-15%, RSSI -70 to -80dBm, hop distances 50-80m
- ❌ **Poor**: Loss >15%, RSSI <-80dBm, hop distances >80m

### Available Command-Line Parameters

```bash
--numDrones=N          # Number of drone relays (default: 2)
--droneInitMode=MODE   # "deploy" (staged), "even" (pre-deployed evenly), or "cluster" (near user)
--totalDistance=M      # Distance between user and AP in meters (for "even" mode)
--userSpeed=S          # User movement speed in m/s (default: 2.5)
```

### Demo Scripts (For Presentations)

Two convenient demo scripts are provided for presentations and quick testing:

#### Quick Demo (5 minutes)
```bash
./quick_demo.sh
```

Runs a single 2-drone test and displays key results. Perfect for:
- Quick validation that the system works
- Showing final statistics
- Time-constrained demos

**Output includes:**
- Deployment events
- Final network topology
- Success/failure indicator based on packet loss

#### Full Interactive Demo (15-20 minutes)
```bash
./demo.sh
```

Runs 4 comprehensive tests with detailed explanations between each. Perfect for:
- Full feature demonstrations
- Educational presentations
- Comparative analysis of different configurations

**Tests included:**
1. Basic 2-Drone Dynamic Deployment
2. 3-Drone Dynamic Deployment
3. Pre-Deployed Even Spacing (Baseline)
4. Fast User Movement (Stress Test)

The script pauses between tests with explanations of what to watch for.

### Key Features

**Dynamic Routing (AODV):**
- Routes automatically update as drones move
- No manual routing table updates needed
- Handles topology changes in real-time

**Smart Deployment:**
- Monitors weakest link in chain (not just direct path)
- Deploys when loss >20%, RTT >100ms, RSSI <-65dBm, OR hop distance >40m
- Places new drone at midpoint of largest gap

**Auto-Balancing:**
- Drones adjust position every 1 second
- Move toward the farther neighbor to equalize hop distances
- Collision prevention ensures minimum 0.5m separation

**Dual Metrics:**
- **Overall**: Cumulative stats for performance evaluation
- **Window**: Recent stats for deployment decisions

### RSSI Implementation
