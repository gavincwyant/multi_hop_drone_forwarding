# Advanced Systems Software group project.

## CONTRIBUTERS: Luke Haidze, Vlad Bordia, Gavin Wyant
 
### The initial idea for this project is the following: 

We will be simulating a wireless network with a single user who is traveling at a constant rate while sending and receiving packets to
an access point. When a certain amount of packets are lost (or some other metric is met), our simulation will spawn another access
point to be equidistant between our user and the initial access point in order to forward packets and increase signal strength. 
This new access point is meant to simulate the deployment of a drone.

The goals of this project are to determine the most effective metric for deciding when our drone will be deployed as well as where 
it should sit between the user and the access point. We also hope to find if this idea is feasible and will scale to any number of 
additional hops between initial access point and the user. 



## Build Information

This repo does not contain ns-3. In order to build this project, you must have ns-3 installed locally.
In order to use the current build script (run_sim.sh), the following directory structure is assumed:

```text
.
├── multi_hop_drone_forwarding
└── ns-3-dev

```

### RSSI Implementation
