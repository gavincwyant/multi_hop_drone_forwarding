# Multi-Hop Drone Forwarding

An ns-3 simulation of drone-assisted relay deployment in a degrading wireless link.

**Advanced Systems Software group project — Luke Haidze, Vlad Bordia, Gavin Wyant**

Five drones held the link out to 295 m across 6 hops at 94% packet delivery,
where the unaided link failed past 50 m. The deployment trigger has to be a
*windowed* loss rate — a cumulative one never crosses the threshold at all.

## The idea

A single user travels away from a fixed access point at constant velocity while
exchanging UDP traffic with it. As distance grows, packet loss climbs. When loss
crosses a threshold, the simulation deploys a relay — standing in for a drone —
between the user and the access point to forward traffic over an additional hop.

The project set out to answer three questions:

1. **When** should a drone be deployed — what metric signals that the link needs help?
2. **Where** should it sit between the user and the access point?
3. **Does it scale** to an arbitrary number of hops?

## What the simulation does

`simulations/drone_wifi_simulation.cc` implements all three.

**Trigger (question 1).** The user's UDP echo traffic is instrumented with `Tx` and
`Rx` trace callbacks. Every `--monitorInterval` seconds the simulation computes a
**sliding-window loss rate** from the change in counters since the previous tick,
and deploys a relay when that rate exceeds `--lossThreshold`.

The window matters. A cumulative loss rate — total lost over total sent since t=0 —
is dominated by early history, responds sluggishly, and can never recover once
relays repair the link, so it cannot serve as a control signal. Both rates are
reported so the difference is visible in the output.

**Placement (question 2).** Relays are spaced evenly along the segment from the
access point to the user: relay *i* of *k* sits at `base + (user - base) * i/(k+1)`.
The chain re-balances on every deployment, since the user keeps moving.

**Scaling (question 3).** `--maxRelays` sets how many drones are available. Nodes
run in ad-hoc mode with **AODV** routing rather than an infrastructure BSS — an
infrastructure network cannot forward user → relay → base, and hard-coding routes
would sidestep the question. AODV discovers each new path after a deployment, so
adding hops is a matter of raising `--maxRelays`.

A deployment cooldown (`--cooldown`) prevents the trigger from firing repeatedly
against stale measurements while AODV is still converging on the new route.

## Results

Two 60 s runs, user departing at 5 m/s, `--lossThreshold=20`:

```sh
./run_sim.sh --simTime=60 --maxRelays=0   # baseline
./run_sim.sh --simTime=60 --maxRelays=5   # with drones
```

| | baseline (no drones) | 5 drones available |
| --- | --- | --- |
| Link usable to | ~50 m | 295 m (end of run) |
| Packets delivered | — link down past ~50 m | 109 / 116 (94%) |
| Cumulative loss at end | 79% by 215 m | 5.3% at 295 m |
| Hops | 1 | up to 6 |

Deployments fired at t = 11, 21, 31, 41 and 51 s, each adding one hop as the
user crossed roughly another 50 m.

### 1. The trigger metric has to be windowed

This is the clearest finding, and it is a negative result for the obvious approach.

At every deployment point the windowed loss rate hit 50% while the cumulative
rate sat near 5%:

| t (s) | distance | window loss | cumulative loss | relays |
| --- | --- | --- | --- | --- |
| 11 | 55 m | 50% | 5.6% | 0 → 1 |
| 21 | 105 m | 50% | 5.3% | 1 → 2 |
| 31 | 155 m | 50% | 5.2% | 2 → 3 |
| 41 | 205 m | 50% | 5.1% | 3 → 4 |
| 51 | 255 m | 50% | 6.1% | 4 → 5 |

Cumulative loss never exceeded **6.1%** across the whole run. A 20% threshold on
the cumulative rate would therefore never have fired, and no drone would ever have
deployed — the link would simply have failed. Averaging over the whole run buries
the signal you are trying to detect in history that is no longer true.

### 2. Even spacing wins, because the chain is only as good as its longest hop

`--placementGamma` controls where relays sit: relay *i* of *k* goes at
`((i+1)/(k+1))^gamma` along the base-to-user segment, so 1.0 is even spacing,
above 1 pulls the chain toward the base station, below 1 pushes it toward the
user. Three 60 s runs with 5 drones:

| gamma | placement | largest gap | outcome |
| --- | --- | --- | --- |
| 0.5 | toward user | 0.408 | link failed at 80 m |
| **1.0** | **even** | **0.167** | **never failed; 109/116 delivered** |
| 2.0 | toward base | 0.306 | link failed at 75 m |

Even spacing is not just a reasonable default, it is the optimum for this
geometry, and the reason is visible in the gap column. A chain of hops fails at
its *widest* gap, no matter how tight the others are. Clustering drones toward
either end necessarily stretches the hop at the opposite end — biasing toward the
user leaves a 0.408 gap from the base station, biasing toward the base leaves a
0.306 gap to the user — and both blow past radio range while the remaining hops
sit uselessly close together. Even spacing is the placement that minimises the
maximum hop, so it is the placement that survives longest.

**Placement also has to be continuous, not one-shot.** An earlier version of this
simulation positioned each relay once, at the moment it deployed. At 5 m/s the
user left that relay behind within seconds and loss stayed pinned at 100% after
deployment — the drone was in the right place for about one tick and the wrong
place forever after. Re-spacing the whole chain every measurement interval is
what makes the scheme work at all.

### 3. It scales linearly in hops, and fails hard when the pool runs out

Range grew about linearly with hop count: each additional relay extended the
usable link by roughly 50 m. Only 6 of 57 measurement intervals in the 60 s run
saw any loss at all, and each was the interval immediately preceding a
deployment — the cost of adding a hop is one brief loss spike while AODV
rediscovers the route.

The 120 s default run shows the other end of that curve. The pool of 5 drones is
exhausted at t = 51 s, and the 6-hop chain sustains the link to ~295 m:

| t (s) | distance | window loss | cumulative | relays |
| --- | --- | --- | --- | --- |
| 59 | 295 m | 0% | 5.3% | 5 (pool exhausted) |
| 63 | 315 m | 100% | 9.0% | 5 |
| 79 | 395 m | 100% | 27.9% | 5 |
| 94 | 470 m | 100% | 39.7% | 5 |

The link fails between 295 m and 315 m and never recovers. There is no graceful
degradation: once the chain cannot span the gap, throughput goes to zero. The
binding constraint is the number of drones available, not where they are placed —
past the ceiling, better placement buys nothing.

Raise `--maxRelays` to push the ceiling out; the ~50 m per hop relationship holds.

## Running it

This repo does not vendor ns-3. You need a local ns-3 checkout, with this repo
beside it:

```text
.
├── multi_hop_drone_forwarding
└── ns-3-dev
```

```sh
./run_sim.sh                                    # defaults
./run_sim.sh --maxRelays=5 --lossThreshold=15   # pass ns-3 args through
./run_sim.sh --csv=results.csv                  # per-tick metrics to CSV
```

### Options

| Flag | Default | Meaning |
| --- | --- | --- |
| `--simTime` | 200 | Simulation duration (s) |
| `--userSpeed` | 5 | User velocity along +x (m/s) |
| `--lossThreshold` | 20 | Windowed loss % that triggers a deployment |
| `--maxRelays` | 3 | Relay drones available |
| `--monitorInterval` | 1 | Seconds between measurements |
| `--cooldown` | 10 | Minimum seconds between deployments |
| `--placementGamma` | 1.0 | Placement exponent: 1 = even, >1 toward base, <1 toward user |
| `--csv` | *(off)* | Write per-tick metrics to this path |
| `--pcap` | off | Enable pcap capture |

CSV columns: `time_s,distance_m,window_loss_pct,cumulative_loss_pct,active_relays`.

## Known limitations

- The user follows a straight line at constant velocity; no mobility model beyond that.
- Relays teleport into position rather than flying there, and re-space instantly
  every interval, so flight time, speed limits, battery, and airspace constraints
  are not modeled. A real drone could not track a 5 m/s user this tightly.
- Placement is searched only over the one-parameter `placementGamma` family, at
  three values. That family contains the even-spacing optimum but is far from
  every possible arrangement — irregular placements were not tested.
- A single UDP echo flow; no competing traffic or interference from other users.
