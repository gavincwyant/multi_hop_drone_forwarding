/* drone_dynamic_deploy_ascii.cc
 *
 * - Linear multi-hop chain with up to N drones.
 * - User moves +X at configurable speed (--userSpeed).
 * - Drones can be staged and dynamically "deployed" (moved into chain) when links deteriorate.
 * - Drones self-balance by adjusting X position to equalize left/right RSSI.
 * - ASCII 1-D visualization printed each monitor tick.
 *
 * Save to scratch/ and run with waf.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/aodv-module.h"

#include <vector>
#include <map>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneDynamicDeployASCII");

NetDeviceContainer CreateWifiHop(Ptr<Node> staNode, Ptr<Node> apNode, const std::string &ssidName);

// ----- Globals & config -----
uint64_t g_txPackets = 0;
uint64_t g_rxPackets = 0;
Ptr<Node> g_user;
Ptr<Node> g_ap;

uint64_t l_rxPackets = 0;
uint64_t l_txPackets = 0;

// Cumulative counters for reporting
uint64_t g_txPackets_total = 0;
uint64_t g_rxPackets_total = 0;
double g_avgRtt_total = 0.0;
uint64_t g_rttSamples_total = 0;

// Sliding window for deployment decisions
uint64_t g_txPackets_window = 0;
uint64_t g_rxPackets_window = 0;
double g_avgRtt_window = 0.0;
uint64_t g_rttSamples_window = 0;

std::map<uint32_t, Time> g_sentTimes;
double g_lastRtt = 0.0;
uint64_t g_rttSamples = 0;
double g_avgRtt = 0.0;

uint32_t g_numDrones = 2;
std::string g_droneInitMode = "deploy"; // even | cluster | deploy
double base_station_x = 0.0;
double g_totalDistance = 0.0;
double g_droneHeight = 10.0;
double g_moveSpeed = 3.0;               // m/s for drone adjustments
double g_rssiMoveThresholdDb = 3.0;     // Distance difference threshold (m) to trigger movement
Time   g_balanceInterval = Seconds(1.0);
Time   g_monitorInterval = Seconds(2.0);
double USER_DEFAULT_SPEED = 2.5;
double g_userSpeed = USER_DEFAULT_SPEED;

// How many meters per ASCII column
double g_asciiStep = 10.0;

// Thresholds for dynamic deployment (tunable) - More aggressive for 3-5 drone deployment
double g_lossDeployThresholdPct = 20.0;   // Was 30.0 - deploy earlier on packet loss
double g_rttDeployThresholdMs = 100.0;    // Was 150.0 - tighter latency requirement
double g_rssiDeployThresholdDb = -65.0;   // Was -75.0 - deploy at weaker signal

// Containers
NodeContainer allNodes;   // [0]=user, [1..N]=drones, [N+1]=ap
NodeContainer droneNodes; // 1..N
std::vector<NetDeviceContainer> hopDevices;
std::vector<Ipv4InterfaceContainer> hopIfaces;
InternetStackHelper internet;

std::vector<bool> droneDeployed; // which drones have been "deployed" into chain (moved from staging)

// ----- Traces -----
void TxTrace(Ptr<const Packet> p)
{
    l_txPackets++;
    g_txPackets++;
    g_txPackets_total++;
    g_txPackets_window++;
    g_sentTimes[p->GetUid()] = Simulator::Now();
}
void RxTrace(Ptr<const Packet> p, const Address &)
{
    l_rxPackets++;
    g_rxPackets++;
    g_rxPackets_total++;
    g_rxPackets_window++;
}
void ClientRxTrace(Ptr<const Packet> p)
{
    uint32_t uid = p->GetUid();
    auto it = g_sentTimes.find(uid);
    if (it != g_sentTimes.end())
    {
        Time rtt = Simulator::Now() - it->second;
        double rttMs = rtt.GetMilliSeconds();
        g_lastRtt = rttMs;

        // Update total average
        g_rttSamples_total++;
        g_avgRtt_total = g_avgRtt_total + (rttMs - g_avgRtt_total) / g_rttSamples_total;

        // Update window average
        g_rttSamples_window++;
        g_avgRtt_window = g_avgRtt_window + (rttMs - g_avgRtt_window) / g_rttSamples_window;

        // Keep old variables for backward compatibility
        g_rttSamples = g_rttSamples_total;
        g_avgRtt = g_avgRtt_total;

        g_sentTimes.erase(it);
    }
}

// ----- RSSI estimator (distance-based like earlier) -----
/*
double rssiCalcFromDistance(double distance)
{
    double Px = 0.03;
    double pathLossExp = 2.0;
    double d0 = 1.0;
    static Ptr<UniformRandomVariable> ur = CreateObject<UniformRandomVariable>();
    double noise = ur->GetValue(5.0, 9.0);
    if (distance <= 0.0) distance = 0.0001;
    double rssi = Px - 10.0 * pathLossExp * std::log10(distance / d0) - noise;
    return rssi;
}*/

double rssiCalcFromDistance(double distance)
{
    // Parameters
    double Pt_dBm = 20.0;         // transmit power in dBm
    double pathLossExp = 2.5;     // realistic urban/suburban path loss
    double d0 = 1.0;              // reference distance in meters
    double noiseStd = 1.0;        // small random noise in dB

    if (distance < d0) distance = d0;  // cap minimum distance

    // log-distance path loss model
    double rssi = Pt_dBm - 10.0 * pathLossExp * std::log10(distance / d0);

    // add small Gaussian noise
    static Ptr<NormalRandomVariable> rng = CreateObject<NormalRandomVariable>();
    rng->SetAttribute("Mean", DoubleValue(0.0));
    rng->SetAttribute("Variance", DoubleValue(noiseStd*noiseStd));

    rssi += rng->GetValue();

    return rssi;
}

// ----- ASCII visualization (sorted by X) -----
void PrintAsciiMap()
{
    uint32_t N = allNodes.GetN();
    std::vector<std::pair<double, std::string>> ents;
    ents.reserve(N);
    double minX = 1e9, maxX = -1e9;

    for (uint32_t i = 0; i < N; ++i)
    {
        Vector p = allNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        std::string label;
        if (i == 0) label = "U";          // user
        else if (i == N-1) label = "A";   // AP
        else {
            std::ostringstream ss; ss << "D" << i; // drones indexed by node index
            label = ss.str();
        }
        ents.emplace_back(p.x, label);
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
    }

    if (maxX - minX < g_asciiStep) maxX = minX + g_asciiStep;

    // Sort by X so display matches physical order
    std::sort(ents.begin(), ents.end(),
              [](const std::pair<double,std::string>& a, const std::pair<double,std::string>& b) {
                  return a.first < b.first;
              });

    uint32_t cols = static_cast<uint32_t>(std::ceil((maxX - minX) / g_asciiStep));
    if (cols < 10) cols = 10;

    std::vector<std::string> row(cols, std::string("-"));

    // place entities; if conflict, later (rightmost in space-sorted order) wins
    for (auto &e : ents)
    {
        uint32_t idx = static_cast<uint32_t>(std::floor((e.first - minX) / g_asciiStep));
        if (idx >= cols) idx = cols - 1;
        row[idx] = e.second;
    }

    std::ostringstream line;
    for (uint32_t c = 0; c < cols; ++c) line << std::setw(4) << row[c];
    NS_LOG_UNCOND("\n[ASCII] " << line.str());

    // print positions under map
    std::ostringstream ruler;
    for (uint32_t c = 0; c < cols; ++c)
    {
        double x = minX + (c + 0.5) * g_asciiStep;
        std::ostringstream sx; sx << std::fixed << std::setprecision(0) << x;
        std::string s = sx.str();
        if (s.size() > 4) s = s.substr(0,4);
        ruler << std::setw(4) << s;
    }
    NS_LOG_UNCOND("[POS]   " << ruler.str());
    NS_LOG_UNCOND("");
}

void DeployNextDroneIfNeeded()
{
    // Use WINDOW metrics for deployment decisions (recent performance)
    double lossRate = 0.0;
    if (g_txPackets_window > 0)
        lossRate = 100.0 * (1.0 - (double)g_rxPackets_window / g_txPackets_window);
    double rtt = g_avgRtt_window;

    // Find weakest hop RSSI in the active chain (not direct User->AP path)
    double minHopRssi = 1000.0;
    std::vector<uint32_t> activeNodes;
    activeNodes.push_back(0);  // User
    for (uint32_t i = 1; i <= g_numDrones; ++i) {
        if (droneDeployed[i-1]) activeNodes.push_back(i);
    }
    activeNodes.push_back(allNodes.GetN() - 1);  // AP

    // Check each hop in the chain
    for (size_t i = 0; i + 1 < activeNodes.size(); ++i) {
        Ptr<MobilityModel> m1 = allNodes.Get(activeNodes[i])->GetObject<MobilityModel>();
        Ptr<MobilityModel> m2 = allNodes.Get(activeNodes[i+1])->GetObject<MobilityModel>();
        double hopDist = m1->GetDistanceFrom(m2);
        double hopRssi = rssiCalcFromDistance(hopDist);
        minHopRssi = std::min(minHopRssi, hopRssi);
    }

    // Deployment criteria: use weakest link RSSI instead of direct path
    bool poor = (lossRate > g_lossDeployThresholdPct) ||
                (rtt > g_rttDeployThresholdMs) ||
                (minHopRssi < g_rssiDeployThresholdDb);

    // Also deploy if any single hop exceeds max distance
    const double MAX_HOP_DISTANCE = 40.0;  // meters
    for (size_t i = 0; i + 1 < activeNodes.size(); ++i) {
        Ptr<MobilityModel> m1 = allNodes.Get(activeNodes[i])->GetObject<MobilityModel>();
        Ptr<MobilityModel> m2 = allNodes.Get(activeNodes[i+1])->GetObject<MobilityModel>();
        double hopDist = m1->GetDistanceFrom(m2);
        if (hopDist > MAX_HOP_DISTANCE) {
            poor = true;
            NS_LOG_UNCOND("[Deploy] Hop distance " << hopDist
                          << "m exceeds max " << MAX_HOP_DISTANCE << "m");
            break;
        }
    }

    int nextIndex = -1;
    for (uint32_t i = 1; i <= g_numDrones; ++i)
    {
        if (!droneDeployed[i-1]) { nextIndex = i; break; }
    }
    if (!poor || nextIndex == -1) return;

    // Compute target X (largest gap in active chain)
    std::vector<double> activeXs;
    activeXs.push_back(allNodes.Get(0)->GetObject<MobilityModel>()->GetPosition().x); // user
    for (uint32_t i = 1; i <= g_numDrones; ++i)
    {
        if (droneDeployed[i-1])
            activeXs.push_back(allNodes.Get(i)->GetObject<MobilityModel>()->GetPosition().x);
    }
    activeXs.push_back(allNodes.Get(allNodes.GetN()-1)->GetObject<MobilityModel>()->GetPosition().x); // AP

    std::sort(activeXs.begin(), activeXs.end());
    double bestGap = -1.0; size_t bestIdx = 0;
    for (size_t j = 0; j + 1 < activeXs.size(); ++j)
    {
        double gap = activeXs[j+1] - activeXs[j];
        if (gap > bestGap) { bestGap = gap; bestIdx = j; }
    }
    double targetX = (activeXs[bestIdx] + activeXs[bestIdx+1]) / 2.0;

    // Move staged drone into chain
    Ptr<Node> droneNode = allNodes.Get(nextIndex);
    Ptr<ConstantVelocityMobilityModel> dm =
        DynamicCast<ConstantVelocityMobilityModel>(droneNode->GetObject<MobilityModel>());
    if (!dm) return;

    Vector oldPos = dm->GetPosition();
    dm->SetPosition(Vector(targetX, oldPos.y, oldPos.z));
    dm->SetVelocity(Vector(0.0, 0.0, 0.0));
    droneDeployed[nextIndex-1] = true;

    // Reset ONLY window metrics for fresh deployment decisions (keep total counters)
    g_txPackets_window = 0;
    g_rxPackets_window = 0;
    g_avgRtt_window = 0.0;
    g_rttSamples_window = 0;
    // Note: g_txPackets, g_rxPackets used by old code - reset for compatibility
    g_txPackets = 0;
    g_rxPackets = 0;

    // AODV handles routing updates automatically - no manual update needed

    NS_LOG_UNCOND("\n[Deploy] Drone node=" << nextIndex << " moved from X="
                   << oldPos.x << " to target X=" << targetX
                   << " due to poor link (loss=" << lossRate
                   << "%, rtt=" << rtt << " ms, weakest_hop_rssi=" << minHopRssi << " dBm)");
}

// ----- Auto-balance movement -----
void AutoBalanceDrones()
{
    if (g_numDrones == 0)
    {
        Simulator::Schedule(g_balanceInterval, &AutoBalanceDrones);
        return;
    }

    // 1. Move deployed drones based on RSSI steering
    for (uint32_t di = 1; di <= g_numDrones; ++di)
    {
        if (!droneDeployed[di-1]) continue;

        Ptr<Node> drone = allNodes.Get(di);
        Ptr<ConstantVelocityMobilityModel> droneMob =
            DynamicCast<ConstantVelocityMobilityModel>(drone->GetObject<MobilityModel>());
        if (!droneMob) continue;

        Ptr<MobilityModel> leftMob  = allNodes.Get(di-1)->GetObject<MobilityModel>();
        Ptr<MobilityModel> rightMob = allNodes.Get(di+1)->GetObject<MobilityModel>();

        double leftDist  = droneMob->GetDistanceFrom(leftMob);
        double rightDist = droneMob->GetDistanceFrom(rightMob);

        // Use distance-based balancing: move toward the farther neighbor to equalize distances
        double diff = leftDist - rightDist;  // positive means left is farther, move left (increase X)
        Vector oldPos = droneMob->GetPosition();
        Vector vel(0,0,0);

        if (diff > g_rssiMoveThresholdDb)  // leftDist > rightDist, move toward left (increase X)
            vel = Vector(g_moveSpeed, 0, 0);
        else if (diff < -g_rssiMoveThresholdDb)  // rightDist > leftDist, move toward right (decrease X)
            vel = Vector(-g_moveSpeed, 0, 0);
        else
            vel = Vector(0, 0, 0);

        droneMob->SetVelocity(vel);

        double dt = g_balanceInterval.GetSeconds();
        double futureX = oldPos.x + vel.x * dt;

        // Allow drones to move freely to follow the user (no artificial boundaries)

        if (std::fabs(futureX - oldPos.x) > 0.001)
        {
            NS_LOG_UNCOND("[Move] Drone node=" << di
                           << " moved from X=" << oldPos.x
                           << " to X=" << futureX
                           << " (Ldist=" << leftDist << "m, Rdist=" << rightDist << "m)");
        }

        droneMob->SetPosition(Vector(futureX, oldPos.y, oldPos.z));
    }

    // Enforce minimum separation and prevent crossing
    const double MIN_SEPARATION = 0.5;  // meters

    std::vector<std::pair<double, Ptr<ConstantVelocityMobilityModel>>> dronePositions;
    for (uint32_t di = 1; di <= g_numDrones; ++di)
    {
        if (!droneDeployed[di-1]) continue;
        Ptr<ConstantVelocityMobilityModel> dm =
            DynamicCast<ConstantVelocityMobilityModel>(allNodes.Get(di)->GetObject<MobilityModel>());
        dronePositions.push_back({dm->GetPosition().x, dm});
    }

    // Sort by X position
    std::sort(dronePositions.begin(), dronePositions.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    // Enforce minimum separation
    for (size_t i = 0; i + 1 < dronePositions.size(); ++i)
    {
        double x1 = dronePositions[i].first;
        double x2 = dronePositions[i+1].first;

        if (x2 - x1 < MIN_SEPARATION)
        {
            double midpoint = (x1 + x2) / 2.0;
            double newX1 = midpoint - MIN_SEPARATION / 2.0;
            double newX2 = midpoint + MIN_SEPARATION / 2.0;

            Vector p1 = dronePositions[i].second->GetPosition();
            Vector p2 = dronePositions[i+1].second->GetPosition();

            dronePositions[i].second->SetPosition(Vector(newX1, p1.y, p1.z));
            dronePositions[i+1].second->SetPosition(Vector(newX2, p2.y, p2.z));

            NS_LOG_UNCOND("[CollisionPrevention] Separated drones: "
                          << x1 << "," << x2 << " -> " << newX1 << "," << newX2);
        }
    }
    // Re-run again later
    Simulator::Schedule(g_balanceInterval, &AutoBalanceDrones);
}


// ----- Create wifi hop helper -----
NetDeviceContainer CreateWifiHop(Ptr<Node> staNode, Ptr<Node> apNode, const std::string &ssidName)
{
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    WifiMacHelper mac;

    Ssid ssid = Ssid(ssidName);

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    NetDeviceContainer sta = wifi.Install(phy, mac, NodeContainer(staNode));

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer ap = wifi.Install(phy, mac, NodeContainer(apNode));

    NetDeviceContainer pair;
    pair.Add(sta.Get(0));
    pair.Add(ap.Get(0));
    return pair;
}

void Monitor(Ptr<WifiPhy> phy, Time interval)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "\n========== " << Simulator::Now().GetSeconds() << "s ==========\n";
    oss << "UserX=" << allNodes.Get(0)->GetObject<MobilityModel>()->GetPosition().x << " m\n";

    // Build active chain
    std::vector<uint32_t> activeNodes;
    activeNodes.push_back(0);  // User
    for (uint32_t i = 1; i <= g_numDrones; ++i) {
        if (droneDeployed[i-1]) activeNodes.push_back(i);
    }
    activeNodes.push_back(allNodes.GetN() - 1);  // AP

    // Print chain topology with positions
    oss << "Chain: ";
    for (size_t i = 0; i < activeNodes.size(); ++i) {
        uint32_t nodeIdx = activeNodes[i];
        double x = allNodes.Get(nodeIdx)->GetObject<MobilityModel>()->GetPosition().x;
        if (nodeIdx == 0) oss << "U";
        else if (nodeIdx == allNodes.GetN() - 1) oss << "A";
        else oss << "D" << nodeIdx;
        oss << "[" << x << "m]";
        if (i + 1 < activeNodes.size()) oss << " <-> ";
    }
    oss << "\n";

    // Print per-hop distances and RSSI
    oss << "Hop Metrics:\n";
    for (size_t i = 0; i + 1 < activeNodes.size(); ++i)
    {
        Ptr<MobilityModel> m1 = allNodes.Get(activeNodes[i])->GetObject<MobilityModel>();
        Ptr<MobilityModel> m2 = allNodes.Get(activeNodes[i+1])->GetObject<MobilityModel>();
        double dist = m1->GetDistanceFrom(m2);
        double rssi = rssiCalcFromDistance(dist);
        oss << "  Hop" << i << " (Node" << activeNodes[i] << "->Node" << activeNodes[i+1]
            << "): dist=" << dist << "m, RSSI=" << rssi << "dBm\n";
    }

    // Overall statistics using TOTAL counters
    double lossTotal = 0.0;
    if (g_txPackets_total > 0)
        lossTotal = 100.0 * (1.0 - (double)g_rxPackets_total / g_txPackets_total);

    oss << "Overall: Tx=" << g_txPackets_total
        << ", Rx=" << g_rxPackets_total
        << ", Loss=" << lossTotal << "%"
        << ", AvgRTT=" << g_avgRtt_total << "ms\n";

    // Window statistics (for deployment)
    double lossWindow = 0.0;
    if (g_txPackets_window > 0)
        lossWindow = 100.0 * (1.0 - (double)g_rxPackets_window / g_txPackets_window);
    oss << "Window: Tx=" << g_txPackets_window
        << ", Rx=" << g_rxPackets_window
        << ", Loss=" << lossWindow << "%"
        << ", AvgRTT=" << g_avgRtt_window << "ms\n";

    NS_LOG_UNCOND(oss.str());

    // ASCII map
    PrintAsciiMap();

    // Check for dynamic deployment
    DeployNextDroneIfNeeded();

    Simulator::Schedule(interval, &Monitor, phy, interval);
}

// ----- main -----
int main(int argc, char *argv[])
{
    CommandLine cmd;
    cmd.AddValue("numDrones", "Number of drone relays (0 = none)", g_numDrones);
    cmd.AddValue("droneInitMode", "Placement: even | cluster | deploy", g_droneInitMode);
    cmd.AddValue("totalDistance", "Meters between user and AP", g_totalDistance);
    cmd.AddValue("userSpeed", "User movement speed (m/s)", g_userSpeed);
    cmd.Parse(argc, argv);

    Time::SetResolution(Time::NS);
    LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
    LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

    // Build nodes: user (0), drones (1..N), ap (N+1)
    allNodes = NodeContainer();
    NodeContainer user;
    user.Create(1);
    g_user = user.Get(0);
    allNodes.Add(g_user);

    droneNodes = NodeContainer();
    if (g_numDrones > 0)
    {
        droneNodes.Create(g_numDrones);
        for (uint32_t i = 0; i < g_numDrones; i++) {
            allNodes.Add(droneNodes.Get(i));
        }
    }

    NodeContainer ap;
    ap.Create(1);
    g_ap = ap.Get(0);
    allNodes.Add(g_ap);

    // Mobility: user (moving) and ap (fixed)
    Ptr<ConstantVelocityMobilityModel> userMob = CreateObject<ConstantVelocityMobilityModel>();
    g_user->AggregateObject(userMob);
    userMob->SetPosition(Vector(0.0, 0.0, 0.0));
    userMob->SetVelocity(Vector(g_userSpeed, 0.0, 0.0)); // moving +g_userSpeed

    Ptr<ConstantPositionMobilityModel> apMob = CreateObject<ConstantPositionMobilityModel>();
    g_ap->AggregateObject(apMob);
    //access point position
    apMob->SetPosition(Vector(base_station_x, 0.0, 0.0));

    // Drone mobility & initial staging positions
    droneDeployed.assign(g_numDrones, false);

    for (uint32_t i = 0; i < g_numDrones; i++)
    {
        Ptr<ConstantVelocityMobilityModel> dm = CreateObject<ConstantVelocityMobilityModel>();
        droneNodes.Get(i)->AggregateObject(dm);
    }

    // initial placement modes:
    // - even: evenly spaced between user and ap (and mark them all deployed)
    // - cluster: cluster near USER (staged near user, mark deployed)
    // - deploy: staged near AP (not deployed) -> will fly out when needed
    if (g_numDrones > 0)
    {
        if (g_droneInitMode == "even")
        {
            for (uint32_t i = 0; i < g_numDrones; ++i)
            {
                double frac = double(i+1) / double(g_numDrones + 1);
                double x = frac * g_totalDistance;
                Ptr<ConstantVelocityMobilityModel> dm = DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
                dm->SetPosition(Vector(x, 0.0, g_droneHeight));
                dm->SetVelocity(Vector(0.0, 0.0, 0.0));
                // mark them as deployed at start
                droneDeployed[i] = true;
                NS_LOG_UNCOND("\n[Init] Drone node=" << (i+1) << " initial deployed at X=" << x);
            }
        }
        else if (g_droneInitMode == "cluster")
        {
            double baseX = 5.0;
            for (uint32_t i = 0; i < g_numDrones; ++i)
            {
                double x = baseX + double(i) * 1.0;
                Ptr<ConstantVelocityMobilityModel> dm = DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
                dm->SetPosition(Vector(x, 0.0, g_droneHeight));
                dm->SetVelocity(Vector(0.0, 0.0, 0.0));
                droneDeployed[i] = true;
                NS_LOG_UNCOND("\n[Init] Drone node=" << (i+1) << " initial deployed at X=" << x);
            }
        }
        else // deploy: staged at AP, not active until deployment
        {
            double baseX = base_station_x;
            for (uint32_t i = 0; i < g_numDrones; ++i)
            {
                double x = baseX - double(i) * 1.0; // staged near AP
                Ptr<ConstantVelocityMobilityModel> dm =
                    DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
                dm->SetPosition(Vector(x, 0.0, g_droneHeight));
                dm->SetVelocity(Vector(0.0, 0.0, 0.0));
                droneDeployed[i] = false; // staged
                NS_LOG_UNCOND("\n[Init] Drone node=" << (i+1) << " staged at X=" << x);
            }
         }
    }

    // Install internet stack with AODV routing on all nodes
    AodvHelper aodv;
    aodv.Set("EnableHello", BooleanValue(true));
    aodv.Set("HelloInterval", TimeValue(Seconds(1.0)));
    aodv.Set("ActiveRouteTimeout", TimeValue(Seconds(10.0)));
    aodv.Set("AllowedHelloLoss", UintegerValue(3));

    internet.SetRoutingHelper(aodv);  // Use AODV instead of static routing
    internet.Install(allNodes);

    // Create single ad-hoc WiFi network for all nodes (required for AODV)
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("HtMcs0"));

    // Ad-hoc MAC - all nodes in same broadcast domain for AODV neighbor discovery
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    // Install WiFi on all nodes
    NetDeviceContainer allDevices = wifi.Install(phy, mac, allNodes);

    // Single subnet for all nodes
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer allInterfaces = address.Assign(allDevices);

    // Server IP for UDP client (AP is last node)
    Ipv4Address serverIp = allInterfaces.GetAddress(allNodes.GetN() - 1);

    // UDP Echo server/client
    uint16_t port = 9;
    UdpEchoServerHelper echoServer(port);
    ApplicationContainer serverApps = echoServer.Install(g_ap);
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(60.0));

    UdpEchoClientHelper echoClient(serverIp, port);
    echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(0.5)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));
    ApplicationContainer clientApps = echoClient.Install(g_user);
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(60.0));

    // traces
    Ptr<UdpEchoClient> clientApp = DynamicCast<UdpEchoClient>(clientApps.Get(0));
    Ptr<UdpEchoServer> serverApp = DynamicCast<UdpEchoServer>(serverApps.Get(0));
    clientApp->TraceConnectWithoutContext("Tx", MakeCallback(&TxTrace));
    serverApp->TraceConnectWithoutContext("Rx", MakeCallback(&RxTrace));
    clientApp->TraceConnectWithoutContext("Rx", MakeCallback(&ClientRxTrace));

    // Monitor using AP's WiFi device
    Ptr<NetDevice> apDevice = allDevices.Get(allNodes.GetN() - 1);
    Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(apDevice);
    Ptr<WifiPhy> phyForMonitor = nullptr;
    if (wifiDev) phyForMonitor = wifiDev->GetPhy();

    // schedule periodic tasks
    Simulator::Schedule(g_balanceInterval, &AutoBalanceDrones);
    Simulator::Schedule(Seconds(2.0), &Monitor, phyForMonitor, g_monitorInterval);

    Simulator::Stop(Seconds(60.0));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
