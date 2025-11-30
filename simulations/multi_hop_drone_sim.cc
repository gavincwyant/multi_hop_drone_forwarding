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

std::map<uint32_t, Time> g_sentTimes;
double g_lastRtt = 0.0;
uint64_t g_rttSamples = 0;
double g_avgRtt = 0.0;

uint32_t g_numDrones = 2;
std::string g_droneInitMode = "deploy"; // even | cluster | deploy
double base_station_x = 0.0;
double g_totalDistance = 0.0;
double g_droneHeight = 10.0;
double g_moveSpeed = 3.0;           // m/s for drone adjustments
double g_rssiMoveThresholdDb = 3.0; // dB to prefer movement
Time   g_balanceInterval = Seconds(1.0);
Time   g_monitorInterval = Seconds(2.0);
double USER_DEFAULT_SPEED = 2.5;
double g_userSpeed = USER_DEFAULT_SPEED;

// How many meters per ASCII column
double g_asciiStep = 10.0;

// Thresholds for dynamic deployment (tunable)
double g_lossDeployThresholdPct = 30.0;
double g_rttDeployThresholdMs = 150.0;
double g_rssiDeployThresholdDb = -75.0; // RSSI below this (dBm) triggers eventual deployment

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
    g_sentTimes[p->GetUid()] = Simulator::Now();
}
void RxTrace(Ptr<const Packet> p, const Address &)
{
    l_rxPackets++;
    g_rxPackets++;
}
void ClientRxTrace(Ptr<const Packet> p)
{
    uint32_t uid = p->GetUid();
    auto it = g_sentTimes.find(uid);
    if (it != g_sentTimes.end())
    {
        Time rtt = Simulator::Now() - it->second;
        g_lastRtt = rtt.GetMilliSeconds();
        g_rttSamples++;
        g_avgRtt = g_avgRtt + (g_lastRtt - g_avgRtt) / g_rttSamples;
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
    // compute metrics
    double lossRate = 0.0;
    if (g_txPackets > 0) lossRate = 100.0 * (1.0 - (double)g_rxPackets / g_txPackets);
    double rtt = g_avgRtt;
    Ptr<MobilityModel> userMob = allNodes.Get(0)->GetObject<MobilityModel>();
    Ptr<MobilityModel> apMob = allNodes.Get(allNodes.GetN()-1)->GetObject<MobilityModel>();
    double directDist = userMob->GetDistanceFrom(apMob);
    double directRssi = rssiCalcFromDistance(directDist);

    bool poor = (lossRate > g_lossDeployThresholdPct) ||
                (rtt > g_rttDeployThresholdMs) ||
                (directRssi < g_rssiDeployThresholdDb);

    int nextIndex = -1;
    for (uint32_t i = 1; i <= g_numDrones; ++i)
    {
        if (!droneDeployed[i-1]) { nextIndex = i; break; }
    }
    if (!poor || nextIndex == -1) return;

    // Compute target X (largest gap in active chain)
    std::vector<double> activeXs;
    activeXs.push_back(userMob->GetPosition().x); // user
    for (uint32_t i = 1; i <= g_numDrones; ++i)
    {
        if (droneDeployed[i-1])
            activeXs.push_back(allNodes.Get(i)->GetObject<MobilityModel>()->GetPosition().x);
    }
    activeXs.push_back(apMob->GetPosition().x); // AP

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

    // Reset metrics so next deployment happens based on fresh stats
    g_txPackets = 0;
    g_rxPackets = 0;
    g_avgRtt = 0.0;
    g_rttSamples = 0;
    g_sentTimes.clear();

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

NS_LOG_UNCOND("\n--- ROUTING TABLES AFTER DEPLOY ---");

// Create an OutputStreamWrapper for stdout
Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(&std::cout);

// Print every node’s routing table
Ipv4GlobalRoutingHelper g;
for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
    g.PrintRoutingTableAt(Seconds(Simulator::Now().GetSeconds()), allNodes.Get(i), stream);
}

    NS_LOG_UNCOND("\n[Deploy] Drone node=" << nextIndex << " moved from X="
                   << oldPos.x << " to target X=" << targetX
                   << " due to poor link (loss=" << lossRate
                   << "%, rtt=" << rtt << " ms, rssi=" << directRssi << " dBm)");
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

        double leftRssi  = rssiCalcFromDistance(leftDist);
        double rightRssi = rssiCalcFromDistance(rightDist);
        //double diff      = leftRssi - rightRssi;
        double diff      = leftDist - rightDist;
        Vector oldPos = droneMob->GetPosition();
        Vector vel(0,0,0);

        if (diff > g_rssiMoveThresholdDb)
            vel = Vector(g_moveSpeed, 0, 0);
        else if (diff < -g_rssiMoveThresholdDb)
            vel = Vector(-g_moveSpeed, 0, 0);
        else
            vel = Vector(0, 0, 0);

        droneMob->SetVelocity(vel);

        double dt = g_balanceInterval.GetSeconds();
        double futureX = oldPos.x + vel.x * dt;

        // respect User/AP boundaries
        double userX = allNodes.Get(0)->GetObject<MobilityModel>()->GetPosition().x;
        double apX   = allNodes.Get(allNodes.GetN()-1)->GetObject<MobilityModel>()->GetPosition().x;
        double minX  = std::min(userX, apX) + 0.1;
        double maxX  = std::max(userX, apX) - 0.1;

        if (futureX < minX) futureX = minX;
        if (futureX > maxX) futureX = maxX;

        if (std::fabs(futureX - oldPos.x) > 0.001)
        {
            NS_LOG_UNCOND("[Move] Drone node=" << di
                           << " moved from X=" << oldPos.x
                           << " to X=" << futureX
                           << " (Lrssi=" << leftRssi << " dB, Rrssi=" << rightRssi << " dB)");
        }

        droneMob->SetPosition(Vector(futureX, oldPos.y, oldPos.z));
    }

    /*
    // -------------------------------------------------------------------
    // 2. NEW FIX: prevent drones from crossing each other (CRITICAL)
    // -------------------------------------------------------------------
    for (uint32_t i = 1; i < g_numDrones; ++i)
    {
        Ptr<MobilityModel> m1 = allNodes.Get(i)->GetObject<MobilityModel>();
        Ptr<MobilityModel> m2 = allNodes.Get(i+1)->GetObject<MobilityModel>();

        double x1 = m1->GetPosition().x;
        double x2 = m2->GetPosition().x;

        if (x1 >= x2)   // crossing detected!
        {
            double mid = (x1 + x2) / 2.0;

            // push them slightly apart, tiny epsilon
            double leftNewX  = mid - 0.01;
            double rightNewX = mid + 0.01;

            // set positions
            Vector p1 = m1->GetPosition();
            Vector p2 = m2->GetPosition();

            m1->SetPosition(Vector(leftNewX,  p1.y, p1.z));
            m2->SetPosition(Vector(rightNewX, p2.y, p2.z));

            NS_LOG_UNCOND("[OrderFix] Corrected crossing between Drone "
                          << i << " and Drone " << (i+1)
                          << " -> newX=(" << leftNewX << ", " << rightNewX << ")");
        }
    }
    */
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
    // Print user X and distances along chain
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << Simulator::Now().GetSeconds() << "s: UserX=" << allNodes.Get(0)->GetObject<MobilityModel>()->GetPosition().x << " m, ";

    //double total = 0.0;
    for (uint32_t i = 0; i + 1 < allNodes.GetN(); ++i)
    {
        double dx = allNodes.Get(i)->GetObject<MobilityModel>()->GetPosition().x -
            allNodes.Get(i+1)->GetObject<MobilityModel>()->GetPosition().x;
        //double segDist = std::abs(dx);
        //total += dx;
        oss << "seg" << i << "-" << (i+1) << "=" << dx << "m, ";
    }

    double loss = 0.0;
    if (g_txPackets > 0) loss = 100.0 * (1.0 - (double)g_rxPackets / g_txPackets);
    double rssi = rssiCalcFromDistance(allNodes.Get(0)->GetObject<MobilityModel>()->GetDistanceFrom(allNodes.Get(allNodes.GetN()-1)->GetObject<MobilityModel>()));
    oss << "Tx=" << g_txPackets << ", Rx=" << g_rxPackets << ", loss=" << loss << "%, RSSI=" << rssi << " dBm, RTT=" << g_avgRtt << " ms";

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

    // Install internet stack on all nodes
    internet.Install(allNodes);

    // Create wifi hops for consecutive nodes: user -> d1 -> d2 -> ... -> dn -> ap
    hopDevices.clear();
    hopIfaces.clear();
    for (uint32_t hop = 0; hop <= g_numDrones; ++hop)
    {
        Ptr<Node> left = allNodes.Get(hop);
        Ptr<Node> right = allNodes.Get(hop+1);
        std::ostringstream ss; ss << "ssid-hop-" << hop;
        NetDeviceContainer pair = CreateWifiHop(left, right, ss.str());
        hopDevices.push_back(pair);

        Ipv4AddressHelper addr;
        std::ostringstream base; base << "10.1." << (hop + 1) << ".0";
        addr.SetBase(base.str().c_str(), "255.255.255.0");
        Ipv4InterfaceContainer ifc = addr.Assign(pair);
        hopIfaces.push_back(ifc);
    }

    // remember IPs
    Ipv4Address serverIp = hopIfaces.back().GetAddress(1); // AP address (right side of last hop)
    //Ipv4Address userIp = hopIfaces.front().GetAddress(0);

    Ipv4GlobalRoutingHelper::RecomputeRoutingTables();

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

    // monitor phy pick last hop AP device
    Ptr<NetDevice> apDevOnLastHop = hopDevices.back().Get(1);
    Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(apDevOnLastHop);
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
