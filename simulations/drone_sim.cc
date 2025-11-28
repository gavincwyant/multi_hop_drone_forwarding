/* drone_linear_chain_selfpos.cc
 *
 * Linear chain of N drones with self-positioning (1D x-axis).
 * Command-line:
 *   --numDrones=N
 *   --droneInitMode=even|cluster|deploy
 *
 * Preserves original Tx/Rx/RTT monitoring.
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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneLinearChainSelfPos");

// Global stats (preserve original)
uint64_t g_txPackets = 0;
uint64_t g_rxPackets = 0;
Ptr<Node> g_user;
Ptr<Node> g_ap; // base station

// RTT tracking (preserve original)
std::map<uint32_t, Time> g_sentTimes;
double g_lastRtt = 0.0;
uint64_t g_rttSamples = 0;
double g_avgRtt = 0.0;

// Configurable parameters
uint32_t g_numDrones = 3;
std::string g_droneInitMode = "even"; // even | cluster | deploy
double g_totalDistance = 100.0;       // meters between user and AP on x-axis
double g_droneHeight = 10.0;          // z coordinate for drones
double g_moveSpeed = 1.0;             // m/s when a drone moves to rebalance
double g_rssiMoveThresholdDb = 3.0;   // dB difference to trigger movement
Time   g_balanceInterval = Seconds(1.0);
Time   g_monitorInterval = Seconds(2.0);

// Containers
NodeContainer allNodes;               // user + drones + ap
NodeContainer droneNodes;             // just drones
std::vector<NetDeviceContainer> hopDevices; // per-hop NetDeviceContainer (sta, ap)
std::vector<Ipv4InterfaceContainer> hopIfaces; // per-hop interfaces
InternetStackHelper g_stackHelper;

// For trace hooking
void TxTrace(Ptr<const Packet> p)
{
  g_txPackets++;
  g_sentTimes[p->GetUid()] = Simulator::Now();
}

void RxTrace(Ptr<const Packet> p, const Address &)
{
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

// Simple RSSI estimator (same style as original)
double rssiCalcFromDistance(double distance)
{
  double Px = 0.05; // placeholder transmit power (dBm)
  double pathLossExponent = 3.0;
  double originDistance = 1.0;
  // small random component to avoid ties
  static Ptr<UniformRandomVariable> uf = CreateObject<UniformRandomVariable>();
  double noise = uf->GetValue(5.0, 9.0);
  if (distance <= 0.0) distance = 0.0001;
  double rssi = Px - 10.0 * pathLossExponent * std::log10(distance / originDistance) - noise;
  return rssi;
}

// Monitor similar to your original Monitor(), using first AP's phy (if exists)
void Monitor(Ptr<WifiPhy> phy, Time interval)
{
  Ptr<MobilityModel> userMob = g_user->GetObject<MobilityModel>();
  Ptr<MobilityModel> apMob = g_ap->GetObject<MobilityModel>();
  double distance = userMob->GetDistanceFrom(apMob);

  double lossRate = 0.0;
  if (g_txPackets > 0)
    lossRate = 100.0 * (1.0 - (double)g_rxPackets / g_txPackets);

  double rssi_value = rssiCalcFromDistance(distance);

  std::cout << Simulator::Now().GetSeconds() << "s: "
            << "Distance=" << distance << "m, "
            << "Tx=" << g_txPackets << ", Rx=" << g_rxPackets
            << " (" << lossRate << "% loss), "
            << "RSSI=" << rssi_value << " dBm, "
            << "RTT=" << g_avgRtt << " ms"
            << std::endl;

  // Schedule next check
  Simulator::Schedule(interval, &Monitor, phy, interval);
}

// Auto-balance drones along x-axis: each drone compares left vs right RSSI and moves toward the weaker link
void AutoBalanceDrones()
{
  if (g_numDrones == 0)
  {
    Simulator::Schedule(g_balanceInterval, &AutoBalanceDrones);
    return;
  }

  // Node indexing:
  // allNodes: [0] = user, [1..N] = drones (if any), [N+1] = ap
  uint32_t totalNodes = allNodes.GetN(); // should be g_numDrones + 2

  // We only move drones (indices 1..N)
  for (uint32_t di = 1; di <= g_numDrones; ++di)
  {
    Ptr<Node> drone = allNodes.Get(di);
    Ptr<ConstantVelocityMobilityModel> droneMob = DynamicCast<ConstantVelocityMobilityModel>(drone->GetObject<MobilityModel>());
    if (!droneMob) continue;

    // left neighbor is node di-1, right neighbor is node di+1
    Ptr<MobilityModel> leftMob = allNodes.Get(di - 1)->GetObject<MobilityModel>();
    Ptr<MobilityModel> rightMob = allNodes.Get(di + 1)->GetObject<MobilityModel>();

    double leftDist = droneMob->GetDistanceFrom(leftMob);
    double rightDist = droneMob->GetDistanceFrom(rightMob);

    double leftRssi = rssiCalcFromDistance(leftDist);
    double rightRssi = rssiCalcFromDistance(rightDist);

    double diff = leftRssi - rightRssi;

    // Movement decision:
    // If left much stronger than right (diff > threshold) -> move right (positive x)
    // If right much stronger than left (diff < -threshold) -> move left (negative x)
    Vector currentPos = droneMob->GetPosition();
    Vector vel(0.0, 0.0, 0.0);

    if (diff > g_rssiMoveThresholdDb)
    {
      // move right toward the right neighbor
      vel = Vector(g_moveSpeed, 0.0, 0.0);
    }
    else if (diff < -g_rssiMoveThresholdDb)
    {
      // move left toward the left neighbor
      vel = Vector(-g_moveSpeed, 0.0, 0.0);
    }
    else
    {
      // hold position
      vel = Vector(0.0, 0.0, 0.0);
    }

    droneMob->SetVelocity(vel);

    // Optionally clamp position so drones don't cross user/AP
    // We'll compute a small future position and clamp bounds [userX, apX]
    Ptr<MobilityModel> userMob = g_user->GetObject<MobilityModel>();
    Ptr<MobilityModel> apMob = g_ap->GetObject<MobilityModel>();
    double userX = userMob->GetPosition().x;
    double apX = apMob->GetPosition().x;
    double dt = g_balanceInterval.GetSeconds();
    double futureX = currentPos.x + vel.x * dt;

    // bounds with a small margin
    double minX = std::min(userX, apX) + 0.1;
    double maxX = std::max(userX, apX) - 0.1;
    if (futureX < minX) futureX = minX;
    if (futureX > maxX) futureX = maxX;

    // Apply future position immediately (this gives smoother movement)
    droneMob->SetPosition(Vector(futureX, currentPos.y, currentPos.z));
  }

  // Schedule next balance check
  Simulator::Schedule(g_balanceInterval, &AutoBalanceDrones);
}

// Helper: create a WIFI hop between node A (STA) and node B (AP) with unique SSID and own PHY/channel.
// Returns NetDeviceContainer {staDev, apDev}
NetDeviceContainer CreateWifiHop(Ptr<Node> staNode, Ptr<Node> apNode, std::string ssidName)
{
  // Create channel & phy for this hop
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();

  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211n);
  WifiMacHelper mac;

  Ssid ssid = Ssid(ssidName);

  mac.SetType("ns3::StaWifiMac",
              "Ssid", SsidValue(ssid),
              "ActiveProbing", BooleanValue(false));
  NetDeviceContainer staDev = wifi.Install(phy, mac, NodeContainer(staNode));

  mac.SetType("ns3::ApWifiMac",
              "Ssid", SsidValue(ssid));
  NetDeviceContainer apDev = wifi.Install(phy, mac, NodeContainer(apNode));

  NetDeviceContainer pair;
  pair.Add(staDev.Get(0));
  pair.Add(apDev.Get(0));
  return pair;
}

int main(int argc, char *argv[])
{
  // Command-line params
  CommandLine cmd;
  cmd.AddValue("numDrones", "Number of drone relays (0 = none)", g_numDrones);
  cmd.AddValue("droneInitMode", "Placement: even | cluster | deploy", g_droneInitMode);
  cmd.AddValue("totalDistance", "Distance (meters) between user and AP", g_totalDistance);
  cmd.Parse(argc, argv);

  Time::SetResolution(Time::NS);
  LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

  // Create nodes: user (0), drones (1..N), ap (N+1)
  allNodes = NodeContainer();
  NodeContainer user;
  user.Create(1);
  g_user = user.Get(0);
  allNodes.Add(user);

  droneNodes = NodeContainer();
  if (g_numDrones > 0)
  {
    droneNodes.Create(g_numDrones);
    for (uint32_t i = 0; i < g_numDrones; ++i)
      allNodes.Add(droneNodes.Get(i));
  }

  NodeContainer baseStation;
  baseStation.Create(1);
  g_ap = baseStation.Get(0);
  allNodes.Add(baseStation);

  // Mobility: user at x=0, ap at x=totalDistance
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(allNodes.Get(0)); // user
  mobility.Install(g_ap);           // ap

  // place user and ap
  Ptr<ConstantPositionMobilityModel> userMob = DynamicCast<ConstantPositionMobilityModel>(g_user->GetObject<MobilityModel>());
  Ptr<ConstantPositionMobilityModel> apMob = DynamicCast<ConstantPositionMobilityModel>(g_ap->GetObject<MobilityModel>());
  userMob->SetPosition(Vector(0.0, 0.0, 0.0));
  apMob->SetPosition(Vector(g_totalDistance, 0.0, 0.0));

  // Drone mobility: use ConstantVelocityMobilityModel to allow movement
  for (uint32_t i = 0; i < g_numDrones; ++i)
  {
    Ptr<Node> dn = droneNodes.Get(i);
    Ptr<ConstantVelocityMobilityModel> cv = CreateObject<ConstantVelocityMobilityModel>();
    dn->AggregateObject(cv); // install
  }

  // Initial placement of drones according to mode
  // Node indexing reminder: user=0, drones 1..N, ap=N+1
  if (g_numDrones > 0)
  {
    if (g_droneInitMode == "even")
    {
      // evenly spaced between user (0) and ap (totalDistance)
      for (uint32_t i = 0; i < g_numDrones; ++i)
      {
        double frac = double(i + 1) / double(g_numDrones + 1);
        double x = frac * g_totalDistance;
        Ptr<ConstantVelocityMobilityModel> dm = DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
        dm->SetPosition(Vector(x, 0.0, g_droneHeight));
        dm->SetVelocity(Vector(0.0, 0.0, 0.0));
      }
    }
    else if (g_droneInitMode == "cluster")
    {
      // cluster near the USER at small offsets
      double baseX = 0.0 + 5.0; // 5 m from user
      for (uint32_t i = 0; i < g_numDrones; ++i)
      {
        double x = baseX + double(i) * 1.0; // 1m spacing
        Ptr<ConstantVelocityMobilityModel> dm = DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
        dm->SetPosition(Vector(x, 0.0, g_droneHeight));
        dm->SetVelocity(Vector(0.0, 0.0, 0.0));
      }
    }
    else if (g_droneInitMode == "deploy")
    {
      // all drones start near the AP and will "fly out"
      double baseX = g_totalDistance - 5.0; // 5 m from AP
      for (uint32_t i = 0; i < g_numDrones; ++i)
      {
        double x = baseX - double(i) * 1.0; // packed near AP
        Ptr<ConstantVelocityMobilityModel> dm = DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
        dm->SetPosition(Vector(x, 0.0, g_droneHeight));
        // Give initial small negative velocity so they "fly" toward user
        dm->SetVelocity(Vector(-0.5, 0.0, 0.0));
      }
    }
    else
    {
      NS_LOG_UNCOND("Unknown droneInitMode '" << g_droneInitMode << "'. Defaulting to even spacing.");
      for (uint32_t i = 0; i < g_numDrones; ++i)
      {
        double frac = double(i + 1) / double(g_numDrones + 1);
        double x = frac * g_totalDistance;
        Ptr<ConstantVelocityMobilityModel> dm = DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
        dm->SetPosition(Vector(x, 0.0, g_droneHeight));
        dm->SetVelocity(Vector(0.0, 0.0, 0.0));
      }
    }
  }

  // Install Internet stack on all nodes
  g_stackHelper.Install(allNodes);

  // Build links (hops) between consecutive nodes: user -> d1 -> d2 -> ... -> dn -> ap
  hopDevices.clear();
  hopIfaces.clear();

  // We'll create per-hop WiFi STA (left node) and AP (right node), with unique SSID per hop
  for (uint32_t hop = 0; hop <= g_numDrones; ++hop)
  {
    // left node index = hop, right node index = hop+1
    Ptr<Node> leftNode = allNodes.Get(hop);
    Ptr<Node> rightNode = allNodes.Get(hop + 1);

    std::ostringstream ss;
    ss << "hop-ssid-" << hop;
    std::string ssidName = ss.str();

    NetDeviceContainer devPair = CreateWifiHop(leftNode, rightNode, ssidName);
    hopDevices.push_back(devPair);

    // Assign IP subnet for this hop: 10.1.(hop+1).0
    Ipv4AddressHelper addr;
    std::ostringstream base;
    base << "10.1." << (hop + 1) << ".0";
    addr.SetBase(base.str().c_str(), "255.255.255.0");

    Ipv4InterfaceContainer ifc = addr.Assign(devPair);
    hopIfaces.push_back(ifc);
  }

  // At this point:
  // hopIfaces[0] = interfaces between user (0) and node 1 (drone1 or ap if numDrones==0)
  // ...
  // hopIfaces[N] = interfaces between node N and node N+1 (ap)

  // Extract server (AP) IP: it is the right-side address of the last hop (hop = g_numDrones)
  Ipv4Address serverIp = hopIfaces.back().GetAddress(1); // right side is AP
  Ipv4Address userIp   = hopIfaces.front().GetAddress(0); // left side of first hop is user

  NS_LOG_UNCOND("Server (AP) IP: " << serverIp);
  NS_LOG_UNCOND("User IP: " << userIp);

  // Populate routing tables
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  // Applications: UDP Echo Server on AP node (g_ap)
  uint16_t port = 9;
  UdpEchoServerHelper echoServer(port);
  ApplicationContainer serverApps = echoServer.Install(g_ap);
  serverApps.Start(Seconds(1.0));
  serverApps.Stop(Seconds(60.0));

  // UDP Echo Client on user node, target serverIp
  UdpEchoClientHelper echoClient(serverIp, port);
  echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
  echoClient.SetAttribute("Interval", TimeValue(Seconds(0.5)));
  echoClient.SetAttribute("PacketSize", UintegerValue(1024));

  ApplicationContainer clientApps = echoClient.Install(g_user);
  clientApps.Start(Seconds(2.0));
  clientApps.Stop(Seconds(60.0));

  // Trace connections for Tx/Rx/ClientRx
  Ptr<UdpEchoClient> clientApp = DynamicCast<UdpEchoClient>(clientApps.Get(0));
  Ptr<UdpEchoServer> serverApp = DynamicCast<UdpEchoServer>(serverApps.Get(0));
  clientApp->TraceConnectWithoutContext("Tx", MakeCallback(&TxTrace));
  serverApp->TraceConnectWithoutContext("Rx", MakeCallback(&RxTrace));
  clientApp->TraceConnectWithoutContext("Rx", MakeCallback(&ClientRxTrace));

  // Pick a phy to pass to Monitor (we can use the AP side of last hop)
  // Find a WifiNetDevice among hopDevices.back()
  Ptr<NetDevice> apDevOnLastHop = hopDevices.back().Get(1);
  Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(apDevOnLastHop);
  Ptr<WifiPhy> monitorPhy = nullptr;
  if (wifiDev)
  {
    monitorPhy = wifiDev->GetPhy();
  }

  // Start auto-balance (self-positioning)
  Simulator::Schedule(g_balanceInterval, &AutoBalanceDrones);

  // Start monitor (if we found a phy)
  if (monitorPhy)
  {
    Simulator::Schedule(Seconds(2.0), &Monitor, monitorPhy, g_monitorInterval);
  }
  else
  {
    // fallback: schedule monitor with a null phy (we only use it for printing)
    Simulator::Schedule(Seconds(2.0), &Monitor, Ptr<WifiPhy>(), g_monitorInterval);
  }

  // Optionally enable PCAP on AP devices for debugging (first and last hop APs)
  // Enable pcap on each AP netdevice if you like; here we enable for last hop AP
  // Note: enabling pcap for each phy requires access to the phy helpers; we used local phys per hop so cannot easily pcap all here
  // If desired, user can enable pcap per CreateWifiHop by exposing the phy.

  Simulator::Stop(Seconds(60.0));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
