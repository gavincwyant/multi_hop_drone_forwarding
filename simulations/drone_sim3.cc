/* drone_experiment_deploy.cc
 *
 * - User starts near base station and walks +X at configurable speed
 * - Direct user<->BS WiFi link exists at t=0
 * - Up to N drones staged (near AP or other modes)
 * - When metrics cross thresholds, next staged drone is "deployed" into the chain
 * - Deployed drones self-position to balance left/right RSSI
 * - ASCII visualization + deployment/move logs
 *
 * Build: place in scratch/ and run with waf
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

NS_LOG_COMPONENT_DEFINE("DroneExperimentDeploy");

// -------------------- Globals & config --------------------
uint64_t g_txPackets = 0;
uint64_t g_rxPackets = 0;
Ptr<Node> g_user;
Ptr<Node> g_ap;
std::map<uint32_t, Time> g_sentTimes;
double g_lastRtt = 0.0;
uint64_t g_rttSamples = 0;
double g_avgRtt = 0.0;

// CLI-configurable
uint32_t g_numDrones = 3;
std::string g_droneInitMode = "deploy"; // even | cluster | deploy
double g_totalDistance = 100.0;
double g_droneHeight = 10.0;
double g_moveSpeed = 1.0;             // m/s movement per balance tick
double g_rssiMoveThresholdDb = 3.0;   // dB threshold to move
Time   g_balanceInterval = Seconds(1.0);
Time   g_monitorInterval = Seconds(2.0);
double USER_DEFAULT_SPEED = 5.0;
double g_userSpeed = USER_DEFAULT_SPEED;

// ASCII map step (meters per column)
double g_asciiStep = 5.0;

// deployment thresholds (tweakable)
double g_lossDeployThresholdPct = 10.0;
double g_rttDeployThresholdMs = 80.0;
double g_rssiDeployThresholdDb = -70.0; // if direct RSSI below this, consider deploy

// Node containers / devices
NodeContainer allNodes;    // [0]=user, [1..N]=drones, [N+1]=ap
NodeContainer droneNodes;  // nodes 1..N
NetDeviceContainer directUserApDev; // direct device pair (user<->ap)
Ipv4InterfaceContainer directIface; // interface for direct link
std::vector<NetDeviceContainer> hopDevices; // user<->d1, d1<->d2, ..., dN<->ap
std::vector<Ipv4InterfaceContainer> hopIfaces; // matching IPs per hop

InternetStackHelper internet;

// flags for which staged drones are deployed into the chain
std::vector<bool> droneDeployed;

// -------------------- Trace callbacks --------------------
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

// -------------------- RSSI / pathloss (safe) --------------------
double rssiFromDistance(double distanceMeters)
{
  // clamp distance to avoid log(0) / huge positive RSSI
  const double minDist = 1.0; // 1 meter reference to avoid overflow
  if (distanceMeters < minDist) distanceMeters = minDist;

  double txPowerDbm = 0.0; // choose 0 dBm as transmit power baseline
  double pathLossExp = 3.0;
  double d0 = 1.0;
  static Ptr<UniformRandomVariable> ur = CreateObject<UniformRandomVariable>();
  double shadowDb = ur->GetValue(5.0, 9.0); // random shadowing
  double rssi = txPowerDbm - 10.0 * pathLossExp * std::log10(distanceMeters / d0) - shadowDb;
  return rssi;
}

// -------------------- ASCII visualization --------------------
void PrintAsciiMap()
{
  uint32_t N = allNodes.GetN();
  std::vector<std::pair<double, std::string>> ents;
  double minX = 1e9, maxX = -1e9;
  for (uint32_t i = 0; i < N; ++i)
  {
    Vector p = allNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
    std::string label;
    if (i == 0) label = "U";
    else if (i == N-1) label = "A";
    else { std::ostringstream ss; ss << "D" << i; label = ss.str(); }
    ents.emplace_back(p.x, label);
    minX = std::min(minX, p.x);
    maxX = std::max(maxX, p.x);
  }

  double pad = g_asciiStep * 2.0;
  minX -= pad; maxX += pad;
  if (maxX - minX < g_asciiStep) maxX = minX + g_asciiStep;
  uint32_t cols = static_cast<uint32_t>(std::ceil((maxX - minX) / g_asciiStep));
  if (cols < 10) cols = 10;

  std::vector<std::string> row(cols, std::string(" - "));
  for (auto &e : ents)
  {
    int idx = static_cast<int>(std::floor((e.first - minX) / g_asciiStep));
    if (idx < 0) idx = 0;
    if ((uint32_t)idx >= cols) idx = cols - 1;
    row[idx] = e.second;
  }

  std::ostringstream line;
  for (uint32_t c = 0; c < cols; ++c)
    line << std::setw(4) << row[c];
  NS_LOG_UNCOND("[ASCII] " << line.str());

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
}

// -------------------- Routing helper to force chain after k deployed --------------------
void InstallChainRoutesUpTo(uint32_t k) // k = number of deployed drones (0..g_numDrones)
{
  // k=0 -> no chain (we rely on default/global routing/direct link)
  // k>=1 -> route: user -> drone1 -> drone2 -> ... -> drone_k -> ap
  // We'll add host routes for server on user, and for server on each deployed drone, and for user on ap.

  // basic helpers
  Ipv4StaticRoutingHelper staticHelper;

  // server (AP) address = hopIfaces[lastHop].GetAddress(1)
  Ipv4Address serverAddr = hopIfaces.back().GetAddress(1);
  Ipv4Address userAddr = hopIfaces.front().GetAddress(0);

  // On user: add host route for server via drone1's IP on hop 0 (user <-> drone1)
  Ptr<Ipv4> ipv4User = allNodes.Get(0)->GetObject<Ipv4>();
  Ptr<Ipv4StaticRouting> userStatic = staticHelper.GetStaticRouting(ipv4User);
  // find interface index for the user device used in hopDevices[0]
  if (k >= 1)
  {
    // nextHop is the IP of drone1 on hop 0 (right side)
    Ipv4Address nextHop = hopIfaces[0].GetAddress(1);
    uint32_t ifIndex = ipv4User->GetInterfaceForDevice(hopDevices[0].Get(0)); // user-side device
    if (ifIndex == -1) ifIndex = 1; // fallback
    userStatic->AddHostRouteTo(serverAddr, nextHop, ifIndex);
    NS_LOG_UNCOND("[Route] User host-route to server " << serverAddr << " via " << nextHop);
  }
  else
  {
    // k==0: ensure any earlier host route is not present — we won't remove here for simplicity.
    NS_LOG_UNCOND("[Route] k=0, keeping default routing (direct link)");
  }

  // On each deployed drone i (node index i), route server via hop i (drone i -> drone i+1)
  for (uint32_t i = 1; i <= k; ++i)
  {
    Ptr<Node> drone = allNodes.Get(i);
    Ptr<Ipv4> ipv4Drone = drone->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> droneStatic = staticHelper.GetStaticRouting(ipv4Drone);
    uint32_t hopIndex = i; // hop i connects node i and i+1
    // nextHop IP is right side of hopIndex (node i+1)
    Ipv4Address nextHop = hopIfaces[hopIndex].GetAddress(1);
    uint32_t ifIndex = ipv4Drone->GetInterfaceForDevice(hopDevices[hopIndex].Get(0)); // drone side device
    if (ifIndex == -1)
    {
      // fallback: pick first non-loopback interface
      ifIndex = 1;
    }
    droneStatic->AddHostRouteTo(serverAddr, nextHop, ifIndex);
    NS_LOG_UNCOND("[Route] Drone node=" << i << " host-route to server " << serverAddr << " via " << nextHop);
  }

  // On AP: add host route back to user via the left neighbor (either last deployed drone or direct)
  Ptr<Ipv4> ipv4Ap = allNodes.Get(allNodes.GetN()-1)->GetObject<Ipv4>();
  Ptr<Ipv4StaticRouting> apStatic = staticHelper.GetStaticRouting(ipv4Ap);

  if (k == 0)
  {
    // Let global routing handle replies via direct link
    NS_LOG_UNCOND("[Route] AP using default routing (direct link to user).");
  }
  else
  {
    // last hop index connecting last deployed drone and AP is hopIndex = (numberOfHops) = g_numDrones
    // But we want actual last deployed drone node index = k
    // hop connecting node k and k+1 is hopIndex = k
    Ipv4Address nextHopToUser = hopIfaces[k].GetAddress(0); // left side = drone_k
    uint32_t ifIndexOnAp = ipv4Ap->GetInterfaceForDevice(hopDevices[k].Get(1)); // ap device on that hop
    if (ifIndexOnAp == -1) ifIndexOnAp = 1;
    apStatic->AddHostRouteTo(userAddr, nextHopToUser, ifIndexOnAp);
    NS_LOG_UNCOND("[Route] AP host-route to user " << userAddr << " via " << nextHopToUser);
  }
}

// -------------------- Deploy logic --------------------
void DeployNextDroneIfNeeded()
{
  // compute metrics
  double lossRate = 0.0;
  if (g_txPackets > 0) lossRate = 100.0 * (1.0 - (double)g_rxPackets / g_txPackets);
  double rtt = g_avgRtt;

  Ptr<MobilityModel> userMob = allNodes.Get(0)->GetObject<MobilityModel>();
  Ptr<MobilityModel> apMob = allNodes.Get(allNodes.GetN()-1)->GetObject<MobilityModel>();
  double directDist = userMob->GetDistanceFrom(apMob);
  double directRssi = rssiFromDistance(directDist);

  bool poor = (lossRate > g_lossDeployThresholdPct) || (rtt > g_rttDeployThresholdMs) || (directRssi < g_rssiDeployThresholdDb);

  // find next staged drone index (node index)
  int nextStagedNodeIndex = -1;
  for (uint32_t i = 0; i < g_numDrones; ++i)
  {
    if (!droneDeployed[i]) { nextStagedNodeIndex = i + 1; break; } // node indices 1..N
  }
  if (!poor || nextStagedNodeIndex == -1) return; // nothing to do

  // compute target X: place new drone in middle of largest gap among active nodes (user + deployed drones + ap)
  std::vector<double> activeXs;
  activeXs.push_back(allNodes.Get(0)->GetObject<MobilityModel>()->GetPosition().x);
  for (uint32_t i = 1; i <= g_numDrones; ++i)
  {
    if (droneDeployed[i-1])
      activeXs.push_back(allNodes.Get(i)->GetObject<MobilityModel>()->GetPosition().x);
  }
  activeXs.push_back(allNodes.Get(allNodes.GetN()-1)->GetObject<MobilityModel>()->GetPosition().x);

  std::sort(activeXs.begin(), activeXs.end());
  double bestGap = -1.0; size_t bestIdx = 0;
  for (size_t j = 0; j + 1 < activeXs.size(); ++j)
  {
    double gap = activeXs[j+1] - activeXs[j];
    if (gap > bestGap) { bestGap = gap; bestIdx = j; }
  }
  double targetX = (activeXs[bestIdx] + activeXs[bestIdx+1]) / 2.0;

  // move staged drone node into place
  Ptr<Node> droneNode = allNodes.Get(nextStagedNodeIndex);
  Ptr<ConstantVelocityMobilityModel> dm = DynamicCast<ConstantVelocityMobilityModel>(droneNode->GetObject<MobilityModel>());
  if (!dm) return;
  Vector old = dm->GetPosition();
  dm->SetPosition(Vector(targetX, old.y, old.z));
  dm->SetVelocity(Vector(0.0, 0.0, 0.0));
  droneDeployed[nextStagedNodeIndex - 1] = true;

  NS_LOG_UNCOND("[Deploy] Drone node=" << nextStagedNodeIndex << " deployed from X=" << old.x << " to X=" << targetX
                                     << " (loss=" << lossRate << "%, rtt=" << rtt << "ms, directRssi=" << directRssi << " dBm)");

  // Install chain routes for k = number of deployed drones
  uint32_t deployedCount = 0;
  for (auto b : droneDeployed) if (b) deployedCount++;
  InstallChainRoutesUpTo(deployedCount);
}

// -------------------- Self-balance movement --------------------
void AutoBalanceDrones()
{
  if (g_numDrones == 0) { Simulator::Schedule(g_balanceInterval, &AutoBalanceDrones); return; }

  for (uint32_t di = 1; di <= g_numDrones; ++di)
  {
    if (!droneDeployed[di-1]) continue; // only active drones move

    Ptr<Node> drone = allNodes.Get(di);
    Ptr<ConstantVelocityMobilityModel> droneMob = DynamicCast<ConstantVelocityMobilityModel>(drone->GetObject<MobilityModel>());
    if (!droneMob) continue;

    Ptr<MobilityModel> leftMob = allNodes.Get(di-1)->GetObject<MobilityModel>();
    Ptr<MobilityModel> rightMob = allNodes.Get(di+1)->GetObject<MobilityModel>();

    double leftDist = droneMob->GetDistanceFrom(leftMob);
    double rightDist = droneMob->GetDistanceFrom(rightMob);

    double leftRssi = rssiFromDistance(leftDist);
    double rightRssi = rssiFromDistance(rightDist);

    double diff = leftRssi - rightRssi;
    Vector oldPos = droneMob->GetPosition();
    Vector vel(0.0, 0.0, 0.0);

    if (diff > g_rssiMoveThresholdDb) vel = Vector(g_moveSpeed, 0.0, 0.0);       // move right
    else if (diff < -g_rssiMoveThresholdDb) vel = Vector(-g_moveSpeed, 0.0, 0.0); // move left
    else vel = Vector(0.0, 0.0, 0.0);

    droneMob->SetVelocity(vel);
    double dt = g_balanceInterval.GetSeconds();
    double futureX = oldPos.x + vel.x * dt;

    double minX = allNodes.Get(0)->GetObject<MobilityModel>()->GetPosition().x + 0.1;
    double maxX = allNodes.Get(allNodes.GetN()-1)->GetObject<MobilityModel>()->GetPosition().x - 0.1;
    if (futureX < minX) futureX = minX;
    if (futureX > maxX) futureX = maxX;

    if (std::fabs(futureX - oldPos.x) > 0.0001)
    {
      NS_LOG_UNCOND("[Move] Drone node=" << di << " moved from X=" << oldPos.x << " to X=" << futureX
                                        << " (L=" << std::fixed << std::setprecision(2) << leftRssi
                                        << " dBm, R=" << rightRssi << " dBm)");
    }
    droneMob->SetPosition(Vector(futureX, oldPos.y, oldPos.z));
  }

  Simulator::Schedule(g_balanceInterval, &AutoBalanceDrones);
}

// -------------------- Create a WiFi hop between two nodes --------------------
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

  NetDeviceContainer pair; pair.Add(sta.Get(0)); pair.Add(ap.Get(0));
  return pair;
}

// -------------------- Monitor --------------------
void Monitor(Ptr<WifiPhy> phy, Time interval)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << Simulator::Now().GetSeconds() << "s: UserX=" << allNodes.Get(0)->GetObject<MobilityModel>()->GetPosition().x << " m, ";

  double total = 0.0;
  for (uint32_t i = 0; i + 1 < allNodes.GetN(); ++i)
  {
    double d = allNodes.Get(i)->GetObject<MobilityModel>()->GetDistanceFrom(allNodes.Get(i+1)->GetObject<MobilityModel>());
    total += d;
    oss << "seg" << i << "-" << (i+1) << "=" << d << "m, ";
  }

  double lossRate = 0.0;
  if (g_txPackets > 0) lossRate = 100.0 * (1.0 - (double)g_rxPackets / g_txPackets);
  double rssi = rssiFromDistance(allNodes.Get(0)->GetObject<MobilityModel>()->GetDistanceFrom(allNodes.Get(allNodes.GetN()-1)->GetObject<MobilityModel>()));
  oss << "Tx=" << g_txPackets << ", Rx=" << g_rxPackets << ", loss=" << lossRate << "%, RSSI=" << std::setprecision(2) << rssi << " dBm, RTT=" << g_avgRtt << " ms";

  NS_LOG_UNCOND(oss.str());

  // ASCII map
  PrintAsciiMap();

  // dynamic deploy check
  DeployNextDroneIfNeeded();

  Simulator::Schedule(interval, &Monitor, phy, interval);
}

// -------------------- main --------------------
int main(int argc, char *argv[])
{
  CommandLine cmd;
  cmd.AddValue("numDrones", "Maximum number of drone relays (0 = none)", g_numDrones);
  cmd.AddValue("droneInitMode", "Placement mode: even | cluster | deploy", g_droneInitMode);
  cmd.AddValue("totalDistance", "Distance between user and AP (meters)", g_totalDistance);
  cmd.AddValue("userSpeed", "User movement speed (m/s)", g_userSpeed);
  cmd.Parse(argc, argv);

  Time::SetResolution(Time::NS);
  LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

  // Create nodes: user (0), drones 1..N, ap (N+1)
  allNodes = NodeContainer();
  NodeContainer user; user.Create(1); g_user = user.Get(0); allNodes.Add(g_user);

  droneNodes = NodeContainer();
  if (g_numDrones > 0)
  {
    droneNodes.Create(g_numDrones);
    for (uint32_t i = 0; i < g_numDrones; ++i) allNodes.Add(droneNodes.Get(i));
  }

  NodeContainer ap; ap.Create(1); g_ap = ap.Get(0); allNodes.Add(g_ap);

  // Mobility: user and AP and drones
  // user starts near AP at x=1.0, AP at 0.0 (very close)
  Ptr<ConstantVelocityMobilityModel> userMob = CreateObject<ConstantVelocityMobilityModel>();
  g_user->AggregateObject(userMob);
  userMob->SetPosition(Vector(1.0, 0.0, 0.0));
  userMob->SetVelocity(Vector(g_userSpeed, 0.0, 0.0)); // moving +X

  Ptr<ConstantPositionMobilityModel> apMob = CreateObject<ConstantPositionMobilityModel>();
  g_ap->AggregateObject(apMob);
  apMob->SetPosition(Vector(0.0, 0.0, 0.0));

  // Drone mobility objects (staged)
  droneDeployed.assign(g_numDrones, false);
  for (uint32_t i = 0; i < g_numDrones; ++i)
  {
    Ptr<ConstantVelocityMobilityModel> dm = CreateObject<ConstantVelocityMobilityModel>();
    droneNodes.Get(i)->AggregateObject(dm);
  }

  // initial staging placement
  if (g_numDrones > 0)
  {
    if (g_droneInitMode == "even")
    {
      for (uint32_t i = 0; i < g_numDrones; ++i)
      {
        double frac = double(i+1)/double(g_numDrones+1);
        double x = frac * g_totalDistance;
        Ptr<ConstantVelocityMobilityModel> dm = DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
        dm->SetPosition(Vector(x, 0.0, g_droneHeight));
        dm->SetVelocity(Vector(0.0,0.0,0.0));
        droneDeployed[i] = true; // already active
        NS_LOG_UNCOND("[Init] Drone node=" << (i+1) << " active at X=" << x);
      }
    }
    else if (g_droneInitMode == "cluster")
    {
      double baseX = 1.0 + 5.0; // near user
      for (uint32_t i = 0; i < g_numDrones; ++i)
      {
        double x = baseX + double(i) * 1.0;
        Ptr<ConstantVelocityMobilityModel> dm = DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
        dm->SetPosition(Vector(x, 0.0, g_droneHeight));
        dm->SetVelocity(Vector(0.0,0.0,0.0));
        droneDeployed[i] = true;
        NS_LOG_UNCOND("[Init] Drone node=" << (i+1) << " active at X=" << x);
      }
    }
    else // deploy mode: staged near AP and not yet deployed
    {
      double baseX = 0.0 + 2.0; // very near AP
      for (uint32_t i = 0; i < g_numDrones; ++i)
      {
        double x = baseX + double(i) * 1.0; // staged marching out from AP
        Ptr<ConstantVelocityMobilityModel> dm = DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
        dm->SetPosition(Vector(x, 0.0, g_droneHeight));
        dm->SetVelocity(Vector(0.0,0.0,0.0));
        droneDeployed[i] = false; // staged, not active
        NS_LOG_UNCOND("[Init] Drone node=" << (i+1) << " staged at X=" << x);
      }
    }
  }

  // Install internet stack
  internet.Install(allNodes);

  // Create direct link user <-> AP (so initial connectivity exists)
  directUserApDev = CreateWifiHop(g_user, g_ap, "base-ap-direct");
  Ipv4AddressHelper directAddr;
  directAddr.SetBase("10.10.100.0", "255.255.255.0");
  directIface = directAddr.Assign(directUserApDev);

  // Create chain hops (user <-> d1, d1<->d2, ..., dN <-> ap)
  hopDevices.clear(); hopIfaces.clear();
  for (uint32_t hop = 0; hop <= g_numDrones; ++hop)
  {
    Ptr<Node> left = allNodes.Get(hop);
    Ptr<Node> right = allNodes.Get(hop+1);
    std::ostringstream ss; ss << "chain-ssid-" << hop;
    NetDeviceContainer pair = CreateWifiHop(left, right, ss.str());
    hopDevices.push_back(pair);

    Ipv4AddressHelper addr;
    std::ostringstream base; base << "10.1." << (hop+1) << ".0";
    addr.SetBase(base.str().c_str(), "255.255.255.0");
    Ipv4InterfaceContainer ifc = addr.Assign(pair);
    hopIfaces.push_back(ifc);
  }

  // remember IPs
  Ipv4Address serverIp = directIface.GetAddress(1); // AP side of direct link
  Ipv4Address userIp = directIface.GetAddress(0);

  // populate initial routing tables (direct link used by default)
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  // Applications (UDP echo) -- server on AP (direct interface)
  uint16_t port = 9;
  UdpEchoServerHelper echoServer(port);
  ApplicationContainer serverApps = echoServer.Install(g_ap);
  serverApps.Start(Seconds(1.0));
  serverApps.Stop(Seconds(120.0));

  UdpEchoClientHelper echoClient(serverIp, port);
  echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
  echoClient.SetAttribute("Interval", TimeValue(Seconds(0.5)));
  echoClient.SetAttribute("PacketSize", UintegerValue(1024));
  ApplicationContainer clientApps = echoClient.Install(g_user);
  clientApps.Start(Seconds(2.0));
  clientApps.Stop(Seconds(120.0));

  // Traces
  Ptr<UdpEchoClient> clientApp = DynamicCast<UdpEchoClient>(clientApps.Get(0));
  Ptr<UdpEchoServer> serverApp = DynamicCast<UdpEchoServer>(serverApps.Get(0));
  clientApp->TraceConnectWithoutContext("Tx", MakeCallback(&TxTrace));
  serverApp->TraceConnectWithoutContext("Rx", MakeCallback(&RxTrace));
  clientApp->TraceConnectWithoutContext("Rx", MakeCallback(&ClientRxTrace));

  // Monitor phy (use AP side of direct link to print stats)
  Ptr<WifiNetDevice> apDirectDev = DynamicCast<WifiNetDevice>(directUserApDev.Get(1));
  Ptr<WifiPhy> monitorPhy = nullptr;
  if (apDirectDev) monitorPhy = apDirectDev->GetPhy();

  // schedule periodic tasks
  Simulator::Schedule(g_balanceInterval, &AutoBalanceDrones);
  Simulator::Schedule(Seconds(2.0), &Monitor, monitorPhy, g_monitorInterval);

  Simulator::Stop(Seconds(120.0));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
