#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/ssid.h"
#include "ns3/config-store-module.h"
#include "ns3/simulator.h"
#include "ns3/building.h"
#include "ns3/building-container.h"
#include "ns3/buildings-module.h"
#include "ns3/hybrid-buildings-propagation-loss-model.h"
#include "ns3/mobility-building-info.h"
#include "ns3/log.h"
#include <map>
#include "ns3/wifi-module.h"
#include "ns3/flow-monitor-module.h"
#include <iostream>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BuildingsExample");

// Globals for tracking
uint64_t g_txPackets = 0;
uint64_t g_rxPackets = 0;
Ptr<Node> g_user;
Ptr<Node> g_ap;

// RTT tracking
std::map<uint32_t, Time> g_sentTimes;
double g_lastRtt = 0.0;
uint64_t g_rttSamples = 0;
double g_avgRtt = 0.0;

// Collect packet Tx/Rx stats
void TxTrace(Ptr<const Packet> p) { g_txPackets++; }
void RxTrace(Ptr<const Packet> p, const Address &) { g_rxPackets++; }

void ClientRxTrace(Ptr<const Packet> p)
{
  uint32_t uid = p->GetUid();
  auto it = g_sentTimes.find(uid);

  if (it != g_sentTimes.end())
  {
    Time rtt = Simulator::Now() - it->second;
    g_lastRtt = rtt.GetMilliSeconds();

    // Calculate running average
    g_rttSamples++;
    g_avgRtt = g_avgRtt + (g_lastRtt - g_avgRtt) / g_rttSamples;

    g_sentTimes.erase(it);
  }
}

// Periodically print network stats
void Monitor(Time interval)
{
  Ptr<MobilityModel> userMob = g_user->GetObject<MobilityModel>();
  Ptr<MobilityModel> apMob = g_ap->GetObject<MobilityModel>();
  double distance = userMob->GetDistanceFrom(apMob);

  double lossRate = 0.0;
  if (g_txPackets > 0)
    lossRate = 100.0 * (1.0 - (double)g_rxPackets / g_txPackets);

  // Print timestamp, distance, and loss
  std::cout << Simulator::Now().GetSeconds() << "s: "
            << "Distance=" << distance << "m, "
            << "Tx=" << g_txPackets << ", Rx=" << g_rxPackets
            << " (" << lossRate << "% loss)"
            << std::endl;

  // Schedule next check
  Simulator::Schedule(interval, &Monitor, interval);
}

int main(int argc, char *argv[]) {
  Time::SetResolution(Time::NS);
  LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

  // Define two rows of buildings creating a corridor:
  // Row 1 (Left side, X=0 to 10)
  // Box(double _xMin, double _xMax, double _yMin, double _yMax, double _zMin, double _zMax)
  for (int i = 0; i < 30; ++i) {
      Ptr<Building> b = CreateObject<Building>();
      b->SetBoundaries(Box(i*10.0, (i + 1)*10.0, -210.0, -200.0, 0.0, 15.0));
      b->SetNFloors(5);
      b->SetBuildingType(Building::Office);
  }
  // Row 2 (Right side, X=30 to 40)
  for (int i = 0; i < 30; ++i) {
      Ptr<Building> b = CreateObject<Building>();
      b->SetBoundaries(Box(i*10.0, (i + 1)*10.0, 200.0, 210.0, 0.0, 15.0));
      b->SetNFloors(5);
      b->SetBuildingType(Building::Office);
  }
  // Corridor of buildings spans from x = 0 to x = 300
  // y coordinates are fixed so that buildings are lined up on both sides of the nodes. The corridor is 30 units in width

  NodeContainer baseStation;
  baseStation.Create(1);
  NodeContainer user;
  user.Create(1);
  g_user = user.Get(0);
  g_ap = baseStation.Get(0);

  NodeContainer allNodes;
  allNodes.Add(baseStation);
  allNodes.Add(user);

  // Mobility
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(user);
  mobility.Install(baseStation);

  user.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(0.0, 0.0, 0.5));
  user.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(5.0, 0.0, 0.0)); // 5 m/s away from spawn

  baseStation.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 0.5));

  InternetStackHelper stack;
  stack.Install(user);
  stack.Install(baseStation);

  // Make the nodes building aware
  //BuildingsHelper::Install(user);
  //BuildingsHelper::Install(baseStation);
  BuildingsHelper::Install(allNodes);

  // Channel + PHY
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
  channel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel",
                            "Frequency", DoubleValue(2.4e9),
                            "RooftopLevel", DoubleValue(15.0),
                            "Environment", StringValue("Urban"));
  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());

  phy.Set("TxPowerStart", DoubleValue(15.0));
  phy.Set("TxPowerEnd", DoubleValue(15.0));

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211n);

  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                              "DataMode", StringValue("DsssRate11Mbps"),
                              "ControlMode", StringValue("DsssRate1Mbps"));

  WifiMacHelper mac;

  Ssid ssid = Ssid("UrbanCanyonNet");
  mac.SetType("ns3::StaWifiMac",
              "Ssid", SsidValue(ssid));
  NetDeviceContainer userDevice = wifi.Install(phy, mac, user);

  mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer apDevice = wifi.Install(phy, mac, baseStation);

  Ipv4AddressHelper address;
  address.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign(NetDeviceContainer(userDevice, apDevice));

  phy.EnablePcapAll("urban-canyon");    

  // UDP Echo
  uint16_t port = 9;
  UdpEchoServerHelper echoServer(port);
  ApplicationContainer serverApps = echoServer.Install(baseStation.Get(0));
  serverApps.Start(Seconds(1.0));
  serverApps.Stop(Seconds(60.0));

  UdpEchoClientHelper echoClient(interfaces.GetAddress(1), port);
  echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
  echoClient.SetAttribute("Interval", TimeValue(Seconds(0.5)));
  echoClient.SetAttribute("PacketSize", UintegerValue(1024));

  ApplicationContainer clientApps = echoClient.Install(user.Get(0));
  clientApps.Start(Seconds(2.0));
  clientApps.Stop(Seconds(60.0));

  // Connect traces for packet tracking
  Ptr<UdpEchoClient> clientApp = DynamicCast<UdpEchoClient>(clientApps.Get(0));
  Ptr<UdpEchoServer> serverApp = DynamicCast<UdpEchoServer>(serverApps.Get(0));

  clientApp->TraceConnectWithoutContext("Tx", MakeCallback(&TxTrace));
  serverApp->TraceConnectWithoutContext("Rx", MakeCallback(&RxTrace));

  // Start periodic monitoring
  Simulator::Schedule(Seconds(2.0), &Monitor, Seconds(2.0));

  phy.EnablePcapAll("drone_wifi_simulation");

  Simulator::Stop(Seconds(60.0));
  Simulator::Run();
  Simulator::Destroy();
  
  return 0;
}