#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/ssid.h"
#include "ns3/config-store-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneWifiSimulation");

// Globals for tracking
uint64_t g_txPackets = 0;
uint64_t g_rxPackets = 0;
Ptr<Node> g_user;
Ptr<Node> g_ap;

// Collect packet Tx/Rx stats
void TxTrace(Ptr<const Packet> p) { g_txPackets++; }
void RxTrace(Ptr<const Packet> p, const Address &) { g_rxPackets++; }

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

int main(int argc, char *argv[])
{
  Time::SetResolution(Time::NS);
  LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

  NodeContainer baseStation;
  baseStation.Create(1);
  NodeContainer user;
  user.Create(1);
  g_user = user.Get(0);
  g_ap = baseStation.Get(0);

  // Channel + PHY
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211n);
  WifiMacHelper mac;

  Ssid ssid("base-ap");
  mac.SetType("ns3::StaWifiMac",
              "Ssid", SsidValue(ssid),
              "ActiveProbing", BooleanValue(false));
  NetDeviceContainer userDevice = wifi.Install(phy, mac, user);

  mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer apDevice = wifi.Install(phy, mac, baseStation);

  // Mobility
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(user);
  mobility.Install(baseStation);

  user.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(0.0, 0.0, 0.0));
  user.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(5.0, 0.0, 0.0)); // 5 m/s away from spawn

  baseStation.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 0.0));

  InternetStackHelper stack;
  stack.Install(user);
  stack.Install(baseStation);

  Ipv4AddressHelper address;
  address.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign(NetDeviceContainer(userDevice, apDevice));

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
