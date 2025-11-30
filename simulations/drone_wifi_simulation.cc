#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/ssid.h"
#include "ns3/config-store-module.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneWifiSimulation");

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
double g_instantaneousRtt = 0.0;

// Collect packet Tx/Rx stats
void TxTrace(Ptr<const Packet> p)
{
  g_txPackets++;
  // Store send time for RTT calculation
  g_sentTimes[p->GetUid()] = Simulator::Now();
}

void RxTrace(Ptr<const Packet> p, const Address &)
{
  g_rxPackets++;
}

// Client receives echo response
void ClientRxTrace(Ptr<const Packet> p)
{
  uint32_t uid = p->GetUid();
  auto it = g_sentTimes.find(uid);

  if (it != g_sentTimes.end())
  {
    Time rtt = Simulator::Now() - it->second;
    g_lastRtt = rtt.GetSeconds() * 1000.0;
    g_instantaneousRtt = g_lastRtt;

    g_rttSamples++;
    g_avgRtt = g_avgRtt + (g_lastRtt - g_avgRtt) / g_rttSamples;

    std::cout << "  [Packet #" << g_rttSamples << "] RTT: "
              << g_instantaneousRtt << "ms, Running Avg: " << g_avgRtt << "ms" << std::endl;

    g_sentTimes.erase(it);
  }
}

// Monitor definition
void Monitor(Ptr<WifiPhy>, Time);

// rssiCalc definition
double rssiCalc(Ptr<WifiPhy>, Ptr<MobilityModel>, Ptr<MobilityModel>, double);

// Periodically print network stats
void Monitor(Ptr<WifiPhy> phy, Time interval)
{
  Ptr<MobilityModel> userMob = g_user->GetObject<MobilityModel>();
  Ptr<MobilityModel> apMob = g_ap->GetObject<MobilityModel>();
  double distance = userMob->GetDistanceFrom(apMob);

  double lossRate = 0.0;
  if (g_txPackets > 0)
    lossRate = 100.0 * (1.0 - (double)g_rxPackets / g_txPackets);

  double rssi_value = rssiCalc(phy, userMob, apMob, distance);

  // Print timestamp, distance, and loss
  std::cout << Simulator::Now().GetSeconds() << "s: "
            << "Distance=" << distance << "m, "
            << "Tx=" << g_txPackets << ", Rx=" << g_rxPackets
            << " (" << lossRate << "% loss), "
            << "RSSI Value= " << rssi_value << ", "
            << "RTT Avg= " << g_avgRtt << "ms, "
            << "Last RTT= " << g_instantaneousRtt << "ms"
            << std::endl;

  // Schedule next check
  Simulator::Schedule(interval, &Monitor, phy, interval);
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
  clientApp->TraceConnectWithoutContext("Rx", MakeCallback(&ClientRxTrace));

  Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(apDevice.Get(0));
  Ptr<WifiPhy> phyPtr = wifiDevice->GetPhy();

  // Start periodic monitoring
  Simulator::Schedule(Seconds(2.0), &Monitor, phyPtr, Seconds(2.0));

  phy.EnablePcapAll("drone_wifi_simulation");

  Simulator::Stop(Seconds(60.0));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}

double rssiCalc(Ptr<WifiPhy> phy, Ptr<MobilityModel> mobility1, Ptr<MobilityModel> mobility2, double distance)
{
  // RSSI = P - 10a*log(d/d0) + Xg

  double txPowerDbm = phy->GetTxPowerStart(); // in dBm

  double pathLossExponent = 3.0; // typical urban area

  double originDistance = 1.0; // reference distance in meters

  // in dB, random value between 5 and 9 dB
  Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();

  double minNoise = 5.0; // in dB
  double maxNoise = 9.0;

  rand->SetAttribute("Min", DoubleValue(minNoise));
  rand->SetAttribute("Max", DoubleValue(maxNoise));

  double noise = rand->GetValue(); // in dB

  double rssi = txPowerDbm - 10 * pathLossExponent * std::log10(distance / originDistance) + noise;

  //NS_LOG_UNCOND("Time: " << Simulator::Now().GetSeconds() << "s, Distance: " << distance << "m, RSSI: " << rssi << " dBm" << std::endl);

  return rssi;

}
