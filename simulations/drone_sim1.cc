/* drone_linear_chain_selfpos.cc
 *
 * Linear chain of N drones with self-positioning (1D x-axis) and moving user.
 * Command-line:
 *   --numDrones=N
 *   --droneInitMode=even|cluster|deploy
 *   --userSpeed=<m/s>
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

// ----------------------- Global Stats -----------------------
uint64_t g_txPackets = 0;
uint64_t g_rxPackets = 0;
Ptr<Node> g_user;
Ptr<Node> g_ap; // base station
std::map<uint32_t, Time> g_sentTimes;
double g_lastRtt = 0.0;
uint64_t g_rttSamples = 0;
double g_avgRtt = 0.0;

// ----------------------- Configuration ----------------------
uint32_t g_numDrones = 3;
std::string g_droneInitMode = "even";
double g_totalDistance = 100.0;       // meters between user and AP
double g_droneHeight = 10.0;          // drones z-coordinate
double g_moveSpeed = 1.0;             // m/s drone adjustment speed
double g_rssiMoveThresholdDb = 3.0;   // dB diff threshold to move drone
Time   g_balanceInterval = Seconds(1.0);
Time   g_monitorInterval = Seconds(2.0);
double USER_DEFAULT_SPEED = 5.0;      // m/s
double g_userSpeed = USER_DEFAULT_SPEED;

// ----------------------- Node Containers -------------------
NodeContainer allNodes;     // user + drones + AP
NodeContainer droneNodes;   // just drones
std::vector<NetDeviceContainer> hopDevices;
std::vector<Ipv4InterfaceContainer> hopIfaces;
InternetStackHelper g_stackHelper;

// ----------------------- Trace Callbacks -------------------
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

// ----------------------- RSSI / Distance -------------------
double rssiCalcFromDistance(double distance)
{
    double Px = 0.05; // dBm
    double pathLossExponent = 3.0;
    double originDistance = 1.0;
    static Ptr<UniformRandomVariable> uf = CreateObject<UniformRandomVariable>();
    double noise = uf->GetValue(5.0, 9.0);
    if (distance <= 0.0) distance = 0.0001;
    double rssi = Px - 10.0 * pathLossExponent * std::log10(distance / originDistance) - noise;
    return rssi;
}

// ----------------------- Monitoring -------------------------
void Monitor(Ptr<WifiPhy> phy, Time interval)
{
    double totalPath = 0.0;
    std::ostringstream oss;
    oss << Simulator::Now().GetSeconds() << "s: ";
    oss << "User X=" << g_user->GetObject<MobilityModel>()->GetPosition().x << " m, ";

    // Distances along chain
    for (uint32_t i = 0; i < allNodes.GetN() - 1; ++i)
    {
        double dist = allNodes.Get(i)->GetObject<MobilityModel>()->GetDistanceFrom(allNodes.Get(i+1)->GetObject<MobilityModel>());
        totalPath += dist;
        oss << "D" << i << "→D" << i+1 << "=" << dist << "m, ";
    }

    double lossRate = 0.0;
    if (g_txPackets > 0)
        lossRate = 100.0 * (1.0 - (double)g_rxPackets / g_txPackets);

    double rssi_value = rssiCalcFromDistance(allNodes.Get(0)->GetObject<MobilityModel>()->GetDistanceFrom(allNodes.Get(allNodes.GetN()-1)->GetObject<MobilityModel>()));

    oss << "Tx=" << g_txPackets << ", Rx=" << g_rxPackets << " (" << lossRate << "% loss), ";
    oss << "RSSI=" << rssi_value << " dBm, RTT=" << g_avgRtt << " ms";

    NS_LOG_UNCOND(oss.str());

    Simulator::Schedule(interval, &Monitor, phy, interval);
}

// ----------------------- Drone Auto-Balance -----------------
void AutoBalanceDrones()
{
    if (g_numDrones == 0)
    {
        Simulator::Schedule(g_balanceInterval, &AutoBalanceDrones);
        return;
    }

    for (uint32_t di = 1; di <= g_numDrones; ++di)
    {
        Ptr<Node> drone = allNodes.Get(di);
        Ptr<ConstantVelocityMobilityModel> droneMob = DynamicCast<ConstantVelocityMobilityModel>(drone->GetObject<MobilityModel>());
        if (!droneMob) continue;

        Ptr<MobilityModel> leftMob = allNodes.Get(di-1)->GetObject<MobilityModel>();
        Ptr<MobilityModel> rightMob = allNodes.Get(di+1)->GetObject<MobilityModel>();

        double leftDist = droneMob->GetDistanceFrom(leftMob);
        double rightDist = droneMob->GetDistanceFrom(rightMob);

        double leftRssi = rssiCalcFromDistance(leftDist);
        double rightRssi = rssiCalcFromDistance(rightDist);
        double diff = leftRssi - rightRssi;

        Vector oldPos = droneMob->GetPosition();
        Vector vel(0,0,0);

        if (diff > g_rssiMoveThresholdDb)
        {
            vel = Vector(g_moveSpeed, 0, 0); // move toward right
        }
        else if (diff < -g_rssiMoveThresholdDb)
        {
            vel = Vector(-g_moveSpeed,0,0); // move toward left
        }
        else
        {
            vel = Vector(0,0,0);
        }
        droneMob->SetVelocity(vel);

        double dt = g_balanceInterval.GetSeconds();
        double futureX = oldPos.x + vel.x * dt;

        double userX = g_user->GetObject<MobilityModel>()->GetPosition().x;
        double apX   = g_ap->GetObject<MobilityModel>()->GetPosition().x;

        double minX = std::min(userX, apX)+0.1;
        double maxX = std::max(userX, apX)-0.1;
        if (futureX < minX) futureX = minX;
        if (futureX > maxX) futureX = maxX;

        if (fabs(futureX - oldPos.x) > 0.001)
        {
            NS_LOG_UNCOND("Drone " << di << " moved from X=" << oldPos.x << " m to X=" << futureX << " m");
        }
        droneMob->SetPosition(Vector(futureX, oldPos.y, oldPos.z));
    }

    Simulator::Schedule(g_balanceInterval, &AutoBalanceDrones);
}

// ----------------------- Create WiFi Hop --------------------
NetDeviceContainer CreateWifiHop(Ptr<Node> staNode, Ptr<Node> apNode, std::string ssidName)
{
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    WifiMacHelper mac;

    Ssid ssid(ssidName);

    mac.SetType("ns3::StaWifiMac","Ssid",SsidValue(ssid),"ActiveProbing",BooleanValue(false));
    NetDeviceContainer staDev = wifi.Install(phy, mac, NodeContainer(staNode));

    mac.SetType("ns3::ApWifiMac","Ssid",SsidValue(ssid));
    NetDeviceContainer apDev = wifi.Install(phy, mac, NodeContainer(apNode));

    NetDeviceContainer pair;
    pair.Add(staDev.Get(0));
    pair.Add(apDev.Get(0));
    return pair;
}

// ----------------------- Main -----------------------------
int main(int argc, char *argv[])
{
    CommandLine cmd;
    cmd.AddValue("numDrones","Number of drone relays (0=none)", g_numDrones);
    cmd.AddValue("droneInitMode","Placement mode: even|cluster|deploy", g_droneInitMode);
    cmd.AddValue("totalDistance","Distance between user and AP", g_totalDistance);
    cmd.AddValue("userSpeed","Speed of moving user (m/s)", g_userSpeed);
    cmd.Parse(argc, argv);

    Time::SetResolution(Time::NS);
    LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
    LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

    // Create nodes
    allNodes = NodeContainer();
    NodeContainer user; user.Create(1); g_user=user.Get(0); allNodes.Add(g_user);
    droneNodes = NodeContainer(); if (g_numDrones>0){droneNodes.Create(g_numDrones); for(uint32_t i=0;i<g_numDrones;i++) allNodes.Add(droneNodes.Get(i));}
    NodeContainer baseStation; baseStation.Create(1); g_ap=baseStation.Get(0); allNodes.Add(g_ap);

    // Mobility: user + AP
    Ptr<ConstantVelocityMobilityModel> userMob = CreateObject<ConstantVelocityMobilityModel>();
    g_user->AggregateObject(userMob);
    userMob->SetPosition(Vector(0,0,0));
    userMob->SetVelocity(Vector(g_userSpeed,0,0));

    Ptr<ConstantPositionMobilityModel> apMob = CreateObject<ConstantPositionMobilityModel>();
    g_ap->AggregateObject(apMob);
    apMob->SetPosition(Vector(g_totalDistance,0,0));

    // Drone mobility
    for(uint32_t i=0;i<g_numDrones;i++)
    {
        Ptr<ConstantVelocityMobilityModel> dm = CreateObject<ConstantVelocityMobilityModel>();
        droneNodes.Get(i)->AggregateObject(dm);
    }

    // Drone initial positions
    if(g_numDrones>0)
    {
        if(g_droneInitMode=="even")
        {
            for(uint32_t i=0;i<g_numDrones;i++)
            {
                double frac=double(i+1)/double(g_numDrones+1);
                Ptr<ConstantVelocityMobilityModel> dm=DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
                dm->SetPosition(Vector(frac*g_totalDistance,0,g_droneHeight));
                dm->SetVelocity(Vector(0,0,0));
                NS_LOG_UNCOND("Drone " << i+1 << " deployed at X=" << frac*g_totalDistance << " m");
            }
        }
        else if(g_droneInitMode=="cluster")
        {
            double baseX=5.0;
            for(uint32_t i=0;i<g_numDrones;i++)
            {
                double x=baseX+double(i)*1.0;
                Ptr<ConstantVelocityMobilityModel> dm=DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
                dm->SetPosition(Vector(x,0,g_droneHeight));
                dm->SetVelocity(Vector(0,0,0));
                NS_LOG_UNCOND("Drone " << i+1 << " deployed at X=" << x << " m");
            }
        }
        else if(g_droneInitMode=="deploy")
        {
            double baseX=g_totalDistance-5.0;
            for(uint32_t i=0;i<g_numDrones;i++)
            {
                double x=baseX-double(i)*1.0;
                Ptr<ConstantVelocityMobilityModel> dm=DynamicCast<ConstantVelocityMobilityModel>(droneNodes.Get(i)->GetObject<MobilityModel>());
                dm->SetPosition(Vector(x,0,g_droneHeight));
                dm->SetVelocity(Vector(-0.5,0,0));
                NS_LOG_UNCOND("Drone " << i+1 << " deployed at X=" << x << " m");
            }
        }
        else
        {
            NS_LOG_UNCOND("Unknown mode '"<<g_droneInitMode<<"', defaulting to even");
        }
    }

    // Internet stack
    g_stackHelper.Install(allNodes);

    // Build hops: user -> drones -> AP
    hopDevices.clear();
    hopIfaces.clear();
    for(uint32_t hop=0;hop<=g_numDrones;hop++)
    {
        Ptr<Node> leftNode=allNodes.Get(hop);
        Ptr<Node> rightNode=allNodes.Get(hop+1);
        std::ostringstream ss; ss<<"hop-ssid-"<<hop;
        NetDeviceContainer devPair = CreateWifiHop(leftNode,rightNode,ss.str());
        hopDevices.push_back(devPair);

        Ipv4AddressHelper addr;
        std::ostringstream base; base<<"10.1."<<(hop+1)<<".0";
        addr.SetBase(base.str().c_str(),"255.255.255.0");
        Ipv4InterfaceContainer ifc=addr.Assign(devPair);
        hopIfaces.push_back(ifc);
    }

    Ipv4Address serverIp=hopIfaces.back().GetAddress(1); // last AP
    Ipv4Address userIp=hopIfaces.front().GetAddress(0);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // UDP Echo
    uint16_t port=9;
    UdpEchoServerHelper echoServer(port);
    ApplicationContainer serverApps = echoServer.Install(g_ap);
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(60.0));

    UdpEchoClientHelper echoClient(serverIp,port);
    echoClient.SetAttribute("MaxPackets",UintegerValue(1000));
    echoClient.SetAttribute("Interval",TimeValue(Seconds(0.5)));
    echoClient.SetAttribute("PacketSize",UintegerValue(1024));
    ApplicationContainer clientApps = echoClient.Install(g_user);
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(60.0));

    Ptr<UdpEchoClient> clientApp=DynamicCast<UdpEchoClient>(clientApps.Get(0));
    Ptr<UdpEchoServer> serverApp=DynamicCast<UdpEchoServer>(serverApps.Get(0));
    clientApp->TraceConnectWithoutContext("Tx",MakeCallback(&TxTrace));
    serverApp->TraceConnectWithoutContext("Rx",MakeCallback(&RxTrace));
    clientApp->TraceConnectWithoutContext("Rx",MakeCallback(&ClientRxTrace));

    // Monitor phy: pick last hop AP device
    Ptr<NetDevice> apDevOnLastHop = hopDevices.back().Get(1);
    Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(apDevOnLastHop);
    Ptr<WifiPhy> monitorPhy = nullptr;
    if(wifiDev) monitorPhy=wifiDev->GetPhy();

    Simulator::Schedule(g_balanceInterval,&AutoBalanceDrones);
    if(monitorPhy) Simulator::Schedule(Seconds(2.0),&Monitor,monitorPhy,g_monitorInterval);

    Simulator::Stop(Seconds(60.0));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
