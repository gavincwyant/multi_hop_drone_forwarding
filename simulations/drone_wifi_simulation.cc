// Multi-hop drone forwarding simulation.
//
// A single user moves away from a fixed base station at constant velocity while
// exchanging UDP traffic with it. As the link degrades, relay "drones" are
// deployed to forward traffic back to the base station over multiple hops.
//
// The simulation answers the three questions posed in the README:
//   1. When should a relay be deployed?  -> a sliding-window loss rate crossing
//      a configurable threshold (see kWindow / --lossThreshold).
//   2. Where should relays be placed?    -> evenly spaced along the base-to-user
//      segment, re-balanced each time a new relay joins (RepositionRelays).
//   3. Does it scale to N hops?          -> --maxRelays controls the hop count;
//      relays run AODV so routes are discovered rather than hard-coded.
//
// Topology is ad-hoc (not infrastructure BSS) because multi-hop forwarding
// requires a routing protocol; AODV discovers the user -> relay* -> base path.

#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/yans-wifi-helper.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneWifiSimulation");

namespace
{

// Packet counters, updated by the trace callbacks below.
uint64_t g_txPackets = 0;
uint64_t g_rxPackets = 0;

// Counter values at the previous monitor tick, used to derive a windowed rate
// instead of a cumulative one. A cumulative rate is dominated by early history
// and cannot recover after relays improve the link, which makes it useless as a
// deployment trigger.
uint64_t g_prevTx = 0;
uint64_t g_prevRx = 0;

Ptr<Node> g_user;
Ptr<Node> g_base;
NodeContainer g_relays;

uint32_t g_activeRelays = 0;
Time g_lastDeployment = Seconds(0);

// Placement exponent. Relay i of k sits at fraction ((i+1)/(k+1))^gamma along
// the base-to-user segment: 1.0 spaces them evenly, >1 pulls the chain toward
// the base station, <1 pushes it toward the user.
double g_placementGamma = 1.0;

std::ofstream g_csv;

void
TxTrace(Ptr<const Packet>)
{
    g_txPackets++;
}

void
RxTrace(Ptr<const Packet>, const Address&)
{
    g_rxPackets++;
}

Vector
PositionOf(Ptr<Node> node)
{
    return node->GetObject<MobilityModel>()->GetPosition();
}

// Space the active relays along the segment from the base station to the user.
// With g_placementGamma == 1 the spacing is even; other values bias the chain
// toward the base or the user so the placement hypothesis can be tested rather
// than assumed. Called every measurement interval, since the user keeps moving.
void
RepositionRelays()
{
    if (g_activeRelays == 0)
    {
        return;
    }

    const Vector base = PositionOf(g_base);
    const Vector user = PositionOf(g_user);

    for (uint32_t i = 0; i < g_activeRelays; ++i)
    {
        const double even = static_cast<double>(i + 1) / (g_activeRelays + 1);
        const double f = std::pow(even, g_placementGamma);
        const Vector pos(base.x + (user.x - base.x) * f,
                         base.y + (user.y - base.y) * f,
                         base.z + (user.z - base.z) * f);
        g_relays.Get(i)->GetObject<MobilityModel>()->SetPosition(pos);
    }
}

// Deploy one more relay, if any remain in the pool.
void
DeployRelay(Time cooldown)
{
    if (g_activeRelays >= g_relays.GetN())
    {
        return;
    }
    if (Simulator::Now() - g_lastDeployment < cooldown)
    {
        // AODV needs time to discover a route through the relay just added;
        // deploying again immediately would react to stale loss measurements.
        return;
    }

    g_activeRelays++;
    g_lastDeployment = Simulator::Now();
    RepositionRelays();

    std::cout << std::fixed << std::setprecision(1) << Simulator::Now().GetSeconds()
              << "s: DEPLOY relay #" << g_activeRelays << " (" << (g_activeRelays + 1)
              << " hops)" << std::endl;
}

void
Monitor(Time interval, double lossThreshold, Time cooldown)
{
    const double distance =
        g_user->GetObject<MobilityModel>()->GetDistanceFrom(g_base->GetObject<MobilityModel>());

    const uint64_t dTx = g_txPackets - g_prevTx;
    const uint64_t dRx = g_rxPackets - g_prevRx;
    g_prevTx = g_txPackets;
    g_prevRx = g_rxPackets;

    const double windowLoss = (dTx > 0) ? 100.0 * (1.0 - static_cast<double>(dRx) / dTx) : 0.0;
    const double cumulativeLoss =
        (g_txPackets > 0) ? 100.0 * (1.0 - static_cast<double>(g_rxPackets) / g_txPackets) : 0.0;

    // Relays track the user continuously: the "equidistant" placement the
    // project set out to test is a moving target, so the chain is re-spaced
    // every tick rather than only when a new relay joins.
    RepositionRelays();

    const double now = Simulator::Now().GetSeconds();

    std::cout << std::fixed << std::setprecision(1) << now << "s: distance=" << distance
              << "m, window loss=" << windowLoss << "%, cumulative=" << cumulativeLoss
              << "%, relays=" << g_activeRelays << std::endl;

    if (g_csv.is_open())
    {
        g_csv << now << "," << distance << "," << windowLoss << "," << cumulativeLoss << ","
              << g_activeRelays << "\n";
    }

    if (windowLoss > lossThreshold)
    {
        DeployRelay(cooldown);
    }

    Simulator::Schedule(interval, &Monitor, interval, lossThreshold, cooldown);
}

} // namespace

int
main(int argc, char* argv[])
{
    double simTime = 120.0;
    double userSpeed = 5.0;
    double lossThreshold = 20.0;
    uint32_t maxRelays = 5;
    double monitorInterval = 1.0;
    double cooldown = 10.0;
    std::string csvPath = "";
    bool pcap = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration in seconds", simTime);
    cmd.AddValue("userSpeed", "User velocity in m/s along +x", userSpeed);
    cmd.AddValue("lossThreshold", "Windowed loss %% that triggers a deployment", lossThreshold);
    cmd.AddValue("maxRelays", "Maximum relay drones available", maxRelays);
    cmd.AddValue("monitorInterval", "Seconds between measurements", monitorInterval);
    cmd.AddValue("cooldown", "Minimum seconds between deployments", cooldown);
    cmd.AddValue("placementGamma",
                 "Relay placement exponent: 1=even, >1=toward base, <1=toward user",
                 g_placementGamma);
    cmd.AddValue("csv", "Write per-tick metrics to this CSV path", csvPath);
    cmd.AddValue("pcap", "Enable pcap capture", pcap);
    cmd.Parse(argc, argv);

    Time::SetResolution(Time::NS);

    NodeContainer base;
    base.Create(1);
    NodeContainer user;
    user.Create(1);
    NodeContainer relays;
    relays.Create(maxRelays);

    g_base = base.Get(0);
    g_user = user.Get(0);
    g_relays = relays;

    NodeContainer all(base, user, relays);

    // Ad-hoc 802.11n. Infrastructure mode cannot forward user -> relay -> base,
    // so every node shares one ad-hoc network and AODV supplies the routes.
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("HtMcs0"),
                                 "ControlMode",
                                 StringValue("HtMcs0"));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(phy, mac, all);

    // Mobility: base fixed at the origin, user departing along +x, relays parked
    // far off-field until deployed so they cannot forward before that.
    MobilityHelper fixedMobility;
    fixedMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    fixedMobility.Install(base);
    fixedMobility.Install(relays);

    MobilityHelper movingMobility;
    movingMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    movingMobility.Install(user);

    g_base->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 0.0));
    g_user->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(0.0, 0.0, 0.0));
    g_user->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(userSpeed, 0.0, 0.0));

    const double kParked = 1.0e6;
    for (uint32_t i = 0; i < relays.GetN(); ++i)
    {
        relays.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(kParked, kParked, 0.0));
    }

    AodvHelper aodv;
    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(all);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // interfaces index order follows `all`: 0 = base, 1 = user, 2.. = relays.
    const Ipv4Address baseAddress = interfaces.GetAddress(0);

    const uint16_t port = 9;
    UdpEchoServerHelper echoServer(port);
    ApplicationContainer serverApps = echoServer.Install(base);
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(simTime));

    UdpEchoClientHelper echoClient(baseAddress, port);
    echoClient.SetAttribute("MaxPackets", UintegerValue(std::numeric_limits<uint32_t>::max()));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(0.5)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));
    ApplicationContainer clientApps = echoClient.Install(user);
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(simTime));

    clientApps.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&TxTrace));
    serverApps.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&RxTrace));

    if (!csvPath.empty())
    {
        g_csv.open(csvPath);
        g_csv << "time_s,distance_m,window_loss_pct,cumulative_loss_pct,active_relays\n";
    }

    if (pcap)
    {
        phy.EnablePcapAll("drone_wifi_simulation");
    }

    Simulator::Schedule(Seconds(2.0 + monitorInterval),
                        &Monitor,
                        Seconds(monitorInterval),
                        lossThreshold,
                        Seconds(cooldown));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    if (g_csv.is_open())
    {
        g_csv.close();
    }

    std::cout << "\nTotals: tx=" << g_txPackets << " rx=" << g_rxPackets << " relays deployed="
              << g_activeRelays << std::endl;

    return 0;
}
