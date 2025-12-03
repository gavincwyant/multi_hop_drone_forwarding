#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/buildings-module.h"
#include "ns3/internet-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UrbanCanyonWifiExample");

int main(int argc, char *argv[])
{
    LogComponentEnable("UrbanCanyonWifiExample", LOG_LEVEL_INFO);
    
    // --- 1. SETUP BUILDINGS ENVIRONMENT (The Corridor) ---
    // Define two rows of buildings creating a corridor:
    // Row 1 (Left side, X=0 to 10)
    // Box(double _xMin, double _xMax, double _yMin, double _yMax, double _zMin, double _zMax)
    for (int i = 0; i < 5; ++i) {
        Ptr<Building> b = CreateObject<Building>();
        b->SetBoundaries(Box(i*10.0, (i + 1)*10.0, -15.0, -5.0, 0.0, 15.0));
        b->SetNFloors(5);
        b->SetBuildingType(Building::Office);
    }
    // Row 2 (Right side, X=30 to 40)
    for (int i = 0; i < 5; ++i) {
        Ptr<Building> b = CreateObject<Building>();
        b->SetBoundaries(Box(i*10.0, (i + 1)*10.0, 5.0, 15.0, 0.0, 15.0));
        b->SetNFloors(5);
        b->SetBuildingType(Building::Office);
    }
    // Corridor of buildings spans from x = 0 to x = 50
    // y coordinates are fixed

    // --- 2. NODE CREATION AND MOBILITY ---
    NodeContainer apNode, staNode; 
    apNode.Create(1);
    staNode.Create(1);
    NodeContainer allNodes;
    allNodes.Add(apNode);
    allNodes.Add(staNode);

    MobilityHelper mobility;
    
    // AP is at the start of the canyon (X=20, Y=5)
    Ptr<ListPositionAllocator> apAlloc = CreateObject<ListPositionAllocator>();
    apAlloc->Add(Vector(20.0, 5.0, 1.5)); 
    mobility.SetPositionAllocator(apAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNode);

    // STA is down the canyon (X=20, Y=85)
    Ptr<ListPositionAllocator> staAlloc = CreateObject<ListPositionAllocator>();
    staAlloc->Add(Vector(20.0, 85.0, 1.5)); 
    mobility.SetPositionAllocator(staAlloc);
    mobility.Install(staNode);

    // CRITICAL STEP: Install MobilityBuildingInfo on all nodes
    BuildingsHelper::Install(allNodes); 

    // --- 3. CHANNEL SETUP (Building-Aware Loss Model) ---
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();

    // Attach the HybridBuildingsPropagationLossModel
    // It uses specialized models for urban NLOS/LOS paths.
    wifiChannel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel",
                                   "Frequency", DoubleValue(2.4e9), // 2.4 GHz band
                                   "RooftopLevel", DoubleValue(20.0),  
                                   "Environment", StringValue("Urban")); 

    Ptr<YansWifiChannel> channel = wifiChannel.Create();

    // --- 4. PHY AND NET DEVICE SETUP ---
    YansWifiPhyHelper wifiPhy;
    // THE LINK: Associate the PHY to the canyon-aware channel
    wifiPhy.SetChannel(channel); 
    
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b); 
    
    wifiPhy.Set("TxPowerStart", DoubleValue(15.0)); 
    wifiPhy.Set("TxPowerEnd", DoubleValue(15.0));

    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", 
                                 "DataMode", StringValue("DsssRate11Mbps"), 
                                 "ControlMode", StringValue("DsssRate1Mbps"));

    WifiMacHelper wifiMac;
    Ssid ssid = Ssid("UrbanCanyonNet");

    // Install AP and STA devices
    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    wifi.Install(wifiPhy, wifiMac, apNode);

    wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    wifi.Install(wifiPhy, wifiMac, staNode);

    // --- 5. RUN AND TRACE ---
    wifiPhy.EnablePcapAll("urban-canyon");

    Simulator::Stop(Seconds(0.1));
    NS_LOG_INFO("Starting simulation...");
    Simulator::Run();
    Simulator::Destroy();
    NS_LOG_INFO("Done.");

    return 0;
}