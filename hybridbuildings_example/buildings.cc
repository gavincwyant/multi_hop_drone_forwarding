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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BuildingsExample");

int main(int argc, char *argv[]) {

  Time::SetResolution(Time::NS);
  LogComponentEnable("BuildingsExample", LOG_LEVEL_INFO);

  Ptr<GridBuildingAllocator>  gridBuildingAllocator;
  gridBuildingAllocator = CreateObject<GridBuildingAllocator>();
  gridBuildingAllocator->SetAttribute("GridWidth", UintegerValue(3));
  gridBuildingAllocator->SetAttribute("LengthX", DoubleValue(7));
  gridBuildingAllocator->SetAttribute("LengthY", DoubleValue(13));
  gridBuildingAllocator->SetAttribute("DeltaX", DoubleValue(3)); // spacing between the buildings
  gridBuildingAllocator->SetAttribute("DeltaY", DoubleValue(3));
  gridBuildingAllocator->SetAttribute("Height", DoubleValue(6));
  gridBuildingAllocator->SetBuildingAttribute("NRoomsX", UintegerValue(2));
  gridBuildingAllocator->SetBuildingAttribute("NRoomsY", UintegerValue(4));
  gridBuildingAllocator->SetBuildingAttribute("NFloors", UintegerValue(2));
  gridBuildingAllocator->SetAttribute("MinX", DoubleValue(0));
  gridBuildingAllocator->SetAttribute("MinY", DoubleValue(0));
  gridBuildingAllocator->Create(6);

  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  NodeContainer ueNodes;
  ueNodes.Create(2); // the equipment nodes (APs or UEs)
  mobility.Install(ueNodes);
  BuildingsHelper::Install(ueNodes);
  Ptr<ConstantPositionMobilityModel> mm0 = ueNodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
  Ptr<ConstantPositionMobilityModel> mm1 = ueNodes.Get(1)->GetObject<ConstantPositionMobilityModel>();
  mm0->SetPosition(Vector(5.0, 5.0, 1.5));
  mm1->SetPosition(Vector(30.0, 40.0, 1.5));

  return 0;
}