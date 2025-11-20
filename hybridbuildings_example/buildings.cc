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