/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */


//
// This ns-3 example demonstrates the use of helper functions to ease
// the construction of simulation scenarios.
//
// The simulation topology consists of a mixed wired and wireless
// scenario in which a hierarchical mobility model is used.
//
// The simulation layout consists of N backbone routers interconnected
// by an ad hoc wifi network.
// Each backbone router also has a local 802.11 network and is connected
// to a local LAN.  An additional set of (K-1) nodes are connected to
// this backbone.  Finally, a local LAN is connected to each router
// on the backbone, with L-1 additional hosts.
//
// The nodes are populated with TCP/IP stacks, and OLSR unicast routing
// on the backbone.  An example UDP transfer is shown.  The simulator
// be configured to output tcpdumps or traces from different nodes.
//
//
//          +--------------------------------------------------------+
//          |                                                        |
//          |              802.11 ad hoc, ns-2 mobility              |
//          |                                                        |
//          +--------------------------------------------------------+
//                   |       o o o (N backbone routers)       |
//               +--------+                               +--------+
//     wired LAN | mobile |                     wired LAN | mobile |
//    -----------| router |                    -----------| router |
//               ---------                                ---------
//                   |                                        |
//          +----------------+                       +----------------+
//          |     802.11     |                       |     802.11     |
//          |   infra net    |                       |   infra net    |
//          |   K-1 hosts    |                       |   K-1 hosts    |
//          +----------------+                       +----------------+
//
// We'll send data from the first wired LAN node on the first wired LAN
// to the last wireless STA on the last infrastructure net, thereby
// causing packets to traverse CSMA to adhoc to infrastructure links
//
// Note that certain mobility patterns may cause packet forwarding
// to fail (if nodes become disconnected)

#include "ns3/command-line.h"
#include "ns3/string.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/qos-txop.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/csma-helper.h"
#include "ns3/animation-interface.h"

using namespace ns3;

//
// Define logging keyword for this file
//
NS_LOG_COMPONENT_DEFINE ("MixedWireless");

//
// This function will be used below as a trace sink, if the command-line
// argument or default value "useCourseChangeCallback" is set to true
//
static void
CourseChangeCallback (std::string path, Ptr<const MobilityModel> model)
{
  Vector position = model->GetPosition ();
  std::cout << "CourseChange " << path << " x=" << position.x << ", y=" << position.y << ", z=" << position.z << std::endl;
}

class AdHocNetwork
{
  public:
    NodeContainer backbone;
    NetDeviceContainer backboneDevices;
    WifiHelper wifi;
    WifiMacHelper mac;
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    OlsrHelper olsr;
    InternetStackHelper internet;
    Ipv4AddressHelper ipAddrs;
    MobilityHelper mobility;

  
    AdHocNetwork (uint32_t backboneNodes){
        backbone.Create (backboneNodes);
        mac.SetType ("ns3::AdhocWifiMac");
        wifiPhy.SetChannel (wifiChannel.Create ());
        backboneDevices = wifi.Install (wifiPhy, mac, backbone);
        internet.SetRoutingHelper (olsr); 
        internet.Install (backbone);        
        ipAddrs.SetBase ("192.168.0.0", "255.255.255.0");
        ipAddrs.Assign (backboneDevices);
        mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                      "MinX", DoubleValue (20.0),
                                      "MinY", DoubleValue (20.0),
                                      "DeltaX", DoubleValue (20.0),
                                      "DeltaY", DoubleValue (20.0),
                                      "GridWidth", UintegerValue (5),
                                      "LayoutType", StringValue ("RowFirst"));
        mobility.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                                  "Bounds", RectangleValue (Rectangle (-500, 500, -500, 500)),
                                  "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=2]"),
                                  "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.2]"));
        mobility.Install (backbone);
    };

    AdHocNetwork (AdHocNetwork &parentAdhoc, uint32_t infraNodes){
        backbone.Create (infraNodes);
        mac.SetType ("ns3::AdhocWifiMac");
        wifiPhy.SetChannel (wifiChannel.Create ());
        backboneDevices = wifi.Install (wifiPhy, mac, backbone);
        internet.SetRoutingHelper (olsr); 
        parentAdhoc.internet.Install (backbone); 
        parentAdhoc.ipAddrs.Assign (backboneDevices);
    }

};


int
main (int argc, char *argv[])
{
  uint32_t backboneNodes = 10; 
  uint32_t infraNodes = 2;
  uint32_t stopTime = 20;
  bool useCourseChangeCallback = false;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("backboneNodes", "number of backbone nodes", backboneNodes);
  cmd.AddValue ("infraNodes", "number of leaf nodes", infraNodes);
  cmd.AddValue ("stopTime", "simulation stop time (seconds)", stopTime);
  cmd.AddValue ("useCourseChangeCallback", "whether to enable course change tracing", useCourseChangeCallback);
  cmd.Parse (argc, argv);

  if (stopTime < 10)
    {
      std::cout << "Use a simulation stop time >= 10 seconds" << std::endl;
      exit (1);
    }

  AdHocNetwork myadhoc (backboneNodes);
  
  for (uint32_t i = 0; i < backboneNodes; ++i)
    {
      NS_LOG_INFO ("Configuring wireless network for backbone node " << i);

      AdHocNetwork myadhocinfra (myadhoc, infraNodes - 1);

      Ptr<ListPositionAllocator> subnetAlloc = CreateObject<ListPositionAllocator> ();
      for (uint32_t j = 0; j < myadhocinfra.backbone.GetN (); ++j)
        {
          subnetAlloc->Add (Vector (0.0, j, 0.0));
        }
      myadhoc.mobility.PushReferenceMobilityModel (myadhoc.backbone.Get (i));
      myadhoc.mobility.SetPositionAllocator (subnetAlloc);
      myadhoc.mobility.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                                 "Bounds", RectangleValue (Rectangle (-10, 10, -10, 10)),
                                 "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=3]"),
                                 "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.4]"));
      myadhoc.mobility.Install (myadhocinfra.backbone);
    }



 
  NS_LOG_INFO ("Create Applications.");
  Config::SetDefault ("ns3::OnOffApplication::PacketSize", StringValue ("1472"));
  Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue ("100kb/s"));
  uint16_t port = 9;   
  Ptr<Node> appSource = NodeList::GetNode (backboneNodes);
  PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));


  NS_LOG_INFO ("Configure Tracing.");
  CsmaHelper csma;
  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("mixed-wireless.tr");
  csma.EnableAsciiAll (stream);
  myadhoc.internet.EnableAsciiIpv4All (stream);
  csma.EnablePcapAll ("mixed-wireless", false);
  myadhoc.wifiPhy.EnablePcap ("mixed-wireless", myadhoc.backboneDevices, false);


  if (useCourseChangeCallback == true)
    {
      Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback (&CourseChangeCallback));
    }

  AnimationInterface anim ("mixed-wireless.xml");

  NS_LOG_INFO ("Run Simulation.");
  Simulator::Stop (Seconds (stopTime));
  Simulator::Run ();
  Simulator::Destroy ();
}