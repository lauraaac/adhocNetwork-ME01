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
};


int
main (int argc, char *argv[])
{
  //
  // First, we declare and initialize a few local variables that control some
  // simulation parameters.
  //
  uint32_t backboneNodes = 10; //we can change it depending on our simulation
  uint32_t infraNodes = 2;
  uint32_t stopTime = 20;
  bool useCourseChangeCallback = false;

  //
  // Simulation defaults are typically set next, before command line
  // arguments are parsed.
  //
  Config::SetDefault ("ns3::OnOffApplication::PacketSize", StringValue ("1472"));
  Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue ("100kb/s"));

  //
  // For convenience, we add the local variables to the command line argument
  // system so that they can be overridden with flags such as
  // "--backboneNodes=20"
  //
  CommandLine cmd (__FILE__);
  cmd.AddValue ("backboneNodes", "number of backbone nodes", backboneNodes);
  cmd.AddValue ("infraNodes", "number of leaf nodes", infraNodes);
  cmd.AddValue ("stopTime", "simulation stop time (seconds)", stopTime);
  cmd.AddValue ("useCourseChangeCallback", "whether to enable course change tracing", useCourseChangeCallback);

  //
  // The system global variables and the local values added to the argument
  // system can be overridden by command line arguments by using this call.
  //
  cmd.Parse (argc, argv);

  if (stopTime < 10)
    {
      std::cout << "Use a simulation stop time >= 10 seconds" << std::endl;
      exit (1);
    }
  ///////////////////////////////////////////////////////////////////////////
  //                                                                       //
  // Construct the backbone                                                //
  //                                                                       //
  ///////////////////////////////////////////////////////////////////////////

  AdHocNetwork myadhoc (backboneNodes);
  
  for (uint32_t i = 0; i < backboneNodes; ++i)
    {
      NS_LOG_INFO ("Configuring wireless network for backbone node " << i);
   
      NodeContainer stas;
      stas.Create (infraNodes - 1);
      // Now, create the container with all nodes on this link
      NodeContainer infra (myadhoc.backbone.Get (i), stas);
      //
      // Create an infrastructure network
      //
      WifiHelper wifiInfra;
      WifiMacHelper macInfra;
      myadhoc.wifiPhy.SetChannel (myadhoc.wifiChannel.Create ());
      // Create unique ssids for these networks
      std::string ssidString ("wifi-infra");
      std::stringstream ss;
      ss << i;
      ssidString += ss.str ();
      Ssid ssid = Ssid (ssidString);
      wifiInfra.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue ("OfdmRate54Mbps"));
      // setup stas
      macInfra.SetType ("ns3::StaWifiMac",
                        "Ssid", SsidValue (ssid));
      NetDeviceContainer staDevices = wifiInfra.Install (myadhoc.wifiPhy, macInfra, stas);
      // setup ap.
      macInfra.SetType ("ns3::ApWifiMac",
                        "Ssid", SsidValue (ssid));
      NetDeviceContainer apDevices = wifiInfra.Install (myadhoc.wifiPhy, macInfra, myadhoc.backbone.Get (i));
      // Collect all of these new devices
      NetDeviceContainer infraDevices (apDevices, staDevices);

      // Add the IPv4 protocol stack to the nodes in our container
      //
      myadhoc.internet.Install (stas);
      //
      // Assign IPv4 addresses to the device drivers (actually to the associated
      // IPv4 interfaces) we just created.
      //
      myadhoc.ipAddrs.Assign (infraDevices);
      //
      // Assign a new network prefix for each mobile network, according to
      // the network mask initialized above
      //
      myadhoc.ipAddrs.NewNetwork ();
      //
      // The new wireless nodes need a mobility model so we aggregate one
      // to each of the nodes we just finished building.
      //
      Ptr<ListPositionAllocator> subnetAlloc =
        CreateObject<ListPositionAllocator> ();
      for (uint32_t j = 0; j < infra.GetN (); ++j)
        {
          subnetAlloc->Add (Vector (0.0, j, 0.0));
        }
      myadhoc.mobility.PushReferenceMobilityModel (myadhoc.backbone.Get (i));
      myadhoc.mobility.SetPositionAllocator (subnetAlloc);
      myadhoc.mobility.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                                 "Bounds", RectangleValue (Rectangle (-10, 10, -10, 10)),
                                 "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=3]"),
                                 "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.4]"));
      myadhoc.mobility.Install (stas);
    }

  ///////////////////////////////////////////////////////////////////////////
  //                                                                       //
  // Application configuration                                             //
  //                                                                       //
  ///////////////////////////////////////////////////////////////////////////

  // Create the OnOff application to send UDP datagrams of size
  // 210 bytes at a rate of 10 Kb/s, between two nodes
  // We'll send data from the first wired LAN node on the first wired LAN
  // to the last wireless STA on the last infrastructure net, thereby
  // causing packets to traverse CSMA to adhoc to infrastructure links

 
 
  NS_LOG_INFO ("Create Applications.");
  uint16_t port = 9;   // Discard port (RFC 863)

  
  // We want the source to be the first node created outside of the backbone
  // Conveniently, the variable "backboneNodes" holds this node index value
  Ptr<Node> appSource = NodeList::GetNode (backboneNodes);
  // We want the sink to be the last node created in the topology.
  

  // Create a packet sink to receive these packets
  PacketSinkHelper sink ("ns3::UdpSocketFactory",
                         InetSocketAddress (Ipv4Address::GetAny (), port));

  ///////////////////////////////////////////////////////////////////////////
  //                                                                       //
  // Tracing configuration                                                 //
  //                                                                       //
  ///////////////////////////////////////////////////////////////////////////

  NS_LOG_INFO ("Configure Tracing.");
  CsmaHelper csma;

  //
  // Let's set up some ns-2-like ascii traces, using another helper class
  //
  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("mixed-wireless.tr");

  csma.EnableAsciiAll (stream);
  myadhoc.internet.EnableAsciiIpv4All (stream);

  // Csma captures in non-promiscuous mode
  csma.EnablePcapAll ("mixed-wireless", false);
  // pcap captures on the backbone wifi devices
  myadhoc.wifiPhy.EnablePcap ("mixed-wireless", myadhoc.backboneDevices, false);


  if (useCourseChangeCallback == true)
    {
      Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback (&CourseChangeCallback));
    }

  AnimationInterface anim ("mixed-wireless.xml");

  ///////////////////////////////////////////////////////////////////////////
  //                                                                       //
  // Run simulation                                                        //
  //                                                                       //
  ///////////////////////////////////////////////////////////////////////////

  NS_LOG_INFO ("Run Simulation.");
  Simulator::Stop (Seconds (stopTime));
  Simulator::Run ();
  Simulator::Destroy ();
}