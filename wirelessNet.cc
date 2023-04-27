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

#include "ns3/animation-interface.h"
#include "ns3/command-line.h"
#include "ns3/csma-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/qos-txop.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/packet-sink.h"

using namespace ns3;

//
// Define logging keyword for this file
//
NS_LOG_COMPONENT_DEFINE("MixedWireless");

//
// This function will be used below as a trace sink, if the command-line
// argument or default value "useCourseChangeCallback" is set to true
//
static void
CourseChangeCallback(std::string path, Ptr<const MobilityModel> model)
{
    Vector position = model->GetPosition();
    std::cout << "CourseChange " << path << " x=" << position.x << ", y=" << position.y
              << ", z=" << position.z << std::endl;
}

class AdHocNetwork
{
  public:
    NodeContainer backbone;
    NetDeviceContainer backboneDevices;
    WifiHelper wifi;
    WifiMacHelper mac;
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel;
    OlsrHelper olsr;
    InternetStackHelper internet;
    Ipv4AddressHelper ipAddrs;
    MobilityHelper mobility;
    ObjectFactory pos;
    Ipv4InterfaceContainer interfaces;

    AdHocNetwork();

    AdHocNetwork(uint32_t backboneNodes)
    {
        backbone.Create(backboneNodes);
        mac.SetType("ns3::AdhocWifiMac");
        wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
        wifiPhy.SetChannel(wifiChannel.Create());

        backboneDevices = wifi.Install(wifiPhy, mac, backbone);
        internet.SetRoutingHelper(olsr);  
        internet.Install(backbone);
        ipAddrs.SetBase("192.168.0.0", "255.255.255.0");
        interfaces = ipAddrs.Assign(backboneDevices);
        ipAddrs.NewNetwork();
        this->setMobilityModel();
    };

    AdHocNetwork(AdHocNetwork& parentAdhoc, uint32_t backboneNodes, uint32_t i)
    {
        backbone.Create(backboneNodes);
        mac.SetType("ns3::AdhocWifiMac");
        wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
        wifiPhy.SetChannel(wifiChannel.Create());


        internet.SetRoutingHelper(parentAdhoc.olsr);    
        parentAdhoc.internet.Install(backbone);
        backbone.Add(parentAdhoc.backbone.Get(i));
        backboneDevices = wifi.Install(wifiPhy, mac, backbone);
        interfaces = parentAdhoc.ipAddrs.Assign(backboneDevices);
        parentAdhoc.ipAddrs.NewNetwork();

        mobility.PushReferenceMobilityModel(parentAdhoc.backbone.Get(i));
        this->setMobilityModel();
    }

  private:
  
    void setWifi(uint32_t backboneNodes)
    {
        backbone.Create(backboneNodes);
        mac.SetType("ns3::AdhocWifiMac");
        wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
        wifiPhy.SetChannel(wifiChannel.Create());
        backboneDevices = wifi.Install(wifiPhy, mac, backbone);
    };

    void setMobilityModel()
    {
        pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
        pos.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
        pos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
        Ptr<PositionAllocator> alloc = pos.Create()->GetObject<PositionAllocator>();
        mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                  "Speed",
                                  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=60.0]"),
                                  "Pause",
                                  StringValue("ns3::ConstantRandomVariable[Constant=0.2]"),
                                  "PositionAllocator",
                                  PointerValue(alloc));

        mobility.SetPositionAllocator(alloc);
        mobility.Install(backbone);
    };
};

void ImprimirResultado (double resultado) {
    // Imprimir el resultado en pantalla
    std::cout << "El resultado es: " << resultado << std::endl;
}

int
main(int argc, char* argv[])
{
    uint32_t backboneNodes = 2;
    uint32_t infraNodes = 6;
    uint32_t stopTime = 50;
    bool useCourseChangeCallback = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("backboneNodes", "number of backbone nodes", backboneNodes);
    cmd.AddValue("infraNodes", "number of leaf nodes", infraNodes);
    cmd.AddValue("stopTime", "simulation stop time (seconds)", stopTime);
    cmd.AddValue("useCourseChangeCallback", "whether to enable course change tracing", useCourseChangeCallback);
    cmd.Parse(argc, argv);

    NS_LOG_INFO("Configure Tracing.");
    CsmaHelper csma;
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream("mixed-wireless.tr");
    csma.EnableAsciiAll(stream);
    csma.EnablePcapAll("mixed-wireless", true);
    csma.Install(NodeContainer::GetGlobal());


    AdHocNetwork myadhoc(backboneNodes);
    myadhoc.internet.EnableAsciiIpv4All(stream);
    myadhoc.wifiPhy.EnablePcap("mixed-wireless", myadhoc.backboneDevices, true);

    for (uint32_t i = 0; i < backboneNodes; ++i)
    {
        NS_LOG_INFO("Configuring wireless network for backbone node " << i);
        AdHocNetwork myadhocinfra(myadhoc, infraNodes, i);
        myadhocinfra.internet.EnableAsciiIpv4All(stream);
        myadhocinfra.wifiPhy.EnablePcap("mixed-wireless", myadhocinfra.backboneDevices, true);
    }

    NS_LOG_INFO("Create Applications.");
    ApplicationContainer apps;

    uint16_t port = 9;
    OnOffHelper onoff("ns3::UdpSocketFactory", Ipv4Address::GetAny());
    onoff.SetAttribute("Remote", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    onoff.SetAttribute("PacketSize", UintegerValue(1472));
    onoff.SetAttribute("DataRate", StringValue("1Mb/s"));

    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    apps.Add(sink.Install(myadhoc.backbone));
    apps.Add(onoff.Install(myadhoc.backbone));
    apps.Start(Seconds(1.0));
    apps.Stop(Seconds(stopTime));
    

    
    

    if (useCourseChangeCallback == true)
    {
        Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback(&CourseChangeCallback));
    }

    AnimationInterface anim("mixed-wireless.xml");
    anim.EnableIpv4RouteTracking("mixed-wireless-route-tracking.xml", Seconds(0), Seconds(9), Seconds(0.25));

    NS_LOG_INFO("Run Simulation.");
    Simulator::Stop(Seconds(stopTime+2));
    Simulator::Run();
    Simulator::Destroy();
}