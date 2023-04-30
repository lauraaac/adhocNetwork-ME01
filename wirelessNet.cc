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
#include "ns3/packet-sink.h"
#include "ns3/qos-txop.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ethernet-header.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/rng-seed-manager.h"
#include <unordered_map>

#define YELLOW_CODE "\033[33m"
#define TEAL_CODE "\033[36m"
#define BOLD_CODE "\033[1m"
#define RED_CODE "\033[91m"
#define END_CODE "\033[0m"

using namespace ns3;

//
// Define logging keyword for this file
//
NS_LOG_COMPONENT_DEFINE("MixedWireless");

//
// This function will be used below as a trace sink, if the command-line
// argument or default value "useCourseChangeCallback" is set to true
//
// static void
// CourseChangeCallback(std::string path, Ptr<const MobilityModel> model)
// {
//     Vector position = model->GetPosition();
//     std::cout << "CourseChange "
//               << " x=" << position.x << ", y=" << position.y << ", z=" << position.z << " * "
//               << " | " << model->GetVelocity().x << " | " << model->GetVelocity().y << " | "
//               << model->GetVelocity().z << std::endl;
// }

// static void
// TxCallback(std::string path, Ptr<const Packet> packet)
// {
//     std::cout << YELLOW_CODE << path << END_CODE << std::endl;
//     EthernetHeader hdr;
//   if (packet->PeekHeader (hdr))
//     {
//       std::cout << "\t" << Now() <<  " Packet from " << hdr.GetSource () << " to " << hdr.GetDestination () << " is experiencing backoff" << std::endl;
//     }

//     std::cout << "\t" << Now() << " Packet from " << packet->GetUid() << " is experiencing backoff" << std::endl;
//     std::cout << "\t" << Now() << " Packet dize " << packet->GetSize() << std::endl;


// }

// static void
// RxCallback(std::string path, Ptr<const Packet> packet, const Address &address)
// {
//     std::cout << "Se ha recibido un paquete en el nodo con " << packet->GetSize() << " bytes." << std::endl;
// }
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
        ipAddrs = parentAdhoc.ipAddrs; 
        parentAdhoc.ipAddrs.NewNetwork();

        this->setMobilityModel();
        mobility.PushReferenceMobilityModel(parentAdhoc.backbone.Get(i));
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
                                  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1.0]"),
                                  "Pause",
                                  StringValue("ns3::ConstantRandomVariable[Constant=0.0]"),
                                  "PositionAllocator",
                                  PointerValue(alloc));

        mobility.SetPositionAllocator(alloc);
        mobility.Install(backbone);
    };
};

int
main(int argc, char* argv[])
{
    uint32_t backboneNodes = 6;
    uint32_t infraNodes = 6;
    uint32_t stopTime = 10;
    bool useCourseChangeCallback = true;
    SeedManager::SetSeed (time(0));

    CommandLine cmd(__FILE__);
    cmd.AddValue("backboneNodes", "number of backbone nodes", backboneNodes);
    cmd.AddValue("infraNodes", "number of leaf nodes", infraNodes);
    cmd.AddValue("stopTime", "simulation stop time (seconds)", stopTime);
    cmd.AddValue("useCourseChangeCallback",
                 "whether to enable course change tracing",
                 useCourseChangeCallback);
    cmd.Parse(argc, argv);

    NS_LOG_INFO("Configure Tracing.");
    CsmaHelper csma;
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream("mixed-wireless.tr");
    csma.EnableAsciiAll(stream);
    csma.EnablePcapAll("mixed-wireless", true);
    csma.Install(NodeContainer::GetGlobal());

    AdHocNetwork myadhoc(backboneNodes); // Cluster Padres
    myadhoc.internet.EnableAsciiIpv4All(stream);
    myadhoc.wifiPhy.EnablePcap("mixed-wireless", myadhoc.backboneDevices, true);
    NS_LOG_INFO("Create Applications.");

    ApplicationContainer apps;
    ApplicationContainer sinkApps;
    uint16_t port = 9;

    for (uint32_t i = 0; i < backboneNodes; ++i)
    {
        NS_LOG_INFO("Configuring wireless network for backbone node " << i);
        AdHocNetwork myadhocinfra(myadhoc, infraNodes, i); // Cluster de hijos
        myadhocinfra.internet.EnableAsciiIpv4All(stream);
        myadhocinfra.wifiPhy.EnablePcap("mixed-wireless", myadhocinfra.backboneDevices, true);
       
    }


    for (size_t i = 0; i < NodeContainer::GetGlobal().GetN(); i++)
    {
        u_int32_t irand = rand() % NodeContainer::GetGlobal().GetN();
        Ptr<Node> nodeRand = NodeContainer::GetGlobal().Get(irand);
        Ptr<Ipv4> ipv4Rand = nodeRand->GetObject<Ipv4>();
        Ipv4Address addrRand = ipv4Rand->GetAddress(1, 0).GetLocal();

        Ptr<Node> node = NodeContainer::GetGlobal().Get(i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        Ipv4Address addr = ipv4->GetAddress(1, 0).GetLocal();
        
        std::cout << "Node " << i << " has address " << addr << " --> \t";
        std::cout << "Node " << irand << " has address " << addrRand << std::endl;

        AddressValue remoteAddress(InetSocketAddress(addrRand, port));
        OnOffHelper onoff("ns3::UdpSocketFactory", addrRand);
        onoff.SetAttribute("Remote", remoteAddress);
        onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        onoff.SetAttribute("PacketSize", UintegerValue(1472));
        onoff.SetAttribute("DataRate", StringValue("512kb/s"));
        apps = onoff.Install(NodeContainer::GetGlobal());
        apps.Start(Seconds(3.0));
        apps.Stop(Seconds(stopTime));
    }
    
    //Config::Connect("/NodeList/*/ApplicationList/0/$ns3::OnOffApplication/Tx", MakeCallback(&TxCallback));
    //Config::Connect("/NodeList/*/ApplicationList/1/$ns3::PacketSink/Rx", MakeCallback(&RxCallback));

    // Config::Connect("/NodeList/0/$ns3::MobilityModel/CourseChange",
    // MakeCallback(&CourseChangeCallback));
    // Config::Connect("/NodeList/*/ApplicationList/0/$ns3::PacketSink/Rx",
    // MakeCallback(&RxCallback)); PacketSinkHelper sink("ns3::UdpSocketFactory",
    // InetSocketAddress(Ipv4Address::GetAny(), port));
    //  apps.Add(sink.Install(NodeContainer::GetGlobal()));
    //  apps.Add(onoff.Install(NodeContainer::GetGlobal()));


    FlowMonitorHelper flowMonitorHelper;
    Ptr<FlowMonitor> flowMonitor = flowMonitorHelper.InstallAll();
    AnimationInterface anim("mixed-wireless.xml");
    anim.EnableIpv4RouteTracking("mixed-wireless-route-tracking.xml",
                                 Seconds(0),
                                 Seconds(9),
                                 Seconds(0.25));

    NS_LOG_INFO("Run Simulation.");
    Simulator::Stop(Seconds(stopTime));
    Simulator::Run();

    flowMonitor->CheckForLostPackets();
    flowMonitor->SerializeToXmlFile("mixed-wireless-flow-monitor.xml", false, false);
    

    // Obtener estadísticas de flujo
    std::map<FlowId, FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats();  
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowMonitorHelper.GetClassifier ());
    std::ofstream myfile("data.csv");
    std::unordered_map<std::string, double> valores;
    std::unordered_map<std::string, double> cantidades;
    

    myfile << "Source Address;Destination Address;TxBytes;RxBytes;FirstTxPacket;LastTxPacket;Duration;Delay;Jitter;LostPackets;TxBitrate;average traffic" << std::endl;

    // Imprimir txBitrate de cada flujo
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        std::stringstream buffer;
        buffer << t.sourceAddress << ";" << t.destinationAddress;
        std::string contenido = buffer.str();
        double bitrate = (i->second.txBytes * 8.0) / (i->second.timeLastTxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1000;
        double Duration = i->second.timeLastTxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds();
        double timemax = i->second.timeLastTxPacket.GetSeconds();
        double percentage = (Duration / timemax);
        double average = bitrate * percentage;
        valores[contenido] += percentage * (bitrate);
        cantidades[contenido] += 1;

        myfile 
        << t.sourceAddress 
        << ";" << t.destinationAddress
        << ";" << i->second.txBytes  
        << ";" << i->second.rxBytes 
        << ";" << i->second.timeFirstTxPacket.GetSeconds() 
        << ";" << timemax
        << ";" << Duration
        << ";" << i->second.delaySum.GetSeconds() / i->second.rxPackets
        << ";" << i->second.jitterSum.GetSeconds() / (i->second.rxPackets - 1)
        << ";" << i->second.lostPackets 
        << ";" << bitrate
        << ";" << average << std::endl;    
    }

    std::ofstream resumenfile("resumen.csv");
    resumenfile << "Source Address;average traffic" << std::endl;
    for (const auto& par : valores) {
        resumenfile << par.first << ";" << par.second << ";" << cantidades[par.first] << std::endl;
    }

    
    
    resumenfile.close();
    myfile.close();
    Simulator::Destroy();
    return 0;
}