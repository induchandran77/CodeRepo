#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/malicious-node.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/propagation-module.h"
#include "ns3/energy-module.h"
#include "ns3/propagation-loss-model.h"

#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <random>
#include <algorithm>


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiAodvSimulation");

//added
uint64_t countAodv = 0, countRreq = 0, countRrep = 0, countRerr = 0, countRrepAck = 0;
Time firstAodv, lastAodv;
uint64_t bytesAodv = 0;
  

void
AodvPacketTrace (std::string context, Ptr<const Packet> packet)
{
  std::string packetType;
  std::ostringstream description;
  aodv::TypeHeader tHeader;
  Ptr<Packet> p = packet->Copy ();
  p->RemoveHeader (tHeader);
  if (!tHeader.IsValid ())
    {
      NS_LOG_DEBUG ("AODV message " << packet->GetUid () << " with unknown type received: " << tHeader.Get () << ". Drop");
      return; // drop
    }
  switch (tHeader.Get ())
    {
    case aodv::MessageType::AODVTYPE_RREQ:
      {
        packetType = "RREQ";
        aodv::RreqHeader rreqHeader;
        p->RemoveHeader (rreqHeader);
        description << "O:" << rreqHeader.GetOrigin () << " D:" << rreqHeader.GetDst () << " Hop:" << (int)rreqHeader.GetHopCount (); 
        countRreq++;
        break;
      }
    case aodv::MessageType::AODVTYPE_RREP:
      {
        packetType = "RREP";
        aodv::RrepHeader rrepHeader;
        p->RemoveHeader (rrepHeader);
        description << "O:" << rrepHeader.GetOrigin () << " D:" << rrepHeader.GetDst () << " Hop:" << (int)rrepHeader.GetHopCount (); 
        countRrep++;
        break;
      }
    case aodv::MessageType::AODVTYPE_RERR:
      {
        packetType = "RERR";
        countRerr++;
        break;
      }
    case aodv::MessageType::AODVTYPE_RREP_ACK:
      {
        packetType = "RREP_ACK";
        countRrepAck++;
        break;
      }
    }

  countAodv++;
  if (countAodv==1)
  {
    firstAodv = Simulator::Now ();
  }
  lastAodv = Simulator::Now ();
  uint64_t packetSize = packet->GetSize ();
  bytesAodv += packetSize;
  NS_LOG_INFO ("AODV stats: " << lastAodv << ", #" << countAodv << ", bytes: " << bytesAodv);
}

void
SimulationRunTime ()
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds ());
  Simulator::Schedule (Seconds (1.0), &SimulationRunTime);
}


uint32_t totalDataPacketsTx = 0;
uint32_t totalDataPacketsRx = 0;

void TxTrace(std::string context, Ptr<const Packet> packet) {
    NS_LOG_UNCOND("Tx: " << context << ", Packet Size = " << packet->GetSize());
    totalDataPacketsTx++;
}


void RxTrace(std::string context, Ptr<const Packet> packet) {
    NS_LOG_UNCOND("Rx: " << context << ", Packet Size = " << packet->GetSize());
    totalDataPacketsRx++;
}


int main(int argc, char *argv[]) {

    double simulationTime = 110;
    uint32_t nNodes = 10;
    // uint32_t numMaliciousNodes = 5;    //1


    NodeContainer nodes;

    for (uint32_t i = 0; i < nNodes; ++i) {
        Ptr<MaliciousNode> maliciousNode = CreateObject<MaliciousNode>();
        nodes.Add(maliciousNode);
    }
// //from here  2
//     std::random_device rd;
//     std::mt19937 gen(rd());

//     std::uniform_int_distribution<> dis(1, nNodes - 2);

//     std::set<int> selectedIndices;

//     while (selectedIndices.size() < numMaliciousNodes) {
//         int randomIndex = dis(gen);
//         selectedIndices.insert(randomIndex);
//     }

//     for (int idx : selectedIndices) {
//         Ptr<MaliciousNode> node = nodes.Get(idx)->GetObject<MaliciousNode>();
//         node->SetIsMalicious(true);
//     }
// //till here 2
    LogComponentEnable("MaliciousNode", LOG_LEVEL_INFO);

    for (uint32_t i = 0; i < nNodes; ++i) {
        Ptr<MaliciousNode> node = nodes.Get(i)->GetObject<MaliciousNode>();
        if (node->IsMalicious()) {
            std::cout << "Node " << i << " is malicious: true" << std::endl;
        }
    }

    // Setup Wi-Fi
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    YansWifiPhyHelper wifiPhy;
    wifiPhy.Set("TxPowerStart", DoubleValue(87.0)); // Transmission power in dBm
    wifiPhy.Set("TxPowerEnd", DoubleValue(87.0));
    wifiPhy.Set("TxGain", DoubleValue(20.0)); // Antenna gain in dBi
    wifiPhy.Set("RxGain", DoubleValue(20.0));
   
    // Set up the propagation channel with the Log-Distance Propagation Loss Model
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                "Exponent", DoubleValue(2.0),  // Path loss exponent
                                "ReferenceDistance", DoubleValue(1.0),  // Reference distance in meters
                                "ReferenceLoss", DoubleValue(46.677));  // Reference loss in dB

    // Set the propagation delay model (optional)
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");


    // wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    // wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    wifiPhy.SetChannel(wifiChannel.Create());
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);


        // Step 3: Configure energy model
    BasicEnergySourceHelper basicSourceHelper;
    // Configure energy source
    basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(1000.0));  // Initial energy in Joules
    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.017));  // Transmission current in Amperes
    radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.010));  // Reception current in Amperes

    // Install energy sources and radio energy models in the nodes
    EnergySourceContainer sources = basicSourceHelper.Install(nodes);
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(devices, sources);

    // Assign IP addresses
    AodvHelper aodv;
    Ipv4ListRoutingHelper list;
    list.Add (aodv, 0);
    InternetStackHelper stack;
    stack.SetRoutingHelper (list);
    // stack.SetRoutingHelper(aodv);
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);


        // Set up mobility
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    mobility.Install(nodes);
    double spacing = 200.0; 

    for (uint32_t nodeId = 0; nodeId < nodes.GetN(); ++nodeId) {
        Ptr<WaypointMobilityModel> mobilityModel = nodes.Get(nodeId)->GetObject<WaypointMobilityModel>();
        Vector startPos(0.0, nodeId * spacing, 300.0); 
        Vector endPos(1000.0, nodeId * spacing, 300.0); 

        mobilityModel->AddWaypoint(Waypoint(Seconds(0.0), startPos));
        mobilityModel->AddWaypoint(Waypoint(Seconds(100.0), endPos));
    }

    // Setup applications
    UdpEchoServerHelper echoServer(9);
    ApplicationContainer serverApps = echoServer.Install(nodes.Get(0));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(simulationTime));

    UdpEchoClientHelper echoClient(interfaces.GetAddress(0), 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(100));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = echoClient.Install(nodes.Get(nNodes-1));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(simulationTime+1));

    // Tracing
    Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpEchoClient/Tx", MakeCallback(&TxTrace));
    Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpEchoServer/Rx", MakeCallback(&RxTrace));

    // Tracing
    Config::Connect("/NodeList/*/$ns3::aodv::RoutingProtocol/Tx", MakeCallback(&AodvPacketTrace));

    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    Simulator::Stop(Seconds(simulationTime+1));
    wifiPhy.EnablePcapAll("line_trace");


    AnimationInterface anim("line_test_aodv.xml");
    for (uint32_t i = 0; i < nNodes; ++i) {
        Ptr<MaliciousNode> node = nodes.Get(i)->GetObject<MaliciousNode>();

        if (i == 0 || i == 9){
          anim.UpdateNodeColor(nodes.Get(i), 0, 255, 0);
        }
        else if (node->IsMalicious()) {
            anim.UpdateNodeColor(nodes.Get(i), 255, 0, 0); // Set malicious nodes color to red
        }
        else {
            anim.UpdateNodeColor(nodes.Get(i), 0, 0, 255); // Set other nodes color to blue
        }
    }
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Run();
    monitor->CheckForLostPackets ();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
        {
        // first 2 FlowIds are for ECHO apps, we don't want to display them
        if (i->first > 2)
            {
            Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
            std::cout << "Flow " << i->first - 2 << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
            std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
            std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
            std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / simulationTime / 1024 / 1024  << " Mbps\n";
            }
        }


    // Simulator::Destroy();
    
    double totalDelay = 0.0;
    uint64_t totalBytesReceived = 0;
    uint32_t totalPacketsReceived = 0;
    // uint32_t totalPacketsSent = 0;
    double totalRemainingEnergy = 0.0;
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<BasicEnergySource> energySource = DynamicCast<BasicEnergySource>(nodes.Get(i)->GetObject<EnergySourceContainer>()->Get(0));
        if (energySource) {
            totalRemainingEnergy += energySource->GetRemainingEnergy();
        }
    }

    double averageRemainingEnergy = totalRemainingEnergy / nodes.GetN();
    

    Simulator::Destroy();

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {
        totalDelay += i->second.delaySum.GetSeconds();  // Sum up all the delays
        totalBytesReceived += i->second.rxBytes;        // Sum up all received bytes
        totalPacketsReceived += i->second.rxPackets;    // Sum up all received packets
        // totalPacketsSent += i->second.txPackets;        // Sum up all sent packets
    }

    // Compute throughput in Mbps
    double throughput = (totalBytesReceived * 8.0 / simulationTime) / (1024 * 1024);

    // Compute average end-to-end delay (if packets were received)
    double averageDelay = totalPacketsReceived > 0 ? totalDelay / totalPacketsReceived : 0;

    // Compute packet delivery rate (if packets were sent)
    double packetDeliveryRate = totalDataPacketsTx > 0 ? (static_cast<double>(totalDataPacketsRx) / totalDataPacketsTx) * 100 : 0;
    double aodvOverheadPercentage = (double(bytesAodv) / totalBytesReceived) * 100;


    std::cout << "bytesAodv:" << double(bytesAodv) << std::endl;
    std::cout << "totalBytesReceived:" << totalBytesReceived << std::endl;


    std::cout << "\nOverall End-to-End Delay: " << averageDelay << " seconds\n";
    std::cout << "Overall Throughput: " << throughput << " Mbps\n";
    std::cout << "Overall Packet Delivery Rate: " << packetDeliveryRate << "%\n";

    
    // Output the results
    std::cout << "Total Data Packets Transmitted: " << totalDataPacketsTx << std::endl;
    std::cout << "Total Data Packets Received: " << totalDataPacketsRx << std::endl;

    std::cout << "AODV overhead [packets]:" << countAodv << std::endl;
    std::cout << "AODV overhead [kB]:" << (double)bytesAodv/1000.0 << std::endl;
    std::cout << "RREQ [packets]:" << countRreq << std::endl;
    std::cout << "RREP [packets]:" << countRrep << std::endl;
    std::cout << "RERR [packets]:" << countRerr << std::endl;
    std::cout << "RREP_ACK [packets]:" << countRrepAck << std::endl;
    std::cout << "aodvOverheadPercentage: " << aodvOverheadPercentage << std::endl;
    std::cout << "Average Remaining Energy per Node: " << averageRemainingEnergy << " Joules" << std::endl;
    return 0;
}
