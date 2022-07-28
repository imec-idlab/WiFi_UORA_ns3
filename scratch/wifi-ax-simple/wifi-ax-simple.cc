/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2022 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Andrey Belogaev <andrei.belogaev@uantwerpen.be>
 */

#include <functional>
#include <ns3/wifi-module.h>
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/progress-bar.h"
#include "ns3/mac48-address.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

using namespace ns3;

struct stats_t
{
  double sinr;
  uint64_t totalBeacons;
};
std::map<Mac48Address, stats_t> g_statsMap; //!< map for statistics for each station

void SinrTrace (Mac48Address addr, double sinr)
{
  g_statsMap[addr].sinr += sinr;
  g_statsMap[addr].totalBeacons++;
}

uint8_t GetTosFromAccessCategory (std::string ac)
{
  if (ac == "AC_BE")
    {
      return 0x70;
    }
  else if (ac == "AC_BK")
    {
      return 0x28;
    }
  else if (ac == "AC_VI")
    {
      return 0xb8;
    }
  else if (ac == "AC_VO")
    {
      return 0xc0;
    }
  else
    {
      NS_FATAL_ERROR ("Wrong access category");
    }
}

int main (int argc, char *argv[])
{
  //LogComponentEnable ("ApWifiMac", LOG_LEVEL_ALL);
  double simulationTime {10}; //seconds
  std::string logDir {"output"};
  double minDistance {1.0}; //meters
  double maxDistance {5.0}; //meters
  std::size_t nStations {1};
  double apHeight {3}; //meters
  double staHeight {1}; //meters
  
  std::string frequency {"5"}; //whether 2.4, 5 or 6 GHz
  uint32_t channelWidth {40}; //for 2.4 <= 40 MHz, for 5 <= 160 MHz
  uint32_t guardInterval {800}; //in ns
  int mcs {11}; // 0..11
  std::string phyModel {"Yans"};
  bool enableShadowing = false;

  std::string dlAckSeqType {"NO-OFDMA"};
  bool enableUlOfdma {false};
  bool enableBsrp {false};
  Time accessReqInterval {0};
  bool useRts {false};
  bool useExtendedBlockAck {false};

  uint32_t payloadSize = 700; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
  bool udp {true};
  bool downlink {true};
  std::string trafficType {"Poisson"};
  double interval {0.00001}; //seconds
  std::string accessCategory {"AC_BE"};

  CommandLine cmd (__FILE__);
  //Experiment set-up
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("logDir", "Directory for the output stats files", logDir);
  cmd.AddValue ("minDistance", "Minimum distance in meters between the station and the access point", minDistance);
  cmd.AddValue ("maxDistance", "Maximum distance in meters between the station and the access point", maxDistance);
  cmd.AddValue ("nStations", "Number of non-AP HE stations", nStations);
  cmd.AddValue ("apHeight", "Height of AP [m]", apHeight);
  cmd.AddValue ("staHeight", "Height of STA [m]", staHeight);
  
  //PHY params
  cmd.AddValue ("frequency", "Whether working in the 2.4, 5 or 6 GHz band (other values gets rejected)", frequency);
  cmd.AddValue ("channelWidth", "Channel width used for transmissions (20, 40, 80, 80+80, 160 MHz)", channelWidth);
  cmd.AddValue ("guardInterval", "The value of the guard interval (800, 1600, 3200 ns)", guardInterval);
  cmd.AddValue ("mcs", "if set, limit testing to a specific MCS (0-11)", mcs);
  cmd.AddValue ("phyModel", "PHY model to use when OFDMA is disabled (Yans or Spectrum). If OFDMA is enabled then Spectrum is automatically selected", phyModel);
  cmd.AddValue ("enableShadowing", "Enable/disable gaussian shadowing", enableShadowing);

  //MAC params
  cmd.AddValue ("useRts", "Enable/disable RTS/CTS", useRts);
  cmd.AddValue ("useExtendedBlockAck", "Enable/disable use of extended BACK", useExtendedBlockAck);
  cmd.AddValue ("dlAckType", "Ack sequence type for DL OFDMA (NO-OFDMA, ACK-SU-FORMAT, MU-BAR, AGGR-MU-BAR)", dlAckSeqType);
  cmd.AddValue ("enableUlOfdma", "Enable UL OFDMA (useful if DL OFDMA is enabled and TCP is used)", enableUlOfdma);
  cmd.AddValue ("enableBsrp", "Enable BSRP (useful if DL and UL OFDMA are enabled and TCP is used)", enableBsrp);
  cmd.AddValue ("muSchedAccessReqInterval", "Duration of the interval between two requests for channel access made by the MU scheduler", accessReqInterval);

  //Traffic params
  cmd.AddValue ("payloadSize", "The application payload size in bytes", payloadSize);
  cmd.AddValue ("udp", "UDP if set to 1, TCP otherwise", udp);
  cmd.AddValue ("downlink", "Generate downlink flows if set to 1, uplink flows otherwise", downlink);
  cmd.AddValue ("trafficType", "Poisson / CBR", trafficType);
  cmd.AddValue ("interval", "Average inter-packet interval [s]", interval);
  cmd.AddValue ("accessCategory", "Access category for traffic: AC_BE, AC_BK, AC_VO, AC_VI", accessCategory);
  
  cmd.Parse (argc, argv);

  /* Change dir. */
  mkdir (logDir.c_str (), 0777);
  if (chdir (logDir.c_str ()) == -1)
    {
      perror ("chdir");
      NS_FATAL_ERROR ("chdir: " << strerror (errno));
    }

  if (useRts)
    {
      Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("0"));
    }

  if (dlAckSeqType == "ACK-SU-FORMAT")
    {
      Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                          EnumValue (WifiAcknowledgment::DL_MU_BAR_BA_SEQUENCE));
    }
  else if (dlAckSeqType == "MU-BAR")
    {
      Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                          EnumValue (WifiAcknowledgment::DL_MU_TF_MU_BAR));
    }
  else if (dlAckSeqType == "AGGR-MU-BAR")
    {
      Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                          EnumValue (WifiAcknowledgment::DL_MU_AGGREGATE_TF));
    }
  else if (dlAckSeqType != "NO-OFDMA")
    {
      NS_ABORT_MSG ("Invalid DL ack sequence type (must be NO-OFDMA, ACK-SU-FORMAT, MU-BAR or AGGR-MU-BAR)");
    }

  if (phyModel != "Yans" && phyModel != "Spectrum")
    {
      NS_ABORT_MSG ("Invalid PHY model (must be Yans or Spectrum)");
    }
  if (dlAckSeqType != "NO-OFDMA")
    {
      // SpectrumWifiPhy is required for OFDMA
      phyModel = "Spectrum";
    }

  if (!udp)
    {
        Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (payloadSize));
    }

  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (nStations);
  NodeContainer wifiApNode;
  wifiApNode.Create (1);

  // Mobility.
  MobilityHelper staMobility;
  Ptr<UniformDiscPositionAllocator> positionAlloc = CreateObject<UniformDiscPositionAllocator> ();
  positionAlloc->SetRho (maxDistance);
  positionAlloc->SetMinDist (minDistance);
  positionAlloc->SetX (0);
  positionAlloc->SetY (0);
  positionAlloc->SetZ (staHeight);
  staMobility.SetPositionAllocator (positionAlloc);
  staMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  staMobility.Install (wifiStaNodes);

  Ptr<ConstantPositionMobilityModel> apMobility = CreateObject<ConstantPositionMobilityModel> ();
  apMobility->SetPosition ({0, 0, apHeight});
  wifiApNode.Get (0)->AggregateObject (apMobility);

  // WI-FI PHY & MAC
  NetDeviceContainer apDevice, staDevices;
  WifiMacHelper mac;
  WifiHelper wifi;
  std::string channelStr ("{0, " + std::to_string (channelWidth) + ", ");

  wifi.SetStandard (WIFI_STANDARD_80211ax);

  if (frequency == "2.4")
    {
      channelStr += "BAND_2_4GHZ, 0}";
    }
  else if (frequency == "5")
    {
      channelStr += "BAND_5GHZ, 0}";
    }
  else if (frequency == "6")
    {
      channelStr += "BAND_6GHZ, 0}";
    }
  else
    {
      NS_FATAL_ERROR ("Wrong frequency value!");
    }

  std::ostringstream oss;
  oss << "HeMcs" << mcs;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue (oss.str ()),
                              "ControlMode", StringValue (oss.str ()));
  // Set guard interval and MPDU buffer size
  wifi.ConfigHeOptions ("GuardInterval", TimeValue (NanoSeconds (guardInterval)),
                      "MpduBufferSize", UintegerValue (useExtendedBlockAck ? 256 : 64));

  Ssid ssid = Ssid ("ns3-80211ax");

  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  Ptr<IeeeIndoorPropagationLossModel> propModel = CreateObject<IeeeIndoorPropagationLossModel> ();
  if (enableShadowing) 
    {
      propModel->SetAttribute ("EnableShadowing", BooleanValue (true));
    }

  if (phyModel == "Spectrum")
    {
      /*
      * SingleModelSpectrumChannel cannot be used with 802.11ax because two
      * spectrum models are required: one with 78.125 kHz bands for HE PPDUs
      * and one with 312.5 kHz bands for, e.g., non-HT PPDUs (for more details,
      * see issue #408 (CLOSED))
      */
      Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();
      spectrumChannel->AddPropagationLossModel (propModel);
      spectrumChannel->SetPropagationDelayModel (delayModel);
      SpectrumWifiPhyHelper phy;
      phy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
      phy.SetChannel (spectrumChannel);

      mac.SetType ("ns3::StaWifiMac",
                  "Ssid", SsidValue (ssid));
      phy.Set ("ChannelSettings", StringValue (channelStr));
      staDevices = wifi.Install (phy, mac, wifiStaNodes);

      if (dlAckSeqType != "NO-OFDMA")
        {
          mac.SetMultiUserScheduler ("ns3::RrMultiUserScheduler",
                                      "EnableUlOfdma", BooleanValue (enableUlOfdma),
                                      "EnableBsrp", BooleanValue (enableBsrp),
                                      "AccessReqInterval", TimeValue (accessReqInterval));
        }
      mac.SetType ("ns3::ApWifiMac",
                  "EnableBeaconJitter", BooleanValue (false),
                  "Ssid", SsidValue (ssid));
      apDevice = wifi.Install (phy, mac, wifiApNode);
    }
  else
    {
      //YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
      Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel> ();
      channel->SetPropagationLossModel (propModel);
      channel->SetPropagationDelayModel (delayModel);

      YansWifiPhyHelper phy;
      phy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
      phy.SetChannel (channel); //channel.Create ()

      mac.SetType ("ns3::StaWifiMac",
                  "Ssid", SsidValue (ssid));
      phy.Set ("ChannelSettings", StringValue (channelStr));
      staDevices = wifi.Install (phy, mac, wifiStaNodes);

      mac.SetType ("ns3::ApWifiMac",
                  "EnableBeaconJitter", BooleanValue (false),
                  "Ssid", SsidValue (ssid));
      apDevice = wifi.Install (phy, mac, wifiApNode);
    }

  /* Internet stack*/
  InternetStackHelper stack;
  stack.Install (wifiApNode);
  stack.Install (wifiStaNodes);

  Ipv4AddressHelper address;
  address.SetBase ("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer staNodeInterfaces;
  Ipv4InterfaceContainer apNodeInterface;

  staNodeInterfaces = address.Assign (staDevices);
  apNodeInterface = address.Assign (apDevice);

  /* Setting applications */
  ApplicationContainer serverApp;
  auto serverNodes = downlink ? std::ref (wifiStaNodes) : std::ref (wifiApNode);
  Ipv4InterfaceContainer serverInterfaces;
  NodeContainer clientNodes;
  for (std::size_t i = 0; i < nStations; i++)
    {
      serverInterfaces.Add (downlink ? staNodeInterfaces.Get (i) : apNodeInterface.Get (0));
      clientNodes.Add (downlink ? wifiApNode.Get (0) : wifiStaNodes.Get (i));
    }

  if (udp)
    {
      //UDP flow
      uint16_t port = 9;
      UdpServerHelper server (port, GetTosFromAccessCategory (accessCategory));
      serverApp = server.Install (serverNodes.get ());
      serverApp.Start (Seconds (0.0));
      serverApp.Stop (Seconds (simulationTime + 1));

      for (std::size_t i = 0; i < nStations; i++)
        {
          InetSocketAddress remoteAddress = InetSocketAddress (serverInterfaces.GetAddress (i), port);
          remoteAddress.SetTos (GetTosFromAccessCategory (accessCategory));
          UdpClientHelper client (remoteAddress);
          client.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
          if (trafficType == "CBR") //CBR traffic
            {
              client.SetAttribute ("EnableRandom", BooleanValue (false));
              client.SetAttribute ("Interval", TimeValue (Time ("0.00001")));
            }
          else //Poisson traffic
            {
              client.SetAttribute ("EnableRandom", BooleanValue (true));
              std::ostringstream intervalDistr;
              intervalDistr << "ns3::ExponentialRandomVariable[Mean=" << interval << "|Bound=0]";
              client.SetAttribute ("IntervalRandomVariable", StringValue (intervalDistr.str()));
            }
          
          client.SetAttribute ("PacketSize", UintegerValue (payloadSize));
          ApplicationContainer clientApp = client.Install (clientNodes.Get (i));
          clientApp.Start (Seconds (1.0));
          clientApp.Stop (Seconds (simulationTime + 1));
        }
    }
  else
    {
      //TCP flow
      uint16_t port = 50000;

      InetSocketAddress localAddress = InetSocketAddress (Ipv4Address::GetAny (), port);
      localAddress.SetTos (GetTosFromAccessCategory (accessCategory));
      PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", localAddress);
      serverApp = packetSinkHelper.Install (serverNodes.get ());
      serverApp.Start (Seconds (0.0));
      serverApp.Stop (Seconds (simulationTime + 1));

      for (std::size_t i = 0; i < nStations; i++)
        {
          OnOffHelper onoff ("ns3::TcpSocketFactory", Ipv4Address::GetAny ());
          onoff.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
          onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
          onoff.SetAttribute ("PacketSize", UintegerValue (payloadSize));
          onoff.SetAttribute ("DataRate", DataRateValue (1000000000)); //bit/s
          InetSocketAddress remoteAddress = InetSocketAddress (serverInterfaces.GetAddress (i), port);
          remoteAddress.SetTos (GetTosFromAccessCategory (accessCategory));
          onoff.SetAttribute ("Remote", AddressValue(remoteAddress));
          ApplicationContainer clientApp = onoff.Install (clientNodes.Get (i));
          clientApp.Start (Seconds (1.0));
          clientApp.Stop (Seconds (simulationTime + 1));
        }
    }

  Simulator::Schedule (Seconds (0), &Ipv4GlobalRoutingHelper::PopulateRoutingTables);

  //Connect traces
  for (uint32_t i = 0; i < staDevices.GetN (); i++)
    {
      staDevices.Get (i)->GetObject <WifiNetDevice> ()->GetPhy ()->GetPhyEntity (WIFI_MOD_CLASS_OFDM)
        ->TraceConnectWithoutContext ("SinrTrace", MakeBoundCallback (&SinrTrace, Mac48Address::ConvertFrom (staDevices.Get (i)->GetAddress ())));
      staDevices.Get (i)->GetObject <WifiNetDevice> ()->GetPhy ()->GetPhyEntity (WIFI_MOD_CLASS_HE)
        ->TraceConnectWithoutContext ("SinrTrace", MakeBoundCallback (&SinrTrace, Mac48Address::ConvertFrom (staDevices.Get (i)->GetAddress ())));
    }

  ProgressBar pg (Seconds(simulationTime + 1));
  Simulator::Stop (Seconds (simulationTime + 1));
  Simulator::Run ();

  // Statistics
  uint64_t rxBytes = 0;
  if (udp)
    {
      for (uint32_t i = 0; i < serverApp.GetN (); i++)
        {
          rxBytes += payloadSize * DynamicCast<UdpServer> (serverApp.Get (i))->GetReceived ();
        }
    }
  else
    {
      for (uint32_t i = 0; i < serverApp.GetN (); i++)
        {
          rxBytes += DynamicCast<PacketSink> (serverApp.Get (i))->GetTotalRx ();
        }
    }
  double throughput = (rxBytes * 8) / (simulationTime * 1000000.0); //Mbit/s

  std::ofstream totalStatsOutStream;
  totalStatsOutStream.open ("throughput.out", std::ios::out);
  totalStatsOutStream << "MCS\tBandwidth, MHz\tGuard interval, ns\tThroughput, Mbps" << std::endl;
  totalStatsOutStream << mcs << "\t" << channelWidth << "\t" << guardInterval << "\t" << throughput << std::endl;
  totalStatsOutStream.close ();

  std::ofstream perStaOutStream;
  perStaOutStream.open ("per_sta_stats.out", std::ios::out);
  perStaOutStream << "MAC addr\tSINR, dB" << std::endl;
  for (auto &staStats : g_statsMap)
    {
      staStats.second.sinr /= staStats.second.totalBeacons;
      perStaOutStream << staStats.first << "\t" << staStats.second.sinr << std::endl;
    }
  perStaOutStream.close ();

  Simulator::Destroy ();

  return 0;
}
