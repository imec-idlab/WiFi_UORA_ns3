/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2024
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
 * Author: Douglas Agbeve <douglas.agbeve@uantwerpen.be>
 */

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <ns3/wifi-module.h>
#include <unistd.h>
#include <vector>
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/mobility-helper.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/application-container.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/progress-bar.h"
#include "ns3/seq-ts-header.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-tag.h"
#include "ns3/ap-server-helper.h"

#include "ns3/arp-cache.h"

using namespace ns3;

struct stats_t
{
  double sinr;
  uint64_t totalBeacons;
  uint64_t rxBytes;
  std::vector<double> pktDelays, genTime, rxTime;
  std::vector<uint64_t> pktTag;
  std::vector<Mac48Address> src;

  //late packet parameters
  uint64_t rxBytesLate;
  std::vector<double> pktDelaysLate, genTimeLate, rxTimeLate;
  std::vector<uint64_t> pktTagLate;
  std::vector<Mac48Address> srcLate;
};

std::map<Mac48Address, stats_t> g_statsMap; //!< map for statistics for each station
std::map<Ipv4Address, Mac48Address> g_Ip2Mac; //!< map for mapping IPv4 to MAC address
std::map<Mac48Address, stats_t> g_statsMapAp;  //!< map for starts for only AP
void SinrTrace (Mac48Address addr, double sinr)
{
  g_statsMap[addr].sinr += sinr;
  g_statsMap[addr].totalBeacons++;
}


void RxTraceWithAddress ( Ptr<const Packet> p, const Address &src, const Address &dst)
{
  Ptr<Packet> pkt = p->Copy ();

  SeqTsHeader seqTs;
  PacketTag tag;
  pkt->RemovePacketTag(tag);
  pkt->RemoveHeader(seqTs);
  //startLogTime *= 1e+9;

  //if (Simulator::Now().GetNanoSeconds () >= startLogTime) {

    g_statsMap[g_Ip2Mac.at (InetSocketAddress::ConvertFrom (src).GetIpv4 ())].rxBytes += p->GetSize ();
    g_statsMap[g_Ip2Mac.at (InetSocketAddress::ConvertFrom (src).GetIpv4 ())].pktDelays.push_back ((Simulator::Now() - seqTs.GetTs ()).GetSeconds ());
    g_statsMap[g_Ip2Mac.at (InetSocketAddress::ConvertFrom (src).GetIpv4 ())].pktTag.push_back (tag.GetValue());
    g_statsMap[g_Ip2Mac.at (InetSocketAddress::ConvertFrom (src).GetIpv4 ())].genTime.push_back(seqTs.GetTs().GetSeconds() );
    g_statsMap[g_Ip2Mac.at (InetSocketAddress::ConvertFrom (src).GetIpv4 ())].rxTime.push_back(Simulator::Now().GetSeconds());
    g_statsMap[g_Ip2Mac.at (InetSocketAddress::ConvertFrom (src).GetIpv4 ())].src.push_back( g_Ip2Mac.at (InetSocketAddress::ConvertFrom (src).GetIpv4 ()));
  //}
}



int main(int argc, char *argv[])
{
  //LogComponentEnable("WifiPhy", LOG_LEVEL_DEBUG);
  //LogComponentEnable("WifiPhyStateHelper", LOG_LEVEL_DEBUG);
  //LogComponentEnable("WifiMac", LOG_LEVEL_DEBUG);
  //LogComponentEnable("ApWifiMac", LOG_LEVEL_DEBUG);
  //LogComponentEnable("WifiDefaultAckManager", LOG_LEVEL_DEBUG);
  //LogComponentEnable("HeFrameExchangeManager", LOG_LEVEL_DEBUG);
  //LogComponentEnable("BlockAckManager", LOG_LEVEL_DEBUG);
  //LogComponentEnable("ChannelAccessManager", LOG_LEVEL_DEBUG);
  //LogComponentEnable("QosTxop", LOG_LEVEL_DEBUG);
  //LogComponentEnable("Txop", LOG_LEVEL_DEBUG);

  double simulationTime {10};//seconds
  std::string logDir {"output"};
  double minDistance {1.0}; //meters
  double maxDistance {4.0}; //meters
  std::size_t totalNStations {1};
  double apHeight {3}; //meters
  double staHeight {1}; //meters
  double startLogTime = {1.030};


  std::string frequency {"5"}; //whether 2.4, 5 or 6 GHz
  uint32_t channelWidth {40}; //for 2.4 <= 40 MHz, for 5 <= 160 MHz
  uint32_t guardInterval {800}; //in ns
  int mcs {8}; // 0..11
  std::string phyModel {"Spectrum"}; //Spectrum or Yans
  bool enableShadowing = false;
  double txPowerStart = {40.0412}; //16.0206
  double txPowerEnd = {40.0412};
  uint32_t nMsdus = 1;
  uint32_t nRaRus {0};


  std::string dlAckSeqType {"AGGR-MU-BAR"};
  bool enableUlOfdma {false};
  bool enableBsrp {false};
  double accessReqInterval = {0}; //microseconds
  bool useRts {false};
  bool useExtendedBlockAck {true};
  bool useMuEdca {true};
  uint32_t nMpdus = 1;
  bool UseCentral26TonesRus = {true};
  bool DelayAccessReqUponAccess = {true};
  double txOpLimits = {2080};  //AC_VO default value in microseconds
  std::string ruAllocationType {"ru-undefined"};
  HeRu::RuType ruAllocType {HeRu::RU_UNDEFINED};


  uint32_t ulPayloadSize = 93; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
  std::string trafficType {"CBR"}; //CBR or not
  double interval {0.005}; //seconds
  bool randomStart {false};
  std::string accessCategory {"AC_VO"};
  bool enableAggregation {true};


  CommandLine cmd (__FILE__);

  //Experiment set-up
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("logDir", "Directory for the output stats files", logDir);
  cmd.AddValue ("minDistance", "Minimum distance in meters between the station and the access point", minDistance);
  cmd.AddValue ("maxDistance", "Maximum distance in meters between the station and the access point", maxDistance);
  cmd.AddValue ("totalNStations", "Total number of non-AP HE stations", totalNStations);
  cmd.AddValue ("apHeight", "Height of AP [m]", apHeight);
  cmd.AddValue ("staHeight", "Height of STA [m]", staHeight);
  cmd.AddValue ("startLogTime","Time to begin logging [seconds]",startLogTime);


  //PHY params
  cmd.AddValue ("frequency", "Whether working in the 2.4, 5 or 6 GHz band (other values gets rejected)", frequency);
  cmd.AddValue ("channelWidth", "Channel width used for transmissions (20, 40, 80, 80+80, 160 MHz)", channelWidth);
  cmd.AddValue ("guardInterval", "The value of the guard interval (800, 1600, 3200 ns)", guardInterval);
  cmd.AddValue ("mcs", "if set, limit testing to a specific MCS (0-11)", mcs);
  cmd.AddValue ("phyModel", "PHY model to use when OFDMA is disabled (Yans or Spectrum). If OFDMA is enabled then Spectrum is automatically selected", phyModel);
  cmd.AddValue ("enableShadowing", "Enable/disable gaussian shadowing", enableShadowing);
  cmd.AddValue ("txPowerStart", "Minimum available transmissions level (dBm)", txPowerStart);
  cmd.AddValue ("txPowerEnd", "Maximum available transmissions level (dBm)", txPowerEnd);
  cmd.AddValue ("UseCentral26TonesRus","Enable/Disable the use of Cenral 26 tone RUs", UseCentral26TonesRus);
  cmd.AddValue ("ruAllocationType","The type of RUs allocated to STAs, if undefined - dynamically selected based on the number of STAs (ru-26-tone, ru-52-tone)", ruAllocationType);
  cmd.AddValue ("nRaRus","Number of RUs to reserve for Random Access", nRaRus);


  //MAC params
  cmd.AddValue ("useRts", "Enable/disable RTS/CTS", useRts);
  cmd.AddValue ("useExtendedBlockAck", "Enable/disable use of extended BACK", useExtendedBlockAck);
  cmd.AddValue ("dlAckType", "Ack sequence type for DL OFDMA (NO-OFDMA, ACK-SU-FORMAT, MU-BAR, AGGR-MU-BAR)", dlAckSeqType);
  cmd.AddValue ("enableUlOfdma", "Enable UL OFDMA (useful if DL OFDMA is enabled and TCP is used)", enableUlOfdma);
  cmd.AddValue ("enableBsrp", "Enable BSRP (useful if DL and UL OFDMA are enabled and TCP is used)", enableBsrp);
  cmd.AddValue ("muSchedAccessReqInterval", "Duration of the interval between two requests for channel access made by the MU scheduler (MicroSeconds)", accessReqInterval);
  cmd.AddValue ("useMuEdca", "Enable/disable Specific EDCA parameters set", useMuEdca);
  cmd.AddValue ("nMsdus", "Number of aggregated MSDUs (=< 11398/740 for VHT/HE/EHT PPDUs)", nMsdus);
  cmd.AddValue ("nMpdus", "Number of aggregated MPDUs (=< 6500631/740 for HE PPDUs)", nMpdus);
  cmd.AddValue ("DelayAccessReqUponAccess", "Enable/Disable delay of access req only upon access grant", DelayAccessReqUponAccess);
  cmd.AddValue ("TxOPLimits", "TX Opportunity limits in microseconds", txOpLimits);


  //Traffic params
  cmd.AddValue ("ulPayloadSize", "the application payload size in bytes for ul traffic", ulPayloadSize);
  cmd.AddValue ("interval", "Average inter-packet interval [s]", interval);
  cmd.AddValue ("accessCategory", "Access category for traffic: AC_BE, AC_BK, AC_VO, AC_VI", accessCategory);
  cmd.AddValue ("enableAggregation", "Enable/disable mpdu/msdu aggregation", enableAggregation);



  cmd.Parse (argc, argv);


  /*
   * make and change into the lodDir
   */
  std::string mkdir {"mkdir -p " + logDir};
  if ( system(mkdir.c_str()) == -1 )
  {
    NS_FATAL_ERROR("Error in creating log directory" << strerror(errno));
  }
  if ( chdir(logDir.c_str()) == -1 )
  {
    perror ("chdir");
    NS_FATAL_ERROR ("Error changing into directory " << strerror (errno));
  }

  /*
   * rts/cts
   * */
  if (useRts)
    {
      Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("0"));
      Config::SetDefault("ns3::WifiDefaultProtectionManager::EnableMuRts", BooleanValue(true));
    }

  /*
   * Downlink ACK type
   * */
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

  /*
   * PHY model
   */
  if (phyModel != "Yans" && phyModel != "Spectrum")
    {
      NS_ABORT_MSG ("Invalid PHY model (must be Yans or Spectrum)");
    }

  if (dlAckSeqType != "NO-OFDMA")
    {
      // SpectrumWifiPhy is required for OFDMA
      phyModel = "Spectrum";
    }

  /*
   * Type of resource unit tones
   */
  if (ruAllocationType == "ru-26-tone" )
    ruAllocType = HeRu::RU_26_TONE;
  else if (ruAllocationType == "ru-52-tone")
    ruAllocType = HeRu::RU_52_TONE;

  /*
   * Create totalNStations of STA and 1 AP
   */
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create(totalNStations);
  NodeContainer wifiApNode;
  wifiApNode.Create(1);

  /*
   * Mobility and Positioning: Constant mobility and randomly distributed nodes around AP
   */
  MobilityHelper staMobility;
  Ptr<UniformDiscPositionAllocator> positionAlloc = CreateObject<UniformDiscPositionAllocator>();
  positionAlloc->SetRho(maxDistance);
  positionAlloc->SetMinDist(minDistance);
  positionAlloc->SetX(0);
  positionAlloc->SetY(0);
  positionAlloc->SetZ(staHeight);
  staMobility.SetPositionAllocator(positionAlloc);
  staMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  staMobility.Install(wifiStaNodes);

  Ptr<ConstantPositionMobilityModel> apMobility = CreateObject<ConstantPositionMobilityModel> ();
  apMobility->SetPosition ({0, 0, apHeight});
  wifiApNode.Get (0)->AggregateObject (apMobility);

  /*
   * To use specific UORA Parameter sets
   */
  if(nRaRus){
    Config::SetDefault ("ns3::HeConfiguration::OCwMin",UintegerValue(7));
    Config::SetDefault("ns3::HeConfiguration::OCwMax", UintegerValue(127));

    //prevent stations from explicitly sending BAR request
    Config::SetDefault("ns3::QosTxop::UseExplicitBarAfterMissedBlockAck", BooleanValue(false));
  }

  /*
   * To use specific MU-EDCA parameters for various AC
   */
  if (useMuEdca)
    {
      //STA AIFSN
      Config::SetDefault ("ns3::HeConfiguration::MuBkAifsn", UintegerValue (0));
      Config::SetDefault ("ns3::HeConfiguration::MuBeAifsn", UintegerValue (0));
      Config::SetDefault ("ns3::HeConfiguration::MuViAifsn", UintegerValue (0));
      Config::SetDefault ("ns3::HeConfiguration::MuVoAifsn", UintegerValue (0));

      //STA CW min
      Config::SetDefault ("ns3::HeConfiguration::MuBkCwMin", UintegerValue (32767));
      Config::SetDefault ("ns3::HeConfiguration::MuBeCwMin", UintegerValue (32767));
      Config::SetDefault ("ns3::HeConfiguration::MuViCwMin", UintegerValue (32767));
      Config::SetDefault ("ns3::HeConfiguration::MuVoCwMin", UintegerValue (32767));

      //STA CW max
      Config::SetDefault ("ns3::HeConfiguration::MuBkCwMax", UintegerValue (32767));
      Config::SetDefault ("ns3::HeConfiguration::MuBeCwMax", UintegerValue (32767));
      Config::SetDefault ("ns3::HeConfiguration::MuViCwMax", UintegerValue (32767));
      Config::SetDefault ("ns3::HeConfiguration::MuVoCwMax", UintegerValue (32767));

      /* Disable EDCA for the duration of the entire simulation:
       *    Calculate the ceiling multiple of
       *    8192 from simulation time
       */
      int  simTime = Seconds(simulationTime).GetMicroSeconds();
      double  muEdcaTimer =  (simTime % 8192 == 0) ? simTime : ( std::round( simTime / 8192.0) * 8192) ;

      //STA timer (validity of set)
      Config::SetDefault ("ns3::HeConfiguration::BkMuEdcaTimer", TimeValue (MicroSeconds(muEdcaTimer) ));
      Config::SetDefault ("ns3::HeConfiguration::BeMuEdcaTimer", TimeValue (MicroSeconds(muEdcaTimer) ));
      Config::SetDefault ("ns3::HeConfiguration::ViMuEdcaTimer", TimeValue (MicroSeconds(muEdcaTimer) ));
      Config::SetDefault ("ns3::HeConfiguration::VoMuEdcaTimer", TimeValue (MicroSeconds(muEdcaTimer) ));

      Config::SetDefault("ns3::HtFrameExchangeManager::DisableEDCA", BooleanValue(true));

    }

  /*
   * SIFS and Slot time
   */
  Config::SetDefault("ns3::WifiPhy::Sifs", TimeValue(MicroSeconds(16)));
  Config::SetDefault("ns3::WifiPhy::Slot", TimeValue(MicroSeconds(9)));

  /*
   * Retry limit
   */
  Config::SetDefault ("ns3::WifiRemoteStationManager::MaxSsrc", UintegerValue (32));
  Config::SetDefault ("ns3::WifiRemoteStationManager::MaxSlrc", UintegerValue (32));

  /*
   * Intrinsic Queue Size reporting
   */
  Config::SetDefault("ns3::QosFrameExchangeManager::SetQueueSize", BooleanValue(false));

  if (enableAggregation)
    Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("990000"));

  /*
   * Set up Network devices
   */
  NetDeviceContainer apNetDevice, staNetDevices;
  WifiMacHelper wifimac;
  WifiHelper wifi;
  std::string channelStr ("{0, " + std::to_string (channelWidth) + ", ");

  wifi.SetStandard(WIFI_STANDARD_80211ax);

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
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager","DataMode", StringValue(oss.str()), "ControlMode", StringValue(oss.str()));

  /*Set guard interval and MPDU buffer size
   */
  wifi.ConfigHeOptions ("GuardInterval", TimeValue (NanoSeconds (guardInterval)),
                      "MpduBufferSize", UintegerValue (useExtendedBlockAck ? 256 : 64));
  /*
   * Setup PHY and MAC layers
   */

  Ssid ssid = Ssid("ns3-80211ax");

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
      phy.Set ("ChannelSettings", StringValue (channelStr));
      phy.Set("TxPowerStart", DoubleValue(txPowerStart ));
      phy.Set("TxPowerEnd", DoubleValue(txPowerEnd ));
      phy.Set("RxNoiseFigure", DoubleValue(7));

      wifimac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid),
                       "VO_BlockAckThreshold", UintegerValue(0));

      staNetDevices = wifi.Install (phy, wifimac, wifiStaNodes);

      if (dlAckSeqType != "NO-OFDMA")
        {
          wifimac.SetMultiUserScheduler ("ns3::RrMultiUserScheduler",
                                      "EnableUlOfdma", BooleanValue (enableUlOfdma),
                                      "EnableBsrp", BooleanValue (enableBsrp),
                                      "AccessReqInterval", TimeValue (MicroSeconds(accessReqInterval)),
                                      "DelayAccessReqUponAccess", BooleanValue (DelayAccessReqUponAccess),
                                      "UseCentral26TonesRus", BooleanValue (UseCentral26TonesRus),
                                      "NStations", UintegerValue (totalNStations),
                                      "AccessReqAc",EnumValue(AcIndex::AC_VO),
                                      "RuAllocationType", EnumValue(ruAllocType),
                                      "NumRandomAccessRus", UintegerValue(nRaRus),
                                      "MaxCredits", TimeValue(Seconds(1)),
                                      "UlPsduSize", UintegerValue(ulPayloadSize)
                                      );
        }
        wifimac.SetType ("ns3::ApWifiMac",
                  "EnableBeaconJitter", BooleanValue (false),
                  "Ssid", SsidValue (ssid),
                  "VO_BlockAckThreshold", UintegerValue(0),
                  "BeaconGeneration", BooleanValue(true),
                  "BsrLifetime", TimeValue(MilliSeconds(20)),
                  "BeaconInterval", TimeValue(MicroSeconds(102400)));

      apNetDevice = wifi.Install (phy, wifimac, wifiApNode);
      //phy.EnablePcap("wifi-ax-uora-ap", apNetDevice); // enable pcap tracing
      //phy.EnablePcap("wifi-ax-uora-sta", staNetDevices);
    }
  else
    {
      NS_ABORT_MSG("Invalid PHY model (must be Spectrum)");
    }

  /*
   * Internet Stack on
   */
  InternetStackHelper internetStack;
  internetStack.Install(wifiApNode);
  internetStack.Install(wifiStaNodes);

  Ipv4AddressHelper address;
  address.SetBase("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer staNodeInterfaces = address.Assign (staNetDevices);
  Ipv4InterfaceContainer apNodeInterface = address.Assign (apNetDevice);

  ArpCache::PopulateArpCache ();


  /*
   * Setup server on AP and client on STAs applications
   */
  uint16_t port = {9};
  InetSocketAddress localAddress = InetSocketAddress (apNodeInterface.GetAddress(0), port);
  localAddress.SetTos (0xc0);
  PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", localAddress);
  ApplicationContainer apServerApp = packetSinkHelper.Install(wifiApNode);
  apServerApp.Start(Seconds (0));
  apServerApp.Stop (Seconds (simulationTime + startLogTime));



  double staStartTime = 0;

  for (size_t i = 0; i < totalNStations; i++)
  {
    staStartTime += 0.00108;
    //InetSocketAddress remoteAddress = InetSocketAddress (serverInterfaces.GetAddress (i), port);
    //remoteAddress.SetTos (0xc0);
    UdpClientHelper staUdpClient (localAddress);
    staUdpClient.SetAttribute("MaxPackets", UintegerValue(4294967295u));
    staUdpClient.SetAttribute("PacketSize", UintegerValue(ulPayloadSize));
    if (trafficType == "CBR")
    {
      staUdpClient.SetAttribute ("EnableRandom", BooleanValue (false));
      staUdpClient.SetAttribute ("Interval", TimeValue (Seconds (interval)));
    }
    else
    {
      staUdpClient.SetAttribute ("EnableRandom", BooleanValue (true));
      std::ostringstream intervalDistr;
      intervalDistr << "ns3::ExponentialRandomVariable[Mean=" << interval << "|Bound=0]";
      staUdpClient.SetAttribute ("IntervalRandomVariable", StringValue (intervalDistr.str()));
    }
    ApplicationContainer staUdpclientApp = staUdpClient.Install (wifiStaNodes.Get(i));
    if (randomStart)
    {
      Ptr<RandomVariableStream> rv = CreateObject<UniformRandomVariable> ();
      rv->SetAttribute("Max", DoubleValue(0.5));
      staUdpclientApp.StartWithJitter (Seconds (0.0), rv);
    }
    else
    {
      staUdpclientApp.Start (Seconds (staStartTime));
    }
    staUdpclientApp.Stop (Seconds (simulationTime + startLogTime));

  }

  Simulator::Schedule (Seconds (0), &Ipv4GlobalRoutingHelper::PopulateRoutingTables);

  for (uint32_t i = 0; i < staNetDevices.GetN (); i++)
    {
      Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> ( staNetDevices.Get(i) );
      Ptr<WifiMac> wifi_mac = dev->GetMac();
      if (enableAggregation) {
        wifi_mac->SetAttribute("VO_MaxAmpduSize", UintegerValue (nMpdus * ulPayloadSize ));
        wifi_mac->SetAttribute("VO_MaxAmsduSize", UintegerValue (nMsdus * ulPayloadSize ));
      }
      wifi_mac->GetQosTxop(AC_VO)->SetTxopLimit(MicroSeconds(txOpLimits));
      auto mac = Mac48Address::ConvertFrom (staNetDevices.Get (i)->GetAddress ());
      auto ip = staNodeInterfaces.Get (i).first->GetAddress (1, 0).GetAddress ();
      g_Ip2Mac[ip] = mac;

      staNetDevices.Get(i)->GetObject<WifiNetDevice>()->GetPhy()->GetPhyEntity(WIFI_MOD_CLASS_OFDM)->TraceConnectWithoutContext("SinrTrace", MakeBoundCallback (&SinrTrace, Mac48Address::ConvertFrom (staNetDevices.Get (i)->GetAddress ())));
      staNetDevices.Get(i)->GetObject<WifiNetDevice>()->GetPhy()->GetPhyEntity(WIFI_MOD_CLASS_HE)->TraceConnectWithoutContext("SinrTrace", MakeBoundCallback (&SinrTrace, Mac48Address::ConvertFrom (staNetDevices.Get (i)->GetAddress ())));
    }


  Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::PacketSink/RxWithAddresses", MakeBoundCallback(&RxTraceWithAddress));


  ProgressBar pg (Seconds(simulationTime + startLogTime ));
  Simulator::Stop (Seconds (simulationTime + startLogTime ));
  Simulator::Run ();

  uint64_t rxBytes = 0;

  for (uint32_t i = 0; i < wifiApNode.GetN (); i++)
    {
      rxBytes += DynamicCast<PacketSink> (apServerApp.Get (i))->GetTotalRx ();
    }


  double throughput = (rxBytes * 8) / (simulationTime * 1000000.0); //Mbit/s

  std::ofstream totalStatsOutStream;
  totalStatsOutStream.open ("throughput.out", std::ios::out);
  totalStatsOutStream << "MCS\tBandwidth_MHz\tGuard_interval_ns\tThroughput_Mbps" << std::endl;
  totalStatsOutStream << mcs << "\t" << channelWidth << "\t" << guardInterval << "\t" << throughput << std::endl;
  totalStatsOutStream.close ();

  std::ofstream perStaOutStream;
  perStaOutStream.open ("per_sta_stats.out", std::ios::out);
  perStaOutStream << "MAC-addr\tSINR_dB\tRX_bytes\tAvg_delay_ms" << std::endl;
  for (auto &staStats : g_statsMap)
    {
      staStats.second.sinr /= staStats.second.totalBeacons;
      double sumDelay = 0.;
      for (auto &delay : staStats.second.pktDelays)
        {
          sumDelay += delay * 1000.; //in ms
        }
      perStaOutStream << staStats.first << "\t" << staStats.second.sinr << "\t" << staStats.second.rxBytes
        << "\t" << sumDelay / staStats.second.pktDelays.size ()  << std::endl;
    }
  perStaOutStream.close ();

  Simulator::Destroy();
  return 0;
}
