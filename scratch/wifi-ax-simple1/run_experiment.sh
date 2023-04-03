#!/bin/bash

set -e

for downlink in "true"; do
    prefix="downlink"
    if [ "$downlink" == "false" ]; then
        prefix="uplink"
    fi
    for run in $(seq 1 1); do
        for nSta in $(seq 6 6); do
            ./../../ns3 run --no-build "wifi-ax-simple1 --RngRun=$run --simulationTime=20 --nStations=$nSta --downlink=$downlink --accessCategory=AC_VO --phyModel=Spectrum --channelWidth=20 --mcs=3\
            --dlAckType=AGGR-MU-BAR --enableUlOfdma=true --enableBsrp=true --useRts=true --logDir=$PWD/results/$run-$prefix-withOfdma-$nSta \
            --udp=true --payloadSize=740 --trafficType=CBR --interval=0.005"
        done
    done
done
