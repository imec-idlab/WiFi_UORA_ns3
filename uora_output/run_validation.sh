#!/bin/bash

set -e

export NS_LOG=""

for run in $(seq 2 2); do
    for nSta in $(seq 9 9); do
        for nRaRus  in $(seq 9 9); do
            ./../ns3 run --no-build "scratch/wifi-ax-uora-validation/wifi-ax-uora-validation \
                --simulationTime=8 --accessCategory=AC_VO --phyModel=Spectrum --channelWidth=20 --mcs=8 --RngRun=${run} \
                --dlAckType=AGGR-MU-BAR --enableUlOfdma=true --enableBsrp=true --useRts=false \
                --logDir=$PWD/uora_validation/${run}-${nSta}-${raru} --ulPayloadSize=1700 \
                --interval=0.00104 --maxDistance=40 --useMuEdca=true --totalNStations=${nSta}  \
                --muSchedAccessReqInterval=124 --TxOPLimits=2080 --activateNaiks=true \
                --UseCentral26TonesRus=true --startLogTime=4 --nMpdus=0 --nMsdus=5 \
                --DelayAccessReqUponAccess=true --enableAggregation=false \
                --ruAllocationType=ru-26-tone --nRaRus=${nRaRus} \ "
            done
        done
    done
