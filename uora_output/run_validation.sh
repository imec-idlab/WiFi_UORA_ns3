#!/bin/bash

#
# For 80 MHz change the following parameters:
# --ruAllocationType=ru-106-tone
# for nRaRus in 0 $(seq 1 2 5) 8
#


set -e

export NS_LOG=""

for run in $(seq 1 5); do
    for nSta in $(seq 9 9 100); do
        for nRaRus in 0 $(seq 1 2 9); do
            ./../ns3 run --no-build "scratch/wifi-ax-uora-validation/wifi-ax-uora-validation \
                --simulationTime=8 --accessCategory=AC_VO --phyModel=Spectrum --channelWidth=80 --mcs=8 --RngRun=${run} \
                --dlAckType=AGGR-MU-BAR --enableUlOfdma=true --enableBsrp=true --useRts=false \
                --logDir=$PWD/uora_validation/${run}-${nSta}-${nRaRus} --ulPayloadSize=1700 \
                --interval=0.00052 --maxDistance=40 --useMuEdca=true --totalNStations=${nSta}  \
                --muSchedAccessReqInterval=124 --TxOPLimits=2080 --activateNaiks=true \
                --UseCentral26TonesRus=false --startLogTime=4 --nMpdus=0 --nMsdus=5 \
                --DelayAccessReqUponAccess=true --enableAggregation=true \
                --ruAllocationType=ru-26-tone --nRaRus=${nRaRus}"
            done
        done
    done
