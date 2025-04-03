#!/bin/bash

set -e

export NS_LOG=""
#export 'NS_LOG=*=level_all|prefix_func|prefix_time' # AGGR-MU-BAR NO-OFDMA
#only edca := phyModel=Yans, dlAckType=NO-OFDMA, useMuEdca=false
##>--txPowerStart=40.0412 --txPowerEnd=40.0412 \

    for run in $(seq 2 2); do
            for nSta in $(seq 10 10); do
                for raru  in $(seq 3 3); do
                ./../ns3 run --gdb --no-build "scratch/wifi-ax-uora-validation/wifi-ax-uora-validation \
                    --simulationTime=8 --accessCategory=AC_VO --phyModel=Spectrum --channelWidth=20 --mcs=8 --RngRun=${run} \
                    --dlAckType=AGGR-MU-BAR --enableUlOfdma=true --enableBsrp=true --useRts=false \
                    --logDir=$PWD/UORA_valocal_naikFalse/${run}-${nSta}-${raru} --ulPayloadSize=1700 \
                    --interval=0.00104 --maxDistance=40 --useMuEdca=true --typeTwoNStations=${nSta}  \
                    --muSchedAccessReqInterval=32 --TxOPLimits=2080 --activateNaiks=false \
                    --UseCentral26TonesRus=true --startLogTime=3 --nMpdus=0 --nMsdus=5 \
                    --DelayAccessReqUponAccess=true --enableAggregation=true \
                    --ruAllocationType=ru-26-tone --nRaRus=${raru} \

                    " #> log_output.txt 2>&1 #
                done
                done
    done
