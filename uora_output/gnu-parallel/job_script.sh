#!/bin/bash

read $(echo "$1" | tr ',' ' ') < <(echo $2 | tr ',' ' ')

../../ns3 run --no-build "scratch/wifi-ax-uora-validation/wifi-ax-uora-validation \
  --simulationTime=20 --totalNStations=${nSta} --RngRun=${run} \
  --accessCategory=AC_VO --phyModel=Spectrum --channelWidth=80 --mcs=8 \
  --enableUlOfdma=true --enableBsrp=true --useRts=false \
  --logDir=$(dirname "$(pwd)")/uora_validation/${run}-${nSta}-${nRaRus} \
  --ulPayloadSize=1024 --interval=0.00104 --maxDistance=40 \
  --useMuEdca=true --muSchedAccessReqInterval=124 --txPowerStart=40.0412 \
  --txPowerEnd=40.0412 --UseCentral26TonesRus=true --TxOPLimits=2080 \
  --startLogTime=4 --DelayAccessReqUponAccess=true --nRaRus=${nRaRus} \
  --enableAggregation=false --ruAllocationType=ru-26-tone --activateNaiks=true"
