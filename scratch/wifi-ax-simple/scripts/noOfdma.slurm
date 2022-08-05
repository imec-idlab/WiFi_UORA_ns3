#!/bin/bash

module purge
ml calcua/supported
ml GCC/9.3.0
ml atools/slurm

# specification (required!)
#SBATCH --ntasks=1 --cpus-per-task=1 --mem-per-cpu=2g

# requested running time (required!)
#SBATCH --time=05:00

#
# send mail notification (optional)
#   FAIL   when job is aborted
#   BEGIN  when job begins
#   END    when job ends
#   ... and more, see the manual page of sbatch.
#SBATCH --mail-type=FAIL
#SBATCH --mail-user=andrei.belogaev@uantwerpen.be

source <(aenv --data noOfdma.csv)

prefix="downlink"
if [ "$downlink" == "false" ]; then
    prefix="uplink"
fi

./wifi-ax-simple --RngRun=$run --logDir=$PWD/output/$run-$prefix-noOfdma-$nStations --nStations=$nStations --downlink=$downlink --accessCategory=AC_BE --udp=true --useRts=false --ns3::WifiMacQueue::MaxSize=10000p
