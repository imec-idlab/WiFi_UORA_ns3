#!/bin/bash

ml atools/slurm

sbatch --array $(arange --data noOfdma.csv) noOfdma.slurm
