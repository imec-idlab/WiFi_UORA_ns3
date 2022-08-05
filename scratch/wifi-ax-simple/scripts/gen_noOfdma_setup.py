#!/apps/antwerpen/rome/centos8/Python/3.8.3-intel-2020a/bin/python

import pandas as pd

run = range (1, 6)
nStations = range (1, 21)
downlink = ['true', 'false']

index = pd.MultiIndex.from_product([run, nStations, downlink], names = ["run", "nStations", "downlink"])
pd.DataFrame(index = index).reset_index().to_csv("noOfdma.csv", index=False)
