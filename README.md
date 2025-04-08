## Wi-Fi UORA in NS-3

This repository houses the IEEE 802.11 Uplink OFDMA Random Access (UORA) feature within the Wi-Fi module of ns-3.

## Configure and Build

- Follow the instructions on [ns-3 Installation Guide](https://www.nsnam.org/wiki/Installation) to prepare all dependencies.
- Clone the project
- Change directory into the WiFi_UORA_ns3 directory
- Configure with
```shell
./ns3 configure --disable-static --disable-gtk --disable-test --disable-examples --cxx-standard=17
```
- Build with
```shell
./ns3 build
```

## Reproduce results of the paper

- Change directory into the uora_output/ directory
- Run with
```shell
./run_validation.sh
```
This runs the simulation in `scratch/wifi-ax-uora-validation/wifi-ax-uora-validation.cc`. Explanations of the parameters are found in the said file.
At the end of the simulation, data is outputed in the same directory.
- To plot, run
```shell
python plotter.py
```
