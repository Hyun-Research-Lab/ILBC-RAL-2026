# ILBC RA-L 2026
This repository provides the code to recreate the experiments in the paper "Learning One-Step Inverse for Performance Improvement of Nonlinear Control Systems: Application to Quadrotor Control".

## Installation
To get started, clone the repository. This project uses submodules, so be sure to clone with `--recursive`.
```
git clone --recursive https://github.com/Hyun-Research-Lab/ILBC-RAL-2026
```
If you already cloned the repository without `--recursive`, you can initialize and update the submodules with:
```
cd ILBC-RAL-2026
git submodule init
git submodule update
```
Next, create a virtual environment:
```
cd ILBC-RAL-2026
python3 -m venv .venv
source .venv/bin/activate
```
Then, install the dependencies:
```
pip install -r requirements.txt
```

## Set up the Crazyflie
To build and flash the Crazyflie firmware, first follow the instructions to install a toolchain in [the Crazyflie building and flashing instructions](./crazyflie-firmware/docs/building-and-flashing/build.md). To build, run:
```
cd crazyflie-firmware/ILBC-RAL-2026
make cf2_defconfig
make -j8
```
To flash the firmware, put the Crazyflie in bootloader mode by holding the power button until the blue lights start flashing and then run:
```
make cload
```
For more information, refer to the [Crazyflie documentation](./crazyflie-firmware/docs/building-and-flashing/build.md).

The system also relies on the usd card deck for logging, see [usd_deck.md](./usd_deck.md) to set it up.

## Run the script
In `main.py`, change the mocap system type, host name, URI, and rigid body name to match your system. Then, choose an inital condition (line 123) and the type of experiment to run (line 128). See the comments in the code for guidance. Then to run an experiment, simply execute the script:
```
python main.py
```