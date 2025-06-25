# Example control for the Pollen Robotics "AmazingHand" (a.k.a. AH!)

## How to use:
- Install Rust: https://www.rust-lang.org/tools/install
- Install uv: https://docs.astral.sh/uv/getting-started/installation/
- Install dora-rs: https://dora-rs.ai/docs/guides/Installation/installing
  - start the daemon: `dora up`

- Clone this repository and in a console from the directory run:
- `uv venv`
- To run in simulation only:
  - `dora build dataflow_simu.yml --uv` (needs to be done only once)
  - `dora run dataflow_simu.yml --uv`
- To run with real hardware:
  - `dora build dataflow_real.yml --uv` (needs to be done only once)
  - `dora run dataflow_real.yml --uv`


## Hand Setup

![alt text](docs/finger.png "Motors naming for each finger")

![alt text](docs/r_hand.png "Fingers naming for each hand")

Be sure to adapt the configuration file [r_hand.toml](AHControl/config/r_hand.toml) for your particular hand.
You can use the software tools located in [AHControl](AHControl).


## Details

- [AHControl](AHControl) contains a dora-rs node to control the motors, along with some useful tools to configure them.
- [AHSimulation](AHSimulation) contains a dora-rs node to simulate the hand and get the inverse kinematics.
- [HandTracking](HandTracking) contains a dora-rs node to track hands from a webcam and use it as target to control AH!.
