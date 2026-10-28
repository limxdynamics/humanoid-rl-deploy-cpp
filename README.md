<p align="center">
  <b>English</b> | <a href="README_cn.md">中文</a>
</p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache--2.0-2f6feb" alt="License: Apache-2.0"></a>
  <img src="https://img.shields.io/badge/Ubuntu-20.04%2B-2f6feb" alt="Ubuntu 20.04 or newer">
  <img src="https://img.shields.io/badge/onnxruntime-1.10.0-2f6feb" alt="onnxruntime 1.10.0">
  <img src="https://img.shields.io/badge/ROS-not%20required-2f6feb" alt="ROS is not required">
</p>

# Deployment of Training Results

## 1. Overview

If there is no need to rely on ROS and you prefer pure C++ development, this is
the deployment example to follow. It covers environment configuration, the
compilation process and basic function implementation. The code structure is
meant to be easy to reuse or extend, which lowers the development threshold and
keeps project setup and iterations short.

## 2. Engineering Environment Setup

Ubuntu 20.04 or newer is assumed. Install the project dependencies as follows.

### 2.1 Compilation environment

```bash
sudo apt update
sudo apt install cmake build-essential libeigen3-dev libyaml-cpp-dev python3 python3-pip -y
sudo pip3 install vcstool colcon-common-extensions
```

### 2.2 onnxruntime

Install the onnxruntime dependency from the
[v1.10.0 release page](https://github.com/microsoft/onnxruntime/releases/tag/v1.10.0),
choosing the version that matches your OS and platform. For Ubuntu 20.04
x86_64, follow these steps:

```bash
wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz

tar xvf onnxruntime-linux-x64-1.10.0.tgz

sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
```

## 3. Create a Workspace

Create an RL deployment and development workspace as follows.

### 3.1 Clone both repositories

The workspace holds two repositories: the MuJoCo simulator and this deployment
example. Both are cloned with `--recurse`, which also fetches their submodules.
Each one carries a `limxsdk-lowlevel` submodule: the simulator's supplies the
wheel installed in 4.1, and this repository's supplies the library linked in
4.2, so a clone without submodules fails at both steps.

```bash
mkdir -p ~/limx_ws
cd ~/limx_ws

# Option 1: HTTPS
git clone --recurse https://github.com/limxdynamics/humanoid-mujoco-sim.git
git clone --recurse https://github.com/limxdynamics/humanoid-rl-deploy-cpp.git

# Option 2: SSH
git clone --recurse git@github.com:limxdynamics/humanoid-mujoco-sim.git
git clone --recurse git@github.com:limxdynamics/humanoid-rl-deploy-cpp.git
```

### 3.2 Set the robot model

`ROBOT_TYPE` selects which configuration directory the controllers load. List
the available robot types first:

```bash
cd ~/limx_ws/humanoid-rl-deploy-cpp/src
tree -L 1 humanoid-rl-deploy-cpp/robot_controllers/mimic_controller/config
```

```text
humanoid-rl-deploy-cpp/robot_controllers/mimic_controller/config
├── HU_D03_03
└── HU_D04_01
```

`tree` comes from the `tree` package, which the apt line in 2.1 does not
include; if the command above reports `command not found`, run
`sudo apt install tree`.

Taking `HU_D04_01` (replace with your actual robot type) as an example, set the
robot model type:

```bash
echo 'export ROBOT_TYPE=HU_D04_01' >> ~/.bashrc && source ~/.bashrc
```

## 4. Simulation Debugging

Three programs run at the same time — the simulator, the controller and the
virtual joystick — so each one gets its own Bash terminal.

### 4.1 MuJoCo simulator

Python 3.8 or higher is recommended. In the first terminal:

```bash
cd ~/limx_ws
pip install -r humanoid-mujoco-sim/requirements.txt
```

That installs MuJoCo (`mujoco>=3.0.0`), which `simulator.py` imports at
start-up. Then install the motion control development library, choosing the
architecture of your machine.

Linux x86_64 environment:

```bash
cd ~/limx_ws
pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/amd64/limxsdk-*.whl
```

Linux aarch64 environment:

```bash
cd ~/limx_ws
pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/aarch64/limxsdk-*.whl
```

Run the MuJoCo simulator:

```bash
cd ~/limx_ws
python humanoid-mujoco-sim/simulator.py
```

A MuJoCo window opens. Nothing drives the robot until the controller in 4.2 is
running, so start the controller first and then click **Reset** in the left
panel.

### 4.2 Controller

In the second terminal:

```bash
# 1. Compile the algorithm code
cd ~/limx_ws/humanoid-rl-deploy-cpp
make

# 2. Run the algorithm
source build/install/setup.bash
robot_hw
```

`robot_hw` takes the robot address as an optional argument. Without it the
address is `127.0.0.1`, which is the simulator; section 5 passes the robot's
address instead. The program then serves a command-line interface on port 8888
(`telnet localhost 8888`) and stays in the foreground until `Ctrl+C`.

![](doc/simulator.gif)

### 4.3 Virtual joystick

Use the virtual remote control to operate the robot during simulation. In the
third terminal:

```bash
~/limx_ws/humanoid-mujoco-sim/robot-joystick/robot-joystick
```

![](doc/robot-joystick.png)

You can now use the virtual joystick to control the robot.

| **Button** | **Mode**         | **Description**                                                    |
| -------- | ---------------- | ----------------------------------------------------------- |
| L1+Y     | Switch to Stand Mode   | If the robot cannot stand, click "Reset" in the MuJoCo interface to reset it. |
| L1+B     | Switch to Greeting Mode |                                                             |
| R1+X     | Switch to Walk Mode|  **You can train your model for this mode!**                                 |

### 4.4 Deploy your own model for robot walking (Optional)

- In the [humanoid-rl-isaaclab](https://github.com/limxdynamics/humanoid-rl-isaaclab) project, run the inference code and export the model. It writes `policy.onnx` into an `exported/` folder next to the checkpoint:

  ```bash
  python scripts/rsl_rl/play.py --task LimX-Oli-31dof-Velocity --checkpoint path-to-model
  ```

- Add the `policy.onnx` to the folder

  ```text
  limx_ws/humanoid-rl-deploy-cpp/src/humanoid-rl-deploy-cpp/robot_controllers/walking_controller/config/HU_D04_01/policy
  ```

- Run `make` again: the `install(DIRECTORY config/.)` rule in
  `walking_controller/CMakeLists.txt` copies that folder into
  `build/install/etc/walking_controller/`, and only the installed copy is
  loaded at run time.

- Follow 4.1 and 4.2 to run the MuJoCo simulator and control the robot.

- Make sure your model runs perfectly in the simulation before deploying it on
  the real robot.

> A walking configuration is shipped for `HU_D04_01` only.
> `robot_controllers/walking_controller/config/` has no `HU_D03_03` directory,
> so that robot needs one added before Walk Mode can load.

## 5. Real Machine Debugging

- Set your computer's IP: ensure your computer is connected to the robot via the external network port. Set your computer's IP address to `10.192.1.200` and verify connectivity with `ping 10.192.1.2`. Configure your computer's IP as shown:

  ![IP configuration](doc/ip.png)

- Robot preparation:

  - Hang the robot by the hooks on its shoulders.
  - Power on the robot, then press the `right joystick` on the remote to start the motors.
  - Press `R1 + DOWN` on the remote to switch to developer mode. This mode persists across reboots. Exit with `R1 + LEFT`.

- Real machine deployment: execute the following shell commands in a Bash terminal to start the control algorithm:

  ```bash
  # 1. Compile the algorithm code
  cd ~/limx_ws/humanoid-rl-deploy-cpp
  make

  # 2. Enter the robot's IP address (10.192.1.2) and run the algorithm
  source build/install/setup.bash
  robot_hw 10.192.1.2
  ```

- Now you can press `L1 + △ (Y)` on the remote control to make the robot stand up.

- Press `L1 + 〇 (B)` on the remote control to make the robot wave.

- Press `R1 + 口 (X)` on the remote control to make the robot walk.
