# PX4-Autopilot-with-ROS2-Micro-XRCE-DDS-and-QGroundControl-Setup-Guide
Welcome! This guide provides step-by-step instructions to set up a simulation environment for [PX4-Autopilot](https://px4.io/) integrated with [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), using [Micro XRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html) for communication, and [QGroundControl](https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl-x86_64.AppImage) for monitoring and control. By the end, you’ll have a simulated drone that you can control via ROS2 and monitor with QGroundControl—no hardware required!

---
## What You’ll Achieve
- Simulate a drone using PX4’s Software-in-the-Loop (SITL) with Gazebo.
- Connect PX4 to ROS2 for publishing/subscribing to drone data.
- Use QGroundControl to visualize and control the simulated drone.
- Optionally, run an example to control the drone programmatically with ROS2.
This is perfect for developers, hobbyists, or researchers looking to test drone applications in a simulated environment before moving to real hardware.
---
## Prerequisites
Before you begin, ensure you have:
- Operating System: Ubuntu 22.04 LTS (recommended for compatibility).
- Hardware: No physical drone needed
- Knowledge: Basic familiarity with Linux command-line tools (e.g., apt, git, terminal navigation).
---
## Setup Instructions
Follow these steps in order. Each includes commands you can copy-paste into your terminal.
### Step 1: Set Up the Development Environment
Prepare your system with PX4 dependencies and tools.

#### 1. Install Git (skip if already installed):
```bash
sudo apt-get update
sudo apt-get install git
```

#### 2. Clone the PX4-Autopilot Repository:
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

#### 3. Run the PX4 Setup Script (installs Gazebo and other dependencies):
```bash
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```
#### 4. Reboot to apply changes:
```bash
sudo reboot
```

### Step 2: Install ROS2 Humble
Add ROS2 Humble to your system for drone control and communication.

#### 1. Install ROS2 Humble:
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
```

#### 2. Source ROS2 (add to your ~/.bashrc for convenience):
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Set Up Micro XRCE-DDS
Install the Micro XRCE-DDS Agent to bridge PX4 and ROS2.

#### 1. Clone and Build the Agent:
```bash
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
```
```bash
cmake ..
```
```bash
make
```
```bash
sudo make install
sudo ldconfig /usr/local/lib/
```

### Step 4: Set Up px4_ros_com for ROS2 Integration
Integrate PX4 with ROS2 using the px4_ros_com package.

#### 1. Create a ROS2 Workspace:
```bash
mkdir -p ~/px4_ros_ws/src
cd ~/px4_ros_ws/src
```

#### 2. Clone PX4 ROS2 Repositories:
```bash
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
```

#### 3. Build the Workspace:
```bash
cd ~/px4_ros_ws
colcon build
```
#### 4. Source the Workspace:
```bash
source ~/px4_ros_ws/install/setup.bash
```

### Step 5: Set Up PX4 Simulation with Gazebo
Launch a simulated drone with PX4 SITL and Gazebo.

#### 1. Run the Simulation:
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```
- You should see Gazebo open with a drone model.

### Step 6: Start the Micro XRCE-DDS Agent
Connect PX4 to ROS2 with the DDS agent.

#### 1. Run the Agent (in a new terminal):
```bash
MicroXRCEAgent udp4 -p 8888
```

### Step 7: Verify Communication Between PX4 and ROS2
Check if PX4 is talking to ROS2.

#### 1. List ROS2 Topics (in a new terminal):
```bash
source ~/px4_ros_ws/install/setup.bash
ros2 topic list
```
- Look for topics like /fmu/out/vehicle_odometry. If you see them, the connection works!

### Step 8: Install and Set Up QGroundControl
Monitor your simulated drone with QGroundControl.

#### 1. Download QGroundControl:
Get the AppImage from the [official site](https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl-x86_64.AppImage).

#### 2. Run QGroundControl:
```bash
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```
- Look for topics like /fmu/out/vehicle_odometry. If you see them, the connection works!

