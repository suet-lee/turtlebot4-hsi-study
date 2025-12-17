# turtlebot4_hsi_study

This repository contains code to run two human-swarm interaction experiments: Human synchronisation via mirroring swarm behaviour, and a collaborative robot sharing task

>[!NOTE]
>It is recommended to deactivate virtual environments when running the ROS nodes in this repository to avoid ROS not finding python dependencies.

## Requirements

The packages in this repository have been developed for ROS2 Jazzy, Ubuntu 24.04, Python version 3.12.

## Overview

This repository can be integrated with real-world tracking systems and gazebo simulation. The expected setup consists of four main components:
1. Central PC: Runs the ROS nodes for broadcasting positional data, managing tasks/teams, sending control commands to robots.

2. Simulation/Motion capture: In the case of simulation, robot positional data is published via a ROS bridge. The central PC subscribes to the specified topics. In the case of Qualisys motion capture, the motion tracking software runs on a dedicated PC and positional data is continuously streamed to a specified port. The central PC uses a qualisys driver (mocap4ros2 package) to access the streamed data via ROS topic `/rigidbodies`.

3. Turtlebot4: The robots should be setup with ROS jazzy. They are connected to the central PC via a discovery server (see section: Discovery server).

4. GUI interface: This is set up as a static website which subscribes and publishes to ROS nodes running on the central PC with websockets using `roslibjs`.

### Packages

- `turtlebot4_broadcast`  
  This package contains the `super_broadcaster_node` which receives positional data from three possible environments (real-world motion tracking systems qualisys and vicon, and gazebo simulation), then broadcasts this data to namespaced topics corresponding to robots.

- `turtlebot4_custom_msg`  
  Custom messages for synchronisation and robot sharing.

- `turtlebot4_sync`  
  This package contains the `sync_node` which should be run for each turtlebot in the synchronisation task. The node implements a swarm milling behaviour.

- `turtlebot4_team`  
  This package contains the `flocking_node` (implements leader-follower flocking behaviour), `task_node` (manages and processes task zones), `teaming_node` (implements the robot sharing mechanism), and `shuffle_node` (test node to check namespacing etc.). The `flocking_node` should be run for each turtlebot, and publishes command velocities based on the position of the leader of the team (which the turtlebot belongs to). Only a single `task_node` and `teaming_node` need to be run.


## Installation
  

### ROS2 Jazzy

Install `ros-jazzy-desktop` following instructions available here: 
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### HSI packages

Create high level workspace `hsi_ws`:
```
cd ~
mkdir -p hsi_ws/src
cd hsi_ws/src
git clone https://github.com/suet-lee/turtlebot4-hsi-study.git
```
Install dependencies:
```
cd ~/hsi_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-path src -yi
```
Compile workspace and source:
```
cd ~/hsi_ws
colcon build --symlink-install
source install/setup.bash
```


### Gazebo simulation

The following packages are required to run with Gazebo Harmonic.
- `turtlebot4`
- `turtlebot4_desktop`
- `turtlebot4_simulator`
- `create3_sim`
- `irobot_create_msgs`

Clone `turtlebot4`, `turtlebot4_desktop` parent repositories:
```
cd ~/hsi_ws/src
git clone https://github.com/turtlebot/turtlebot4.git -b jazzy
git clone https://github.com/turtlebot/turtlebot4_desktop.git -b jazzy
```
Clone `turtlebot4_simulator`, `create3_sim`, `irobot_create_msgs` forked repositories:
```
cd ~/hsi_ws/src
git clone https://github.com/suet-lee/turtlebot4_simulator.git -b jazzy
git clone https://github.com/paoloelle/irobot_create_msgs -b jazzy 
git clone https://github.com/paoloelle/create3_sim.git -b jazzy
```
>[!NOTE]
>Changes have been made in the forked repositories for the specific needs of the HSI study: these include removing the docking station for turtlebots (description and spawning: `turtlebot4_simulator/turtlebot4_gz_bringup/launch/turtlebot4_spawn.launch.py`), removing the sensor plugin from turtlebot model (prevents reloading of the plugin when spawning multiple robots, which causes errors: `create3_sim/irobot_create_common/irobot_create_description/urdf/create3.urdf.xacro`).  

Install dependencies:
```
cd ~/hsi_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-path src -yi
```
Compile workspace:
```
cd ~/hsi_ws
colcon build --symlink-install
source install/setup.bash
```
Install Gazebo harmonic
```
sudo apt-get install ros-jazzy-ros-gz
```
To start the simulation with one spawned robot:
```
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```
To 
The warehouse world is loaded by default.

#### Known issues:

- Firewall/network:
  If you see errors relating to "Requesting list of world names" or "GUI requesting list of world names. The server may be busy downloading resources", this may be a network issue. Run:
  ```
  sudo ufw allow in proto udp to 224.0.0.0/4
  sudo ufw allow in proto udp from 224.0.0.0/4
  ```
  To test:
  ```
  gz sim -v 4 empty.sdf
  ```
  Gazebo should load the world with no problems.

- Start gazebo with -r flag:
  If the simulation is not started immediately with the -r flag, this can cause issues when spawning the robot. This issue has been resolved with a merged PR: 

- Gazebo crashing on launch
  Error: Segmentation fault (Address not mapped to object [0x8])
  ```
  export QT_QPA_PLATFORM=xcb
  ```
  See: https://github.com/gazebosim/gz-sim/issues/2510

### Qualisys tracking system

Integration of qualisys makes use of the `mocap4ros2_qualisys` package. The original repository has been forked to enable access to the subject labels (rigid body names) assigned in the software. Instructions for installation can be followed as per the original repository, repeated here for convenience:

Recursively clone the repository:
```
cd ~/hsi_ws/src
git clone --recursive https://github.com/suet-lee/mocap4ros2_qualisys.git
```
Install dependencies:
```
vcs import < mocap4ros2_qualisys/dependency_repos.repos
```
Compile workspace and source:
```
cd ~/hsi_ws && colcon build --symlink-install
source install/setup.bash
```
Setup qualisys configuration:
```
src/mocap4ros2_qualisys/qualisys_driver/config/qualisys_driver_params.yaml
```
The `host_name` should correspond to the IP address of the dedicated PC running the qualisys tracking software. The `host_port` should be 22223.

To run:
```
ros2 launch qualisys_driver qualisys.launch.py
```
This publishes robot positional data to a `/rigid_bodies` topic.

#### Known issues:

- Errors: `mocap/mocap4r2_control/rqt_mocap4r2_control/include/rqt_mocap4r2_control/SystemController.hpp:0: Note: No relevant classes found. No output generated.`
  `ReceiveRTPacket(CRTPacket::EPacketType &eType, bool bSkipEvents = true, int nTimeout = cWaitForDataTimeout); // nTimeout < 0 : Blocking receive`
  On second compile, these errors seem to resolve themselves.
  

### Vicon tracking system

Instructions for setup can be found here: https://github.com/CPS-Konstanz/turtlebot4-vicon-setup


### GUI interface

The GUI interface is implemented as a static website, served on a specified port using a web server. The following instructions are for usage with the web server nginx. There are two website GUIs available in this repository in folder `www`: `robot_share_gui` and `robot_sync_gui`.

#### Setup website:

Put website files in directory, e.g. `robot_share_gui`:
```
/var/wwww/{robot_share_gui}
```
Install nginx:
```
sudo apt update
sudo apt install nginx
```
Create an nginx configuration file for the website:
```
sudo nano /etc/nginx/sites-available/{robot_share_gui}
```
Add contents to file:
```
server {
  listen 4444;
  listen [::]:4444;


  root /var/www/{robot_share_gui}/html;
  index index.html index.htm index.nginx-debian.html;


  server_name 0.0.0.0;


  location / {
          try_files $uri $uri/ =404;
  }
}
```
The website will be served on port 4444.

Enable the site by creating a symlink:
```
ln -s /etc/nginx/sites-available/{robot_share_gui} /etc/nginx/sites-enabled/
```
Enable nginx, this also automatically starts nginx:
```
sudo systemctl enable nginx
```
Now you can access the website on your machine at http://localhost:4444

To make the website accessible on a network, you also need to allow tcp connections through the firewall, if enabled:
```
sudo ufw allow 4444/tcp
```
Now you should be able to access the web app at http://{your_machine_ip}:4444

>[!NOTE]
>To disable the website simply remove the symbolic link: `sudo rm /etc/nginx/sites-enabled/{robot_share_gui}`


>[!NOTE]
>To delete firewall rules: you can find the relevant rule number using `sudo ufw status numbered`. Then to delete: `sudo ufw delete {rule-number-here}`.

#### Setup rosbridge:

Rosbridge enables clients on the network to connect via a websocket. The website GUI uses `roslibjs`, a javascript library enabling websocket connection.

Clone `rosbridge_suite`:
```
cd ~/hsi_ws/src
git clone https://github.com/RobotWebTools/rosbridge_suite.git
```
Install dependencies:
```
cd ~/hsi_ws
rosdep install --from-path src -yi
```
Compile workspace and source:
```
cd ~/hsi_ws && colcon build --symlink-install
source install/setup.bash
```
To run:
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
The websocket is opened at localhost:9090 by default. To allow other clients on the network to connect, the websocket needs to be open at 0.0.0.0:{port_number}:
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0
```
The websocket is now open at {your_machine_ip}:9090. It is also possible to specify the port using the port:={port_number}.

If a firewall is enabled, tcp connections via the websocket port needs to be allowed:
```
sudo ufw allow 9090/tcp
```

See https://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality for an example implementation for connecting to the websocket.

>[!NOTE]
>You can also add a self-signed SSL certificate, which can enable implementation of features such as phone vibration (not tested).

## Run pipeline for qualisys tracking

This pipeline assumes that the turtlebots are set up with the correct discovery server settings and ROS_DOMAIN_ID=0. See https://github.com/suet-lee/turtlebot4-hsi-study/ref/0_discovery_server_setup.md for more details.

1. In terminal 1: Start fastdds discovery server
  ```
  source ~/hsi_ws/src/turtlebot4_hsi_study/scripts/setup-ros2-discovery.sh pc
  fastdds discovery -l {your_machine_ip} -p 11811 --server-id 1
  ```

>[!NOTE]
>The setup script needs to be sourced (with `'pc'` argument) in any terminal which is publishing to topics that need to be seen by other clients. It connects to the fastdds discovery server (id 1).

2. In terminal 2: Start ROS qualisys node
  ```
  source ~/hsi_ws/src/turtlebot4_hsi_study/scripts/setup-ros2-discovery.sh pc
  source ~/hsi_ws/install/setup.bash
  ros2 launch qualisys_driver qualisys.launch.py
  ```

3. In terminal 3: Start ROS super_broadcaster_node
  ```
  source ~/hsi_ws/src/turtlebot4_hsi_study/scripts/setup-ros2-discovery.sh pc
  source ~/hsi_ws/install/setup.bash
  ros2 launch turtlebot4_broadcast super_broadcaster_launch.py
  ```
  You should ensure that the turtlebot namespaces are correctly matched to the subject labels assigned in the qualisys tracking software: these can be set in `~/hsi_ws/src/turtlebot4_hsi_study/src/turtlebot4_broadcast/config/rigid_body_names.yaml`.

4. In terminal 4: Start ROS bridge for GUI interface
  ```
  source ~/hsi_ws/src/turtlebot4_hsi_study/scripts/setup-ros2-discovery.sh pc
  source ~/hsi_ws/install/setup.bash
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0
  ```

5. In terminals 5+n: Start Turtlebot4 control nodes for n+1 robots
  For each turtlebot:
  ```
  source ~/hsi_ws/src/turtlebot4_hsi_study/scripts/setup-ros2-discovery.sh tb {turtlebot4_ip}
  source ~/hsi_ws/install/setup.bash
  ```
  The ROS topics for the turtlebot should be visible. You should also see the topic available from the nodes running on the central PC. Check:
  ```
  ros2 topic list
  ```
  Run the desired nodes:
  ```
  ros2 launch [...]
  ```

## Run pipeline for Gazebo simulator

This pipeline assumes that spawned robots are namespaced as `'turtlebot4_{n}'`. The config files for synchronisation (`~/hsi_ws/src/turtlebot4_hsi_study/src/turtlebot4_sync/config/sync_teams.yaml`) and teaming (`~/hsi_ws/src/turtlebot4_hsi_study/src/turtlebot4_team/config/share_teams.yaml`) should correspond to the spawned robot namespaces, to run the synchronisation and flocking behaviour respectively.

>[!NOTE]
>Robots must be spawned in separate terminals.

1. In terminal 1: Start Gazebo simulation with one spawned robot (default namespace `'turtlebot4_0'`)
  ```
  cd ~/hsi_ws
  source install/setup.bash
  cd src/turtlebot4_hsi_study/scripts
  ./start_sim.sh {no_spawned_robots}
  ```
  Where the first argument to the script `start_sim.sh` is the total number of spawned robots to track positional data. The script runs a ROS-Gazebo bridge so that positional data is published from Gazebo to a specified ROS topic.

2. In terminal 2: Spawn an additional robot with namespace `'turtlebot4_1'`
  ```
  cd ~/hsi_ws
  source install/setup.bash
  cd src/turtlebot4_hsi_study/scripts
  ./spawn_robot.sh 1
  ```

3. In terminal 3: Start ROS super_broadcaster_node
  ```
  cd ~/hsi_ws
  source install/setup.bash
  ros2 launch turtlebot4_broadcast super_broadcaster_launch.py n_robots:={no_spawned_robots}
  ```

4. In terminal 4: Run desired node
  E.g. flocking behaviour for `'turtlebot4_1'`:
  ```
  cd ~/hsi_ws
  source install/setup.bash
  ros2 launch turtlebot4_team flocking_launch.py namespace:=turtlebot4_1
  ```