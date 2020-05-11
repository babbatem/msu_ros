# msu_ros

Implements ```Motors Skill Units``` or whatever we're calling it these days.

# Installation:
## mujoco
download & extract mujoco150 in ~/.mujoco
download mjkey.txt from slack/email/Ben and place it in ~/.mujoco
set env variables (preferably in .bashrc so they are loaded on shell creation)
```
export MUJOCO_VERSION="mjpro151"
export MUJOCO_PRO_PATH="$HOME/.mujoco/${MUJOCO_VERSION}/"
export MUJOCO_KEY_PATH="$HOME/.mujoco/mjkey.txt"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/${MUJOCO_VERSION}/bin
```

## ros workspace
create a workspace
```
mkdir -p ~/msu_ws/src/
cd ~/msu_ws
catkin build
```

## msu_ros
download my fork of mujoco_ros_pkgs
(modified CMakeLists.txt and source code to use env variables above, adding depth image publishing)
```
cd ~/msu_ws/src
git clone https://github.com/babbatem/mujoco_ros_pkgs.git
```

download kinova_ros (our test platform, for now)
```
cd ~/msu_ws/src
git clone https://github.com/Kinovarobotics/kinova-ros.git
```

download msu_ros
```
git clone https://github.com/babbatem/msu_ros.git
```

compile
```
cd ~/msu_ws/
catkin build
```

if you run into issues here, let me know and we can work through them. I suspect they will culminate in more general formulations of CMakeLists.txt in mujoco_ros_pkgs.

test (load mujoco HWSim and control it using kinova & moveit packages)
```
source devel/setup.bash
roslaunch msu_ros moveit_integration.launch &
python msu_ros/src/scripts/msu_ros_test.py
```

## TODOs
turn this structure into submodules
