# msu_ros

Implements ```Motors Skill Units``` or whatever

# Installation:
## mujoco
download & install mujoco150 in ~/.mujoco
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
download my fork of mujoco_ros_pkgs
```
cd ~/msu_ws/src
git clone https://github.com/babbatem/mujoco_ros_pkgs.git
```

download kinova_ros
