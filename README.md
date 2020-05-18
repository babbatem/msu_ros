# msu_ros

Implements ```Motors Skill Units``` aka ```Composable Interaction Primitives```

# Installation:
## mujoco
1. download & extract mujoco150 in ~/.mujoco
2. download mjkey.txt from slack/email/Ben and place it in ~/.mujoco
3. set env variables (preferably in .bashrc so they are loaded on shell creation)
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
1. download my fork of mujoco_ros_pkgs  
```
cd ~/msu_ws/src
git clone https://github.com/babbatem/mujoco_ros_pkgs.git
```  

2. download kinova_ros (our test platform, for now)   
```
cd ~/msu_ws/src
git clone https://github.com/Kinovarobotics/kinova-ros.git
```

3. download msu_ros  
```
git clone https://github.com/babbatem/msu_ros.git
```
please change the hard-coded filepaths in ```launch/moveit_integration.launch```


4. compile  
```
cd ~/msu_ws/
catkin build
```

if you run into issues here, let me know and we can work through them.

5. test (load mujoco HWSim and control it using kinova & moveit packages)  
```
source devel/setup.bash
roslaunch msu_ros moveit_integration.launch &
python msu_ros/src/scripts/msu_ros_test.py
```

For a brief python kdl example, see src/scripts/kdl_test.py.
Note that the robot has changed a bit, and the model is in msu_ros/assets/kinova...
