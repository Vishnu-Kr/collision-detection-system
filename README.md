# Mobile Robot Collision Detection System
Implementation of collision detection system for a mobile robot in a simulated environment
&nbsp;
&nbsp;

&nbsp;


## Procedure:
Create a ros workspace:
 ```
 mkdir -p catkin_ws/src
 ```
Clone this repository into it's source folder
 ```
 cd src
 https://github.com/Vishnu-Kr/collision-detection-system.git
 ```

Update Dependencies
 ```
 cd ..
 rosdep install --from-paths src --ignore-src -r -y
 ```
Build and Source:
```
catkin build
source devel/setup.bash
```

&nbsp;

## Run:
open a terminal and launch the below:
```
roslaunch kshoonya_gazebo world.launch
```
open 2nd terminal and launch the below:
```
roslaunch kshoonya_gazebo robot.launch
```


open 3rd terminal and launch the below:
```
cd src/kshoonya_core/scripts
python3 collision_avoidance.py
```
open 4th terminal and launch the teleop Node:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```
&nbsp;
<a href="https://youtu.be/rOPMT2CvkBc">Video Demonstration </a>
&nbsp;

## References
- <a href="http://wiki.ros.org/Robots/Husky">Husky Robot Model </a>
