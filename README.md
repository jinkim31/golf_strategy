# Rospy Boilerplate
Barebone boilerplate for ROS-Python nodes

## Getting started

- Compile & run
```
$ cd catkin_ws/src
$ git clone https://github.com/jinkim31/rospy_boilerplate.git
$ cd ~/catkin_ws && catkin_make
$ rosrun rospy_boilerplate main.py
```

- (optional) Install dependencies(e.g. opencv, scipy)
```
$ sudo apt-get install python-catkin-tools
$ sudo apt-get install python3-opencv
$ sudo apt-get install python3-scipy
```

- (optional) For those machines that use Python2 for default, install Python3 
```
$ sudo apt install python3-pip python3-all-dev python3-rospkg
$ sudo apt install ros-melodic-desktop-full --fix-missing
```

## Using with Anaconda virtual environment

1. Install Anaconda: https://www.anaconda.com/


2. Create new virtual environment
```
conda create -n ros python=3.8
```

3. Activate virtual environment
```
conda activate ros
```

4. Install ROS dependencies
```
pip install -U rospkg
pip install netifaces
```

5. (optional) Install additional dependencies
```
pip install --upgrade tensorflow
pip install tensorflow-probability
pip install opencv-python
pip install scipy
pip install gym
pip install matplotlib
pip install pygame
```

6. Set node interpreter
```
in src/main.py, change the first line:

#! /usr/bin/env python3

to

#! /home/candy/anaconda3/envs/ros/bin/python
```

7. Compile & run
```
$ cd ~/catkin_ws && catkin_make
$ rosrun rospy_boilerplate main.py
```
