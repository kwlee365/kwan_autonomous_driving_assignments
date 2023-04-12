# CARLA Simulation

## Topics Intelligent Convergence Systems (2023) 

Disclaimer: Please note that I am not responsible for any negative consequences that may result from referring to or using this code. Use at your own risk.

## Environments setup

1. Ubuntu 18.04 or 20.04 and ROS (melodic or noetic)
2. Python3: recommend to use 3.6 or upper
3. Pip: A library manager for python 
- Run ‘manual_control.py’ first and download each library (such as numpy, pygame, …) step-by-step.
4. Dependencies for tracker or heightmap (ROS packages): Eigen, Boost, OpenCV, PCL, CV_bridge
- Build these packages (tracker, heightmap) first before checking above libraries.
- If it gives you errors, then solve compatibility issues step-by-step.

## How to install

### 1. Lanuch the server

Ref: <https://carla.readthedocs.io/en/0.9.12/build_linux/>

1. Software requirements
```
$ sudo apt-get update &&
  sudo apt-get install wget software-properties-common &&
  sudo add-apt-repository ppa:ubuntu-toolchain-r/test &&
  wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add - &&
  sudo apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-8 main" &&
  sudo apt-get update
```

2. Ubuntu 20.04. (The following commands depend on your Ubuntu version. Make sure to choose accordingly.)

```
$ sudo apt-get install build-essential clang-8 lld-8 g++-7 cmake ninja-build libvulkan1 python python-pip python-dev python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git
```

3. All Ubuntu Systems (To avoid compatibility issues)
```
$ sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 180 &&
  sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 180
```

4. Run CARLA
```
$ ./CarlaUE4.sh
```

### 2. Lanuch the agent

Ref: <https://pypi.org/project/carla/0.9.12/#files>
1. Choose .whl file compatible with your python version and download it. 

2. Using pip, install carla at your download folder. Now, you can import carla in python.
```
$ pip3 install <wheel-file-name>.whl
```

3.  launch manual_control.py (from <https://github.com/kwlee365/carla_AVs>)
```
$ python3 manual_control.py
```

### 3. Lanuch the controller
1. Put the "tracker" and "height_map" into your ros workspace (usually, at ‘home/catkin_ws’)

2. Launch the controller
```
$ roscore
$ roslaunch tracker tracker.launch
```
