# ninjacar_mppi

The tools contained within this repository have been used to implement an [MPPI](https://ieeexplore.ieee.org/document/7487277) controller based off of GeorgiaTech's [AutoRally](https://github.com/AutoRally/autorally) implementation.

A [Notion website](https://www.notion.so/MPPI-Code-Walkthrough-d6f159b261fa48db9d41d51e2ba90466) was developed to gain a deeper understanding on how the algorithm is structured in CUDA language.

![](https://github.com/JaumeAlbardaner/ninjacar_mppi/blob/master/gif/ninjacar.gif)
## Setup Instructions

### Contents

1. [Install Prerequisites](#1-install-prerequisites)
2. [Clone repository](#2-clone-or-fork-repositories)
3. [Run MPPI controller](#3-run-mppi-controller)
4. [Sidenotes](#4-sidenotes)

### 1. Install Prerequisites

1. __Install [Ubuntu 18.04 64-bit](http://www.ubuntu.com)__

2. __Install required packages__

   ```sudo apt install git doxygen openssh-server libusb-dev texinfo```

3. __Install [ros-melodic-desktop-full](https://wiki.ros.org/melodic/Installation/Ubuntu)__

4. __Install Dependencies__
    The core idea behind MPPI is to sample thousands of trajectories really fast. This is accomplished by implementing the sampling step on a GPU, for which you will need CUDA. We also use an external library to load python's numpy zip archives (.npz files) into C++.
    * [Install CUDA](https://developer.nvidia.com/cuda-downloads)

    * __Install CNPY__
        ```
        mkdir -p ~/tmp ; cd ~/tmp
        git clone git@github.com:rogersce/cnpy.git
        cd cnpy && mkdir build ; cd build && cmake ..
        make
        sudo make install
        ```

    * __Install gtsam__

        ```
        mkdir -p ~/tmp ; cd ~/tmp
        git clone git@github.com:borglab/gtsam.git
        cd gtsam && mkdir build && cd build
        cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF .. 
        make
        sudo make install
        ```


### 2. Clone or Fork Repositories
Get the autorally repository in a [catkin workspace](http://wiki.ros.org/catkin/workspaces). The suggested location is  `~/catkin_ws/src/ `, but any valid catkin worskspace source folder will work. 

1. __Create workspace if non-existent and switch into it__
    ```
    mkdir ~/catkin_ws/src/ ; cd ~/catkin_ws/src/
    ```
2. __Clone repository and its dependencies__
    ```
    git clone git@github.com:arpg/ninja_car_driving_control.git --recurse-submodules
    git clone git@github.com:JaumeAlbardaner/ninjacar_mppi.git
    ```
3. __Set variables in ~/.bashrc__

    Your `~/.bashrc` should include these lines (adapted for your PC):
    ```
    export CUDA_HOME=/usr/local/cuda
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64
    export PATH=$PATH:$CUDA_HOME/bin

    source /opt/ros/melodic/setup.bash
    source /home/yauma/catkin_ws/devel/setup.bash

    export AR_MPPI_PARAMS_PATH=`rospack find autorally_control`/src/path_integral/params/
    ```

4. __Install AutoRally ROS Dependencies__
    
    Within the catkin workspace folder, run this command to install the packages this project depends on.
    ```
    cd ~/catkin_ws/ && rosdep install --from-path src --ignore-src -y
    ```
5. __Build ninjacar_mppi__
    
    It is recommended that you use `catkin build`:
    ```
    sudo apt-get install python-catkin-tools
    ```
    ```
    cd ~/catkin_ws/ && catkin build autorrally_control
    ```
    If problems arise building `drive_control`, contact @mmiles19 . If the problem is related to a missing library, you may try to comment lines 105-144 of the [CMakeLists.txt](https://github.com/arpg/ninja_car_driving_control/blob/81c7bed59b76d6b244e06053628c601160752b8e/ros_pololu_servo/CMakeLists.txt#L105-L144) file within the package ros_polulu_servo, building again, descommenting them, and building again. The issue should subside.

6. __Compilation & Running__

    First, check your Eigen version with 
    ```
    pkg-config --modversion eigen3
    ```
    If you don't have at least version 3.3.5, update it:
    ```
    mkdir -p ~/tmp ; cd ~/tmp
    git clone git@github.com:eigenteam/eigen-git-mirror.git
    cd eigen-git-mirror && mkdir build && cd build
    cmake .. 
    sudo make install
    ```

### 3. Run MPPI controller

1. __Change  into the workspace__
    ```
    cd ~/catkin_ws/
    ```
2. __Source into the workspace's environment__
    ```
    source devel/setup.bash
    ```
3. __Connect to the VRPN server (if in the lab, simulation still works otherwise)__
    ```
    roslaunch vrpn_client_ros sample.launch server:=tracker
    ```
4. __Run the MPPI controller__
    
    If the neural network model wants to be launched:
    ```
    roslaunch autorally_control path_integral_nn.launch
    ```

    If the base functions model wants to be launched instead:
    ```
    roslaunch autorally_control path_integral_bf.launch
    ```

### 4. Sidenotes

1.  All of the variables set in the launch files are described in the [MPPI Wiki](https://github.com/AutoRally/autorally/wiki/Model-Predictive-Path-Integral-Controller-(MPPI)).

2.  Issues with Eigen occurred during compilation, hence why in some [CMakeFiles](https://github.com/JaumeAlbardaner/ninjacar_mppi/blob/d24dc287ba3fdbd5cfb960ab8881d0b1f0fb6c35/autorally_control/CMakeLists.txt#L21-L22), a different c++ compiler is used.

3.  The republisher called in both launchers: [command_republisher.py](https://github.com/JaumeAlbardaner/ninjacar_mppi/blob/master/autorally_control/src/path_integral/scripts/command_republisher.py), is a patch made to regulate that the output of MPPI actually is within a range. Otherwise it would still output values outside the _min_ and _max_ values set in the `.launch` file.

4.  When the `path_integral_bf.launch` is called, the variable _use_feedback_gains_ __must__ be set to false. The DDP algorithm does not work properly when the pose messages are not given consistently, which does not hold true in the motion capture space. 

5. The republisher [command_rosbag.py](https://github.com/JaumeAlbardaner/ninjacar_mppi/blob/master/autorally_control/src/path_integral/scripts/command_rosbag.py) is to be called every time one wants to capture a rosbag to train a neural network using [mppi_trainer](https://github.com/JaumeAlbardaner/mppi_trainer).

6. The costMap building tool is located in the [ninjacar_map](https://github.com/JaumeAlbardaner/ninjacar_map) repository.