# ninjacar_mppi

The tools contained within this repository have been used to implement an [MPPI](https://ieeexplore.ieee.org/document/7487277) controller based off of GeorgiaTech's [AutoRally](https://github.com/AutoRally/autorally) implementation.

## Setup Instructions

### Contents

1. [Install Prerequisites](#1-install-prerequisites)
2. [Clone repository](#2-clone-or-fork-repositories)
3. [Run MPPI controller](#3-run-mppi-controller)

### 1. Install Prerequisites

1. __Install [Ubuntu 18.04 64-bit](http://www.ubuntu.com)__

2. __Install required packages__

   ```sudo apt install git doxygen openssh-server libusb-dev texinfo```

3. __[Install](http://www.ros.org/install/) ros-melodic-desktop-full__

4. __Install MPPI Dependencies (if you have a GPU and will run MPPI)__
    The core idea behind MPPI is to sample thousands of trajectories really fast. This is accomplished by implementing the sampling step on a GPU, for which you will need CUDA. We also use an external library to load python's numpy zip archives (.npz files) into C++.
    * [Install CUDA](https://developer.nvidia.com/cuda-downloads)
    * [Install CNPY](https://github.com/rogersce/cnpy)

5. __Install gtsam__
   Follow the gtsam [Quick Start](https://bitbucket.org/gtborg/gtsam/) guide to clone and install the _develop_ branch of gtsam.
   Instead of `cmake ..`, use:
   ```cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF ..```
   Once install is complete, make sure linux can see the shared library:
   ```sudo ldconfig```

### 2. Clone or Fork Repositories
Get the autorally repository in a [catkin workspace](http://wiki.ros.org/catkin/workspaces). The suggested location is  `~/catkin_ws/src/ `, but any valid catkin worskspace source folder will work. 

1. __Create workspace if non-existent and switch into it__
    ```
    mkdir ~/catkin_ws/src/ ; cd ~/catkin_ws/src/
    ```
2. __Clone repository__
    ```
    git clone git@github.com:JaumeAlbardaner/ninjacar_mppi.git
    ```
3. __Install AutoRally ROS Dependencies__
    
    Within the catkin workspace folder, run this command to install the packages this project depends on.
    ```
    cd ~/catkin_ws/ && rosdep install --from-path src --ignore-src -y
    ```
4. __Build ninjacar_mppi__
    ```
    cd ~/catkin_ws/ && catkin build mppi_autorrally_control
    ```

5. __Compilation & Running__

    First, check your Eigen version with 
    ```
    pkg-config --modversion eigen3
    ```
    If you don't have at least version 3.3.5, [upgrade Eigen](https://github.com/eigenteam/eigen-git-mirror).

### 3. Run MPPI controller

1. __Change  into the workspace__
    ```
    cd ~/catkin_ws/
    ```
2. __Source into the workspace's environment__
    ```
    source devel/setup.bash
    ```
3. __Instantiate a required variable__
    ```
    export AR_MPPI_PARAMS_PATH=`rospack find mppi_autorally_control`/src/path_integral/params/
    ```
4. __Connect to the VRPN server__
    ```
    roslaunch vrpn_client_ros sample.launch server:=tracker
    ```
5. __Run the MPPI controller__
    ```
    roslaunch mppi_autorally_control path_integral_nn.launch
    ```