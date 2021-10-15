# carla1s-ros

本仓库主要结合CARLA ROS-Bridge以及主流的Navigation相关算法，提供一整套完整的自动驾驶导航算法验证平台。

## 硬件需求

* Intel i7-10700kf及更高单核性能CPU 
* 16G+ RAM
* Nvidia Geforce RTX2060及更高级别GPU
* 开发者测试环境： Intel i7-10700kf / 32G DDR4 RAM / RTX3070

## 环境需求

* Ubuntu 20.04 (其他版本建议使用Docker)
* Docker 19.03及以上
* ROS Noetic
* CARLA 0.9.12 (carla1s:sustech-main)
* [Nvidia-Docker2](https://github.com/NVIDIA/nvidia-docker)

## 支持模块

- [x] Fake Localization 
- [x] Behevior Tree Based Decision
- [x] Route Planner
- [ ] Path Planner
- [x] Path Tracking

## 本地编译

### 安装ROS

http://wiki.ros.org/cn/noetic/Installation

### 安装系统依赖

```bash
sudo apt install -y \
    libpng16-16 \
    ros-noetic-tf \
    ros-noetic-derived-object-msgs \
    ros-noetic-cv-bridge \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-map-server \
    ros-noetic-ackermann-msgs \
    ros-noetic-derived-object-msgs \
    ros-noetic-behaviortree-cpp-v3 \
    vim git wget curl \
    ninja-build \
    libomp-dev
    
pip3 --no-cache-dir install --upgrade networkx distro pygame simple-pid numpy==1.18.4 transforms3d pep8 autopep8 cmake_format==0.6.11 pylint pexpect scipy empy catkin_pkg netifaces defusedxml
```

### 下载预编译版本Carla

```bash

# 下载并解压CARLA 0.9.12 原始版本
wget -c https://mirrors.sustech.edu.cn/carla/carla/0.9.12/CARLA_0.9.12.tar.gz -O CARLA.tar.gz
mkdir -p ~/carla1s/carla
tar -xzvf CARLA.tar.gz -c ~/carla1s/carla

# 配置环境变量(写入 ~/.bashrc 或 ~/.zshrc)
export CARLA_ROOT=/home/YOUR_USERNAME/carla1s/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla/
# 其中[PATH_TO_CARLA]代表carla服务端（CarlaUE4.sh）所在目录
# 例如~/carla
# [CARLA_PYTHON_EGG_FILENAME]是egg文件的文件名，于文件夹./PythonAPI/carla/dist/
```

### 测试carla1s-ros

#### 下载并编译carla1s-ros
```bash

mkdir -p ~/carla1s/ros1_ws/src
cd ~/carla1s/ros1_ws/src
git clone --recurse-submodules git@gitlab.isus.tech:carla1s/ros-agent/carla1s-ros.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release --use-ninja
source devel/setup.bash
#source devel/setup.zsh
```

#### 运行carla1s-ros

```bash
#运行Carla服务端
cd $CARLA_ROOT
./CarlaUE4.sh -vulkan -RenderOffscreen

#测试代码
# For bash
source ~/carla1s/ros1_ws/devel/setup.bash
# For zsh
# source ~/carla1s/ros1_ws/devel/setup.zsh

roslaunch carla1s_bringup carla_example_ego_vehicle.launch town:=Town02
#roslaunch carla1s_bringup carla_example_ego_vehicle.launch town:=ParkingLot

# A->B导航实例
roslaunch carla1s_decision carla_decision_test.launch

# A->B 感知+导航实例
roslaunch carla1s_decision carla_decision_test_with_fake_perception.launch
```

