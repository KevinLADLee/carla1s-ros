# carla1s-ros

本仓库主要结合CARLA ROS-Bridge以及主流的Navigation相关算法，集成打包方便用于各模块算法的验证测试。

## 硬件需求

* Intel i7-10700kf及更高单核性能CPU 
* 32G+ RAM
* Nvidia Geforce RTX2060及更高级别GPU
* 开发者测试环境： Intel i7-10700kf / 32G DDR4 RAM / RTX3070

## 环境需求

* Ubuntu 20.04 (其他版本建议使用Docker)
* Docker 19.03及以上
* ROS Noetic
* CARLA 0.9.12 (carla1s:sustech-main)
* [Nvidia-Docker2](https://github.com/NVIDIA/nvidia-docker)

## 支持模块

- [x] SLAM (SC-Lego-LOAM)
- [x] Fake Localization 
- [x] Behevior Tree Based Decision
- [x] Path Planner
- [ ] Trajectory Planner
- [x] Path Tracking (Pure-Pursuit / PID)

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

### 下载源码编译并测试

从cdn上下载carla源码：[CARLA_cdn_url](http://cdn.isus.tech/cdn/carla/carla/)

解压源码至文件夹

```bash
tar -xzvf CARLA_0.9.12.tar.gz -C ~/carla
```

下载并编译carla1s-ros

```bash
#下载并编译carla1s-ros
mkdir -p carla_ws/src
cd carla_ws/src
git clone --recurse-submodules https://gitlab.isus.tech/carla1s/carla1s-ros.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release --use-ninja
```

添加环境变量：

```bash
gedit ~/.bashrc
# 需添加以下环境变量至bashrc或zshrc中
# 其中[PATH_TO_CARLA]代表carla服务端（CarlaUE4.sh）所在目录
# 例如~/carla
# [CARLA_PYTHON_EGG_FILENAME]是egg文件的文件名，于文件夹./PythonAPI/carla/dist/
# 例如 carla-0.9.12-py3.7-linux-x86_64.egg
export CARLA_ROOT=[PATH_TO_CARLA]
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/[CARLA_PYTHON_EGG_FILENAME]:$CARLA_ROOT/PythonAPI/carla/
source devel/setup.bash
#source devel/setup.zsh
# 例如：
# export CARLA_ROOT=~/carla
# export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla/
# source /home/tongda/workspace/carla_ws/devel/setup.bash
source ~/.bashrc
```

运行carla服务器

```bash
#运行Carla服务端
cd $CARLA_ROOT
./CarlaUE4.sh -vulkan -RenderOffscreen

#测试代码
roslaunch carla_2d_nav carla_example_ego_vehicle.launch town:=Town02
#roslaunch carla_2d_nav carla_example_ego_vehicle.launch town:=ParkingLot
roslaunch carla_decision carla_decision_test.launch
```

