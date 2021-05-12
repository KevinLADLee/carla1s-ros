# CARLA-ROS 集成包

本仓库主要结合CARLA ROS-Bridge以及主流的Navigation相关算法，集成打包方便用于各模块算法的验证测试。

## 硬件需求

* Intel i7-9700k及更高单核性能CPU 
* 32G+ RAM
* Nvidia 2060及更高级别GPU

## 环境需求

* Ubuntu 20.04 (其他版本建议使用Docker)
* Docker 19.03及以上
* ROS Noetic
* CARLA 0.9.11
* [Nvidia-Docker2](https://github.com/NVIDIA/nvidia-docker)

## 支持模块

- [x] SLAM (SC-Lego-LOAM)
- [x] Fake Localization 
- [ ] Behevior Tree Based Decision
- [ ] Path Planner
- [ ] Trajectory Planner
- [x] Path Tracking (Pure-Pursuit)
- [x] Ackermann Controller

## 使用Docker快速测试


启动`xserver`访问权限
```bash
xhost +
```

启动CARLA服务端

```bash
docker run -d --name=carla-server --ipc=host --network=host --runtime=nvidia --gpus all -e SDL_VIDEODRIVER=offscreen --restart=always harbor.isus.tech/carlasim/carla:0.9.11 bash CarlaUE4.sh
```

启动CARLA ROS
```
 docker run -it --rm --name=carla-ros --network=host -e DISPLAY --runtime=nvidia harbor.isus.tech/carla-ros/carla-ros:0.9.11 bash
```

## 本地编译

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
    ninja-build
    
pip3 --no-cache-dir install --upgrade \
        networkx distro pygame simple-pid numpy==1.18.4 transforms3d    
```

### 安装GTSAM (optional)
此依赖为`carla-sc-lego-loam`使用，如不需要，请在`carla-sc-lego-loam`文件夹下添加`CATKIN_IGNORE`文件。

```bash
curl -o gtsam.tar.gz -LJ https://github.com/borglab/gtsam/archive/refs/tags/4.0.3.tar.gz
tar -zxf gtsam.tar.gz
cd gtsam-4.0.3
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
sudo make install
```

### 下载源码编译并测试

```bash
mkdir -p carla_ws/src
cd carla_ws/src
git clone --recurse-submodules https://gitlab.isus.tech/kevinlad/carla-ros-bridge.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release --use-ninja

source devel/setup.zsh

roslaunch carla_2d_nav carla_example_ego_vehicle.launch town:=Town02
roslaunch carla_decision carla_decision_test.launch
```

## 整体设计与开发情况

![carla-ros-bridge.png](/carla-ros-bridge.png)
