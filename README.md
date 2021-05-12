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

## 整体设计与开发情况

![carla-ros-bridge.png](/carla-ros-bridge.png)
