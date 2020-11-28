# Docker版 Carla-ROS Bridge使用指南

## 本机启动Carla服务器

```bash
docker run -d --name=carla-server --ipc=host --network=host --runtime=nvidia --gpus all -e SDL_VIDEODRIVER=offscreen --restart=always registry.gitlab.isus.tech/kevinlad/carla:0.9.10.1 bash CarlaUE4.sh
```

## 本机启动ROS-Bridge

```bash
docker run -it --rm --network=host harbor.isus.tech/carla-ros/carla-ros:0.9.10.1-noetic bash

roslaunch carla_ego_vehicle carla_example_ego_vehicle.launch
```