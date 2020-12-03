# Docker版 Carla-ROS Bridge使用指南

依赖： 

* [docker](https://docs.docker.com/engine/install/ubuntu/)
* [nvidia-docker2](https://github.com/NVIDIA/nvidia-docker) 

```bash
# 拉取最新源码
git clone https://gitlab.isus.tech/kevinlad/carla-ros-bridge

cd carla-ros-bridge/docker

# 如果之前使用过此demo，建议重新清除本地的旧版本docker image
./clean_docker_image.sh

# 启动A-LOAM算法演示
./run_carla_aloam.sh

# 启动Costmap演示
./run_carla_move_base.sh
```