#!/bin/bash

if [ "$(docker ps -a | grep carla-server)" ]; then
    echo "restart carla-server..."
    docker restart carla-server
else
    echo "start carla-server..."
    docker run -d --name=carla-server --ipc=host --network=host --runtime=nvidia --gpus all -e SDL_VIDEODRIVER=offscreen --restart=always registry.gitlab.isus.tech/kevinlad/carla:0.9.10.1 bash CarlaUE4.sh
fi    

if [ "$(docker ps -a | grep carla-ros)" ]; then
    echo "stop carla-ros..."
    docker stop carla-ros && docker rm carla-ros
fi

sleep 2

echo "start carla-ros..."
docker run -it --rm --name=carla-ros --network=host -e DISPLAY --runtime=nvidia harbor.isus.tech/carla-ros/carla-ros:0.9.10.1-noetic bash /move_base.sh

