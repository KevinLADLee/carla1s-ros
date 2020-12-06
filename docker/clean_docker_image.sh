#!/bin/bash

if [ "$(docker ps -a | grep carla-ros)" ]; then
    echo "stop carla-ros..."
    docker stop carla-ros && docker rm carla-ros
fi

docker image rm harbor.isus.tech/carla-ros/carla-ros:0.9.10.1-noetic

if [ "$(docker ps -a | grep carla-server)" ]; then
    echo "stop carla-server..."
    docker stop carla-server && docker rm carla-server
fi

docker image rm harbor.isus.tech/carlasim/carla:0.9.10.1
