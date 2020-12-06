#!/bin/bash

docker pull harbor.isus.tech/carla-ros/orbslam3:latest

if [ "$(docker ps -a | grep carla-ros)" ]; then
    echo "stop carla-ros..."
    docker stop carla-ros && docker rm carla-ros
fi

docker pull harbor.isus.tech/carla-ros/carla-ros:0.9.10.1-noetic

if [ "$(docker ps -a | grep carla-server)" ]; then
    echo "stop carla-server..."
    docker stop carla-server && docker rm carla-server
fi

docker pull harbor.isus.tech/carlasim/carla:0.9.10.1
