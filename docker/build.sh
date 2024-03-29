#!/bin/bash

docker pull harbor.isus.tech/osrf/ros:noetic-desktop
docker pull harbor.isus.tech/carlasim/carla:0.9.12
docker build --network=host -t harbor.isus.tech/carla-ros/carla-ros:0.9.12 -f Dockerfile ./..
