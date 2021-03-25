#!/bin/bash

TAG=$1
docker build -t harbor.isus.tech/carla-ros/carla-ros:$TAG -f Dockerfile ./..
