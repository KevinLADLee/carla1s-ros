#!/bin/bash

docker build --network=host -t harbor.isus.tech/carla-ros/carla-ros:0.9.11 -f Dockerfile ./..
