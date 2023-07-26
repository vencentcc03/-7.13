#!/bin/bash

for ((i=0; i<=$1; i++))
do
  roslaunch demo01_gazebo spwan_car.launch model:=$i
done
