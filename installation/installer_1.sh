#!/bin/bash

# this script install tools like compilers and git as well as the nvidia driver
# and restarts the machine afterwards

# install tools
sudo apt -qq update && sudo apt -qq upgrade -y
sudo apt install -y gcc g++ cmake make git wget curl openssh-server

# install nvidia driver
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt-get -qq update

sudo apt install -y nvidia-384

# restart after driver install is required
sudo reboot

