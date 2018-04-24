# Installation scripts for production environment

These scripts can be used to set up a fully functional production environment
for the project based on a fresh **Ubuntu 16.04** installation. This can either be used
to set up a local machine or on a remote server. Running on a remote server this has the advantage
of distributing the computational load. The simulator can be run on the client while everything ros related and especially
the detection node runs on the server.

## How to use the installer
1. Download cudnn v6 and place it in this directory. Make sure it is the correct version with name `cudnn-8.0-linux-x64-v6.0.tar`
2. Download the inference graph protobuf and put it in this directory. `frozen_inference_graph.pb`
3. Run `bash installer_1.sh`. The script will install the NVIDIA driver and reboot your system afterwards
4. Once the system has rebooted, run the second script: `bash -i installer_2.sh`. This will install CUDA, cudnn, anaconda, websockets and 
ROS. Finally it will run a tensorflow session to test if everything works.

## Connect the simulator via ssh to a remote host
1. ssh into remote and start the ROS project
2. Open another terminal and forward the simulator port from your machine to the remote `ssh -N -L 4567:127.0.1.1:4567 remote` where remote is the ip
of the remote server