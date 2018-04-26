#!/bin/bash

# after the driver has been installed, this script continues to 
# install anaconda, cuda, ros and websockets

# do not run as sudo!
# run with bash -i option
INSTALL_DIR=$(pwd)

# install cuda
# tf 1.3 docs: https://www.tensorflow.org/versions/r1.3/install/install_linux
sudo apt-get install -y libcupti-dev

wget -O cuda8.deb https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda-repo-ubuntu1604-8-0-local-ga2_8.0.61-1_amd64-deb

sudo dpkg -i cuda8.deb
sudo apt -qq update
sudo apt install -y cuda-8-0

# add cuda to path
echo "export PATH=/usr/local/cuda-8.0/bin${PATH:+:${PATH}}" >> ~/.bashrc


# install cudnn
tar -xvf cudnn-8.0-linux-x64-v6.0.tar 
sudo cp cuda/include/cudnn.h /usr/local/cuda/include
sudo cp cuda/lib64/* /usr/local/cuda/lib64/
sudo chmod a+r /usr/local/cuda/lib64/libcudnn*

# install anaconda
CONDAFILE=Anaconda3-5.1.0-Linux-x86_64.sh
curl -O https://repo.continuum.io/archive/$CONDAFILE
# b and p are options for silent mode in anaconda's installer
# https://conda.io/docs/user-guide/install/macos.html#install-macos-silent
bash ./$CONDAFILE -b -p ~/anaconda3
echo "export PATH=~/anaconda3/bin:$PATH" >> ~/.bashrc
rm $CONDAFILE

source ~/.bashrc


# create conda env and alias to activate rospy env
if conda create -n rospy python=2.7
then
    echo "alias rpy='source activate rospy'" >> ~/.bashrc

    REPO_TARGET=~/Desktop/destiny_awaits

    # clone repo and set up env
    git clone https://github.com/wAuner/destiny_awaits.git $REPO_TARGET
     
    # copy inference graph into project directory
    cp $INSTALL_DIR/frozen_inference_graph.pb $REPO_TARGET/ros/src/tl_detector/light_classification/frozen_inference_graph.pb

    if source activate rospy 
    then
        pip install -r $REPO_TARGET/requirements.txt
        pip install catkin_pkg empy wstool rosinstall
        source deactivate
    fi

else
    echo "Error while creating conda env."
    exit 1
fi



# install ros and dbw
# http://wiki.ros.org/kinetic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt -qq update
sudo apt-get install -y ros-kinetic-desktop-full
sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# installs in system python, maybe need to install in conda env
#sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# install dbw
bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_mkz_ros/raw/default/dbw_mkz/scripts/sdk_install.bash)


# install websockets
bash $INSTALL_DIR/websockets_install.sh

# Finish installation
echo "Installation process complete."
echo "Testing tensorflow-gpu:"

source ~/.bashrc
rpy
python $INSTALL_DIR/tf_test.py

