#!/usr/bin/env bash

# Install dependencies
sudo apt-get install libglew-dev -y
sudo apt-get install cmake -y

# Build Pangolin from source and install it
curr_dir=`pwd`
cd /tmp
git clone https://github.com/gaunthan/Pangolin
cd Pangolin
mkdir build
cd build
cmake ..
make
sudo make install
cd "$curr_dir"

# Install OpenCV python binding
sudo apt install python-opencv python3-opencv -y

# Install Eigen3
curr_dir=`pwd`
cd /tmp
wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
tar xzvf 3.3.4.tar.gz
cd eigen-eigen-*
mkdir build
cd build
cmake ..
sudo make install
