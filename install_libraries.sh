#!/bin/bash

# 更新并安装一些常见依赖
sudo apt-get update
sudo apt-get install -y build-essential cmake git

# 克隆并安装第三方库（librealsense 2.50.0）
cd ~
mkdir librealsense
cd librealsense
git clone --branch v2.50.0 --single-branch https://github.com/IntelRealSense/librealsense.git

cd librealsense
sudo apt-get update
sudo apt-get install -y cmake libusb-1.0-0-dev pkg-config
sudo apt-get install -y libssl-dev libcurl4-openssl-dev
mkdir build && cd build
cmake ..

make
make install


# 其他需要的安装步骤可以继续添加
