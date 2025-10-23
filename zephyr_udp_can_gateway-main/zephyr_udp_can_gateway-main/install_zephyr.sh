#! /usr/bin/bash

# Copyright (c) 2025 Robert Bosch GmbH.
#
# This program and the accompanying materials are made available under the
# terms of the MIT License which is available at
# https://opensource.org/licenses/MIT.
# 
# SPDX-License-Identifier: MIT

sudo apt update -y
sudo apt upgrade -y
sudo apt install --no-install-recommends git cmake ninja-build gperf   ccache dfu-util device-tree-compiler wget   python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file   make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1
sudo apt install python3-venv
python3 -m venv ~/zephyrproject/.venv
source ~/zephyrproject/.venv/bin/activate

pip install west
west init ~/zephyrproject
cd ~/zephyrproject
west update
west zephyr-export
pip install -r ~/zephyrproject/zephyr/scripts/requirements.txt
cd ~
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_linux-x86_64.tar.xz
tar xvf zephyr-sdk-0.16.8_linux-x86_64.tar.xz
cd zephyr-sdk-0.16.8/
./setup.sh
sudo cp sysroots/x86_64-pokysdk-linux/usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d
sudo usermod -a -G plugdev $USER
sudo udevadm control --reload-rules && sudo udevadm trigger 
sudo apt-get install minicom
mkdir ~/zephyrproject/projects