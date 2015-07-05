# brokenGlasses
Vision that works, but isn't perfect. 

A first attempt at vision processing using ROS and the OpenCV library.

Friendly script to set up everything (minus ROS):

#/bin/bash
clear
sudo apt-get -y install libgtk2.0-dev
sudo apt-get -y install git
sudo apt-get -y install cmake
sudo apt-get -y install qt5-default libvtk6-dev
sudo apt-get -y install zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev
sudo apt-get -y install libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev
sudo apt-get -y install libtbb-dev libeigen3-dev
sudo apt-get -y install python-dev python-tk python-numpy python3-dev python3-tk python3-numpy
sudo apt-get -y install ant default-jdk
sudo apt-get -y install doxygen sphinx-common texlive-latex-extra


cd ~
git clone https://github.com/Itseez/opencv
cd opencv
sudo mkdir Release
cd Release
sudo cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_GTK=ON -D WITH_OPENGL=ON ..
sudo make
sudo make install
cd ~
sudo ldconfig
