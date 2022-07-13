# Install and Run ORB_SLAM2 in ROS2

#### Make directory for cloning related packages
```
mkdir orb_slam_dependency
cd orb_slam_dependency  # 'opencv' and 'pangolin' folder will be cloned here.
```
#### Install OpenCV 3.x [[installation reference](https://www.notion.so/opencv-3-x-364951de38eb47d0a57cdba7996600a0)]
```
# Check installed opencv ('not found' should appear)
pkg-config --modversion opencv

# Download opencv
mkdir opencv
cd opencv
wget -O opencv-3.4.11.zip https://github.com/opencv/opencv/archive/3.4.11.zip
wget -O opencv_contrib-3.4.11.zip https://github.com/opencv/opencv_contrib/archive/3.4.11.zip
unzip opencv-3.4.11.zip
unzip opencv_contrib-3.4.11.zip

cd opencv-3.4.11/
mkdir build
cd build

# Cmake (unable GPU usage)
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D WITH_TBB=OFF \
-D WITH_IPP=OFF \
-D WITH_1394=OFF \
-D BUILD_WITH_DEBUG_INFO=OFF \
-D BUILD_DOCS=OFF \
-D INSTALL_C_EXAMPLES=ON \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D BUILD_EXAMPLES=OFF \
-D BUILD_TESTS=OFF \
-D BUILD_PERF_TESTS=OFF \
-D WITH_QT=ON \
-D WITH_CUDA=OFF \
-D BUILD_opencv_gpu=OFF \
-D WITH_OPENGL=ON \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.11/modules \
-D WITH_V4L=ON \
-D WITH_FFMPEG=ON \
-D WITH_XINE=ON \
-D WITH_NVCUVID=OFF \
-D BUILD_NEW_PYTHON_SUPPORT=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON ../

# Cmake (enable GPU usage)
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D WITH_TBB=OFF \
-D WITH_IPP=OFF \
-D WITH_1394=OFF \
-D BUILD_WITH_DEBUG_INFO=OFF \
-D BUILD_DOCS=OFF \
-D INSTALL_C_EXAMPLES=ON \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D BUILD_EXAMPLES=OFF \
-D BUILD_TESTS=OFF \
-D BUILD_PERF_TESTS=OFF \
-D WITH_QT=ON \
-D WITH_CUDA=ON \
-D WITH_OPENGL=ON \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.11/modules \
-D WITH_V4L=ON \
-D WITH_FFMPEG=ON \
-D WITH_XINE=ON \
-D WITH_NVCUVID=OFF \
-D BUILD_NEW_PYTHON_SUPPORT=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON ../

# Install
time make -j $(nproc)
sudo make install
sudo sh -c 'echo '/usr/local/lib' > /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig

# Check installed opencv ('3.4.11' should appear)
pkg-config --modversion opencv
```
#### Install Pangolin
```
sudo git clone https://github.com/stevenlovegrove/Pangolin pangolin
cd pangolin
mkdir build
cd build
cmake ..
make
sudo make install
```
#### Install ORB_slam2 (some installation errors are fixed from the [original repo](https://github.com/raulmur/ORB_SLAM2)) [[modified repo](https://github.com/awesomericky/ORB_SLAM2)] [[installation reference](https://www.notion.so/ORB-SLAM2-09ad4624eccb4478b23a9be8341348bc)]
```
git clone git@github.com:awesomericky/ORB_SLAM2.git
cd ORB_SLAM2
chmod +x build.sh
./build.sh

# If 'opencv Qt..' error occurs .. export the path and rebuild it.
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/:$LD_LIBRARY_PATH
rm -rf build
./build.sh
```

#### Install ROS2 wrapper of ORB_slam2 [[repo](https://github.com/awesomericky/ros2-ORB_SLAM2)]
```
mkdir -p orb_slam_ws/src
cd orb_slam_ws/src
git clone git@github.com:awesomericky/ros2-ORB_SLAM2.git
cd ..
export ORB_SLAM2_ROOT_DIR=/path/to/ORB_SLAM2
colcon build
```
#### Run ORB_slam2
```
# source 
source install/setup.zsh

# export ORB_SLAM2 library and its dependencies 
export LD_LIBRARY_PATH=/home/awesomericky/ROS_workspace/orb_slam_dependancy/pangolin/build:/home/awesomericky/ROS_workspace/ORB_SLAM2/Thirdparty/DBoW2/lib:/home/awesomericky/ROS_workspace/ORB_SLAM2/Thirdparty/g2o/lib:/home/awesomericky/ROS_workspace/ORB_SLAM2/lib:/usr/lib/x86_64-linux-gnu/:$LD_LIBRARY_PATH

# run orb2_slam
ros2 run ros2_orbslam mono ../ORB_SLAM2/Vocabulary/ORBvoc.txt src/ros2-ORB_SLAM2/src/rgbd/d435i.yaml  # mono slam
ros2 run ros2_orbslam rgbd ../ORB_SLAM2/Vocabulary/ORBvoc.txt src/ros2-ORB_SLAM2/src/rgbd/d435i.yaml  # rgbd slam
ros2 topic echo /orb_pose  # view estimated pose
```
#### Install and run sensors [[repo](https://github.com/awesomericky/raibo-smd_ros)]
```
source /opt/ros/foxy/setup.zsh
source ../realsense_ws/install/local_setup.zsh  (realsense-ros from source)
source install/local_setup.zsh  (main workspace with sensor streaming)
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/:$LD_LIBRARY_PATH  (need in local desktop to make rviz2 available)
ros2 launch raibo-smd_ros head_launch.py
```
----------------------------------------------------------------------------------------------

# ros2-ORB_SLAM2
ROS2 node wrapping the ORB_SLAM2 library

If you want to integrate ORB_SLAM2 inside your ROS2 system, consider trying [this](https://github.com/alsora/ORB_SLAM2) fork of ORB_SLAM2 library which drops Pangolin dependency and streams all SLAM data through ROS2 topics.

### Requirements

 - [ROS2 Foxy](https://github.com/ros2/ros2/wiki/Installation)
 - [Pangolin](https://github.com/stevenlovegrove/Pangolin)
 - [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
 - [OpenCV3](https://docs.opencv.org/3.0-beta/doc/tutorials/introduction/linux_install/linux_install.html)
 - [vision_opencv](https://github.com/ros-perception/vision_opencv/tree/ros2)
 - [message_filters](https://github.com/ros2/message_filters)

Note: The `vision_opencv` package requires OpenCV3. Make sure to build ORB_SLAM2 with the same OpenCV version otherwise strange run errors could appear.

The `message_filters` package is not required if you want to use only the Monocular SLAM node. 


### Build

This repository contains a Dockerfile providing an Ubuntu environment with this package and all its dependencies already installed.
In order to use it:

    $ cd docker
    $ bash build.sh
    $ bash run.sh

Otherwise you can build the package on your system.
If you built ORB_SLAM2 following the instructions provided in its repository, you will have to tell CMake where to find it by exporting an environment variable that points to the cloned repository (as the library and include files will be in there).

    $ export ORB_SLAM2_ROOT_DIR=/path/to/ORB_SLAM2

Then you can build this package

    $ mkdir -p ws/src
    $ cd ws/src
    $ git clone https://github.com/alsora/ros2-ORB_SLAM2
    $ cd ..
    $ colcon build

### Usage

First source the workspace

    $ source ws/install/setup.sh

Then add to the LD_LIBRARY_PATH the location of ORB_SLAM2 library and its dependencies (the following paths may be different on your machine)

    $ export LD_LIBRARY_PATH=~/Pangolin/build/src/:~/ORB_SLAM2/Thirdparty/DBoW2/lib:~/ORB_SLAM2/Thirdparty/g2o/lib:~/ORB_SLAM2/lib:$LD_LIBRARY_PATH

Run the monocular SLAM node

    $ ros2 run ros2_orbslam mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE

You can find the vocabulary file in the ORB_SLAM2 repository (e.g. `ORB_SLAM2/Vocabulary/ORBvoc.txt`), while the config file can be found within this repo (e.g. `ros2-ORB_SLAM2/src/monocular/TUM1.yaml` for monocular SLAM).

This node subscribes to the ROS2 topic `camera` and waits for Image messages.
For example you can stream frames from your laptop webcam using:

    $ ros2 run image_tools cam2image -t camera

The other nodes can be run in a very similar way.

You can run the `rgbd` node by using 

    $ ros2 run ros2_orbslam rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE

You can run the `stereo` node by using 

    $ ros2 run ros2_orbslam stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
