# Loading from nvidia-opengl for visualization capability
FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04

# install packages
RUN apt-get update && apt-get install -q -y  \
  dirmngr \
  gnupg2 \
  lsb-release \
  && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install ros packages
ENV ROS_DISTRO kinetic

RUN apt-get update && apt-get install -y  \
  ros-kinetic-ros-core=1.3.2-0* \
  && rm -rf /var/lib/apt/lists/*

# Install packages for Baxter base  
RUN apt-get update && apt-get install -y  \
  git-core \
  python-argparse \
  python-wstool \
  python-vcstools \
  python-rosdep \
  ros-kinetic-control-msgs \
  vim

# Install packages for gazebo
RUN apt-get update && apt-get install -y  \
  gazebo7 \
  ros-kinetic-qt-build \
  ros-kinetic-gazebo-ros-control \
  ros-kinetic-gazebo-ros-pkgs \
  ros-kinetic-ros-control \
  ros-kinetic-control-toolbox \
  ros-kinetic-realtime-tools \
  ros-kinetic-ros-controllers \
  ros-kinetic-xacro \
  ros-kinetic-tf-conversions \
  ros-kinetic-kdl-parser \
  && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt-get update && apt-get install -y  \
  python-pip \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y  \
  ros-kinetic-robot-state-publisher \
  && rm -rf /var/lib/apt/lists/*

# Install Catkin
RUN  pip install rospkg
RUN  pip install -U catkin_tools

# bootstrap rosdep
RUN rosdep init \
  && rosdep update

RUN apt-get update && apt-get install -y  \
  ros-kinetic-gazebo-ros-pkgs \
  ros-kinetic-gazebo-ros-control \
&& rm -rf /var/lib/apt/lists/*

# taken from moveit source install Dockerfile (https://github.com/ros-planning/moveit/blob/kinetic-devel/.docker/source/Dockerfile)
# Replacing shell with bash for later docker build commands
RUN mv /bin/sh /bin/sh-old && \
  ln -s /bin/bash /bin/sh

# create ROS ENV
ENV CATKIN_WS=/root/catkin_ws
RUN source /opt/ros/kinetic/setup.bash
RUN mkdir -p $CATKIN_WS/src
WORKDIR ${CATKIN_WS} 
RUN catkin init
RUN catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release 
WORKDIR $CATKIN_WS/src

# Download the baxter core files
RUN wstool init . \
  && wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall \
  && wstool update

WORKDIR $CATKIN_WS
ENV PYTHONIOENCODING UTF-8
RUN catkin build
RUN source devel/setup.bash

WORKDIR ${CATKIN_WS}/src

# Download the simulation files
RUN git clone -b kinetic-devel https://github.com/RethinkRobotics/baxter_simulator.git # get the regular simulator

WORKDIR $CATKIN_WS
RUN catkin build
RUN source devel/setup.bash

WORKDIR ${CATKIN_WS}/src

# Download moveit source
# RUN wstool init . && \
RUN wstool merge https://raw.githubusercontent.com/ros-planning/moveit/${ROS_DISTRO}-devel/moveit.rosinstall && \
  # RUN wstool merge https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall && \
  # RUN wstool merge https://raw.githubusercontent.com/ros-planning/moveit/melodic-devel/moveit.rosinstall && \
  wstool update

# Update apt-get because osrf image clears this cache. download deps
# Note that because we're building on top of kinetic-ci, there should not be any deps installed
# unless something has changed in the source code since the other container was made
# (they are triggered together so should only be one-build out of sync)
RUN apt-get -qq update && \
  apt-get -qq install -y  \
  wget && \
  rosdep update && \
  rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
  rm -rf /var/lib/apt/lists/*

# Build repo
WORKDIR $CATKIN_WS
RUN catkin build  
RUN source devel/setup.bash

WORKDIR ${CATKIN_WS}/src

# Download and install Baxter MoveIt
RUN git clone https://github.com/ros-planning/moveit_robots.git

# Download the baxter_moveit_experiments repository which has some of the key launch file capability, 
# and copy in local version of python script which has some necessary changes for MPNet experiment to run
RUN git clone https://github.com/anthonysimeonov/baxter_moveit_experiments.git
COPY ./motion_planning_data_gen.py ${CATKIN_WS}/src/baxter_moveit_experiments/

# Python package dependencies
RUN pip install --upgrade setuptools
RUN pip install matplotlib==2.0.2 pyassimp==4.1.3
RUN pip install cython
RUN pip install quadprog nltk pypcd
RUN pip install numpy --upgrade

# Build workspace now with MoveIt
WORKDIR ${CATKIN_WS}
RUN catkin build
RUN source devel/setup.bash

# remove OMPL debians to prepare for OMPL source installation
RUN apt-get remove ros-kinetic-ompl -y
RUN apt-get purge *ompl* -y

# OMPL source install
RUN mkdir ${CATKIN_WS}/src/ompl
WORKDIR ${CATKIN_WS}/src/ompl
RUN wget https://raw.githubusercontent.com/ros-gbp/ompl-release/debian/kinetic/xenial/ompl/package.xml
RUN git init

# Checkout our lab's fork of OMPL
RUN git remote add -t 1.4.2 -f origin https://github.com/anthonysimeonov/ompl.git
RUN git checkout 1.4.2

# Copy in the CMakeLists file for ompl_interface that reorders include paths (necessary to build OMPL from source)
ENV OMPL_INTERFACE moveit/moveit_planners/ompl
COPY ./moveit_cmake/CMakeLists.txt ${CATKIN_WS}/src/${OMPL_INTERFACE}/

WORKDIR ${CATKIN_WS}
RUN catkin clean -y
RUN catkin build
RUN source devel/setup.bash

# Copy in the moveit and baxter_moveit source code to register a new planner, and to turn off simplify solutions/interpolation
COPY ./moveit_mpnet/ompl_planning.yaml ${CATKIN_WS}/src/moveit_robots/baxter/baxter_moveit_config/config/
COPY ./moveit_mpnet/planning_context_manager.cpp ${CATKIN_WS}/src/${OMPL_INTERFACE}/ompl_interface/src/
COPY ./moveit_mpnet/model_based_planning_context.cpp ${CATKIN_WS}/src/${OMPL_INTERFACE}/ompl_interface/src/

WORKDIR ${CATKIN_WS}
RUN catkin clean -y
RUN catkin build
RUN source devel/setup.bash

#################################### CUDA 9.0 ##################################
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates apt-transport-https gnupg-curl && \
  rm -rf /var/lib/apt/lists/* && \
  NVIDIA_GPGKEY_SUM=d1be581509378368edeec8c1eb2958702feedf3bc3d17011adbf24efacce4ab5 && \
  NVIDIA_GPGKEY_FPR=ae09fe4bbd223a84b2ccfce3f60f4b3d7fa2af80 && \
  apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub && \
  apt-key adv --export --no-emit-version -a $NVIDIA_GPGKEY_FPR | tail -n +5 > cudasign.pub && \
  echo "$NVIDIA_GPGKEY_SUM  cudasign.pub" | sha256sum -c --strict - && rm cudasign.pub && \
  echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
  echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDA_VERSION 9.0.176

ENV CUDA_PKG_VERSION 9-0=$CUDA_VERSION-1
RUN apt-get update && apt-get install -y --no-install-recommends \
  cuda-cudart-$CUDA_PKG_VERSION && \
  ln -s cuda-9.0 /usr/local/cuda && \
  rm -rf /var/lib/apt/lists/*

# nvidia-docker 1.0
LABEL com.nvidia.volumes.needed="nvidia_driver"
LABEL com.nvidia.cuda.version="${CUDA_VERSION}"

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
  echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=9.0"

# Adding TF dependencies
# Pick up some TF dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  build-essential \
  cuda-command-line-tools-9-0 \
  cuda-cublas-9-0 \
  cuda-cufft-9-0 \
  cuda-curand-9-0 \
  cuda-cusolver-9-0 \
  cuda-cusparse-9-0 \
  libcudnn7=7.2.1.38-1+cuda9.0 \
  libnccl2=2.2.13-1+cuda9.0 \
  libfreetype6-dev \
  libhdf5-serial-dev \
  libpng12-dev \
  libzmq3-dev \
  pkg-config \
  software-properties-common \
  unzip \
  && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
  apt-get install nvinfer-runtime-trt-repo-ubuntu1604-4.0.1-ga-cuda9.0 && \
  apt-get update && \
  apt-get install libnvinfer4=4.1.2-1+cuda9.0

# Adding CUBLAS
# See : https://gitlab.com/nvidia/cuda/blob/ubuntu16.04/9.0/runtime/Dockerfile
ENV NCCL_VERSION 2.4.2

RUN apt-get update && apt-get install -y --no-install-recommends \
  cuda-libraries-$CUDA_PKG_VERSION \
  cuda-cublas-9-0=9.0.176.4-1 \
  libnccl2=$NCCL_VERSION-1+cuda9.0 && \
  apt-mark hold libnccl2 && \
  rm -rf /var/lib/apt/lists/*


# Adding CUDNN
# See : https://gitlab.com/nvidia/cuda/blob/ubuntu16.04/9.0/runtime/cudnn7/Dockerfile
# ENV CUDNN_VERSION 7.4.2.24
# LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

# RUN apt-get update && apt-get install -y --no-install-recommends \
#   libcudnn7=$CUDNN_VERSION-1+cuda9.0 && \
#   apt-mark hold libcudnn7 && \
#   rm -rf /var/lib/apt/lists/*

# Copy in files for building CUDA with cuDNN
COPY cudnn/*.deb /opt/
RUN dpkg -i /opt/libcudnn7_7.6.0.64-1+cuda9.0_amd64.deb \
  && dpkg -i /opt/libcudnn7-dev_7.6.0.64-1+cuda9.0_amd64.deb \
  && cp /usr/include/cudnn.h /usr/lib/x86_64-linux-gnu/ \
  && rm -f /opt/libcudnn*.deb

# Necessary changes to library files to get things to build and environment variables
RUN export CUDA_NVRTC_LIB="/usr/local/cuda-9.0/lib64/"
RUN cp /usr/local/cuda-9.0/lib64/libcusparse.so.9.0 /usr/local/cuda-9.0/lib64/libcusparse.so && \
    cp /usr/local/cuda-9.0/lib64/libcurand.so.9.0 /usr/local/cuda-9.0/lib64/libcurand.so && \
    cp /usr/local/cuda-9.0/lib64/libnvrtc.so.9.0 /usr/local/cuda-9.0/lib64/libnvrtc.so && \
    cp /usr/local/cuda-9.0/lib64/libcublas.so.9.0 /usr/local/cuda-9.0/lib64/libcublas.so && \
    cp /usr/local/cuda-9.0/lib64/libcufft.so.9.0 /usr/local/cuda-9.0/lib64/libcufft.so

# dev versions give the header files that pytorch source needs
RUN apt-get update && apt-get install -y \
    cuda-cusparse-dev-9-0 \
    cuda-nvrtc-dev-9-0 \
    cuda-cublas-dev-9-0 \
    cuda-cufft-dev-9-0 \
    cuda-curand-dev-9-0 \
    cuda-cusolver-dev-9-0 && \
    rm -rf /var/lib/apt/lists/*
    
# #########################################################################

# for torch to work
RUN pip install future glog
RUN apt-get update && \
    apt-get install -y libgoogle-glog-dev libgflags-dev && \
    rm -rf /var/lib/apt/lists/*

# Prepare for torch source install
WORKDIR /
RUN export CMAKE_PREFIX_PATH="/usr/bin/"
RUN pip install numpy pyyaml mkl mkl-include setuptools cmake cffi typing
RUN git clone --recursive https://github.com/pytorch/pytorch

# Build pytorch with setup script
WORKDIR /pytorch  
# Set below environment variable for compatibility with newer RTX cards
ENV TORCH_CUDA_ARCH_LIST 7.0
RUN python setup.py install 

# Copy in more moveit source code necessary for running MPNet in OMPL (turn off simplify solutions and path interpolation)   
COPY ./moveit_mpnet/model_based_planning_context.cpp ${CATKIN_WS}/src/${OMPL_INTERFACE}/ompl_interface/src/
WORKDIR ${CATKIN_WS}
RUN source devel/setup.bash && \
    catkin build

# Copy in modified CMakeLists.txt files for OMPL to build against pytorch
COPY ./ompl_cmake/CMakeLists.txt ${CATKIN_WS}/src/ompl/
COPY ./ompl_cmake/src/ompl/CMakeLists.txt ${CATKIN_WS}/src/ompl/src/ompl/

# OMPL tests breaking with cmake prefix path appended for some reason, temp hack to not build them
COPY ./ompl_cmake/tests/CMakeLists.txt ${CATKIN_WS}/src/ompl/tests/

# Finally, copy in local version of the MPNet source code and header file so that experiment runs right away
COPY ./moveit_mpnet/MPNet.cpp ${CATKIN_WS}/src/ompl/src/ompl/geometric/planners/mpnet/src/
COPY ./moveit_mpnet/MPNet.h ${CATKIN_WS}/src/ompl/src/ompl/geometric/planners/mpnet/

# Append prefix path so that OMPL links against pytorch, and build ompl package
WORKDIR ${CATKIN_WS}
RUN catkin config -a --cmake-args -DCMAKE_PREFIX_PATH=/pytorch/torch
RUN source devel/setup.bash && \
    catkin build ompl

# Exposing the ports
EXPOSE 11311

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
  ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
