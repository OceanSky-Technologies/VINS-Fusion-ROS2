FROM ros:humble

ENV CATKIN_WS=/root/catkin_ws

RUN apt update \
      && apt install -y wget curl cmake pkg-config g++ gcc libeigen3-dev \
      && rm -rf /var/lib/apt/lists/*

# opencv with vulkan
RUN apt update \
      && apt install -y ffmpeg libavformat-dev libavcodec-dev libswscale-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev mesa-vulkan-drivers vulkan-tools \
      && rm -rf /var/lib/apt/lists/*

ENV OPENCV_VERSION=4.11.0
RUN wget https://github.com/opencv/opencv/archive/refs/tags/$OPENCV_VERSION.tar.gz \
      && tar -xf $OPENCV_VERSION.tar.gz && rm $OPENCV_VERSION.tar.gz \
      && cd opencv-$OPENCV_VERSION \
      && mkdir build && cd build \
      && cmake -DWITH_VULKAN=ON .. \
      && cmake --build . -j $(nproc) --target install

RUN apt update \
      && apt install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev \
      && rm -rf /var/lib/apt/lists/*

ENV CERES_SOLVER_VERSION=2.2.0
RUN wget http://ceres-solver.org/ceres-solver-$CERES_SOLVER_VERSION.tar.gz \
      && tar -xf ceres-solver-$CERES_SOLVER_VERSION.tar.gz && rm ceres-solver-$CERES_SOLVER_VERSION.tar.gz \
      && cd ceres-solver-$CERES_SOLVER_VERSION \
      && mkdir build && cd build \
      && cmake .. \
      && cmake --build . -j $(nproc) --target install
