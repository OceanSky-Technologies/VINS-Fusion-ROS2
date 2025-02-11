FROM dtcooper/raspberrypi-os:bookworm

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
      && cmake -DWITH_VULKAN=ON \
            -DBUILD_TESTS=OFF \
            -DBUILD_PERF_TESTS=OFF \
            -DBUILD_EXAMPLES=OFF \
            .. \
      && cmake --build . -j $(nproc) --target install

# ceres
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

RUN apt update \
      && apt install -y gstreamer1.0-* ninja-build \
      && rm -rf /var/lib/apt/lists/*

# ROS2
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    locales \
    wget \
    curl \
    git \
    build-essential \
    cmake \
    python3-pip \
    libssl-dev \
    libasio-dev \
    libtinyxml2-dev \
    python3-empy \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANGUAGE=en_US:en

RUN pip3 install --break-system-packages -U \
setuptools \
    colcon-common-extensions \
    rosdep \
    vcstool \
    rosinstall_generator \
    rosdistro \
    rospkg

RUN rosdep init || echo "rosdep is already initialized."
RUN rosdep update

WORKDIR /ros2_ws
RUN mkdir -p src

RUN rosinstall_generator --rosdistro humble desktop --deps --tar > humble-desktop.rosinstall
RUN vcs import src < humble-desktop.rosinstall

RUN apt-get update && rosdep install \
      --skip-keys "ignition_math6_vendor fastcdr rti-connext-dds-6.0.1 urdfdom_headers ignition-cmake2 ignition-math6 python3-catkin-pkg-modules python3-rosdistro-modules python3-vcstool" \
      --from-paths src --ignore-src -y --rosdistro humble \
      && rm -rf /var/lib/apt/lists/*

# Build ROS 2 with colcon
RUN colcon build --merge-install --symlink-install --packages-skip rviz_default_plugins rviz2 desktop

RUN echo "LC_ALL=en_US.UTF-8" >> /etc/environment \
      && echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen \
      && echo "LANG=en_US.UTF-8" > /etc/locale.conf \
      && locale-gen en_US.UTF-8

# libcamera
RUN pip3 install meson --break-system-packages
RUN apt update \
      && apt install -y libcamera-dev libepoxy-dev libjpeg-dev libtiff5-dev libpng-dev libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev \
            cmake libboost-program-options-dev libdrm-dev libexif-dev \
      && rm -rf /var/lib/apt/lists/*

# rpicam-apps
RUN git clone --depth=1 https://github.com/raspberrypi/rpicam-apps.git \
      && cd rpicam-apps && meson setup build --buildtype=release -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled \
      && meson compile -C build && meson install -C build
RUN echo "/usr/local/lib/aarch64-linux-gnu" >> /etc/ld.so.conf.d/rpicam.conf \
      && ldconfig

WORKDIR /root

# 8. Source setup in the container environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Default to a bash shell
CMD ["/bin/bash"]