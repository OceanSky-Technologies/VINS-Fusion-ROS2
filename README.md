# VINS-Fusion

## RPI / ROS2 version of VINS-Fusion.

This is a fork of https://github.com/zinuok/VINS-Fusion-ROS2.

It runs on a Raspberry Pi 5 using the Global Shutter camera, BMI088 IMU and a RS485 CAN HAT.

## Setup

First, install docker on the RPI:

```bash
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Then build the docker container for this repo:

```bash
cd docker
make build
```

For running on a Jetson board open `feature_tracker.h` and uncomment

  ```bash
  #define GPU_MODE 1
  ```

### run with loop fusion

```bash
docker run \
  -it \
  --rm \
  --net=host \
  -v $(git rev-parse --show-toplevel):/root/catkin_ws/src/VINS-Fusion/ \
  ros:vins-fusion \
  /bin/bash -c \
  "cd /root/catkin_ws/; \
  catkin config \
          --env-cache \
          --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
          -DCMAKE_BUILD_TYPE=Release; \
      catkin build; \
      source devel/setup.bash; \
      rosrun loop_fusion loop_fusion_node ${CONFIG_IN_DOCKER} & \
      rosrun vins kitti_odom_test ${CONFIG_IN_DOCKER} /root/kitti_dataset/"


ros2 launch vins vins_rviz.launch.xml &
ros2 run vins vins_node ./config/euroc/euroc_mono_imu_config.yaml &
ros2 run loop_fusion loop_fusion_node ./config/euroc/euroc_mono_imu_config.yaml &

# Use the "world" frame
```


## play bag recorded at ROS1
Unfortunately, you can't just play back the bag file recorded at ROS1. 
This is because the filesystem structure for bag file has been changed significantly.
The bag file at ROS2 needs the folder with some meta data for each bag file, which is done using following commands.
- you have to install [this pkg](https://gitlab.com/ternaris/rosbags)
```bash
sudo apt install pipx
pipx install rosbags
```

- run
```bash
export PATH=$PATH:~/.local/bin
rosbags-convert --src foo.bag --dst /path/to/bar
```






## Original Readme:

## 8. Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, a generic [camera model](https://github.com/hengli/camodocal) and [GeographicLib](https://geographiclib.sourceforge.io/).

## 9. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong Qin <qintonguavATgmail.com>.

For commercial inquiries, please contact Shaojie Shen <eeshaojieATust.hk>.
