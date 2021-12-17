
# Jetson 빌드 시 주의사항 
2021-10-17 기준 최신 버전에 해당하는 Jetpack 4.6에서는 ros2_galactic 빌드 시 에러가 발생함.
Jetpack 4.6 버전 사용자의 경우 반드시 사전에 4.5.1로 다운그레이드를 진행하여야 함.

# ROS2 빌드 사전 준비
# 링크 : https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html       

locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

apt-cache policy | grep universe # check Ubuntu Universe repository (for apt)

sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing

python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools

# Only in ubuntu 18.04 !!!
python3 -m pip install -U importlib-metadata importlib-resources

sudo rosdep init
rosdep update

# QT development library
sudo apt-get install qtquickcontrols2-5-dev qtscript5-dev libqt5serialport5-dev qml-module-qtquick-controls2


python3 -m pip install open3d cupoch

#################################### ROS2 빌드 전 주의사항 ####################################

ROS2는 Build를 진행하기 전 source로 가져온 다른 workspace에서 패키지들을 불러오게 되어 있음.
그러므로 ws의 빌드 순서를 틀리거나 ws를 불러오지 않고 빌드를 시도하는 경우 에러가 발생할 수 있음.
또한 workspace들의 경로가 바뀌거나 혹은 다른 사람으로부터 workspace를 받은 경우 
반드시 Clean Build를 시켜줘야 함.
# Clean build 방법 : ws에서 bulid, install, log 파일 제거 후 colcon build ~~~ 진행
특히 다른 사람으로부터 workspace를 받은 경우에는 Build를 진행하기 전 rosdep을 
이용하여 dependency 패키지들을 설치해주어야 함.

# ws 불러오는 방법 : 빌드가 완료된 후에 ws를 불러오는게 가능함.
source (ws 경로)/install/setup.bash

# ex) source ~/Documents/GitHub/Platform_ROS2_ws/workspaces/install/ros2_galactic/setup.bash

# ws dependency 설치 방법 : 처음 빌드할 때 or 패키지 dependency가 변동되는 경우 실행
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} 

#############################################################################################

# ROS2 소스 빌드 설치

cd ~/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic
rosdep install --from-paths src --ignore-src --rosdistro galactic -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"
colcon build --symlink-install

# ROS2 예제 실행
# ~/.bashrc에 추가한 경우 이후 과정에서 source ~~~는 생략해도 OK

# Terminal 1
source ~/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/setup.bash 
ros2 run demo_nodes_cpp talker

# Terminal 2
source ~/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/setup.bash 
ros2 run demo_nodes_cpp listener

# ROS2 Talker, listener 실행 후 안되면 해당 명령어 실행
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4

# self_drive_ws 빌드
cd ~/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} 
sudo apt-get install python3-sphinx

source ~/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/setup.bash 
colcon build --symlink-install

# For librealsense
source ~/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/install/setup.bash 
realsense-viewer
sudo cp ~/.99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger

# simulation_ws 빌드 방법
# 링크 : http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install

# gazebo 설치
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt install libgazebo11-dev 


cd ~/Documents/GitHub/Platform_ROS2_ws/workspaces/simulation_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro=${ROS_DISTRO}

# 빌드
source ~/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/setup.bash 
source ~/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/install/setup.bash 
colcon build --symlink-install 

# ~/.bashrc 최하단에 해당 내용 추가. 반드시 최하단에 놓아야됨.
# Github Desktop으로 clone을 진행하였을 때 경로 기준
source ~/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/setup.bash
source ~/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/install/setup.bash
source ~/Documents/GitHub/Platform_ROS2_ws/workspaces/simulation_ws/install/setup.bash

# 미들웨어 선택 (fastrtps(fast dds) 추천, ~/.bashrc 추가)
# 둘 중 하나만 선택할 것.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 


# (Optional) Cyclone DDS URI 경로 추가 (~/.bashrc)
# 만약 ros2: using network interface enp... (udp/...) selected arbitrarily from: enp..., tun0 ~~
# 에러가 발생하는 경우 MULTICAST 플래그를 가진 네트워크 인터페이스로 선택하여 설정해 주어야 함.

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>(인터페이스)</></></></>'

# ex.

user@user:~$ ifconfig

# Choose this.. (Look flags= ~~~~,"MULTICAST">)
enp4s0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.0.xxx  netmask 255.255.255.0  broadcast 192.168.0.xxx
        inet6 xxxx::xxxx:xxxx:xxxx:xxxx  prefixlen 64  scopeid 0x20<link>
        ether 70:4d:7b:a3:3b:f3  txqueuelen 1000  (Ethernet)
        RX packets 2813463  bytes 2573310320 (2.5 GB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 3824336  bytes 1395161691 (1.3 GB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        RX packets 7822068  bytes 16193692285 (16.1 GB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 7822068  bytes 16193692285 (16.1 GB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

# In ~/.bashrc
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>enp4s0</></></></>'



#################################### WS 수정 시 주의사항 #############################################

환경 이슈 최소화 및 설치 간편화를 위해 소스 빌드 방식을 채택하였음.
대부분의 패키지는 github에서 받은 패키지 초기 소스와 다르게 dependency에 해당하는 라이브러리를 함께 
동봉하였으므로 패키지 재설치 or 최신 버전으로 변경 시 이를 유의하여야 함.
추가된 패키지는 대부분 ~~_dependencies(ex. nav2_dependencies) 폴더에 들어있음.

# 참고 : 현재 self_drive_ws에서 build 에러로 인해 법
#        cartographer/mapping/3d/hybrid_grid_test.cc를 제거하였음
#        (관련링크 : https://github.com/cartographer-project/cartographer/issues/1653)

# 동적 파라미터 변동을 위해 nav2_costmap_2d 패키지를 일부 수정하였음.
# (관련링크: https://github.com/ros-planning/navigation2/pull/2592)
# (수정파일: costmap_2d_ros.cpp/hpp, inflation_layer.cpp/hpp)


#####################################################################################################

#################################### 패키지 사용 시 주의사항 ##########################################

# rqt 사용 중 not found ~~~/plagin.xml 에러 발생 시 (주로 ROS2 버전 변경 시 발생함)
rqt --force-discover

#####################################################################################################



