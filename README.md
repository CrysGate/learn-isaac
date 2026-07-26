# learn-isaac

## isaacsim env setup

```bash
uv sync
```

## install ros2-jazzy-desktop in ubuntu24.04

```bash
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository universe

sudo apt update
sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F "tag_name" \
  | awk -F'"' '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade -y

sudo apt install ros-jazzy-desktop -y
```

optional for development:
```bash
sudo apt install ros-dev-tools -y
```
### activate ros2 in terminal
```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
```

### test ros2
terminal 1:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```
terminal 2:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```
