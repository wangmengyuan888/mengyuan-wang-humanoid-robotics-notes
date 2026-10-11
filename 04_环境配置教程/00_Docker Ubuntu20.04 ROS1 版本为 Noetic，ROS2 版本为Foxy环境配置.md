# 环境配置与ROS桥接：Mengyuan Wang 作业参考笔记

作者：Mengyuan Wang。原始来源：个人 Docker 环境文档，第 1 至 562 行。

本文件保留个人记录中的步骤、代码、排错与结果。

## 使用与复核说明

Docker 服务代理、容器内代理与 APT 代理作用范围不同。ROS 1/ROS 2 桥接终端需要同时具备对应环境。原文的 GPU、显示和权限命令属于特定主机的历史配置；删除容器、重启服务、扩大设备权限等命令应逐项核对后再使用，不能把整个文档连续执行。

## 原始实验记录整理

### Docker Ubuntu20.04 ROS1 版本为 Noetic，ROS2 版本为Foxy

### Docker 守护进程与终端代理的区别

#### 一、先确认你的VPN本地代理端口（举例，按你自己的来）

下面仅为代理地址示例。实际 HTTP/SOCKS 协议与监听端口必须在客户端配置中确认，7890 不是所有工具的默认端口。

- Clash/V2Ray：`socks5://127.0.0.1:7890`

- HTTP代理：`http://127.0.0.1:7890`

#### 二、给Docker服务配置全局代理（永久生效）

1. 创建docker服务配置目录

```Plain Text
sudo mkdir -p /etc/systemd/system/docker.service.d
```

2. 新建代理配置文件

```Plain Text
sudo nano /etc/systemd/system/docker.service.d/http-proxy.conf
```

写入下面内容，**替换成你自己的代理地址**

```Plain Text
[Service]
Environment="HTTP_PROXY=http://127.0.0.1:7890"
Environment="HTTPS_PROXY=http://127.0.0.1:7890"
Environment="NO_PROXY=localhost,127.0.0.1,.docker.internal"
```

保存：Ctrl+O → 回车；退出：Ctrl+X

3. 重载服务、重启Docker

```Plain Text
sudo systemctl daemon-reload
sudo systemctl restart docker
```

#### 三、终端临时变量与容器环境

`export` 影响当前 shell 后续启动的子进程；在宿主机设置不会自动传进已有容器，容器内需另设或创建时传入。由 systemd 管理的 Docker 守护进程通常不继承交互终端变量，拉镜像代理应配置守护进程。参见 [Docker 官方代理说明](https://docs.docker.com/engine/daemon/proxy/)。

```Plain Text
# 临时给当前终端设置代理（容器内生效，下载镜像不生效）
export HTTP_PROXY=http://127.0.0.1:7890
export HTTPS_PROXY=http://127.0.0.1:7890
```

#### 四、测试

配置完成后再次执行：

```Plain Text
docker run hello-world
```

正常拉取镜像并输出欢迎文字即成功。

#### 补充踩坑点

1. 关键是 Docker 守护进程能访问所配置的代理服务，并不一定要启用系统全局代理；仅浏览器设置通常不改变守护进程环境。

2. 优先使用客户端实际提供的 HTTP 代理地址；SOCKS 支持应核对 Docker 版本和代理配置，不能只改 URI 就保证兼容。

3. 关闭VPN时，记得删除 `/etc/systemd/system/docker.service.d/http-proxy.conf` 再重启docker，否则会再次超时。

### Docker安装

```Markdown
# 更新源
sudo apt update
# 安装依赖
sudo apt install -y apt-transport-https ca-certificates curl gnupg lsb-release
# 添加docker密钥
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
# 添加docker软件源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
# 再次更新并安装docker
sudo apt update
sudo apt install docker.io
# 将当前用户加入docker组，免sudo
sudo usermod -aG docker $USER
# 刷新用户组（新开终端也可）
newgrp docker
# 验证docker安装成功
docker run hello-world
```

### ROS安装

#### 方案一：开启 VPN 后给 Docker 全局配置代理（一劳永逸）

##### 1、创建 docker 后台服务代理配置文件夹

```Plain Text
sudo mkdir -p /etc/systemd/system/docker.service.d
```

##### 2、写入代理配置（以下以本地 HTTP 端口 7890 举例）

```Plain Text
sudo tee /etc/systemd/system/docker.service.d/http-proxy.conf <<-'EOF'
[Service]
Environment="HTTP_PROXY=http://127.0.0.1:7890"
Environment="HTTPS_PROXY=http://127.0.0.1:7890"
Environment="NO_PROXY=localhost,127.0.0.1,.docker.internal"
EOF
```

##### 3、重载系统服务、重启 docker

```Plain Text
sudo systemctl daemon-reload
sudo systemctl restart docker
```

##### 4、现在重新拉取镜像

```Plain Text
sudo docker pull ubuntu:20.04
```

#### 拉取纯净 ubuntu20.04 基础镜像

```Plain Text
sudo docker pull ubuntu:20.04
```

#### 创建【ROS1 Noetic 容器】

```Bash
xhost +local:
docker run -it \
    --name ros1_noetic \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/ros1_ws:/root/ros1_ws \
    ubuntu:20.04
    
   
# 更新系统软件源
apt update && apt install -y curl gnupg lsb-release

# 导入ROS公钥
curl -s https://mirrors.ustc.edu.cn/rosdistro/ros.key | apt-key add -

# 生成20.04对应的Noetic源（中科大源，选1那个）
echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
# 更新ROS软件源
apt update
# 直接安装 Noetic 完整版
apt install -y ros-noetic-desktop-full
# 配置环境变量
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 新开终端，创建【ROS2 Foxy 容器】

```Bash
xhost +local:
docker run -it \
    --name ros2_foxy \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/ros2_ws:/root/ros2_ws \
    ubuntu:20.04
    
    
# 容器内安装前置依赖  
apt update && apt install -y curl gnupg lsb-release
# 写入 ROS2 官方原始源  
echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2.list
# 重新添加 ROS 官方密钥（避免签名校验失败）
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
# 刷新源缓存
apt update
# 安装 Foxy 完整版桌面包
apt install -y ros-foxy-desktop
apt install -y --fix-missing ros-foxy-desktop
# 配置环境变量
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 在选定桥接容器中安装 ROS 1、ROS 2 与 bridge

```Bash
# 清理旧失效ros2源
rm -f /etc/apt/sources.list.d/ros2.list
# 写入官方ros2源
echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2.list
# 导入密钥
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
# 更新缓存
apt update

apt install -y ros-foxy-ros1-bridge

```

原记录在 Noetic 容器安装 bridge，却在 Foxy 容器启动，是需要补齐的环境条件。桥接进程所在容器必须实际安装对应的 ROS 1/ROS 2 库和 bridge；`--net=host` 共享网络，不共享另一个容器的文件系统。若在 `ros2_foxy` 启动，必须先确认该容器中 Noetic 与 bridge 也可用。下方保留原终端布局，本轮未完成安装复测。

1. 分两个终端（或两个容器，都加`--net=host`）

- 终端 A（ROS1 Noetic）  终端 1（ros1_noetic）：

```Bash
# 先把停止的容器启动起来
sudo docker start ros1_noetic
# 再进入容器终端
sudo docker exec -it ros1_noetic bash

source /opt/ros/noetic/setup.bash
# 启动 ROS1 核心（必须先启动，桥接程序依赖 roscore）
roscore

```

- 终端 B（ROS2 Foxy，启动桥）新开宿主机终端，进入 ros2_foxy 容器启动桥接  终端 2（ros2_foxy 桥进程，双环境）

```Bash
sudo docker start ros2_foxy
sudo docker exec -it ros2_foxy bash

# 第一步：加载ROS1 Noetic环境（桥程序同时需要两套消息定义）
source /opt/ros/noetic/setup.bash
# 第二步：加载ROS2 Foxy环境
source /opt/ros/foxy/setup.bash

# 以下两个启动方式二选一，不要连续启动两个桥
export ROS_MASTER_URI=http://localhost:11311
# 默认按需桥接，需存在匹配的发布/订阅端及受支持的类型
ros2 run ros1_bridge dynamic_bridge
# 可选替代：按该版本帮助，桥接全部可匹配话题
# ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

`Created 2-way bridge` 表示相应桥接端点建立，不证明全部话题或业务链路正确。动态桥可以发现后加入的话题；默认按匹配发布/订阅关系桥接。ROS 2 echo 可显式给出类型，ROS 1 无该选项，反向测试必要时采用该版本的全部 ROS 2 到 ROS 1 话题桥接选项。两方向均收到预期消息后才算完成本节通信测试。参见 [ros1_bridge 官方 README](https://github.com/ros2/ros1_bridge)。

---

#### 双向话题收发测试（验证桥接生效）

https://github.com/ros2/ros1_bridge

##### 测试 1：ROS1 发消息，ROS2 接收 

1. 新开终端进入 Noetic 容器，发布话题  终端 3（ros1_noetic 发布话题）

```Plain Text
sudo docker exec -it ros1_noetic bash
source /opt/ros/noetic/setup.bash
rostopic pub /chatter std_msgs/String "data: 'hello bridge'" -r 1
```

2. 新开终端进入 Foxy 容器，监听话题  终端 4（ros2_foxy 订阅，仅 Foxy 环境）

```Plain Text
sudo docker exec -it ros2_foxy bash
source /opt/ros/foxy/setup.bash
ros2 topic echo /chatter std_msgs/msg/String
```

能打印字符串代表 ROS1→ROS2 转发成功。

##### 测试 2：ROS2 发消息，ROS1 接收

1. Foxy 容器发布消息

```Plain Text
sudo docker exec -it ros2_foxy bash
source /opt/ros/foxy/setup.bash

ros2 topic pub /reply std_msgs/msg/String '{data: "Reply from ROS2 Foxy"}' -r 1
```

2. Noetic 容器监听话题

```Plain Text
sudo docker exec -it ros1_noetic bash
source /opt/ros/noetic/setup.bash
rostopic echo /reply
```

##### 可视化问题：容器Docker内执行

```Bash
# 1. 安装缺失的 OpenGL 驱动库（关键！）
apt update && apt install -y libgl1-mesa-glx libgl1-mesa-dri

# 2. 强制启用间接渲染（解决 libGL 错误）
echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc

# 3. 禁用 MIT-SHM 共享内存（避免 X11 通信错误）
echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc

# 4. 激活配置
source ~/.bashrc
```

### 安装环境

```Bash
sudo usermod -aG docker $USER
newgrp docker


sudo docker stop ros1_noetic
sudo docker start ros1_noetic
sudo docker exec -it ros1_noetic bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装依赖
sudo apt install -y \
    cmake \
    curl \
    git \
    libfreeimage-dev \
    libprotoc-dev \
    protobuf-compiler \
    libignition-math6-dev \
    libsqlite3-dev \
    libtinyxml2-dev \
    libgflags-dev \
    libavformat-dev \
    libavcodec-dev
# 添加 Gazebo 官方软件源​
sudo apt install -y wget
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 安装 Gazebo​
sudo apt update
sudo apt install -y gazebo11 libgazebo11-dev
# 配置环境变量
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc
# 验证安装
export DISPLAY=$DISPLAY
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_INDIRECT=1
# 出现问题参考下面
gazebo worlds/empty.world
```

#### gazebo出现运行不出来问题：修改之前的Docker(需要安装nvidia-container-toolkit)

##### 安装nvidia-container-toolkit

```Bash
sudo rm -f /etc/apt/sources.list.d/nvidia-*.list
sudo rm -f /usr/share/keyrings/nvidia-*.gpg
# 重载全局代理（你已执行，重新执行确保生效）
export HTTP_PROXY=http://127.0.0.1:7890
export HTTPS_PROXY=http://127.0.0.1:7890
# 新增socks代理兜底（多数7890端口软件同时开启socks5）
export ALL_PROXY=socks5://127.0.0.1:7890
# curl 禁用 TLS1.3，规避握手断连（关键参数）
# 新增参数 --tlsv1.2 强制使用 TLS1.2，解决unexpected eof：
curl -4 --tlsv1.2 --retry 5 --retry-delay 2 -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
# 写入官方适配 Ubuntu22.04 jammy 的源
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
arch=$(dpkg --print-architecture)
echo "deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://nvidia.github.io/libnvidia-container/stable/deb/$arch /" | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

```

###### 给 APT 单独配置代理（推荐，一次性生效）

###### 创建 apt 代理配置文件

```Plain Text
sudo nano /etc/apt/apt.conf.d/99-proxy
```

粘贴下面内容：

```Plain Text
Acquire::http::Proxy "http://127.0.0.1:7890";
Acquire::https::Proxy "http://127.0.0.1:7890";
```

保存退出：`Ctrl+O` → 回车 → `Ctrl+X`

###### 重新执行 apt 更新

```Plain Text
sudo apt update
```

配置生效且代理可达时，APT 可经示例代理下载元数据；仍报握手错误时需检查协议、证书、仓库地址与网络，不能保证配置后必然消失。

###### 安装工具包

```Plain Text
sudo apt install -y nvidia-container-toolkit
```

##### 修改Docker

```Bash
# 允许容器访问显示器
xhost +local:

# 把当前容器打包（保留所有已安装的东西）
docker commit ros1_noetic ros1_noetic_backup

# 删掉旧容器
docker stop ros1_noetic
docker rm ros1_noetic

# 用合并后的参数重新创建
docker run -it \
    --name ros1_noetic \
    --gpus all \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/ros1_ws:/root/ros1_ws \
    ros1_noetic_backup
```

```Bash
# 允许容器访问显示器
xhost +local:

# 把当前容器打包（保留已安装的东西）
docker commit ros2_foxy ros2_foxy_backup

# 删掉旧容器
docker stop ros2_foxy
docker rm ros2_foxy

# 用合并后的参数重新创建
docker run -it \
    --name ros2_foxy \
    --gpus all \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/ros2_ws:/root/ros2_ws \
     ros2_foxy_backup
     
# 最新镜像
docker run -it \
    --name ros2_foxy \
    --gpus all \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/ros2_ws:/root/ros2_ws \
     ros2_foxy:backup_20260925
```

#### 解压工作空间到挂载目录下面(继续安装)

```Bash
# Eigen安装，版本为3.4
sudo unzip /home/mengyuan/下载/robocup_g1.zip -d /home/mengyuan/ros1_ws
cd ros1_ws
cd robocup_g1/eigen-3.4.0
cd build
rm -rf *
cmake ..
sudo make install
```

```Bash
# RealSense SDK 安装
sudo apt-get install ros-noetic-realsense2-camera
```

```Bash
# Ignition-math 安装
sudo apt install libignition-math4-dev
```

```Bash
# 遥控器 ROS 驱动 ros-noetic-joy 安装
sudo apt-get install ros-noetic-joy
```

（可选）如果遇到缺少以下依赖项，可以参考安装，没有遇到则不需要安装

```Bash
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-joint-state-controller ros-noetic-ros-control
ros-noetic-ros-controllers
sudo apt-get install ros-noetic-robot-state-publisher
sudo apt-get install ros-noetic-xacro
```

```Bash
# 解压缩后，进行编译
cd ~/ros1_ws/robocup_g1/
catkin_make
```

#### 启动流程

在宿主机（不是容器内）的终端执行：

```Bash
xhost +local:root
```

```Bash
# 启动 Gazebo
# 这里对 CPU 内存需求比较大，建议16G+
source ~/ros1_ws/robocup_g1/devel/setup.bash
roslaunch unitree_guide gazebo.launch
```

```Bash
# 重新开一个终端启动运控
sudo docker exec -it ros1_noetic bash
source /opt/ros/noetic/setup.bash
cd /root/ros1_ws/robocup_g1

source ~/ros1_ws/robocup_g1/devel/setup.bash
./devel/lib/unitree_guide/junior_ctrl
```

#### 切换模式

•在`junior_ctrl`主界面输入`2`  ，控制机器人从 State_Passive 切换到 fixed stand。

•  回到 Gazebo 主界面，按下暂停键，然后在主菜单中选择 Edit/Reset Model Poses 以重置机器人的

位姿。

• 在 `junior_ctrl` 主界面输入 `4` ，控制机器人从 fixed stand 切换到 LOCOMOTION。

• 回到 Gazebo 主界面，点击播放键，重启应用。
