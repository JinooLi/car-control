FROM ubuntu:20.04

RUN apt update
RUN apt install sudo -y

# Set the locale
RUN sudo apt install locales -y
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# set the timezone
RUN ln -sf /usr/share/zoneinfo/Asia/Seoul /etc/localtime
RUN ln -sf /usr/share/zoneinfo/GMT /etc/localtime

# ensure the Ubuntu Universe repository is enabled
RUN sudo apt install software-properties-common -y
RUN sudo add-apt-repository universe

# Add the ROS 2 GPG key wit apt
RUN sudo apt update && sudo apt install curl -y
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository to sources list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo apt update
RUN sudo apt upgrade

# Install terminator
RUN sudo apt install terminator -y

# Install ROS 2 packages
# DESKTOP INSTALL (Recommended): ROS, RViz, demos, tutorials.
RUN sudo apt install ros-foxy-desktop python3-argcomplete -y

# Development tools: Compilers and other tools to build ROS packages
RUN sudo apt install ros-dev-tools -y

# Install dependencies for f1tenth gym
RUN sudo apt install -y ros-foxy-ackermann-msgs ros-foxy-joint-state-publisher 
RUN sudo apt install -y ros-foxy-xacro ros-foxy-nav2-map-server 
RUN sudo apt install -y ros-foxy-nav2-lifecycle-manager 
RUN rosdep init
RUN rosdep update
RUN sudo apt install python3-pip -y

# set bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc