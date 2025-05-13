# ==============================================================================
#               *** BASE SETUP - 以下内容请勿擅自改动 ***
# ==============================================================================

FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    python3-pip \
    curl \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

RUN sed -i 's/http:\/\/archive.ubuntu.com/http:\/\/mirrors.tuna.tsinghua.edu.cn\/ubuntu/g' /etc/apt/sources.list

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -

RUN echo "deb [arch=$(dpkg --print-architecture)] https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu focal main" | tee /etc/apt/sources.list.d/ros.list > /dev/null

RUN mkdir -p /etc/profile.d/ && \
    echo 'export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml' > /etc/profile.d/ros.sh

RUN pip install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple --upgrade pip \
    && pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID  -m -s /bin/bash $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ARG WORKSPACE=/home/$USERNAME/ros_ws
# RUN mkdir -p /etc/profile.d/ && \
#     echo "if [ -f \${WORKSPACE}/install/setup.bash ]; then source \${WORKSPACE}/install/setup.bash; fi" > /etc/profile.d/ros_setup.sh

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN usermod -aG audio ${USERNAME}

# ==============================================================================
#               *** BASE SETUP - 以上内容请勿擅自改动 ***
# ==============================================================================


# ==============================================================================
#            >>> 允许修改下列依赖相关内容，在写入dockerfile前请先测试<<<
# ==============================================================================

## 安装 Ubuntu 以及 ROS 相关依赖
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    sudo \
    python3-pip \
    build-essential \
    cmake \
    pkg-config \
    libjpeg-dev \
    libtiff5-dev \
    librealsense2-dev \
    librealsense2-utils \
    libpng-dev \
    make \
    libeigen3-dev\
    gfortran \
    ros-noetic-joy \
    ros-noetic-octomap-msgs \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-octomap-ros \
    ros-noetic-octomap \
    ros-noetic-octomap-server \
    ros-noetic-map-server \
    ros-noetic-sbpl \
    portaudio19-dev \
    ros-$(rosversion -d)-audio-common \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# 安装 python 依赖
RUN pip install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple \
    numpy \
    pyaudio \
    algorithm \
    motion \
    scipy \
    openai \
    pyalsaaudio \
    pydub \
    sounddevice \
    opencc-python-reimplemented

# ==============================================================================
#              <<< END USER MODIFICATION SECTION >>>
# ==============================================================================


# ==============================================================================
#            *** ENVIRONMENT SETUP - DO NOT MODIFY BELOW ***
# ==============================================================================

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home/$USERNAME

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc

RUN chown -R $USERNAME:$USERNAME /home/$USERNAME
USER $USERNAME
RUN echo "if [ -f /workspace/devel/setup.bash ]; then source /workspace/devel/setup.bash; else echo '未找到工作空间 /workspace/devel/setup.bash，跳过 source。'; fi" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]