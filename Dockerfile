FROM resin/beaglebone-black-buildpack-deps:jessie

# Switch on systemd init system in container and set various other variables
ENV INITSYSTEM="on" \
    TERM="xterm" \
    PYTHONIOENCODING="UTF-8"

# Variables for ROS distribution, configuration, and installation
ENV ROS_DISTRO="indigo" \
    ROS_CONFIG="ros_comm" \
    ROS_EXTRA_PACKAGES="sensor_msgs pcl_conversions pointcloud_to_laserscan"
ENV ROS_INSTALL_DIR="/opt/ros/${ROS_DISTRO}"

RUN apt-get -qq update \
  && apt-get install -yq --no-install-recommends \
    python-dev python-pip \
    libpcl-dev libpcl1.7

# Install ROS-related Python tools
COPY ./requirements.txt .
RUN pip install -q -r requirements.txt

RUN rosdep init \
    && rosdep update

RUN mkdir -p /usr/src/catkin_ws/src ${ROS_INSTALL_DIR}

WORKDIR /usr/src/catkin_ws

RUN rosinstall_generator ${ROS_CONFIG} ${ROS_EXTRA_PACKAGES} \
    --rosdistro ${ROS_DISTRO} --deps --tar > .rosinstall \
    && wstool init src .rosinstall \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
       --skip-keys python-rosdep \
       --skip-keys python-rospkg \
       --skip-keys python-catkin-pkg \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN catkin init \
    && catkin config --install --install-space ${ROS_INSTALL_DIR} \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build --no-status --no-summary --no-notify \
    && catkin clean -y --logs --build --devel \
    && rm -rf src/*

WORKDIR /usr/src

# Setting -DDUMMY=On is a temporary workaround due to a linking error
RUN git clone https://github.com/scanse/sweep-sdk.git \
  && cd sweep-sdk/libsweep \
  && mkdir -p build && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release -DDUMMY=On \
  && cmake --build . \
  && cmake --build . --target install \
  && ldconfig \
  && cd /usr/src && rm -rf sweep-sdk

COPY . /usr/src/app

WORKDIR /usr/src/app

# Debugging .dockerignore
RUN cat .dockerignore && ls -lah

ENTRYPOINT ["./entrypoint.sh"]

CMD ["roscore"]
