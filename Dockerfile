FROM resin/beaglebone-black-buildpack-deps:jessie

#switch on systemd init system in container
ENV INITSYSTEM="on" \
    TERM="xterm" \
    LANG="en_US.UTF-8" \
    ROS_DISTRO="indigo"

# Setup apt keys
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# Add apt sources
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get -q update \
  && apt-get install -yq --no-install-recommends \
    locales locales-all \
    python-dev python-pip \
    python-rosdep python-catkin-tools \
    ros-${ROS_DISTRO}-ros-base \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

RUN rosdep init \
    && rosdep update

COPY . /usr/src/app

WORKDIR /usr/src/app

RUN ls -lah

ENTRYPOINT ["./entrypoint.sh"]

CMD ["roscore"]
