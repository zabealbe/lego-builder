FROM osrf/ros:noetic-desktop-focal

ENV TZ=Europe/Rome
ARG DEBIAN_FRONTEND=noninteractive

# Update repos and install apt-utils
RUN apt update && apt install -y apt-utils

# Install base dependencies
RUN apt install -y curl gnupg lsb-release wget build-essential

#
# Install Torch & Yolo dependencies
#
# Install python
RUN apt install -y python3 python3-pip python3-tk && ln -s /usr/bin/python3 /usr/bin/python

#
# install ros packages
#
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1*
RUN apt install -y ros-noetic-ros-control ros-noetic-ros-controllers # Controllers stuff
RUN apt install -y ros-noetic-soem # Ethercat stuff

# Install ROS & ROS dependencies
#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

#RUN apt update && apt install -y --no-install-recommends ros-noetic-desktop-full=1.5.0-1*
#RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool
#RUN apt install -y ros-noetic-moveit # Moveit

#
# Install project dependencies
#
RUN pip install -r https://raw.githubusercontent.com/ultralytics/yolov5/v6.0/requirements.txt

COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# Cleanup
#RUN rm -rf /var/lib/apt/lists/*

# Add ros user with home folder
RUN useradd -ms /bin/bash -G sudo ros
RUN chown -R ros:ros /home/ros

# Yolo font
ADD https://ultralytics.com/assets/Arial.ttf /home/ros/.config/Ultralytics/Arial.ttf
RUN chown -R ros:ros /home/ros/.config/Ultralytics

# Enable sudo without password
RUN sed -i 's/(ALL:ALL) ALL/(ALL) NOPASSWD: ALL/' /etc/sudoers

# Switch to the newly created user
USER ros

# Source ros on startup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
WORKDIR /home/ros

# Update ros dependencies
RUN rosdep update

ENTRYPOINT ["/bin/bash"]
