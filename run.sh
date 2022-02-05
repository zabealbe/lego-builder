#!/usr/bin/env bash

DOCKER_IMAGE=ros-noetic
cmd=$0

function in_docker(){
	[ -f /.dockerenv ] || { echo "This option is only allowed while in Docker!" && exit 1; };
}

function docker_build() {
	docker build -t ${DOCKER_IMAGE} Docker
}
function docker_start() {
	# Docker build if not already
	[[ ! "$(docker images -q ${DOCKER_IMAGE}:latest 2> /dev/null)" == "" ]] || docker_build
	xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ros-noetic`
	docker run \
		-dt --rm \
		-e ROS_HOSTNAME=localhost \
		-e ROS_IP=localhost \
		-e ROS_MASTER_URI=http://localhost:11311 \
		-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v ~/.Xauthority:/home/ros/.Xauthority:rw -v /dev/dri/card0:/dev/dri/card0 -v /dev/dri/card1:/dev/dri/card1 \
		-v $PWD:/home/ros/Workspace \
		--workdir /home/ros/Workspace \
		--name ${DOCKER_IMAGE} \
		${DOCKER_IMAGE}
}
function docker_bash() {
    # Docker start if not already
    [[ $(docker ps --filter "name=^/$DOCKER_IMAGE$" --format '{{.Names}}') == $DOCKER_IMAGE ]] || docker_start
    #
    docker exec -it ${DOCKER_IMAGE} bash
}
function docker_stop() {
    docker stop ${DOCKER_IMAGE}
}
function devel() {
    in_docker
    
    setup

    . devel/setup.bash
    roslaunch lego_builder_gazebo base.launch
}
function setup() {
    in_docker

    mkdir -p src
    cd src
    catkin_init_workspace

    cd ..
    #rosdep update
    rosdep install -y --rosdistro $ROS_DISTRO --ignore-src --from-paths src
    catkin_make
}
function launch() {
    launch.sh
}
function clean() {
    docker_stop
}

# Utilities
function view() {
    rosrun image_view image_view image:=/camera/color/image_raw
}
function view_depth() {
    rosrun image_view image_view image:=/camera/depth/image_raw
}
function run_vision() {
    source devel/setup.bash

    rosrun lego_builder_vision main.py
}

function run_kinematics() {
  source devel/setup.bash

  rosrun lego_builder_kinematics main.py
}

eval $1
