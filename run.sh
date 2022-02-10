#!/usr/bin/env bash

DOCKER_IMAGE=ros-noetic
cmd=$0

# Exit if we are not in a docker container
function in_docker(){
	[ -f /.dockerenv ] || { echo "This option is only allowed while in Docker!" && exit 1; };
}

# Build the docker image
function docker_build() {
	docker build -t ${DOCKER_IMAGE} Docker
}

# Start the docker container
function docker_start() {
  # Build the docker image if it does not exist
	[[ ! "$(docker images -q ${DOCKER_IMAGE}:latest 2> /dev/null)" == "" ]] || docker_build
	# Enable x11 forwarding
	xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ros-noetic`
	# Launch the docker container
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
  # Source the ROS setup script
  docker exec -it ${DOCKER_IMAGE} bash -c "echo 'source ~/Workspace/devel/setup.bash' >> ~/.bashrc"
}
function docker_bash() {
    # Docker start if not already
    [[ $(docker ps --filter "name=^/$DOCKER_IMAGE$" --format '{{.Names}}') == $DOCKER_IMAGE ]] || docker_start
    # Create a new bash session in the docker container
    docker exec -it ${DOCKER_IMAGE} bash
}
function docker_stop() {
    # Stop the docker container
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
function clean() {
    docker_stop
}

function run_master() {
    devel
}

function run_vision() {
    rosrun lego_builder_vision main.py
}

function run_kinematics() {
    rosrun lego_builder_kinematics main.ass3.py
}

function run_all() {
    (run_master) > /dev/null &
    (run_vision) > /dev/null &
    run_kinematics
}

function run_launch() {
    roslaunch
}

eval $1
