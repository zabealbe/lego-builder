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

    # Setup the ROS workspace
    docker exec -i ${DOCKER_IMAGE} bash -c "source /opt/ros/noetic/setup.bash && ./run.sh setup"

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
    # Setup the environment and launch gazebo world
    # param $1: world name

    roslaunch lego_builder_gazebo $1
    #roslaunch lego_builder_gazebo $1
}

function run_vision() {
    # Launch the vision ros node
    rosrun lego_builder_vision main.py
}

function run_kinematics() {
    # Setup the environment and launch gazebo world
    # param $1: mode name

    if [ -z "$1" ]; then
        echo "Please provide a mode name"
        exit 1
    fi

    # Launch the kinematics ros node
    case $1 in
        stack)
            rosrun lego_builder_kinematics main.stack.py
        ;;
        build)
            rosrun lego_builder_kinematics main.build.py
        ;;
        *)
            echo "Unknown mode"
            exit 1
        ;;
    esac
}

function launch() {
    # Start gazebo world and launch all the nodes
    # param $1: launch file name
    # param $2: mode name

    if [ -z "$1" ]; then
        echo "Please provide a launch file name"
        exit 1
    fi

    (run_master "$1.launch") > /dev/null &
    (run_vision) > /dev/null &
    run_kinematics $2
}

function launch_mode_build() {
    launch $1 build
}

function launch_mode_stack() {
    launch $1 stack
}

function launch_ass2() {
    # Launch the assignment 2
    launch ass2 stack
}

function launch_ass3() {
    # Launch the assignment 3
    launch ass3 stack
}

function launch_ass4() {
    # Launch the assignment 4
    launch ass4 build
}
eval $@
