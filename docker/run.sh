#!/bin/bash

# Go to the directory of this file -- needed for docker-compose
FILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $FILE_DIR

if [ -z ${1+x} ];
then
        SERVICE_NAME="vision_project_dev";
else
        SERVICE_NAME=$1;
fi

if [ -z ${2+x} ];
then
	MODE="popup";
else
	MODE=$2;
fi

ROS_MASTER_URI=$ROS_MASTER_URI docker-compose up --build
case "$MODE" in
	terminal | terminal_debug | t )
		ROS_MASTER_URI=$ROS_MASTER_URI docker-compose run $SERVICE_NAME bash -c "source ~/catkin_ws/devel/setup.bash && bash"
		;;
	stop | kill | down )
		docker-compose down
		;;
	terminator | popup )
		xhost +local: # share display on ubuntu platforms
		ROS_MASTER_URI=$ROS_MASTER_URI docker-compose run $SERVICE_NAME bash -c "terminator -e \"echo 'Entering Docker Container...' && bash\""
		;;
	* )
		echo "Not a valid command"
		;;
esac
