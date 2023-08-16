#!/bin/bash
#* Hide popd and pushd stdout by defining new commands.
popdq () {
	command popd "$@" > /dev/null
}
pushdq () {
	command pushd "$@" > /dev/null
}
#* Change the cwd to the script dir temporarily until the script exits for any reason.
#* (If it exists use BASH_SOURCE, otherwise fall back to $0.)
trap popdq EXIT
pushdq "$(dirname ${BASH_SOURCE[0]:-$0})"

#* project's relative path with respect to this script
PROJECT_PATH=".."
TOWR_PATH="../../solo12_catkin_ws/src/towr_solo12/towr"


mkdir -p ~/.ros/data/heightfields/from_pybullet
rsync -a $PROJECT_PATH/data/heightfields/from_pybullet/towr_heightfield.txt  ~/.ros/data/heightfields/from_pybullet/ --delete
rsync -a $PROJECT_PATH/data/heightfields/from_pybullet/towr_heightfield.txt  $TOWR_PATH/data/heightfields/from_pybullet/ --delete