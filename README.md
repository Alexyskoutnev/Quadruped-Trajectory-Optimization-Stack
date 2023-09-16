# Quadruped Trajectory Optimization Stack
[Alexy Skoutnev](https://alexyskoutnev.github.io/alexyskoutnev-github.io/index.html), [Andrew Cinfal](https://github.com/cinaral), [Praful Sigdel](https://praful22.github.io/), [Forrest Laine](https://github.com/forrestlaine)

[Project](https://alexyskoutnev.github.io/Quadruped-Trajectory-Optimization-Stack/) | [arXiv](https://arxiv.org/)

## Introduction

Robotic trajectory simulation package built on top of Pybullet. Used for testing trajectories (TOWR) and dynamics between robot model and enviroment  (SOLO12).

If you find our work useful in your research, please consider [citing](#citing).

## Dependencies

- Python 3.10 (recommended)
- [PyBullet](https://github.com/bulletphysics/bullet3/)
- [Towr](https://github.com/ethz-adrl/towr)
- [Docker](https://www.docker.com/)

## Installation and Environment Setup
To help keep all the packages together in one enviroment, please create conda enviroment.

```console
conda create -n QTOS python=3.10
conda activate QTOS
pip3 install -r requirements.txt
pip3 install -e .
```
Installing Docker [Ubuntu].
```console
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo docker run hello-world
```
Installing Docker [macOS]
```console
sudo hdiutil attach Docker.dmg
sudo /Volumes/Docker/Docker.app/Contents/MacOS/install
sudo hdiutil detach /Volumes/Docker
```

## Docker Installation
```console
DOCKER_BUILDKIT=1 docker build --no-cache -t towr -f Dockerfile .
docker run -d towr
```
Now you should a Docker image named towr which will be used in the towr_run.py script to strat the Towr MPC loop.
To check that the container was built correctly, use the follwing command (should output that the container is a ancestor of the towr image).
```console
docker ps -f ancestor=towr
```
To build a towr enviroment with a GUI, you would need to build anthor image named, Dockerfile_GUI. However this image is slow and should be only used to debug towr source code/binaries. 
```console
DOCKER_BUILDKIT=1 docker build --no-cache -t towr-gui -f Dockerfile_GUI .
docker run -p 6080:80 -v /dev/shm:/dev/shm towr-gui
```
Now open a web browser and type in,
```
http://localhost:6080/
```
To launch a visual towr interface in the Towr Docker GUI, use the commands
```console
cd ~/catkin_ws
source /opt/ros/melodic/setup.sh
/opt/ros/melodic/setup.sh >> ~/.bashrc
source ~/.bashrc
catkin_make_isolated -DCMAKE_BUILD_TYPE=Release -j4
source devel_isolated/setup.sh
roslaunch towr_ros towr_ros.launch
```

# Usage

## Running Basic Simulation
Make sure you run all script from the home direction (~/SOLO12_SIM_CONTROL), to run the simulator use run.py. The configuration files for this scripts are ~/data/config/solo12.yml and ~/data/config/simulation.yml. Make all the neccesary changes in the .yml files to configure starting state of the simulation and robot.
```
cd ~/SOLO12_SIM_CONTROL
python3 scripts/run.py

```

## Running MPC Simulation 
Make sure you run all script from the home direction (~/SOLO12_SIM_CONTROL), to run the MPC simulation use towr_run.py. The configuration files for this scripts are ~/data/config/solo12.yml and ~/data/config/simulation.yml. Make all the neccesary changes in the .yml files to configure starting state of the simulation and robot.
```
cd ~/SOLO12_SIM_CONTROL
python3 scripts/towr_run.py
```

## Running Debugger 
Make sure you run all script from the home direction (~/SOLO12_SIM_CONTROL), to run the MPC simulation use towr_run.py. The configuration files for this scripts are ~/data/config/solo12_debug.yml and ~/data/config/debug.yml. Make all the neccesary changes in the .yml files to configure starting state of the simulation and robot.
```
cd ~/SOLO12_SIM_CONTROL
python3 scripts/debug.py
```


## Citing
```
@inproceedings{skoutnev2023qtos,
   title={Quadruped Trajectory Optimization Stack},
   author={Skoutnev, Alexy and Cinral, Andrew and Sigdel, Praful and Laine, Forrest},
   archivePrefix={arXiv}
   primaryClass={cs.RO}
   year={2023}
}
```