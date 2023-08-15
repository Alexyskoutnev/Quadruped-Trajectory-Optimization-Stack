Robotic trajectory simulation package built on top of Pybullet. Used for testing trajectories (TOWR) and dynamics between robot model and enviroment  (SOLO12).
# Docker Installation Steps #
```console
DOCKER_BUILDKIT=1 docker build --no-cache -t towr -f Dockerfile
docker run -it towr
```
Now you should a Docker image named towr which will be used in the towr_run.py script to strat the Towr MPC loop.
To check that the container was built correctly, use the follwing command (should output that the container is a ancestor of the towr image).
```console
docker ps -f ancestor=towr
```
To build a towr enviroment with a GUI, you would need to build anthor image named, Dockerfile_GUI. However this image is slow and should be only used to debug towr source code/binaries. 
```console
DOCKER_BUILDKIT=1 docker build --no-cache -t towr-gui -f Dockerfile_GUI
docker run -p 6080:80 -b /dev/shm:/dev/shem towr-gui
```
Now open a web browser and type in,
```
http://localhost:6080/
```
# Python Installation Steps #
```
conda create --name soloSim python=3.10
conda activate soloSim
conda install -c conda-forge pybullet
pip3 install -e .
conda install pyyaml
conda install scipy
conda install matplotlib
```
# Running Basic Simulation #
Make sure you run all script from the home direction (~/SOLO12_SIM_CONTROL), to run the simulator use run.py. The configuration files for this scripts are ~/data/config/solo12.yml and ~/data/config/simulation.yml. Make all the neccesary changes in the .yml files to configure starting state of the simulation and robot.
```
cd ~/SOLO12_SIM_CONTROL
python3 scripts/run.py

```

# Running MPC Simulation #
Make sure you run all script from the home direction (~/SOLO12_SIM_CONTROL), to run the MPC simulation use towr_run.py. The configuration files for this scripts are ~/data/config/solo12.yml and ~/data/config/simulation.yml. Make all the neccesary changes in the .yml files to configure starting state of the simulation and robot.
```
cd ~/SOLO12_SIM_CONTROL
python3 scripts/towr_run.py
```

# Running Debugger #
Make sure you run all script from the home direction (~/SOLO12_SIM_CONTROL), to run the MPC simulation use towr_run.py. The configuration files for this scripts are ~/data/config/solo12_debug.yml and ~/data/config/debug.yml. Make all the neccesary changes in the .yml files to configure starting state of the simulation and robot.
```
cd ~/SOLO12_SIM_CONTROL
python3 scripts/debug.py
```