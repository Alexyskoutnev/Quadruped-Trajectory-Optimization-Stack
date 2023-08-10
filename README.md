Robotic trajectory simulation package built on top of Pybullet. Used for testing trajectories (TOWR) and dynamics between robot model and enviroment  (SOLO12).
# Docker Installation Steps #
```console
docker build -t solo12 .
docker run -it solo12
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
python3 scripts/towr_run.py
```