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
conda install pybullet
pip3 install -e .
```
