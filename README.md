# SliceIt! - A Dual Simulator Framework for Learning Robot Food Slicing

A real2sim2real approach to learning robotic food slicing tasks with rigid robot manipulators.

Checkout our [paper](https://arxiv.org/abs/2404.02569) and our [project page](https://omron-sinicx.github.io/sliceit/).

## Installation

#### Prerequisites
- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

### Install & Build
1. Download this repository and set up its submodules
``` sh
git clone https://github.com/omron-sinicx/sliceit.git
cd sliceit
git submodule update --init --recursive
``` 
2. Build docker image
``` sh
./BUILD-DOCKER-IMAGE.sh
```
3. Create and run docker container
``` sh
./RUN-DOCKER-CONTAINER.sh
```
4. Build ROS environment inside container
``` sh
o2ac-build-catkin-workspace
``` 
5. Install DiSECt as instructed in [disect/README.md](disect/README.md) including downloading the dataset
```shell
pip install -r disect/requirements.txt
pip install -e /root/o2ac-ur/disect/meshing
pip install -e /root/o2ac-ur/disect
pip install -e /root/o2ac-ur/underlay_ws/src/ur_python_utilities/dependencies/tf2rl
```

## Demo

### Gazebo Only
- Visualization (simple env)
``` sh
roslaunch o2ac_gazebo b_bot_gazebo.launch
rosrun ur3e_rl simple_env.py -e 1
```

### Gazebo + DiSECt
- Visualization (simple env)
``` sh
roslaunch o2ac_gazebo b_bot_gazebo.launch
cd ~/o2ac-ur/disect && python disect/cutting/ros_visualizer.py
rosrun ur3e_rl simple_env.py -e 3
```
- Policy learning
``` sh
rosrun ur3e_rl tf2rl_sac.py -e 3
```

Authors:
- Cristian C. Beltran-Hernandez
- Nicolas Erbetti
- Masashi Hamaya

This repository is based on [Team O2AC repository](https://github.com/o2ac/o2ac-ur)

```
@article{beltran2024sliceit,
  title={SliceIt!--A Dual Simulator Framework for Learning Robot Food Slicing},
  author={Beltran-Hernandez, Cristian C and Erbetti, Nicolas and Hamaya, Masashi},
  journal={arXiv preprint arXiv:2404.02569},
  year={2024}
```
