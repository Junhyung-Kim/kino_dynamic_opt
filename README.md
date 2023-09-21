# Kino-dynamic Trajectory Optimization for Multiped Robots

This software provides methods for kino-dynamic optimization for multiped robots.
It contains a list of solvers and optimization problem for robotics optimal control.

## Install required packages

Please make sure to install the following packages:

- the qp solver quadprog, the code manager treep and the python resource manager:
```
pip install lightargs==1.22 quadprog treep importlib_resources
```
- yaml-cpp for reading the parameters of the optimal control problem:
  - ubuntu:
  ```
  sudo apt install libyaml-cpp-dev
  ```
  - MacOs
  ```
  brew install yaml-cpp
  ```

- And colcon to build packages (more info about the build system https://design.ros2.org/articles/build_tool.html)
  - Ubuntu
  ```
  sudo apt install python3-colcon-common-extensions
  ```
  - MacOs
  ```
  python3 -m pip install colcon-common-extensions
  ```

## Getting started

Clone the repository in the desired work folder <work_folder>
```
mkdir devel
cd devel
git clone https://github.com/machines-in-motion/treep_machines_in_motion.git
treep --clone KINO_DYN_PLANNER
```
This operation should have cloned pacakges in `workspace/src/` which will be
refered as the <work_folder>\
Compile the code, by running the following commands in the <work_folder>
```
cd workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Once the code has been compiled, you can source the setup.bash file in
`install/setup.bash`
```
. install/setup.bash
```

## Running a demo
```
cd <work_folder>/kino_dynamic_opt/momentumopt/demos
python3 ../nodes/kino_dyn_planner_solo -i <path_to_config_file>
```
For example plan and execute a jumping motion with
```
python3 ../nodes/kino_dyn_planner -i ../config/cfg_solo8_jump.yaml
python3 ../nodes/kino_dyn_planner -i ../config/cfg_solo12_jump.yaml --solo12
python3 ../nodes/kino_dyn_planner -i ../config/cfg_bolt_jump.yaml --bolt
python3 ../nodes/kino_dyn_planner -i ../config/cfg_tocabi_walk.yaml --walk
```

#### Configuration overview
Different configuration files are available in
```
<work_folder>/kino_dynamic_opt/momentumopt/config
```
The available motions are:

* cfg_solo12_jump.yaml
* cfg_bolt_jump.yaml
* cfg_solo8_jump.yaml
* cfg_solo8_fast_short_trot.yaml
* cfg_solo8_trot.yaml
* cfg_tocabi_walk.yaml

For an explanation of the different settings in the configuration files, refer to [cfg_solo_jump.yaml].

#### Select your desired inverse kinematics formulation
We have two different inverse kinematics formulations implemented, you can choose
which one you would like to use. The default version is a first order inverse kinematics
that optimizes for generalized velocities. If you wanna use second order inverse
kinematics, you would need to change the boolean `use_second_order_inv_kin` to `True`
in your config (.yaml) file.
