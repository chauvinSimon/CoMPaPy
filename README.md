# `CoMPaPy` :robot: :panda_face: * :snake:

:speaking_head: (:fr:): [kɔm papi] :older_man:

`Co`ntrol with `M`oveIt of `P`anda robot in `Py`thon

**a simple python interface for basic control of a real [panda robot](https://www.franka.de/)
with [moveit](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)**

- using only official and maintained repos: [`franka_ros`](https://frankaemika.github.io/docs/franka_ros.html)
  and [`MoveIt`](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
- both in `rviz` and on real robot
- no `cpp`, just `python`

**Keywords:** `moveit!`, `franka_emika`, `franka_ros`, `libfranka`, `panda`, `fr3`, `ROS`, `python`

### :warning: limitations

- only tested on the following combination
    - (`ubuntu 20`, `noetic`, `fr3`, `libfranka 0.10.0`, `franka_ros 0.10.1`)
- only basic actions
    - `move_j`
    - `move_l`
    - `open_gripper`
    - `close_gripper`
- some parameter tuning may be required
    - e.g. for the cartesian plan in `move_l`
- control of `panda_link8` instead of the gripper
    - todo: `https://answers.ros.org/question/334902/moveit-control-gripper-instead-of-panda_link8-eff/`

## :wrench: installation

note: installing `libfranka` and `franka_ros` with `sudo apt install ros-noetic- ...` is not an option at the time of
writing: it gives incompatible versions

### :dart: expected structure

<details>
  <summary>click to expand</summary>

```
tree ~/catkin_ws/src

.
├── compapy
│     ├── config
│     ├── launch
│     └── scripts
├── franka_ros
│     ├── cmake
│     ├── franka_control
│     ├── franka_description
│     ├── franka_example_controllers
│     ├── franka_gazebo
│     ├── franka_gripper
│     ├── franka_hw
│     ├── franka_msgs
│     ├── franka_ros
│     └── franka_visualization
├── geometric_shapes
│     ├── cmake
│     ├── include
│     ├── src
│     └── test
├── moveit
│     ├── moveit
│     ├── moveit_commander
│     ├── moveit_core
│     ├── moveit_experimental
│     ├── moveit_kinematics
│     ├── moveit_planners
│     ├── moveit_plugins
│     ├── moveit_ros
│     ├── moveit_runtime
│     └── moveit_setup_assistant
├── moveit_msgs
│     ├── action
│     ├── dox
│     ├── msg
│     └── srv
├── moveit_resources
│     ├── dual_panda_moveit_config
│     ├── fanuc_description
│     ├── fanuc_moveit_config
│     ├── moveit_resources
│     ├── panda_description
│     ├── panda_moveit_config
│     ├── pr2_description
│     ├── prbt_ikfast_manipulator_plugin
│     ├── prbt_moveit_config
│     ├── prbt_pg70_support
│     └── prbt_support
├── moveit_tutorials
│     ├── doc
│     ├── _scripts
│     ├── _static
│     └── _themes
├── moveit_visual_tools
│     ├── include
│     ├── launch
│     ├── resources
│     └── src
├── panda_moveit_config
│     ├── config
│     └── launch
├── rviz_visual_tools
│     ├── icons
│     ├── include
│     ├── launch
│     ├── resources
│     ├── src
│     └── tests
├── srdfdom
│     ├── include
│     ├── scripts
│     ├── src
│     └── test
└── ven
    ├── bin
    ├── include
    ├── lib
    ├── lib64 -> lib
    └── share
```
</details>


### :pick: building from Source

only tested with these versions ([source](https://frankaemika.github.io/docs/compatibility.html)):

| Robot system version | libfranka version | franka_ros version | Ubuntu / ROS    |
| -------------------- | ----------------- | ------------------ | ----------------|
| > = 5.2.0 (FR3)       | > = 0.10.0         | > = 0.10.0          | 20.04 / noetic  |

#### `libfranka`

following [these build instructions](https://frankaemika.github.io/docs/installation_linux.html#building-libfranka)

```
cd build/
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
cpack -G DEB
sudo dpkg -i libfranka*.deb
```

#### :package: ROS packages

##### :panda_face: `franka_ros`

follow [these build instructions](https://frankaemika.github.io/docs/installation_linux.html#building-the-ros-packages)

- prefer `catkin build` to `catkin_make`

```
catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```

follow [these instructions](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
to set up the real-time kernel

- check with `uname -r`

##### :cartwheeling: `moveit_panda_config`

follow [these build instructions](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)

- prefer `~/catkin_ws/src` to `~/ws_moveit/src`

#### :performing_arts: `venv`

```
cd ~/catkin_ws/src
python3 -m venv ven --system-site-packages
source ven/bin/activate
~/Downloads/pycharm-2021.3/bin/pycharm.sh
```

#### :pencil2: `~/.bashrc`

I find convenient to add this alias

```
alias source_catkin="source ~/catkin_ws/devel/setup.bash; deactivate; source ~/catkin_ws/src/ven/bin/activate; cd ~/catkin_ws/src"
```

#### :bug: troubleshooting

> "Unable to find either executable 'empy' or Python module 'em'..."

- `(ven) ~/catkin_ws$ catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3`

### :writing_hand: PyCharm

follow [this great video](`https://www.youtube.com/watch?v=lTew9mbXrAs`)

find `pycharm.sh` with

```
sudo apt install locate
sudo updatedb
locate pycharm.sh
```

## :checkered_flag: usage

### rviz only

todo

### :robot: real robot

todo

## :+1: acknowledgements

the following resources helped me understand how to control the robot

- todo