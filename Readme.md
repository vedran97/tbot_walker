# tbot_walker

This package demoes use of tbot as a roomba-like device<br>
This package was built and tested for ros2-humble on an ubuntu 22.04 distro.<br>
The model of turtlebot3 is waffle_pi

## Install process

1. Clone this repo in a ros2 workspace's src folder : ```git clone https://github.com/vedran97/tbot_walker.git```
2. Install dependencies by running ```rosdep install -i --from-path src --rosdistro humble -y``` in the root of your workspace.

## Ensure you have necessary packages installed

1. ```sudo apt -y install ros-humble-desktop-full```
2. ```sudo apt -y install ros-humble-gazebo-ros-pkgs```
3. ```sudo apt -y install ros-humble-turtlebot3*```

## Building process

1. Build the workspace:```colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers $(nproc)```
2. Source the workspace: ```source install/setup.bash```

## Instructions to run the cpptools

```bash
# run clang-format from workspace root

  cd src/tbot_walker && clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^(./build/|./install/|./log/)") && cd -

# run cppcheck from inside the pkg directory
  mkdir results -p && cppcheck --enable=all --std=c++17 -I include/ --suppress=missingInclude --inline-suppr $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cppcheck

# run cpplint from inside the pkg directory

  mkdir results -p && cpplint  --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cpplint

```

## Run code

0. Build the package,then source the package in 2 terminals ```source install/setup.bash```
1. run this in 1 terminal ```export TURTLEBOT3_MODEL=waffle_pi && ros2 launch tbot_walker spawn_tbot.py``` to spawn the robot
2. run this or cmd in #3 in another terminal ```ros2 launch tbot_walker walker.py``` to just run the walker node without rosbag record // note : Run this only after the robot spawns in gazebo and simulation starts running
3. ```ros2 launch tbot_walker walker.py rosbag_record:=true``` to launch walker node and record all topics except camera and scan. // note : Run this only after the robot spawns in gazebo and simulation starts running

## Results and replaying the ros bag

1. Results are kept inside the results folder.
2. cpplint and cppcheck results can be found there
3. Recorded bag for a run of the walker is stored inside results/walker_bag folder
4. Inspecting the ros bag ```ros2 bag info src/tbot_walker/results/walker_bag/walker_bag_0.db3``` from root of the workspace
5. Playing back the ros bag ```ros2 bag play src/tbot_walker/results/walker_bag/walker_bag_0.db3```.

## Testability

1. This package already depends on gtest, and exports the walker node as a linkable "walker_lib" with a dedicated header file. This makes it test ready.
