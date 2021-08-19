# ROS-planning navigation RRT rrt* 双向RRT 剪枝

## navigation package
https://github.com/ros-planning/navigation

- 在ros-planning模块的navigation包的基础上，添加了各类RRT路径规划功能。
- 使用时需将该package放在workspace下，运行程序时应优先调用该目录下程序，而非apt install的navigation库（默认在/opt下）

## nav_sim
https://github.com/ZJUYH/nav_sim

- navigation模块实现的demo
- 与navigation放在同一workspace/src下

~~~
{your_workspace}$catkin_make
{your_workspace}$source devel/setup.bash
{your_workspace}$roslaunch nav_sim myrobot_world.launch
{your_workspace}$roslaunch nav_sim move_base.launch
~~~
在rviz中使用2D Nav Goal设置目标点，自动进行路径规划