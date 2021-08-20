# ROS-planning navigation RRT rrt* 双向RRT 剪枝

## navigation package
https://github.com/ros-planning/navigation

- ros原版的navigation包中，global_planner仅给出了A*，dijkstra等功能
- 本项目在ros-planning模块的navigation包的基础上，对接原装接口，仿照`global_planner`下的`astar.cpp`/`dijkstra.cpp`添加了各类RRT路径规划功能`rrt.cpp`。
- 使用不同rrt算法可通过`nav_sim/cfg/global_planner_params.yaml`更改参数设置。如需在其他项目中使用，请自行添加参数文件。
- 参数说明：
    - use_connect 双向rrt
    - use_rrt_star rrt*
    - use_cut_bridge 进行rrt剪枝，此参数可单独配置
    - use_goal_guild 进行目标导向，使用该参数将使得树以一定概率直接向终点延伸
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