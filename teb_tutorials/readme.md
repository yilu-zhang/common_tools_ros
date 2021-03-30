环境：Ubuntu 18.04 + ROS melodic
```bash
$ mkdir -p ~/teb_ws/src
$ cd ~/teb_ws/src
$ catkin_init_workspace
$ git clone https://github.com/rst-tu-dortmund/teb_local_planner.git
$ git clone https://github.com/rst-tu-dortmund/teb_local_planner_tutorials.git
$ git clone https://github.com/rst-tu-dortmund/costmap_converter.git
$ git clone https://github.com/AMRobots/stage_ros.git
$ cd ~/teb_ws
$ catkin_make
$ source devel/setup.bash
# 检查是否正确安装
$ roslaunch teb_local_planner_tutorials dyn_obst_corridor_scenario.launch
```
然后将teb_tutorials文件夹中文件拷到~/teb_ws/src/teb_local_planner_tutorials对应文件夹下
dyn_obst_corridor_scenario_skew.launch -> launch
corridor_skew.yaml -> maps
move_obstacle.py - > scripts (替换原文件)
move_obstacle_skew_l.py - > scripts
move_obstacle_skew_r.py - > scripts
corridor_skew.world -> stage

```bash
$ roslaunch teb_local_planner_tutorials dyn_obst_corridor_scenario_skew.launch
```

