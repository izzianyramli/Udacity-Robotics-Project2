# Udacity-Robotics-Project2
Udacity Robotics Software Engineer Nanodegree - Project 2: Go Chase It!

![Alt test] (GoChaseIt.png)


## Build and Launch

1. Clone and initialize project with a catkin workspace
```
$ mkdir ~/catkin_ws && cd ~/catkin_ws
$ git clone https://github.com/izzianyramli/Udacity-Robotics-Project2.git
$ mv Udacity-Robotics-Project2 src
$ cd src && catkin_init_workspace
```

2. Go to `catkin_ws/` and build
```
$ cd ~/catkin_ws
$ catkin_make
```

3. Launch the robot world
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

4. Open another terminal, and launch the `ball_chaser`
```
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```

5. Pick up the white ball and place in front of the mobile robot.

   The robot will follow the ball.
