# TE3003B-puzzlebot-slam

This repository contains the code for using a physical Puzzlebot to implement SLAM. 

This repository uses code from [Equipo1IRSTecCCM/TE3003B-M3-RobotsAutonomos](https://github.com/Equipo1IRSTecCCM/TE3003B-M3-RobotsAutonomos) and [Equipo1IRSTecCCM/TE3003B-Manchester](https://github.com/Equipo1IRSTecCCM/TE3003B-Manchester)


To run this repository you have to use a puzzlebot jetson lidar edition.

To run SLAM with just one robot, you'll need to follow the instructions shown in the [README_lidar.md](https://github.com/DiegoRR00/TE3003B-puzzlebot-slam/blob/main/catkin_ws/README_lidar.md).

After that you need to run the launch file [slam.launch](https://github.com/DiegoRR00/TE3003B-puzzlebot-slam/blob/main/catkin_ws/src/puzzlebot_slam/launch/slam.launch) with the command:

    roslaunch puzzlebot_slam slam.launch

For running multiple robots, you have to launch

    roslaunch puzzlebot_slam coolab_slam.launch

For this project it's important to use the following IPs to simplify our work (Work in progress):

| Device        | IP            | Namespace |
|---------------|---------------|-----------|
| puzzlebot-master   | 192.168.0.117 | --        |
| hernan        | 192.168.0.114 | he        |
| frank-bot     | 192.168.0.120 | fb        |
| moto-mami     | 192.168.0.123 | mm        |

