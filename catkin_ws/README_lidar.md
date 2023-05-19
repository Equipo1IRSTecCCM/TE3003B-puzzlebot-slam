# RUN LIDAR IN THE FUCKING PUZZLEBOT JETSON NANO EDITION

## STEP 1

Create a .launch file inside the puzzlebot_autostart node inside your puzzlebot.

> File name: puzzlebot.launch

> Contents:

    <launch>
        <include file="$(find rplidar_ros)/launch/rplidar.launch"/>
        <node name="serial_node" pkg="rosserial_python" type="serial_node.py">
            <param name="port" value="/dev/ttyUSB1"/>
        </node>
    </launch>

> Assumptions: The MCR_1204 board it's in the /dev/ttyUSB1 port and the RPLIDAR A1 it's in the /dev/ttyUSB0 port

## STEP 2

Connect your computer to the same Wi-Fi network as the puzzlebot. We'll assume the following conditions:

> Master IP (your computer) = 10.42.0.84

> Puzzlebot IP = 10.42.0.1

## STEP 3

Connect through an SSH connection to the puzzlebot by typing the command:

    ssh puzzlebot@10.42.0.1

Type the password and the type the following commands:

    export ROS_MASTER_URI=http://10.42.0.84:11311

    export ROS_IP=10.42.0.1

## STEP 4

On a terminal in your computer run the following commands:

    export ROS_MASTER_URI=http://10.42.0.84:11311

    export ROS_IP=10.42.0.84

    roscore

This way your computer is running the roscore and the puzzlebot is expecting to find the master in your computer's IP

## STEP 5

To view the data in RVIZ, you have to run the following command in a new terminal:

    export ROS_MASTER_URI=http://10.42.0.84:11311

    export ROS_IP=10.42.0.84

    rosrun rviz rviz

Open a new terminal and run:

    export ROS_MASTER_URI=http://10.42.0.84:11311

    export ROS_IP=10.42.0.84

    rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map laser


This way you'll be able to see in RVIZ the LaserScan message displayed

If you want to run any further commands in a new terminal you have to run the followings command:
 
    export ROS_MASTER_URI=http://10.42.0.84:11311

    export ROS_IP=10.42.0.84    # This IP is the one of the device you're using, for this example, this command is meant to be run on the computer


## STEP 6

Run the following command in the terminal connected trhough SSH to the puzzlebot

    roslaunch puzzlebot_autostart puzzlebot.launch
