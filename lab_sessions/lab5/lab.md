---
layout: page
title: Lab 5 - Actionlib & ROS Navigation
permalink: /lab_sessions/lab5
parent: Lab Sessions
nav_order: 5
---
**Lab 5 submission due on 11<sup>th</sup> June 2022, 23:59**{: .label .label-red }

## Table of contents
{: .no_toc .text-delta }

- TOC
{:toc}

# Prelab (1%)

## Start of Lab
1. We will have a short MCQ quiz on concepts that have been covered in the lecture and those that will be needed during this lab session, concepts covered will be from the readings found below.

## Readings
1. [ROS Navigation - Setup](http://wiki.ros.org/navigation/Tutorials/RobotSetup)
2. [ROS Navigation - AMCL](http://wiki.ros.org/amcl)
3. [ROS Navigation - Move_base](http://wiki.ros.org/move_base)
4. [ActionLib - Python Tutorials](http://wiki.ros.org/actionlib_tutorials/Tutorials)

## Materials

----

# Setup
* Be in your teams of 5
* Tasks & report should be performed by all **group members individually** unless told otherwise.

## Lab Report and Submission
* Throughout this lab, there are tasks that you are supposed to perform and record observations/deductions.
* You can share common experimental data, but not explanations, code or deductions for the lab report.
* Discrepancies between report results and code submissions are liable for loss of marks.
* Each task will be clearly labelled and will need to be included in your lab report, which is in the format "**lab5\_report\_<STUDENT\_ID>.doc / pdf**", include your name, student_id at the begining of the report.
* Zip up your lab report and other requirements (if present) and name it "**lab5\_<STUDENT\_ID>.zip**" and upload it.

## Learning Outcomes
By the end of lab 5, you will have:
1. learnt about action servers/clients
2. learnt how to navigate using amcl/move_base

----

# Lab 5 (5%) 

## Single Waypoint Navigation
### **Task 1: Navigation using amcl & move_base**{: .label .label-green}
Basic navigation using ROS navigation stack in created Gazebo world.

![overview_tf_small](assets/overview_tf_small.png)

1. Load Gazebo virtual environment and map
    1. Launch Gazebo simulation
        
        ```bash
        roslaunch limo_gazebo_sim limo_four_diff.launch
        ```

    2. In another terminal, load map using map_server ros package

        ```bash
        rosrun map_server map_server -f <path-to-map>
        ``` 
2. Launch navigation
    1. In another terminal, launch navigation launch file 

        ```bash
        roslaunch limo_bringup limo_navigation_diff.launch
        ```

    2. LaserScan in RViz  will not display anything as it is not subscribing to the correct topic. Change to "/limo/scan"

     ![Laserscan](assets/Laserscan.png)

3. Initial pose estimate and single point navigation using RViz
    1. At the top of RViz window, click "2D Pose Estimate". 
    2. On the map, click and hold left mouse click at estimated location of Limo. 
    3. At the top of RViz window, click "2D Nav Goal". 
    4. On the map, click and hold left mouse click at targeted location. 

**Task 1a**{: .label .label-blue}What topics do "2D Pose Estimate" and "2D Nav Goal" publish to. Are the message types the same?

**Task 1b**{: .label .label-blue}Briefly explain why there are no visuals displayed when LaserScan topic is set to /scan. 

**Task 1c**{: .label .label-blue}You will see 3 Map topics on the left window pane in the RViz, yet you only loaded one map, briefly describe what they are. 

**Task 1d**{: .label .label-blue}Similarly, you will see 2 Path topics on the left window pane in the RViz, yet the robot only follows one path, briefly describe what they are. 

**Optional Task**{: .label .label-blue}Instead of typing out the command to load the map each time, we have multiple ways to streamline this task. One such method is the roslaunch feature. Edit the navigation launch file to run the map server node. 

## Action Servers/Client
### **Task 2: Sending a Goal to move_base action server**{: .label .label-green}

## Multi Waypoint Navigation
### **Task 3: Navigation using a node**{: .label .label-green}


## Submission
Zip up your lab report into a zip file called "**lab5\_<STUDENT\_ID>.zip**" and submit by 11<sup>th</sup> June 2022, 23:59.