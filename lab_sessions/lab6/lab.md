---
layout: page
title: Lab 6 - ROS Navigation (Recap & Costmap)
permalink: /lab_sessions/lab6
parent: Lab Sessions
nav_order: 6
---
**Lab 6 submission due on 18<sup>th</sup> June 2022, 23:59**{: .label .label-red }

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
4. [ROS Navigation - Map_Server](http://wiki.ros.org/map_server)
5. [A* Algorithm](https://brilliant.org/wiki/a-star-search/)
6. [Dijkstra's Algorithm](https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/)

## Materials

----

# Setup
* Be in your teams of 5
* Tasks & report should be performed by all **group members individually** unless told otherwise.

## Lab Report and Submission
* Throughout this lab, there are tasks that you are supposed to perform and record observations/deductions.
* You can share common experimental data, but not explanations, code or deductions for the lab report.
* Discrepancies between report results and code submissions are liable for loss of marks.
* Each task will be clearly labelled and will need to be included in your lab report, which is in the format "**lab6\_report\_team\_<TEAM\_ID>.doc / pdf**", include your name, student_id at the beginning of the report.
* Zip up your lab report and other requirements (if present) and name it "**lab6\_team\_<TEAM\_ID>.zip**" and upload it.

## Learning Outcomes
By the end of lab 6, you will have:
1. Recapped what you have learnt over the last 4 labs.
2. Navigated your limo robot through an real-life environment
3. Learnt how to manipulate the planners' costmap by editing the map.


----

# Lab 6 (4%)

Today's lab will be done as a team, therefore each team will only have to create 1 report for this lab. However, each member will still need to submit that same report **individually** on the submission page.

## Recap
Over the past few labs, you have mapped and navigated through a virtual gazebo environment. Today we will attempt to do the same in a real-life environment.

### **Task 1: Mapping of a real-life environment**{: .label .label-green}
For this task, each team is tasked to use the limo robot to map a section of your classroom(or any area of your choosing). The section and size of your map is up to each team, but try to map an area big enough for the limo to move and navigate through. Since the area might be chaotic, the generated map do not have to be very clean. We can fix that further down this lab.

* **Task 1a**{: .label .label-blue}Include the generated map and pictures of the real-life space with your report.

### **Task 2: Navigation of a real-life environment**{: .label .label-green}
Using the map you have generated and the limo_navigator node you have made in lab 5 task 2 (You can choose among your members), navigate the limo through 4 different way-points through your environment by running the navigator node on your computer.

* **Task 2a**{: .label .label-blue}Pick 4 way-points and list these in your report.
* **Task 2b**{: .label .label-blue}Take a video of your limo navigating through these 4 way-points and a screen recording of the rviz window showing the navigation. Include these 2 videos with your submissions, name them as task2_limo and task2_screen.


## Map manipulation
### **Task 3: (Cost)Map Manipulation**{: .label .label-green}
There are many reason why we would have to manipulate a map, from a cleaning up a "messy" map to "dis-allowing" movement within a certain space.

The map uses primarily uses 3 colours to denote different types of "spaces" or "cells"...
  1. White denotes "free space" where the robot can freely move in.
  2. Black denotes "occupied space" or "obstacles" where the robot cannot move in.
  3. Gray denotes "no data" meaning we do not know what is in that space.

By editing the map (.pgm) file, we can indirectly influence the global/local planners' *static costmaps*. This in turn affects how these planners plan and navigate through the environment.

* **Task 3a**{: .label .label-blue}Using a image editing software, remove any obstacles in your map that should not be there (these includes things like obstacles that are no longer there, people's feet etc...). Include this new map with your report, name it as cleaned_map.pgm.
  * You can use any editing software you are familiar with, a software that can be apt installed and relatively easy to use on ubuntu is gimp.
* **Task 3b**{: .label .label-blue}Between any 2 consecutive way-points chosen in task 2a, draw a wall obstacle that will cause the limo to navigate around this wall. Include this new map with your report, name it as added_obstacle_map.pgm.
* **Task 3c**{: .label .label-blue}Rerun task 2b with the new map in task 3b (You might need to rename the new map to the original name to run it.). Include these 2 videos with your submissions, name them as task3_limo and task3_screen.


## Submission
Zip up your lab report into a zip file called "**lab6\_team\_<TEAM\_ID>.zip**" and submit by 18<sup>th</sup> June 2022, 23:59.