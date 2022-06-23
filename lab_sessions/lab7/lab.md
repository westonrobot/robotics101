---
layout: page
title: Lab 7 - Introduction to computer vision with OpenCV
permalink: /lab_sessions/lab7
parent: Lab Sessions
nav_order: 7
---
**Lab 7 submission due on 2<sup>nd</sup> July 2022, 23:59**{: .label .label-red }

## Table of contents
{: .no_toc .text-delta }

- TOC
{:toc}

# Prelab (1%)

## Start of Lab
1. We will have a short MCQ quiz on concepts that have been covered in the lecture and those that will be needed during this lab session, concepts covered will be from the readings found below.

## Readings
1. [LIMO Guide - Depth Camera + LiDAR](https://github.com/agilexrobotics/limo-doc/blob/master/Limo%20user%20manual(EN).md#7-depth-camera--lidar-mapping)
2. [ROS - rosparam](http://wiki.ros.org/rosparam)

## Materials
1. [Colour Test Image]({{ site.baseurl }}{% link lab_sessions/lab7/assets/colour_test_img.png %})
2. [Colour Venn Diagram]({{ site.baseurl }}{% link lab_sessions/lab7/assets/colour_venn_diagram.jpg %})
3. [Camera Adapter Mount Stp File]({{ site.baseurl }}{% link lab_sessions/lab7/assets/colour_test_img.png %})

----

# Setup
* Be in your teams of 5
* Tasks & report should be performed by all **group members individually** unless told otherwise.

## Lab Report and Submission
* Throughout this lab, there are tasks that you are supposed to perform and record observations/deductions.
* You can share common experimental data, but not explanations, code or deductions for the lab report.
* Discrepancies between report results and code submissions are liable for loss of marks.
* Each task will be clearly labelled and will need to be included in your lab report, which is in the format "**lab7\_report\_<STUDENT\_ID>.doc / pdf**", include your name, student_id at the beginning of the report.
* Zip up your lab report and other requirements (if present) and name it "**lab7\_report\_<STUDENT\_ID>.doc / pdf**" and upload it.

## Learning Outcomes
By the end of lab 7, you will have:
1. Learnt how to start and subscribe to a video stream from limo's camera
2. Manipulate and analyze images using OpenCV

----

# Lab 7 (4%)
Before starting with any of the task below, set your network so that each student's computer can communicate with your LIMO through a local network. (Refer to lab 2 task 2)

## Working with Images + ROS
Each of your limo is equipped with a forward facing camera mounted on its "head". The ROS driver (astra_camera) needed to run this camera has already been loaded onto the "~/agilex_ws" catkin workspace on the jetson nano's home folder.

### **Task 1: Starting the camera stream**{: .label .label-green}
For this task, you will need to utilise the image stream from your LIMO's camera. The camera's ROS driver mentioned above publishes the different types of "images" from the camera over the ROS network. We are mainly interested in the RGB image stream.

* **Task 1a**{: .label .label-blue}Source the agilex_ws setup.bash and run the camera driver using the command below, include an image of the resultant ROS network from your computer in your report.
  ```bash
  roslaunch astra_camera dabai_u3.launch
  ```
* **Task 1b**{: .label .label-blue}When running, this driver dynamically creates parameters that are exposed to the ROS network.
  * What parameters are used to state the height and width of the image captured?
  * We can change these parameters using 2 methods, state these 2 methods.
* **Task 1c**{: .label .label-blue}Out of all the different image streams published we are mainly interested in 2; "/camera/rgb/image_raw" & "/camera/rgb/image_raw/compressed". What are the data formats used to publish these 2 topics?

### **Task 2: Utilising the camera stream**{: .label .label-green}
For this task, you will need to write a node in its own package that subscribes and displays the "/camera/rgb/image_raw" image stream in its own window.

#### **Node description**
* Your package should be called "limo_pov"
* Your node should be called "limo_pov_node"
* Your node should follow the behaviour described below
   * Subscribe to "/camera/rgb/image_raw" topic
   * For each image in the stream, display it in a window called "LIMO POV" using OpenCV's "imshow" function

1. **Task 2a**{: .label .label-blue}Create your custom node in its own package (behaviour listed above).
2. **Task 2b**{: .label .label-blue}Create a launch file to launch limo_pov_node.
3. **Task 2c**{: .label .label-blue}In what color space is this image stream in? (you can use the colour test image provided)

## Working with Images

### **Task 3: Analysing & Processing Predictable Images**{: .label .label-green}
For this task, you will use the custom package you have made in task 2 and further process and analysis the image you receive. We will test using the given colour test image.

The different combinations of the primary colours in the RGB colour space gives us a total of 6 primary/secondary colours which are
* Red
* Green
* Blue
* Yellow
* Cyan
* Magenta


1. **Task 3a**{: .label .label-blue}Create a python script called pure_color_extractor.py and include this script with your report. This script should
   1. Read the given colour_test_img.png image.
   2. Extract each of the 6 colours listed above
   3. Display each extracted colour on its own window (Total 7 including the original) named after the color. An example showing extracted red is shown below.

![pure red extracted](assets/example_extract_pure.png)

### **Task 4: Analysing & Processing "real" Images**{: .label .label-green}
The colour_test_img.png image in task 3 have what are known as "pure" colours, meaning they only have the component colours necessary to make that colour. However, images from the real world rarely so, having a mix of colours at different intensities but are still considered a specific colour. This can be the result of lighting, camera parameters, etc.

1. **Task 4a**{: .label .label-blue}Create a python script called mixed_color_extractor.py and include this script with your report. This script should
   1. Read the given colour_test_img.png image.
   2. Extract each of the 6 colours listed above
   3. Display each extracted colour on its own window (Total 7 including the original) named after the color. An example showing extracted red is shown below.

![mixed red extracted](assets/example_extract_mix.png)
