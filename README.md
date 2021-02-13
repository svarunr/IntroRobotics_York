# EECS 4421, Introduction to Robotics
The course consists of 5 assignments ranging from basics of ROS, Gazebo and Python scripting all the way up to controlling a real robot arm remotely through SSH. The assignment structure and breakdown is provided below in an attempt to maintain a timeline of concepts learnt in the course, and compare them with the expected learning outcomes in the course syllabus.

### Resources
Here are some useful links for ROS:
1. [ROS tutorials](http://wiki.ros.org/ROS/Tutorials)
1. [ROS names and namespaces](https://wiki.ros.org/Names)
1. [Remapping arugments in ROS](https://wiki.ros.org/Remapping%20Arguments)
1. [ROS launch files](http://wiki.ros.org/roslaunch/XML)

Here are some useful links for code documentation and syntax:
1. [ROS style guide](http://wiki.ros.org/StyleGuide)
1. [Instructions on auto-formatting code](https://github.com/davetcoleman/roscpp_code_format)

Here are some useful links for `git`:
1. [Quick reference cheat sheet](https://ndpsoftware.com/git-cheatsheet.html)
1. [Git documentation](https://git-scm.com/docs)
1. [Git training manual](https://githubtraining.github.io/training-manual/book.pdf)

### Report Format
Quoting the eClass submission link:
> Your submission must be provided as a single pdf, and must not exceed 100mb in size. 
> You should provide documented code when code is provided, but imagery, text, etc. are certainly appropriate. 
> Do not submit external code (I can look it up -- just provide a URL).

## 1 *Basics of ROS, Gazebo and Getting Familiar with Model Libraries*
Goal: to get an understanding of the Robot Operating System, ROS for short, and how robots are simulated using Gazebo.

Tasks:
- [x] Simplify intertial calculations using `xacro`.
- [x] Add a hat to the `block_robot` that shows the forward direction.
- [x] Add a visual light camera that points in the x-direction. 
- [x] Download the  URDF description of some robot and drive it around the world.
- [x] Create a namespace for the `block_robot` and make it subscribe/publish to it.

Expected Learning Outcomes: to familiarize ourselves with creating robots using .urdf and .xacro files, understanding how nodes publish/subscribe to topics, assign namespaces to easily distinguish multiple robot simulations, modifying launch files.
Grade:

## 2 *Using Python to Simulate Signal Noise*
Goal: to simulate sensor noise in the `block_robot` and observe how much of it is required to prevent the robot from following instructions.

Tasks: 
- [] Create and add Gaussian noise signals using `/noisy_odom`.
- [] Make the `block_robot` drive in a square and get an understanding of how much noise is required to stall it.
- [] Some other requirement that needs to be updated.
- [] Some other requirement that needs to be updated.

Expected Learning Outcomes: to familiarize ourselves with Python/C++ to create nodes and understand the effect of signal noise in controlling robots.
