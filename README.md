![image](https://github.com/Guri-cccc/KAIST_EE405A_2024_1/assets/70877236/699932d7-a550-431a-949b-a7da675493ba)# KAIST_EE405A_2024_1
KAIST EE405A 2024 spring semester github

In this class, students are invited to build an AI-enabled autonomous rover, based on a ground vehicle platform with GPU-powered computers. Students will learn the basics of robotics including computer vision, navigation, control as well as latest deep learning based detection and decision making. Students will do individual simulation based homework and then build an autonomous rover as a team effort. After building the robot system, students are invited write software that drives the system using latest AI technologies to perform autonomous exploration just like a real life planetary rovers looking for the evidence of life in other planets. 

# Class materials
## Hardware platform
<details>
  
<summary> Hiwonder ArmPi Pro </summary>

[Link](https://www.hiwonder.com/collections/robotic-car/products/armpi-pro?variant=40308380958807) <br/>
- 4 omni-directional mecanum wheels
- 6DOF arm (5DOF+gripper)
- FPV camera at the end-effector

</details>

<details>
  
<summary> Intel Realsense D435 </summary>

[Link](https://www.intelrealsense.com/depth-camera-d435/) <br/>

</details>

## Onboard computer
<details>
  
<summary> NVIDIA® Jetson Orin™ Nano Developer Kit </summary>

[Link](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit) <br/>

</details>

# Environments
- Ubuntu 20.04
- ROS Noetic


***

# Syllabus

<details>
<summary> Week 1 - Class Introduction </summary>


</details>

<details>
<summary> Week 2 - Introduction to Ubuntu & ROS </summary>

<!-- [Lecture Note](Week2/Materials/) <br/> -->
- Instructions for installing Ubuntu (Linux-based OS)
- Understanding the Robotics Operating System (ROS)
- Installation & initial setup for ROS
- Basic ROS examples
- Hands-on learn of ROS programming

</details>

<details>
<summary> Week 3 - Simulating Robot in Virtual Environment  </summary>

<!-- [Lecture Note](Week 3/Materials/) <br/> -->
- ROS Gazebo
- Robot model and environment
  - URDF: Unified Robot Description Format
  - World
- Spawn your own URDF robot in the World
- Virtual Sensors in Gazebo

</details>

<details>  
<summary> Week 4 - Overall System Architecture </summary>

<!-- [Lecture Note](Week 4/Materials/) <br/> -->
- Autonomous system configuration
- Modules
    - Control
    - Localization
    - Perception
    - Path planning
    - Task planning

</details>

<details>
<summary> Week 5 - Vehicle Control </summary>

<!-- [Lecture Note](Week 5/Materials/) <br/> -->
- Learn how to design the Mecanum Wheels Robot
    - Mechanism of Mecanum Wheels Robot
    - Kinematics of Mecanum Wheels Robot
    - How to operate Hiwonder ArmPi Pro centering on ROS

</details>

<details>
<summary> Week 6 - Robot Configuration </summary>

<!-- [Lecture Note](Week 6/Materials/) <br/> -->
- Hardware introduction
- Component description
- Before assembling hardware
- Basic soldering tips

</details>

<details>
<summary> Week 7 - Visual SLAM </summary>

<!-- [Lecture Note](Week 7/Materials/) <br/> -->
- TFs in mobile manipulator
  - Robot TFs
  - TFs for perception
  - TFs for localization
- SLAM
  - ORB-SLAM
- Waypoints for global path planning

</details>

  ( Week 8 - Midterm exam week )

<details>
<summary> Week 9 - Perception </summary>

<!-- [Lecture Note](Week 9/Materials/) <br/> -->
- Depth Image and Pointcloud
    - Depth estimation using vision
    - Stereo vision
    - Other methods
    - Object detection
    - Point cloud segmentation
- Image
    - Object detection
    - Image segmentation
    - IPM (Inverse Perspective Mapping)

</details>

<details>        
<summary> Week 10 - Motion Planning </summary>

<!-- [Lecture Note](Week10/Materials/) <br/> -->
- Occupancy grid map
- Cost map generation
- Collision checking
- Motion planning methods
  - Graph-based approaches
  - Sampling-based approaches
- Algorithms
  - A* algorithm
  - Rapidly Exploring Random Tree (RRT)
  
</details>

<details>    
<summary> Week 11 - Exploration </summary>

<!-- [Lecture Note](Week11/Materials/) <br/> -->

</details>

<details>
<summary> Week 12 - Large Language Model in Robotics </summary>

<!-- [Lecture Note](Week12/Materials/) <br/> -->

</details>

<details>
<summary> Week 13 - System Operation </summary>

<!-- [Lecture Note](Week13/Materials/) <br/> -->
- Review of overall system architecture
- Hardware settings check
- Review of exploration strategy
  - Localization
  - Perception
  - Large language model

</details>

<details>
<summary> Week 14 - Final Project Preparation </summary>

<!-- [Lecture Note](Week14/Materials/) <br/> -->
- Review of the race rules
- Testings
- QnA

</details>

<details>
<summary> Week 15 - Final Race </summary>

<!-- [Lecture Note](Week15/Materials/) <br/> -->

</details>

  ( Week 16 - Final exam week )
