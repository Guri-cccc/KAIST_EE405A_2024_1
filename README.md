# KAIST_EE405A_2024_1
KAIST EE405A 2024 spring semester github

In this class, students are invited to build an AI-enabled autonomous rover, based on a ground vehicle platform with GPU-powered computers. Students will learn the basics of robotics including computer vision, navigation, control as well as latest deep learning based detection and decision making. Students will do individual simulation based homework and then build an autonomous rover as a team effort. After building the robot system, students are invited write software that drives the system using latest AI technologies to perform autonomous exploration just like a real life planetary rovers looking for the evidence of life in other planets. 

# Class materials
## Hardware platform
<details>
  
<summary> Base robot info </summary>

Hiwonder ArmPi Pro

[Robot link](https://www.hiwonder.com/collections/robotic-car/products/armpi-pro?variant=40308380958807) <br/>
- 4 omni-directional mecanum wheels
- 6DOF arm (5DOF+gripper)
- FPV camera at the end-effector

</details>

<details>
  
<summary> Additional Camera </summary>

Intel Realsense D435

[Camera link](https://www.hiwonder.com/collections/robotic-car/products/armpi-pro?variant=40308380958807) <br/>

</details>

## Computer
NVIDIA® Jetson Orin™ Nano Developer Kit

***

# Syllabus

<details>
<summary> Week 1 - Class introduction </summary>


</details>

<details>
<summary> Week 2 - Ubuntu Installation & ROS </summary>

<!-- [Lecture Note](Week2/Materials/) <br/> -->
- Brief tips on installing Ubuntu (Linux-based OS)
- Understand the Robotics Operating System (ROS) (1)
- Install & Setup ROS
- Run ROS tutorial
- Learn ROS programming

</details>

<details>
<summary> Week 3 - Simulation </summary>

<!-- [Lecture Note](Week 3/Materials/) <br/> -->
- Gazebo
- Robot model
  - URDF
  - World
- Control car model in the simulation
- Sensor inputs

</details>

<details>  
<summary> Week 4 - Overall system architecture </summary>

<!-- [Lecture Note](Week 4/Materials/) <br/> -->

</details>

<details>
<summary> Week 5 - Vehicle control </summary>

<!-- [Lecture Note](Week 5/Materials/) <br/> -->
- Learn how to design the vehicle controller
    - Vehicle kinematics model
    - Longitudinal controller using PID control
    - Geometry for lateral vehicle control
    - Lateral controller based on Pure Pursuit & Stanley Method

</details>

<details>
<summary> Week 6 - Hardware configuration </summary>

<!-- [Lecture Note](Week 6/Materials/) <br/> -->
- Hardware architecture
- Electronics
- Chassis

</details>

<details>
<summary> Week 7 - Visual SLAM </summary>

<!-- [Lecture Note](Week 7/Materials/) <br/> -->
- Visual Odometry
  - ORB-SLAM
- TFs in car like robots
  - TFs for perception
  - TFs for localization
- Waypoints for global path planning

</details>

  Week 8 - Midterm exam week

<details>
<summary> Week 9 - Perception </summary>

<!-- [Lecture Note](Week 9/Materials/) <br/> -->
- LiDAR-based perception
    - Object detection
    - Segmentation
- Camera-based perception
    - Object detection
    - Segmentation
    - Depth estimation using vision
    - Stereo vision
    - Other methods
- Post-processing
    - IPM (Inverse Perspective Mapping)
    - RGB-point cloud
    - Cost map generation

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
  - Motion primitive-based approaches
- Algorithms
  - A* algorithm
  - Rapidly Exploring Random Tree (RRT)
  - Motion primitive-based path planning

</details>

<details>    
<summary> Week 11 - Exploration </summary>

<!-- [Lecture Note](Week11/Materials/) <br/> -->

</details>

<details>
<summary> Week 12 - Large language model in robotics </summary>

<!-- [Lecture Note](Week12/Materials/) <br/> -->

</details>

<details>
<summary> Week 13 - Final project for exploration </summary>

<!-- [Lecture Note](Week13/Materials/) <br/> -->

</details>

<details>
<summary> Week 14 - Final project preparation (Exploration) </summary>

<!-- [Lecture Note](Week14/Materials/) <br/> -->

</details>

<details>
<summary> Week 15 - Final Race </summary>

<!-- [Lecture Note](Week15/Materials/) <br/> -->

</details>

  Week 16 - Final exam week
