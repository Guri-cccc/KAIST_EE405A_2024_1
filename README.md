# KAIST EE405A 2024 Spring Semester
## Topic: AI-enabled autonomous rover

In this class, students are invited to build an AI-enabled autonomous rover, based on a ground vehicle platform with GPU-powered computers. Students will learn the basics of robotics including computer vision, navigation, control as well as latest deep learning based detection and decision making. Students will do individual simulation based homework and then build an autonomous rover as a team effort. After building the robot system, students are invited write software that drives the system using latest AI technologies to perform autonomous exploration just like a real life planetary rovers looking for the evidence of life in other planets.

## Related projects
[WPI] A Robotic Solution to Safely Finding and Destroying Land Mines [Video](https://youtu.be/LhUmatv10n4?feature=shared) <br/>
[NASA JPL] Mars Science Laboratory Curiosity Rover [Animation](https://youtu.be/P4boyXQuUIw?feature=shared) <br/>
[RoSys Group] A mobile robotic manipulator for plastic waste collection [Video](https://youtu.be/dNyUeop_Ihc?feature=shared) <br/>
[USRG] MBZIR-UGV Operating a Valve [Video](https://youtu.be/NRCp4iheQsg?feature=shared) <br/>
[USRG] Autonomous Exploration and Object Detection in Complex Multi-Floor Environments [Video](https://youtu.be/ua-bf6es4ac?feature=shared) <br/>

# Hardware platform
<img src="./images/robot_image.png" align="center" width="40%">

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

<details>
  
<summary> NVIDIA® Jetson Orin™ Nano Developer Kit </summary>

[Link](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit) <br/>

</details>

# Environments
- Ubuntu 20.04
- ROS Noetic

***
# Autonomous Robot System Architecture (Example)

<img src="./images/system_architecture_sample_.png" align="center" width="80%">

***

# Experiment schedule

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
- Hands-on learn of ROS programming <br/>
  ROS Tutorial [Link](http://wiki.ros.org/ROS/Tutorials) 1.1.2 ~ 1.1.18 <br/>
  After following the tutorial, try to complete the Week 2 practice package as demonstrated video. <br/>
  [Link](https://www.youtube.com/watch?v=c1Ax88TbL9s) <br/>

</details>

<details>
<summary> Week 3 - Simulating Robot in Virtual Environment :computer: <p>$\color{#5ad7b7}HW:\ Simulating\ Robot\ on \ ROS\ Gazebo$</p> </summary>

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
<summary> Week 5 - Vehicle Control :computer: <p>$\color{#5ad7b7}HW:\ Pure\ pursuit$</p></summary>

<!-- [Lecture Note](Week 5/Materials/) <br/> -->
- Learn how to design the Mecanum Wheels Robot
    - Mechanism of Mecanum Wheels Robot
    - Kinematics of Mecanum Wheels Robot
    - Kinematics of 6-DOF robot arm
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
<summary> Week 7 - Visual SLAM :computer: <p>$\color{#5ad7b7}HW:\ ORB-SLAM$</p></summary></summary>

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
<summary> Week 10 - Motion Planning :computer: <p>$\color{#5ad7b7}HW:\ Plannig\ collision\ avoidance\ path$</p></summary>

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
- Introduction to Exploration
- Core Papers Review (Exploration)
  - "Topological Exploration using Segmented Map with Keyframe Contribution in Subterranean Environments", "GBPlanner", "FAEL", and etc.
</details>

<details>
<summary> Week 12 - Large Language Model in Robotics :computer: <p>$\color{#5ad7b7}HW:\ GPT\ API$</p></summary>

<!-- [Lecture Note](Week12/Materials/) <br/> -->
- Introduction to LLMs (GPT, BERT, PaLM, and etc.)
  - GPT, BERT, PaLM, and etc.
  - Vision + LLM
  - Robotics + LLM
- Instruction of GPT API
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
<summary> Week 15 - Final Race :blue_car: <p>$\huge{\rm{\color{#DD6565}Sample\ Collection\ using\ Autonomous\ Rover}}$</p></summary>

<!-- [Lecture Note](Week15/Materials/) <br/> -->

</details>

  ( Week 16 - Final exam week )
