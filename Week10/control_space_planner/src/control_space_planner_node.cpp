#include "control_space_planner/control_space_planner_node.hpp"

/* ----- Class Functions ----- */
MotionPlanner::MotionPlanner(ros::NodeHandle& nh) : nh_(nh)
{
  // Subscriber
  subOccupancyGrid = nh.subscribe("/map/local_map/obstacle",1, &MotionPlanner::CallbackOccupancyGrid, this);
  subEgoOdom = nh.subscribe("/odom",1, &MotionPlanner::CallbackEgoOdom, this);
  subGoalPoint = nh.subscribe("/move_base_simple/goal",1, &MotionPlanner::CallbackGoalPoint, this);
  // Publisher
  pubSelectedMotion = nh_.advertise<sensor_msgs::PointCloud2>("/points/selected_motion", 1, true);
  pubMotionPrimitives = nh_.advertise<sensor_msgs::PointCloud2>("/points/motion_primitives", 1, true);
  pubCommand = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  pubTruncTarget = nh_.advertise<geometry_msgs::PoseStamped>("/car/trunc_target", 1, true);
  
};

MotionPlanner::~MotionPlanner() 
{    
    ROS_INFO("MotionPlanner destructor.");
}

/* ----- ROS Functions ----- */

void MotionPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
  this->localMap = msg;
  this->origin_x = msg.info.origin.position.x;
  this->origin_y = msg.info.origin.position.y;
  this->frame_id = msg.header.frame_id;
  this->mapResol = msg.info.resolution;
  bGetMap = true;
}

void MotionPlanner::CallbackGoalPoint(const geometry_msgs::PoseStamped& msg)
{
  this->goalPose = msg;
  // - position
  this->goal_x = msg.pose.position.x;
  this->goal_y = msg.pose.position.y;
  // - orientation
  // -- quaternion to RPY (global)
  tf2::Quaternion goal;
  double goal_roll, goal_pitch, goal_yaw;
  // --- copy quaternion from odom
  tf2::convert(msg.pose.orientation, goal);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_goal(goal);
  m_goal.getRPY(goal_roll, goal_pitch, goal_yaw);
  this->goal_yaw = goal_yaw;
  
  this->bGetGoal = true;
}

void MotionPlanner::CallbackEgoOdom(const nav_msgs::Odometry& msg)
{
  this->egoOdom = msg;
  // - position
  this->ego_x = msg.pose.pose.position.x;
  this->ego_y = msg.pose.pose.position.y;
  // - orientation
  // -- quaternion to RPY (global)
  tf2::Quaternion ego;
  double ego_roll, ego_pitch, ego_yaw;
  // --- copy quaternion from odom
  tf2::convert(msg.pose.pose.orientation, ego);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_ego(ego);
  m_ego.getRPY(ego_roll, ego_pitch, ego_yaw);
  this->ego_yaw = ego_yaw;
  
  this->bGetEgoOdom = true;
}

void MotionPlanner::PublishSelectedMotion(std::vector<Node> motionMinCost)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // publish selected motion primitive as point cloud
  for (auto motion : motionMinCost) {
    pcl::PointXYZI pointTmp;
    pointTmp.x = motion.x;
    pointTmp.y = motion.y;
    cloud_in_ptr->points.push_back(pointTmp);
  }

  sensor_msgs::PointCloud2 motionCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionCloudMsg);
  motionCloudMsg.header.frame_id = this->frame_id;
  motionCloudMsg.header.stamp = ros::Time::now();
  pubSelectedMotion.publish(motionCloudMsg);
}

void MotionPlanner::PublishMotionPrimitives(std::vector<std::vector<Node>> motionPrimitives)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // publish motion primitives as point cloud
  for (auto& motionPrimitive : motionPrimitives) {
    double cost_total = motionPrimitive.back().cost_total;
    for (auto motion : motionPrimitive) {
      pcl::PointXYZI pointTmp;
      pointTmp.x = motion.x;
      pointTmp.y = motion.y;
      pointTmp.z = cost_total;
      pointTmp.intensity = cost_total;
      cloud_in_ptr->points.push_back(pointTmp);
    }
  }
  
  sensor_msgs::PointCloud2 motionPrimitivesCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionPrimitivesCloudMsg);
  motionPrimitivesCloudMsg.header.frame_id = this->frame_id;
  motionPrimitivesCloudMsg.header.stamp = ros::Time::now();
  pubMotionPrimitives.publish(motionPrimitivesCloudMsg);
}

void MotionPlanner::PublishCommand(std::vector<Node> motionMinCost)
{

  geometry_msgs::Twist command;
  // low-level control
  double steering_angle = motionMinCost.back().delta;
  double yaw = motionMinCost.back().yaw;
  double speed_norm = 0.1 * this->MOTION_VEL * motionMinCost.size() / (this->MAX_PROGRESS / this->DIST_RESOL);
  
  command.angular.z = speed_norm * tan(steering_angle) / this->WHEELBASE * 0.5;
  command.linear.x  = speed_norm * cos(yaw);
  command.linear.y  = speed_norm * sin(yaw);

  // arrival rule
  if (bGetGoal && bGetLocalNode) {
    double distToGoal = sqrt(this->localNode.x*this->localNode.x + this->localNode.y*this->localNode.y);

    if (distToGoal < this->ARRIVAL_THRES) {
      command.angular.z = 0.0;
      command.linear.x = 0.0;
    }
    else if (distToGoal < 2*this->ARRIVAL_THRES) {
      command.linear.x = command.linear.x * pow(distToGoal / 2*this->ARRIVAL_THRES, 3.0);
    }
  }

  pubCommand.publish(command);
}

/* ----- Algorithm Functions ----- */

void MotionPlanner::Plan()
{
  // Compute current LOS target pose
  if (this->bGetEgoOdom && this->bGetGoal) {
    Node goalNode;
    goalNode.x = this->goal_x;
    goalNode.y = this->goal_y;
    goalNode.yaw = this->goal_yaw;
    localNode = GlobalToLocalCoordinate(goalNode, this->egoOdom);
    // - compute truncated local node pose within local map
    Node tmpLocalNode;
    memcpy(&tmpLocalNode, &localNode, sizeof(struct Node));
    tmpLocalNode.x = std::max(this->mapMinX, std::min(tmpLocalNode.x, this->mapMaxX));
    tmpLocalNode.y = std::max(this->mapMinY, std::min(tmpLocalNode.y, this->mapMaxY));
    truncLocalNode = tmpLocalNode;

    // for debug
    geometry_msgs::PoseStamped localPose = GlobalToLocalCoordinate(this->goalPose, this->egoOdom);
    localPose.header.frame_id = "base_link";
    pubTruncTarget.publish(localPose);

    this->bGetLocalNode = true;
  }

  // Motion generation
  motionCandidates = GenerateMotionPrimitives(this->localMap);
  
  // Select motion
  std::vector<Node> motionMinCost = SelectMotion(motionCandidates);

  // Publish data
  PublishData(motionMinCost, motionCandidates);
}

std::vector<std::vector<Node>> MotionPlanner::GenerateMotionPrimitives(nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: Generate motion primitives
    - you can change the below process if you need.
    - you can calculate cost of each motion if you need.
  */


  // initialize motion primitives
  std::vector<std::vector<Node>> motionPrimitives;

  // compute params w.r.t. uncertainty
  int num_candidates = this->MAX_DELTA*2 / this->DELTA_RESOL; // *2 for considering both left/right direction

  // max progress of each motion
  double maxProgress = this->MAX_PROGRESS;
  for (int i=0; i<num_candidates+1; i++) {
    // current steering delta
    double angle_delta = this->MAX_DELTA - i * this->DELTA_RESOL;

    // init start node
    Node startNode(0, 0, 0, 0, angle_delta, 0, 0, 0, -1, false);
    
    // rollout to generate motion
    std::vector<Node> motionPrimitive = RolloutMotion(startNode, maxProgress, this->localMap);

    // add current motionPrimitive
    motionPrimitives.push_back(motionPrimitive);
  }

  return motionPrimitives;
}

std::vector<Node> MotionPlanner::RolloutMotion(Node startNode,
                                              double maxProgress,
                                              nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: rollout to generate a motion primitive based on the current steering angle
    - calculate cost terms here if you need
    - check collision / sensor range if you need
    1. Update motion node using current steering angle delta based on the vehicle kinematics equation.
    2. collision checking
    3. range checking
  */

  // Initialize motionPrimitive
  std::vector<Node> motionPrimitive;

  // Check collision and compute traversability cost for each motion node of primitive (in planner coordinate)
  Node currMotionNode(startNode.x, startNode.y, 0, 0, startNode.delta, 0, 0, 0, -1, false);
  double progress = this->DIST_RESOL;

  // for compute closest distance toward goal point. You can use in SelectMotion function to calculate goal distance cost
  double minDistGoal = 987654321;
  if (this->bGetLocalNode) {
    minDistGoal = sqrt((startNode.x-truncLocalNode.x)*(startNode.x-truncLocalNode.x) +
                       (startNode.y-truncLocalNode.y)*(startNode.y-truncLocalNode.y));
  }

  //! 1. Update motion node using current steering angle delta based on the vehicle kinematics equation
  // - while loop until maximum progress of a motion

  // Loop for rollout
  while (progress < maxProgress) {
    // x_t+1   := x_t + x_dot * dt
    // y_t+1   := y_t + y_dot * dt
    // yaw_t+1 := yaw_t + yaw_dot * dt
    currMotionNode.x += 0.0;
    currMotionNode.y += 0.0;
    currMotionNode.yaw += 0.0;

    // Calculate minimum distance toward goal
    if (this->bGetLocalNode) {
      double distGoal = sqrt((currMotionNode.x-truncLocalNode.x)*(currMotionNode.x-truncLocalNode.x) +
                             (currMotionNode.y-truncLocalNode.y)*(currMotionNode.y-truncLocalNode.y));
      if (minDistGoal > distGoal) {
        minDistGoal = distGoal;
      }
    }
    currMotionNode.minDistGoal = minDistGoal; // save current minDistGoal at current node
    
    // collision/range chekcing with "lookahead" concept
    // - lookahead point
    // double aheadYaw = currMotionNode.yaw + this->MOTION_VEL * tan(startNode.delta) / this->WHEELBASE * this->TIME_RESOL;
    double aheadYaw = currMotionNode.yaw;
    double aheadX = currMotionNode.x + this->MOTION_VEL * cos(aheadYaw) * this->TIME_RESOL;
    double aheadY = currMotionNode.y + this->MOTION_VEL * sin(aheadYaw) * this->TIME_RESOL;

    //! 2. collision checking
    // - local to map coordinate transform
    Node collisionPointNode(currMotionNode.x, currMotionNode.y, currMotionNode.yaw, currMotionNode.delta, 0, 0, 0, -1, false);
    Node collisionPointNodeMap = LocalToPlannerCorrdinate(collisionPointNode);
    if (CheckCollision(collisionPointNodeMap, localMap)) {
      // - do some process when collision occurs.
      // - you can save collision information & calculate collision cost here.
      // - you can break and return current motion primitive or keep generate rollout.

      return motionPrimitive;
    }

    //! 3. range checking
    // - if you want to filter out motion points out of the sensor range, calculate the Line-Of-Sight (LOS) distance & yaw angle of the node
    // - LOS distance := sqrt(x^2 + y^2)
    // - LOS yaw := atan2(y, x)
    // - if LOS distance > MAX_SENSOR_RANGE or abs(LOS_yaw) > FOV*0.5 <-- outside of sensor range 
    // - if LOS distance <= MAX_SENSOR_RANGE and abs(LOS_yaw) <= FOV*0.5 <-- inside of sensor range
    // - use params in header file (MAX_SENSOR_RANGE, FOV)
    double LOS_DIST = 0.0;
    double LOS_YAW = 0.0;
    if (LOS_DIST > this->MAX_SENSOR_RANGE || abs(LOS_YAW) > this->FOV*0.5) {
      // -- do some process when out-of-range occurs.
      // -- you can break and return current motion primitive or keep generate rollout.

      return motionPrimitive;
    } 

    // append collision-free motion in the current motionPrimitive
    motionPrimitive.push_back(currMotionNode);

    // update progress of motion
    progress += this->DIST_RESOL;
  }
  
  // return current motion
  return motionPrimitive;
}


std::vector<Node> MotionPlanner::SelectMotion(std::vector<std::vector<Node>> motionPrimitives)
{
  /*
    TODO: select the minimum cost motion primitive
  
    1. Calculate cost terms
    2. Calculate total cost (weighted sum of all cost terms)
    3. Compare & Find minimum cost (double minCost) & minimum cost motion (std::vector<Node> motionMinCost)
    4. Return minimum cost motion
  */

  double minCost = 9999999;
  std::vector<Node> motionMinCost; // initialize as odom

  // check size of motion primitives
  if (motionPrimitives.size() != 0) {
    // Iterate all motion primitive (motionPrimitive) in motionPrimitives
    for (auto& motionPrimitive : motionPrimitives) {
      //!1. Calculate cost terms

      //! 2. Calculate total cost ex) collision cost, goal distance, goal direction, progress cost, steering cost....
      double cost_total = 0.0;

      //! 3. Compare & Find minimum cost & minimum cost motion
      if (cost_total < minCost) {
          motionMinCost = motionPrimitive;
          minCost = cost_total;
      }
    }
  }
  //! 4. Return minimum cost motion
  return motionMinCost;
}

/* ----- Util Functions ----- */

bool MotionPlanner::CheckCollision(Node goalNodePlanner, nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: check collision of the current node
    - the position x of the node should be in a range of [0, map width]
    - the position y of the node should be in a range of [0, map height]
    - check all map values within the inflation area of the current node
    e.g.,

    for loop i in range(0, inflation_size)
      for loop j in range(0, inflation_size)
        tmp_x := currentNodeMap.x + i - 0.5*inflation_size <- you need to check whether this tmp_x is in [0, map width]
        tmp_y := currentNodeMap.y + j - 0.5*inflation_size <- you need to check whether this tmp_x is in [0, map height]
        map_index := "index of the grid at the position (tmp_x, tmp_y)" <-- map_index should be int, not double!
        map_value = static_cast<int16_t>(localMap.data[map_index])
        if (map_value > map_value_threshold) OR (map_value < 0)
          return true
    return false

    - use params in header file: INFLATION_SIZE, OCCUPANCY_THRES
  */

  return false;
  
}

bool MotionPlanner::CheckRunCondition()
{
  if (this->bGetMap && this->bGetGoal) {
  // if (this->bGetMap) {
    return true;
  }
  else {
    std::cout << "Run condition is not satisfied!!!" << "bGetMap : " << bGetMap << " bGetGoal : " << bGetGoal << std::endl;
    return false;
  }
}

Node MotionPlanner::GlobalToLocalCoordinate(Node globalNode, nav_msgs::Odometry egoOdom)
{
  // Coordinate transformation from global to local
  Node tmpLocalNode;
  // - Copy data globalNode to tmpLocalNode
  memcpy(&tmpLocalNode, &globalNode, sizeof(struct Node));

  // - Coordinate transform
  // -- translatioonal transform
  double delX = globalNode.x - egoOdom.pose.pose.position.x;
  double delY = globalNode.y - egoOdom.pose.pose.position.y;
  double delZ = globalNode.z - egoOdom.pose.pose.position.z;

  // -- rotational transform
  tf2::Quaternion q_ego;
  double egoR, egoP, egoY;
  // --- copy quaternion from odom
  tf2::convert(egoOdom.pose.pose.orientation, q_ego);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_odom(q_ego);
  m_odom.getRPY(egoR, egoP, egoY);

  // - calculate new pose
  double newX = cos(-egoY) * delX - sin(-egoY) * delY;
  double newY = sin(-egoY) * delX + cos(-egoY) * delY;
  double newZ = delZ;
  double newYaw = globalNode.yaw - egoY;

  // - Update pose
  tmpLocalNode.x = newX;
  tmpLocalNode.y = newY;
  tmpLocalNode.z = newZ;
  tmpLocalNode.yaw = newYaw;

  return tmpLocalNode;
}

geometry_msgs::PoseStamped MotionPlanner::GlobalToLocalCoordinate(geometry_msgs::PoseStamped poseGlobal, nav_msgs::Odometry egoOdom)
{
  // Coordinate transformation from global to local
  // - Copy data nodeGlobal to nodeLocal
  geometry_msgs::PoseStamped poseLocal;

  // - Coordinate transform
  // -- translatioonal transform
  double delX = poseGlobal.pose.position.x - egoOdom.pose.pose.position.x;
  double delY = poseGlobal.pose.position.y - egoOdom.pose.pose.position.y;
  double delZ = poseGlobal.pose.position.z - egoOdom.pose.pose.position.z;

  // -- rotational transform
  tf2::Quaternion q_goal, q_ego;
  double goalR, goalP, goalY;
  double egoR, egoP, egoY;
  // --- copy quaternion from odom
  tf2::convert(poseGlobal.pose.orientation, q_goal);
  tf2::convert(egoOdom.pose.pose.orientation, q_ego);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_goal(q_goal);
  tf2::Matrix3x3 m_odom(q_ego);
  m_goal.getRPY(goalR, goalP, goalY);
  m_odom.getRPY(egoR, egoP, egoY);

  // - calculate new pose
  double newX = cos(-egoY) * delX - sin(-egoY) * delY;
  double newY = sin(-egoY) * delX + cos(-egoY) * delY;
  double newZ = delZ;
  double newYaw = goalY - egoY;

  // - Update pose
  // -- quaternion to RPY 
  // -- RPY to Quaternion
  tf2::Quaternion globalQ, globalQ_new;
  globalQ.setRPY(0.0, 0.0, newYaw);
  globalQ_new = globalQ.normalize();

  poseLocal.pose.position.x = newX;
  poseLocal.pose.position.y = newY;
  poseLocal.pose.position.z = newZ;
  tf2::convert(globalQ_new, poseLocal.pose.orientation);

  return poseLocal;
}

Node MotionPlanner::LocalToPlannerCorrdinate(Node nodeLocal)
{
  /*
    TODO: Transform from local to occupancy grid map coordinate
    - local coordinate ([m]): x [map min x, map max x], y [map min y, map max y]
    - map coordinate ([cell]): x [0, map width], y [map height]
    - convert [m] to [cell] using map resolution ([m]/[cell])
  */
  // Copy data nodeLocal to nodeMap
  Node nodeMap;
  memcpy(&nodeMap, &nodeLocal, sizeof(struct Node));
  // Transform from local (min x, max x) [m] to map (0, map width) [grid] coordinate
  nodeMap.x = 0;
  // Transform from local (min y, max y) [m] to map (0, map height) [grid] coordinate
  nodeMap.y = 0;

  return nodeMap;
}


/* ----- Publisher ----- */

void MotionPlanner::PublishData(std::vector<Node> motionMinCost, std::vector<std::vector<Node>> motionPrimitives)
{
  // Publisher
  // - visualize selected motion primitive
  PublishSelectedMotion(motionMinCost);
  // - visualize motion primitives
  PublishMotionPrimitives(motionPrimitives);
  // - publish command
  PublishCommand(motionMinCost);
}

/* ----- Main ----- */

int main(int argc, char* argv[])
{ 
  std::cout << "start main process" << std::endl;

  ros::init(argc, argv, "control_space_planner");
  // for subscribe
  ros::NodeHandle nh;
  ros::Rate rate(50.0);
  MotionPlanner MotionPlanner(nh);

  // Planning loop
  while (MotionPlanner.nh_.ok()) {
      // Spin ROS
      ros::spinOnce();
      // check run condition
      if (MotionPlanner.CheckRunCondition()) {
        // Run algorithm
        MotionPlanner.Plan();
      }
      rate.sleep();
  }

  return 0;

}
