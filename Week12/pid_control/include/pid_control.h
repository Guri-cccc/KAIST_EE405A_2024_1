#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "tf2/LinearMath/Transform.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <chassis_control/SetVelocity.h>
#include <llm_msgs/task_plan.h>

// headers in STL
#include <algorithm>
#include <bits/stdc++.h>
#include <cmath>
#include <deque>
#include <float.h>
#include <memory>
#include <mutex>
#include <queue>
#include <stdio.h>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <vector>
#include <tuple>

class PidControl
{
public:
  PidControl(ros::NodeHandle nh);
  ~PidControl();

  void TaskPlanningCallback(const llm_msgs::task_planConstPtr &msg);
  void EgoOdometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void MotionPlanningCallback(const nav_msgs::PathConstPtr &msg);
  void GoalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void TimerCallback(const ros::WallTimerEvent &e);
  void run();

  double normalizeEulerAngle(double euler);
  std::tuple<std::vector<double>, std::vector<double>> loadCSVfile(const std::string &wpt_file_path_);
  nav_msgs::Path xyVec2Path(std::tuple<std::vector<double>, std::vector<double>> xy);
  geometry_msgs::PoseStamped GlobalToLocal(const geometry_msgs::PoseStamped& msg);


private:
  ros::NodeHandle nh_;
  ros::Subscriber ego_odom_sub_;
  ros::Subscriber motin_planning_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber task_sub_;

  ros::Publisher vel_pub_;
  ros::Publisher arm_pi_vel_pub_;
  ros::Publisher predifined_path_pub_;
  ros::Publisher pubCommand;

  ros::WallTimer Timer;

  nav_msgs::Odometry m_odom;
  geometry_msgs::PoseStamped current_goal;
  nav_msgs::Path predefined_global_path;
  nav_msgs::Path predefined_local_path;
  nav_msgs::Path motion_planning_path;

  std::string file_path_left_line_ = "/home/jaeha/hw_ws/src/pid_control/wpt_data/wpt_data_dense.csv";

  double x_error_sum;
  double y_error_sum;
  double yaw_error_sum;

  bool bGetPath = false;
  bool bGetMotion = false;
  bool bGetOdometry = false;
  bool bGetGoal = false;

  const double DEG2RAD = 0.0174533;
  const double RAD2DEG = 57.2958;
  const double ARRIVAL_THRES = 0.2; // [m] distance threshold for arrival
  double veh_roll, veh_pitch, veh_yaw;
  llm_msgs::task_plan m_task_plan;
  std::string mode = "Navigation";
  std::string parameter = "";
  double value = 0;
};