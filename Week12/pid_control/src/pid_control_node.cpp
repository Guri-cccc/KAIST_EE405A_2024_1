#include <pid_control.h>

PidControl::PidControl(ros::NodeHandle nh)
    : nh_(nh)
{
    // PUBLISHER
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    arm_pi_vel_pub_ = nh_.advertise<chassis_control::SetVelocity>("/chassis_control/set_velocity", 1, true);
    predifined_path_pub_ = nh_.advertise<nav_msgs::Path>("/predifined_path", 1, true);
    pubCommand = nh_.advertise<ackermann_msgs::AckermannDrive>("/car_1/command", 1, true);

    // SUBSCRIBER
    ego_odom_sub_ = nh_.subscribe("/odom", 10, &PidControl::EgoOdometryCallback, this);
    goal_sub_ = nh_.subscribe("/car/trunc_target", 10, &PidControl::GoalCallback, this);
    motin_planning_sub_ = nh_.subscribe("/path/selected_motion", 10, &PidControl::MotionPlanningCallback, this);
    task_sub_ = nh_.subscribe("/task_plan", 10, &PidControl::TaskPlanningCallback, this);

    Timer = nh_.createWallTimer(ros::WallDuration(0.03), &PidControl::TimerCallback, this);

};

PidControl::~PidControl() { ROS_INFO("PidControl destructor."); }

void PidControl::TaskPlanningCallback(const llm_msgs::task_planConstPtr &msg)
{
    m_task_plan=*msg;
    mode = m_task_plan.mode;
    parameter = m_task_plan.parameter;
    value = m_task_plan.value;
}

void PidControl::EgoOdometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    m_odom = *msg;

    tf2::Quaternion veh_quat;
    tf2::fromMsg(m_odom.pose.pose.orientation, veh_quat);
    veh_quat.normalize();
    tf2::Matrix3x3 veh_mat(veh_quat);
    // double veh_roll, veh_pitch, veh_yaw;
    veh_mat.getRPY(veh_roll, veh_pitch, veh_yaw);
    bGetOdometry = true;
}

void PidControl::MotionPlanningCallback(const nav_msgs::PathConstPtr &msg)
{
    motion_planning_path = *msg;
    bGetMotion = true;
}

void PidControl::GoalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    current_goal = *msg;
    bGetGoal = true;
}

void PidControl::TimerCallback(const ros::WallTimerEvent &e)
{
    // if (bGetMotion && bGetOdometry && bGetGoal)
    if (bGetGoal)
    {
        run();
    }
    else
    {
        chassis_control::SetVelocity arm_vel_command;
        arm_vel_command.velocity = 0.0;
        arm_vel_command.direction = 90.0;
        arm_vel_command.angular = 0.0;
        arm_pi_vel_pub_.publish(arm_vel_command);
    }
    bGetGoal = false;
}

void PidControl::run()
{
    geometry_msgs::Twist vel_command;

    double P_gain = 0.5;     // Need to change this value
    double I_gain = 0.1;     // Need to change this value
    double D_gain = 0.05;    // Need to change this value
    int lookahead_index = 2; // Need to change this value

    // TODO 2. Transform the target point from the global coordinate system to the local coordinate system.

    //! In this homework, we cannot use motion planning results.
    // double control_target_x = motion_planning_path.poses[lookahead_index].pose.position.x;
    // double control_target_y = motion_planning_path.poses[lookahead_index].pose.position.y;

    double control_target_x = 1.0;
    double control_target_y = 0.0;
    //! In this homework, we cannot use motion planning results. 
    //! If control target position set to (1.0,0.0), robot will go front continuously.

    // TODO 3. Calculate X, Y error and yaw error in local coordinate.
    double local_x_error = control_target_x;
    double local_y_error = control_target_y;
    double local_yaw_error = normalizeEulerAngle(atan2(local_y_error, local_x_error));
    // TODO 3. Calculate X, Y error and yaw error in local coordinate.

    // TODO 4. Calculate vx, vy, yaw rate command (PI(PID) control using 3.results)
    double integral_x_error = 0.0;
    double integral_y_error = 0.0;
    double integral_yaw_error = 0.0;
    
    double proportional_x = P_gain * local_x_error;
    double proportional_y = P_gain * local_y_error;
    double proportional_yaw = P_gain * local_yaw_error;

    integral_x_error += local_x_error;
    integral_y_error += local_y_error;
    integral_yaw_error += local_yaw_error;
    double integral_term_x = I_gain * integral_x_error;
    double integral_term_y = I_gain * integral_y_error;
    double integral_term_yaw = I_gain * integral_yaw_error;

    double derivative_term_x = D_gain * local_x_error;
    double derivative_term_y = D_gain * local_y_error;
    double derivative_term_yaw = D_gain * local_yaw_error;

    vel_command.linear.x = proportional_x + integral_term_x + derivative_term_x;
    vel_command.linear.y = 0.0;
    vel_command.angular.z = proportional_y + integral_term_y + proportional_yaw + integral_term_yaw + derivative_term_yaw;


    //! In this homework, we cannot use motion planning results. 
    //! Setting arbitrary goal distance.
    // double goal_distance = std::sqrt(std::pow(m_odom.pose.pose.position.x - current_goal.pose.position.x, 2) 
                                //   + std::pow(m_odom.pose.pose.position.y - current_goal.pose.position.y, 2));
    double goal_distance = std::sqrt(std::pow(current_goal.pose.position.x, 2) 
                                  + std::pow(current_goal.pose.position.y, 2));
    //! In this homework, we cannot use motion planning results. 
    //! Setting arbitrary goal distance.

    // ROS_INFO_THROTTLE(0.5, "goal_distance: %f", goal_distance);

    geometry_msgs::PoseStamped current_goal_local = GlobalToLocal(current_goal);

    double local_yaw = std::atan2(current_goal_local.pose.position.y, current_goal_local.pose.position.x);
    // ROS_INFO_THROTTLE(0.5, "local_yaw: %f", local_yaw);

    if(bGetGoal)
    {
        if(mode == "Exploration")
        {
            if(parameter == "Left")
            {
                ROS_WARN_THROTTLE(0.5, "FORCED LEFT");
                vel_command.linear.x = 0.0;
                vel_command.linear.y = 0.0;
                vel_command.angular.z = value * DEG2RAD / 18;
            }
            else
            {
                ROS_WARN_THROTTLE(0.5, "FORCED RIGHT");
                vel_command.linear.x = 0.0;
                vel_command.linear.y = 0.0;
                vel_command.angular.z = value * DEG2RAD / 18;
            }
        }
        else if(mode == "Grasping")
        {
            ROS_WARN_THROTTLE(0.5, "GRASPING");
            vel_command.linear.x = 0.0;
            vel_command.linear.y = 0.0;
            vel_command.angular.z = 0.0;
        }
        else
        {
            ROS_WARN_THROTTLE(0.5, "NAVIGATION");
            if(goal_distance < this->ARRIVAL_THRES)
            {
                vel_command.linear.x = 0.0;
                vel_command.linear.y = 0.0;
                vel_command.angular.z = 0.0;
            }
            else if (goal_distance < 2*this->ARRIVAL_THRES) //! If the goal position is near, slow down the velocity.
            {
                vel_command.linear.x = vel_command.linear.x * pow(goal_distance / 2*this->ARRIVAL_THRES, 3.0);
            }
        }
    }
    else
    {
        vel_command.linear.x = 0.0;
        vel_command.linear.y = 0.0;
        vel_command.angular.z = 0.0;
    }
    // TODO 4. Calculate vx, vy, yaw rate command (PI(PID) control using 3.results)

    vel_pub_.publish(vel_command);

    //! Generate Armpi pro base control commmand. 
    double vel_norm = std::sqrt(std::pow(vel_command.linear.x, 2) 
                    + std::pow(vel_command.linear.y, 2)) * 100.0; //! m/s -> cm/s

    chassis_control::SetVelocity arm_vel_command;
    arm_vel_command.velocity = vel_norm;
    arm_vel_command.direction = 90.0 + atan2(vel_command.linear.y, vel_command.linear.x)*RAD2DEG;
    arm_vel_command.angular = vel_command.angular.z;
    arm_pi_vel_pub_.publish(arm_vel_command);
}

geometry_msgs::PoseStamped PidControl::GlobalToLocal(const geometry_msgs::PoseStamped& msg)
{
    geometry_msgs::PoseStamped OnGlobalCoord = msg;
    geometry_msgs::PoseStamped OnBodyCoord;

    OnBodyCoord.pose.position.x =
        (OnGlobalCoord.pose.position.x - m_odom.pose.pose.position.x) *
            std::cos(veh_yaw) +
        (OnGlobalCoord.pose.position.y - m_odom.pose.pose.position.y) *
            std::sin(veh_yaw);
    OnBodyCoord.pose.position.y =
        (OnGlobalCoord.pose.position.x - m_odom.pose.pose.position.x) *
            -std::sin(veh_yaw) +
        (OnGlobalCoord.pose.position.y - m_odom.pose.pose.position.y) *
            std::cos(veh_yaw);

    return OnBodyCoord;
}

double PidControl::normalizeEulerAngle(double euler)
{
    double res = euler;
    while (res > M_PI)
    {
        res -= (2.0 * M_PI);
    }
    while (res < -M_PI)
    {
        res += 2.0 * M_PI;
    }

    return res;
}

std::tuple<std::vector<double>, std::vector<double>>
PidControl::loadCSVfile(const std::string &wpt_file_path_)
{
    std::ifstream inputFile(wpt_file_path_);
    std::vector<double> vec_x, vec_y;

    while (inputFile)
    {
        std::string s;
        if (!std::getline(inputFile, s))
        {
            break;
        }
        if (s[0] != '#')
        {
            std::istringstream ss(s);
            int cnt = 0;
            bool nan_flg = false;
            while (ss)
            {
                std::string line;
                if (!std::getline(ss, line, ','))
                {
                    break;
                }
                try
                {
                    if (cnt == 0)
                    {
                        vec_x.push_back(stof(line));
                    }
                    else if (cnt == 1)
                    {
                        vec_y.push_back(stof(line));
                    }
                }
                catch (const std::invalid_argument e)
                {
                    std::cout << "NaN found in file " << wpt_file_path_ << std::endl;
                    e.what();
                    nan_flg = true;
                }
                cnt++;
            }
        }
    }

    if (!inputFile.eof())
    {
        std::cerr << "Could not read file " << wpt_file_path_ << "\n";
        std::__throw_invalid_argument("File not found.");
    }

    if (vec_x.size() == 0 || vec_y.size() == 0 ||
        (vec_x.size() != vec_y.size()))
    {
        std::__throw_invalid_argument("WPT SIZE ERROR.");
    }

    return std::make_tuple(vec_x, vec_y);
}

nav_msgs::Path
PidControl::xyVec2Path(std::tuple<std::vector<double>, std::vector<double>> xy)
{
    nav_msgs::Path output;
    output.header.frame_id = "odom";
    auto x_ = std::get<0>(xy);
    auto y_ = std::get<1>(xy);
    for (int i = 0; i < x_.size(); i++)
    {
        geometry_msgs::PoseStamped pt;
        pt.pose.position.x = x_[i];
        pt.pose.position.y = y_[i];
        pt.pose.position.z = 0.0;

        // orientation
        int next_idx = (i + 1) % x_.size();
        double next_x = x_[next_idx];
        double next_y = y_[next_idx];
        tf2::Quaternion quat_ekf;
        double yaw = atan2(next_y - pt.pose.position.y, next_x - pt.pose.position.x);
        if (std::isnan(yaw))
        {
            yaw = 0;
        }
        quat_ekf.setRPY(0, 0, yaw);
        quat_ekf.normalize();
        pt.pose.orientation = tf2::toMsg(quat_ekf);

        // push_back
        output.poses.push_back(pt);
    }

    bGetPath = true;
    return output;
}
