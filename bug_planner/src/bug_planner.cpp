#include "bug_planner.hpp"

BugPlanner::BugPlanner() : time_of_last_obstacle_(ros::Time::now()), obstacle_in_sight_(false)
{
  nh_ = ros::NodeHandle("~");

  fcu_input_sub_ = nh_.subscribe("/mavros/trajectory/desired", 1, &BugPlanner::fcuInputGoalCallback, this);
  depth_cam_sub_ = nh_.subscribe("/camera/depth/points", 1, &BugPlanner::depthCameraCallback, this);
  pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &BugPlanner::poseCallback, this);
  mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>("/mavros/trajectory/generated", 10);
  current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
  goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal_pose", 10);
}

BugPlanner::~BugPlanner(){};

void BugPlanner::fcuInputGoalCallback(const mavros_msgs::Trajectory& msg)
{
  // Save the current pose
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, msg.point_1.yaw);
  current_pose_.header = msg.header;
  current_pose_.pose.position.x = msg.point_1.position.x;
  current_pose_.pose.position.y = msg.point_1.position.y;
  current_pose_.pose.position.z = msg.point_1.position.z;
  current_pose_.pose.orientation.w = q.getW();
  current_pose_.pose.orientation.x = q.getX();
  current_pose_.pose.orientation.y = q.getY();
  current_pose_.pose.orientation.z = q.getZ();

  // Save the goal pose
  q.setRPY(0.0, 0.0, msg.point_2.yaw);
  goal_pose_.header = msg.header;
  goal_pose_.pose.position.x = msg.point_2.position.x;
  goal_pose_.pose.position.y = msg.point_2.position.y;
  goal_pose_.pose.position.z = msg.point_2.position.z;
  goal_pose_.pose.orientation.w = q.getW();
  goal_pose_.pose.orientation.x = q.getX();
  goal_pose_.pose.orientation.y = q.getY();
  goal_pose_.pose.orientation.z = q.getZ();
}

void BugPlanner::poseCallback(const geometry_msgs::PoseStamped& msg) {
	current_pose_ = msg;
	current_yaw_ = tf::getYaw(current_pose_.pose.orientation);
	x_dir_ = (goal_pose_.pose.position.x - current_pose_.pose.position.x);
	y_dir_ = (goal_pose_.pose.position.y - current_pose_.pose.position.y);
	z_dir_ = (goal_pose_.pose.position.z - current_pose_.pose.position.z);
	double mag = sqrt(x_dir_ * x_dir_ + y_dir_ * y_dir_ + z_dir_ * z_dir_);
	if(mag >= DESIRED_SPEED_M_P_S){// only normalize if far away from goal
	  x_dir_ /= mag/DESIRED_SPEED_M_P_S;
	  y_dir_ /= mag/DESIRED_SPEED_M_P_S;
	  z_dir_ /= mag/DESIRED_SPEED_M_P_S;
	}
}

void BugPlanner::fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point)
{
  point.position.x = NAN;
  point.position.y = NAN;
  point.position.z = NAN;
  point.velocity.x = NAN;
  point.velocity.y = NAN;
  point.velocity.z = NAN;
  point.acceleration_or_force.x = NAN;
  point.acceleration_or_force.y = NAN;
  point.acceleration_or_force.z = NAN;
  point.yaw = NAN;
  point.yaw_rate = NAN;
}

void BugPlanner::avoid()
{
  mavros_msgs::Trajectory obst_avoid = {};
  obst_avoid.header = current_pose_.header;
  obst_avoid.type = 0;
  obst_avoid.point_1.position.x = NAN;
  obst_avoid.point_1.position.y = NAN;
  obst_avoid.point_1.position.z = NAN;
  obst_avoid.point_1.yaw = NAN;
  obst_avoid.point_1.acceleration_or_force.x = NAN;
  obst_avoid.point_1.acceleration_or_force.y = NAN;
  obst_avoid.point_1.acceleration_or_force.z = NAN;

  // if an obstacle is ahead, just yaw and find a "free direction"
  if (obstacle_in_sight_)
  {
    obst_avoid.point_1.velocity.x = 0.0f;
    obst_avoid.point_1.velocity.y = 0.0f;
    obst_avoid.point_1.velocity.z = 0.0f;
    obst_avoid.point_1.yaw_rate = YAW_RATE_RAD_P_S;
    std::cout << "Obstacle ahead!" << std::endl;
  }

  // if there is no obstacle ahead, but we have recently had one, go straight
  else if (ros::Time::now().toSec() - time_of_last_obstacle_.toSec() < FLY_STRAIGHT_FOR_S)
  {
	std::cout << "No obstacle ahead, but recently there was" << std::endl;
    obst_avoid.point_1.velocity.x = cos(current_yaw_) * 1.5;
    obst_avoid.point_1.velocity.y = sin(current_yaw_) * 1.5;
    obst_avoid.point_1.velocity.z = 0.0f;
    obst_avoid.point_1.yaw_rate = 0.0f;
  }

  // if there's no obstacle ahead, but we're not facing the goal, just yaw and/or climb/descend
  else if (std::fabs(current_yaw_ -
                     (atan2(goal_pose_.pose.position.y - current_pose_.pose.position.y,
                            goal_pose_.pose.position.x - current_pose_.pose.position.x))) > FACING_GOAL_YAW_THR_RAD)
  {
	  std::cout << "No obstacle ahead, but not facing goal" << std::endl;
    obst_avoid.point_1.velocity.x = 0.0f;
    obst_avoid.point_1.velocity.y = 0.0f;
    obst_avoid.point_1.velocity.z = (goal_pose_.pose.position.z - current_pose_.pose.position.z)
    		* DESIRED_SPEED_M_P_S;
    obst_avoid.point_1.yaw_rate = wrapToPi(current_yaw_ - atan2(goal_pose_.pose.position.y - current_pose_.pose.position.y,
                   goal_pose_.pose.position.x - current_pose_.pose.position.x)) * YAW_RATE_RAD_P_S;
  }

  // if there is no obstacle ahead, and it's been a while that there was, we can fly toward the goal
  else
  {
	  std::cout << "No obstacle ahead, fly toward goal" << std::endl;
    obst_avoid.point_1.velocity.x = x_dir_;
    obst_avoid.point_1.velocity.y = y_dir_;
    obst_avoid.point_1.velocity.z = z_dir_;
    obst_avoid.point_1.yaw_rate = 0.0;
  }

  fillUnusedTrajectoryPoint(obst_avoid.point_2);
  fillUnusedTrajectoryPoint(obst_avoid.point_3);
  fillUnusedTrajectoryPoint(obst_avoid.point_4);
  fillUnusedTrajectoryPoint(obst_avoid.point_5);
  obst_avoid.time_horizon = { NAN, NAN, NAN, NAN, NAN };
  obst_avoid.point_valid = { true, false, false, false, false };

  mavros_obstacle_free_path_pub_.publish(obst_avoid);
  current_pose_pub_.publish(current_pose_);
  goal_pose_pub_.publish(goal_pose_);

}

double BugPlanner::wrapToPi(const double& in){
	double ret = in;
    if (in>0)
        ret = fmod(ret+M_PI, 2.0*M_PI)-M_PI;
    else
        ret = fmod(ret-M_PI, 2.0*M_PI)+M_PI;
    return ret;

}

void BugPlanner::depthCameraCallback(const sensor_msgs::PointCloud2& msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;  // Easier to loop through pcl::PointCloud
  pcl::fromROSMsg(msg, cloud);

  // Look through the point cloud, only if we have reached a height higher than our
  // CLOSEST_OBSTACLE_THRESHOLD since otherwise the ground would trigger an obstacle
  if (current_pose_.pose.position.z > CLOSEST_OBSTACLE_THRESHOLD_M)
  {
    for (const auto& p : cloud)
    {
      if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
      {
        // points in camera frame -> if p.y is close to zero it is roughly on the same height as the drone
        if (p.y > -0.5 && p.y < 0.5 &&
            (p.x * p.x + p.y * p.y + p.z * p.z) < CLOSEST_OBSTACLE_THRESHOLD_M * CLOSEST_OBSTACLE_THRESHOLD_M)
        {
          time_of_last_obstacle_ = ros::Time::now();
          obstacle_in_sight_ = true;
          return;
        }
      }
    }
  }
  obstacle_in_sight_ = false;
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bug_planner");
  BugPlanner planner;
  ros::Rate loop_rate(10.0);

    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
      planner.avoid();
    }
  ros::spin();
  return 0;
}
