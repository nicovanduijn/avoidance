#include "bug_planner.hpp"

BugPlanner::BugPlanner() : time_of_last_obstacle_(ros::Time::now()), obstacle_in_sight_(false)
{
  nh_ = ros::NodeHandle("~");

  fcu_input_sub_ = nh_.subscribe("/mavros/trajectory/desired", 1, &BugPlanner::fcuInputGoalCallback, this);
  depth_cam_sub_ = nh_.subscribe("/camera/depth/points", 1, &BugPlanner::depthCameraCallback, this);
  mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>("/mavros/trajectory/generated", 10);
  current_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_pos", 10);
  current_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal_pos", 10);
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

  mavros_msgs::Trajectory adjusted_traj_ = {};
  avoid(adjusted_traj_, msg);

  mavros_obstacle_free_path_pub_.publish(adjusted_traj_);
  current_pos_pub_.publish(current_pose_);
  current_goal_pub_.publish(goal_pose_);
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

void BugPlanner::avoid(mavros_msgs::Trajectory& obst_avoid, const mavros_msgs::Trajectory& msg)
{
  obst_avoid.header = msg.header;
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
  }

  // if there is no obstacle ahead, but we have recently had one, go straight
  else if (ros::Time::now().toSec() - time_of_last_obstacle_.toSec() < FLY_STRAIGHT_FOR_S)
  {
    obst_avoid.point_1.velocity.x = cos(msg.point_1.yaw) * 1.5;
    obst_avoid.point_1.velocity.y = sin(msg.point_1.yaw) * 1.5;
    obst_avoid.point_1.velocity.z = 0.0f;
    obst_avoid.point_1.yaw_rate = 0.0f;
  }

  // if there's no obstacle ahead, but we're not facing the goal, just yaw and/or climb/descend
  else if (std::fabs(tf::getYaw(current_pose_.pose.orientation) -
                     (atan2(goal_pose_.pose.position.y - current_pose_.pose.position.y,
                            goal_pose_.pose.position.x - current_pose_.pose.position.x))) > FACING_GOAL_YAW_THR_RAD)
  {
    obst_avoid.point_1.velocity.x = 0.0f;
    obst_avoid.point_1.velocity.y = 0.0f;
    obst_avoid.point_1.velocity.z = msg.point_1.velocity.z;
    obst_avoid.point_1.yaw_rate = msg.point_1.yaw_rate;
  }

  // if there is no obstacle ahead, and it's been a while that there was, we can fly toward the goal
  else
  {
    obst_avoid.point_1.velocity.x = msg.point_1.velocity.x * FRACTION_OF_DESIRED_SPEED;
    obst_avoid.point_1.velocity.y = msg.point_1.velocity.y * FRACTION_OF_DESIRED_SPEED;
    obst_avoid.point_1.velocity.z = msg.point_1.velocity.z;
    obst_avoid.point_1.yaw_rate = msg.point_1.yaw_rate;
  }

  fillUnusedTrajectoryPoint(obst_avoid.point_2);
  fillUnusedTrajectoryPoint(obst_avoid.point_3);
  fillUnusedTrajectoryPoint(obst_avoid.point_4);
  fillUnusedTrajectoryPoint(obst_avoid.point_5);
  obst_avoid.time_horizon = { NAN, NAN, NAN, NAN, NAN };
  obst_avoid.point_valid = { true, false, false, false, false };
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

  ros::spin();
  return 0;
}
