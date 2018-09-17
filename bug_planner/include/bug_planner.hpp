#ifndef BUG_PLANNER_H
#define BUG_PLANNER_H

#include <mavros_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

class BugPlanner
{
public:
  BugPlanner();
  ~BugPlanner();

  void avoid();

private:
  ros::NodeHandle nh_;

  ros::Subscriber fcu_input_sub_;
  ros::Subscriber depth_cam_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher mavros_obstacle_free_path_pub_;
  ros::Publisher current_pose_pub_;
  ros::Publisher goal_pose_pub_;

  geometry_msgs::PoseStamped current_pose_;

  geometry_msgs::PoseStamped goal_pose_;
  ros::Time time_of_last_obstacle_;
  bool obstacle_in_sight_;
  double current_yaw_;
  double x_dir_, y_dir_, z_dir_; // Point toward goal with length DESIRED_SPEED_M_P_S



  // Hard-coded parameters

  // start breaking when obstacles are closer than this [m]
  const float CLOSEST_OBSTACLE_THRESHOLD_M = 3.0;

  // yaw rate to use when looking for a free direction
  const float YAW_RATE_RAD_P_S = 0.5;

  // Time to fly straight before turning toward goal when a new free direction is found
  const float FLY_STRAIGHT_FOR_S = 2.0;

  // Desired forward speed to use
  const float DESIRED_SPEED_M_P_S = 2.0;

  // Accept current heading as "facing the goal" when this many radians off
  const float FACING_GOAL_YAW_THR_RAD = 0.1;

  void fcuInputGoalCallback(const mavros_msgs::Trajectory& msg);
  void depthCameraCallback(const sensor_msgs::PointCloud2& msg);
  void poseCallback(const geometry_msgs::PoseStamped& msg);
  void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point);
  double wrapToPi(const double& in);

};

#endif  // BUG_PLANNER_H
