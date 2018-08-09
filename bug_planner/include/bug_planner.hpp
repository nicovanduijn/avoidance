#ifndef BUG_PLANNER_H
#define BUG_PLANNER_H


#include <mavros_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

class BugPlanner {
 public:

  BugPlanner();
  ~BugPlanner();

 private:
  ros::NodeHandle nh_;
 
  
  ros::Subscriber fcu_input_sub_;
  ros::Subscriber depth_cam_sub_;
  ros::Publisher mavros_obstacle_free_path_pub_;
  ros::Publisher current_pos_pub_;
  ros::Publisher current_goal_pub_;

  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::PoseStamped goal_pose_;
  const float CLOSEST_OBSTACLE_THRESHOLD = 3.0;
  const float YAW_RATE = 0.5;
  const float OBS_TIME_THR = 2.0; 

  ros::Time time_of_last_obstacle_;
  bool obstacle_in_sight_;
 
  void fcuInputGoalCallback(const mavros_msgs::Trajectory& msg);
  void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point);
  void avoid(mavros_msgs::Trajectory& obst_avoid,
                                        const mavros_msgs::Trajectory& msg);
  void depthCameraCallback(const sensor_msgs::PointCloud2& msg);
};


#endif  // BUG_PLANNER_H
