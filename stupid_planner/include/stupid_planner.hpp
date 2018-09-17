#ifndef STUPID_PLANNER_H
#define STUPID_PLANNER_H


#include <mavros_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

class StupidPlanner {
 public:
	// This planner utilises the carrot-and-stick method to trick the
	// position controller to follow a trajectory toward the goal. The
	// CARROT_STICK_LENGTH sort-of defines the speed
	const float CARROT_STICK_LENGTH = 2.0;

  StupidPlanner();
  ~StupidPlanner();

  void publishOutput();

 private:
  ros::NodeHandle nh_;
 
  
  ros::Subscriber fcu_input_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber vel_sub_;
  ros::Publisher mavros_obstacle_free_path_pub_;
  ros::Publisher current_pose_pub_;
  ros::Publisher goal_pose_pub_;

  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::TwistStamped current_vel_;
  geometry_msgs::PoseStamped goal_pose_;
  mavros_msgs::Trajectory obst_free_path_;


  void fcuInputGoalCallback(const mavros_msgs::Trajectory& msg);
  void velCallback(const geometry_msgs::TwistStamped& msg);
  void poseCallback(const geometry_msgs::PoseStamped& msg);
  void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point);

};


#endif  // STUPID_PLANNER_H
