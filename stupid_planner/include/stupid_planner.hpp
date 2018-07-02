#ifndef STUPID_PLANNER_H
#define STUPID_PLANNER_H


#include <mavros_msgs/Trajectory.h>
#include <ros/ros.h>

class StupidPlanner {
 public:

  StupidPlanner();
  ~StupidPlanner();

 private:
  ros::NodeHandle nh_;
 
  
  ros::Subscriber fcu_input_sub_;
  ros::Publisher mavros_obstacle_free_path_pub_;

 
  void fcuInputGoalCallback(const mavros_msgs::Trajectory& msg);
  void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point);
  void transformPoseToObstacleAvoidance(mavros_msgs::Trajectory& obst_avoid,
                                        const mavros_msgs::Trajectory& msg);
};


#endif  // STUPID_PLANNER_H
