#include "stupid_planner.hpp"
//#define POSITION_SETPOINTS

StupidPlanner::StupidPlanner() {
  nh_ = ros::NodeHandle("~");

  fcu_input_sub_ =
      nh_.subscribe("/mavros/trajectory/desired", 1,
                    &StupidPlanner::fcuInputGoalCallback, this);

  mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>(
      "/mavros/trajectory/generated", 10);
}

StupidPlanner::~StupidPlanner() {};

void StupidPlanner::fcuInputGoalCallback(
    const mavros_msgs::Trajectory& msg) {

  ROS_INFO("input p1: %.3f, %.3f, %.3f | %.3f, %.3f, %.3f",msg.point_1.position.x, msg.point_1.position.y, msg.point_1.position.z, msg.point_1.velocity.x, msg.point_1.velocity.y, msg.point_1.velocity.z);
 
  mavros_msgs::Trajectory obst_free_path = {};
  transformPoseToObstacleAvoidance(obst_free_path, msg);
  mavros_obstacle_free_path_pub_.publish(obst_free_path);

  ROS_INFO("pub: %.3f, %.3f, %.3f | %.3f, %.3f, %.3f",obst_free_path.point_1.position.x, obst_free_path.point_1.position.y, obst_free_path.point_1.position.z,
           obst_free_path.point_1.velocity.x, obst_free_path.point_1.velocity.y, obst_free_path.point_1.velocity.z);
}

void StupidPlanner::fillUnusedTrajectoryPoint(
    mavros_msgs::PositionTarget& point) {
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

void StupidPlanner::transformPoseToObstacleAvoidance(
    mavros_msgs::Trajectory& obst_avoid, const mavros_msgs::Trajectory& msg) {
  obst_avoid.header = msg.header;
  obst_avoid.type = 0;


#ifdef POSITION_SETPOINTS
  double x_dir =0.0d, y_dir = 0.0d, z_dir=1.0d;
  // If no goal is defined, just return current position
if(msg.point_valid[1] == true && msg.point_1.position.z >= 0.5f){
     x_dir = (msg.point_2.position.x - msg.point_1.position.x);
     y_dir = (msg.point_2.position.y - msg.point_1.position.y);
     z_dir = (msg.point_2.position.z - msg.point_1.position.z);
    double mag = sqrt(x_dir*x_dir+y_dir*y_dir + z_dir*z_dir);
    if(mag >= 1.0d){// only normalize if far away from goal
      x_dir /= mag;
      y_dir /= mag;
      z_dir /= mag;
     }
}

  obst_avoid.point_1.position.x = msg.point_1.position.x + x_dir;
  obst_avoid.point_1.position.y = msg.point_1.position.y + y_dir;
  obst_avoid.point_1.position.z = msg.point_1.position.z + z_dir;
  obst_avoid.point_1.velocity.x = NAN;
  obst_avoid.point_1.velocity.y = NAN;
  obst_avoid.point_1.velocity.z = NAN;
  obst_avoid.point_1.yaw = atan2(y_dir,x_dir);
  obst_avoid.point_1.yaw_rate = NAN;
#else
  obst_avoid.point_1.position.x = NAN;
  obst_avoid.point_1.position.y = NAN;
  obst_avoid.point_1.position.z = NAN;
  obst_avoid.point_1.velocity.x = msg.point_1.velocity.x;
  obst_avoid.point_1.velocity.y = msg.point_1.velocity.y;
  obst_avoid.point_1.velocity.z = msg.point_1.velocity.z;
  obst_avoid.point_1.yaw = NAN;
  obst_avoid.point_1.yaw_rate = msg.point_1.yaw_rate;
#endif

  obst_avoid.point_1.acceleration_or_force.x = NAN;
  obst_avoid.point_1.acceleration_or_force.y = NAN;
  obst_avoid.point_1.acceleration_or_force.z = NAN;

  fillUnusedTrajectoryPoint(obst_avoid.point_2);
  fillUnusedTrajectoryPoint(obst_avoid.point_3);
  fillUnusedTrajectoryPoint(obst_avoid.point_4);
  fillUnusedTrajectoryPoint(obst_avoid.point_5);

  obst_avoid.time_horizon = {NAN, NAN, NAN, NAN, NAN};

  obst_avoid.point_valid = {true, false, false, false, false};
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "stupid_planner");
  StupidPlanner planner;

  ros::spin();
  return 0;
}
