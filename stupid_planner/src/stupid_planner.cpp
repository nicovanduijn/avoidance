#include "stupid_planner.hpp"
#define POSITION_SETPOINTS
#define VERBOSE

StupidPlanner::StupidPlanner() {
  nh_ = ros::NodeHandle("~");

  fcu_input_sub_ =
      nh_.subscribe("/mavros/trajectory/desired", 1,
                    &StupidPlanner::fcuInputGoalCallback, this);
  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>(
        "/mavros/local_position/pose", 1, &StupidPlanner::poseCallback,
        this);
  vel_sub_ = nh_.subscribe<const geometry_msgs::TwistStamped&>("/mavros/local_position/velocity", 1,
                                  &StupidPlanner::velCallback, this);

  mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>(
      "/mavros/trajectory/generated", 10);
  current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
  goal_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/goal_pose", 10);


}

StupidPlanner::~StupidPlanner() {};

void StupidPlanner::velCallback(const geometry_msgs::TwistStamped& msg) {
	current_vel_ = msg;
}

void StupidPlanner::poseCallback(const geometry_msgs::PoseStamped& msg) {
	current_pose_ = msg;
}

void StupidPlanner::fcuInputGoalCallback(
    const mavros_msgs::Trajectory& msg) {

#ifdef VERBOSE
  ROS_INFO("input p1 pos|vel: %.3f, %.3f, %.3f, %.3f| %.3f, %.3f, %.3f, %.3f",
    msg.point_1.position.x, msg.point_1.position.y, msg.point_1.position.z, msg.point_1.yaw,
     msg.point_1.velocity.x, msg.point_1.velocity.y, msg.point_1.velocity.z, msg.point_1.yaw_rate);
  ROS_INFO("input p2 pos|vel: %.3f, %.3f, %.3f, %.3f| %.3f, %.3f, %.3f, %.3f",
    msg.point_2.position.x, msg.point_2.position.y, msg.point_2.position.z, msg.point_2.yaw,
     msg.point_2.velocity.x, msg.point_2.velocity.y, msg.point_2.velocity.z, msg.point_2.yaw_rate);
#endif
 
  if(msg.point_valid[1] == true){
	    goal_pose_.header = msg.header;
	    goal_pose_.pose.position.x = msg.point_2.position.x;
	    goal_pose_.pose.position.y = msg.point_2.position.y;
	    goal_pose_.pose.position.z = msg.point_2.position.z;
  }
}

void StupidPlanner::publishOutput(){
	// some logic here
#ifdef POSITION_SETPOINTS
	double x_dir =0.0d, y_dir = 0.0d, z_dir=CARROT_STICK_LENGTH;

	  // Only go toward goal, if a certain height is reached
	if(current_pose_.pose.position.z >= CARROT_STICK_LENGTH){
	     x_dir = (goal_pose_.pose.position.x - current_pose_.pose.position.x);
	     y_dir = (goal_pose_.pose.position.y - current_pose_.pose.position.y);
	     z_dir = (goal_pose_.pose.position.z - current_pose_.pose.position.z);
	    double mag = sqrt(x_dir*x_dir+y_dir*y_dir + z_dir*z_dir);
	    if(mag >= CARROT_STICK_LENGTH){// only normalize if far away from goal
	      x_dir /= mag/CARROT_STICK_LENGTH;
	      y_dir /= mag/CARROT_STICK_LENGTH;
	      z_dir /= mag/CARROT_STICK_LENGTH;
	     }
	}

	// Carrot-and-stick method...
	obst_free_path_.point_1.position.x = current_pose_.pose.position.x + x_dir;
	obst_free_path_.point_1.position.y = current_pose_.pose.position.y + y_dir;
	obst_free_path_.point_1.position.z = current_pose_.pose.position.z + z_dir;
	obst_free_path_.point_1.velocity.x = NAN;
	obst_free_path_.point_1.velocity.y = NAN;
	obst_free_path_.point_1.velocity.z = NAN;
	obst_free_path_.point_1.yaw = atan2(y_dir,x_dir);
	obst_free_path_.point_1.yaw_rate = NAN;
#else
	obst_free_path_.point_1.position.x = NAN;
	obst_free_path_.point_1.position.y = NAN;
	obst_free_path_.point_1.position.z = NAN;
	obst_free_path_.point_1.velocity.x = x_dir;
	obst_free_path_.point_1.velocity.y = y_dir;
	obst_free_path_.point_1.velocity.z = z_dir;
	obst_free_path_.point_1.yaw = NAN;
	obst_free_path_.point_1.yaw_rate = msg.point_1.yaw_rate;
#endif

	obst_free_path_.point_1.acceleration_or_force.x = NAN;
	obst_free_path_.point_1.acceleration_or_force.y = NAN;
	obst_free_path_.point_1.acceleration_or_force.z = NAN;
	fillUnusedTrajectoryPoint(obst_free_path_.point_2);
	fillUnusedTrajectoryPoint(obst_free_path_.point_3);
	fillUnusedTrajectoryPoint(obst_free_path_.point_4);
	fillUnusedTrajectoryPoint(obst_free_path_.point_5);
	obst_free_path_.time_horizon = {NAN, NAN, NAN, NAN, NAN};
	obst_free_path_.point_valid = {true, false, false, false, false};

	  mavros_obstacle_free_path_pub_.publish(obst_free_path_);
	  current_pose_pub_.publish(current_pose_);
	  goal_pose_pub_.publish(goal_pose_);

#ifdef VERBOSE
	  ROS_INFO("output p1 pos|vel: %.3f, %.3f, %.3f, %.3f| %.3f, %.3f, %.3f, %.3f",
			  obst_free_path_.point_1.position.x, obst_free_path_.point_1.position.y,
			  obst_free_path_.point_1.position.z, obst_free_path_.point_1.yaw,
			  obst_free_path_.point_1.velocity.x, obst_free_path_.point_1.velocity.y,
			  obst_free_path_.point_1.velocity.z, obst_free_path_.point_1.yaw_rate);
	  ROS_INFO("output p2 pos|vel: %.3f, %.3f, %.3f, %.3f| %.3f, %.3f, %.3f, %.3f",
			  obst_free_path_.point_2.position.x, obst_free_path_.point_2.position.y,
			  obst_free_path_.point_2.position.z, obst_free_path_.point_2.yaw,
			  obst_free_path_.point_2.velocity.x, obst_free_path_.point_2.velocity.y,
			  obst_free_path_.point_2.velocity.z, obst_free_path_.point_2.yaw_rate);
#endif

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "stupid_planner");
  StupidPlanner planner;
  ros::Rate loop_rate(10.0);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    planner.publishOutput();
  }


  ros::spin();
  return 0;
}
