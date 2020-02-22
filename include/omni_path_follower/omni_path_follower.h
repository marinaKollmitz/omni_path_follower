/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <omni_path_follower/Config.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <omni_path_follower/PathFollowerConfig.h>

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace omni_path_follower
{

#define EPSILON 0.0001

class PathFollower : public nav_core::BaseLocalPlanner
{
public:

  PathFollower();

  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros);
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
  bool isGoalReached();
  bool setPlan(const std::vector< geometry_msgs::PoseStamped > &plan);
  int getPathIndex(const std::vector< geometry_msgs::PoseStamped > &plan,
                   const tf::Pose &robot_pose);

private:
  void reconfigureCallback(omni_path_follower::PathFollowerConfig &config, u_int32_t level);
  bool posesEqual(geometry_msgs::PoseStamped first, geometry_msgs::PoseStamped second);

  dynamic_reconfigure::Server<omni_path_follower::PathFollowerConfig> *server_;
  double in_path_vel_;
  double to_path_k_;
  double angle_k_;
  double parking_scale_;
  double goal_threshold_;
  double max_lin_vel_;
  double max_ang_vel_;
  double min_lin_vel_;
  double min_ang_vel_;
  double max_path_offset_;
  bool rotate_to_path_;
  bool rotate_at_start_;
  bool rotating_;
  ros::Publisher waypoint_pub_;

  geometry_msgs::Pose last_waypoint_;
  geometry_msgs::Pose next_waypoint_;
  int path_length_;
  int path_index_;
  int path_index_offset_; //follower later waypoint, has a path smoothing effect
  // TODO keeping the current path index is not ideal
  // when the global plan is updated before the goal is reached ...

  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped goal_;
  geometry_msgs::PoseStamped last_start_;
  tf::TransformListener* tfl_;
  costmap_2d::Costmap2DROS* costmap_ros_;

  bool initialized_;
  bool goal_reached_;
};

} //namespace omni_path_follower

#endif
