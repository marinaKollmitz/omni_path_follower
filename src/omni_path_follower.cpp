#include <omni_path_follower/omni_path_follower.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(omni_path_follower::PathFollower, nav_core::BaseLocalPlanner)

namespace omni_path_follower
{
  PathFollower::PathFollower() :
     initialized_(false)
   {
   }

//  bool PathFollower::posesEqual(geometry_msgs::PoseStamped first, geometry_msgs::PoseStamped second)
//  {
//    //    first.pose.position - second.pose.position;
//    if(fabs(first.pose.position.x - second.pose.position.x) < EPSILON &&
//       fabs(first.pose.position.y - second.pose.position.y) < EPSILON &&
//       fabs(first.pose.position.z - second.pose.position.z) < EPSILON &&
//       fabs(first.pose.orientation.x - second.pose.orientation.x) < EPSILON &&
//       fabs(first.pose.orientation.y - second.pose.orientation.y) < EPSILON &&
//       fabs(first.pose.orientation.z - second.pose.orientation.z) < EPSILON &&
//       fabs(first.pose.orientation.w - second.pose.orientation.w) < EPSILON)
//      return true;

//    return false;
//  }

  void PathFollower::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
  {
    tfl_ = tf;
    costmap_ros_ = costmap_ros;

    in_path_vel_ = 0.5;
    to_path_k_ = 0.5;
    angle_k_ = 1.0;
    goal_threshold_ = 0.05;
    rotate_to_path_ = true;
    rotate_at_start_ = false;

//    //initialize empty global plan
//    std::vector<geometry_msgs::PoseStamped> empty_plan;
//    empty_plan.push_back(geometry_msgs::PoseStamped());
//    global_plan_ = empty_plan;

    goal_reached_ = false;
    initialized_ = true;
    return;
  }

  bool PathFollower::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    geometry_msgs::Twist zero_vel;

    //make sure planner had been initialized
    if(!initialized_)
    {
      ROS_ERROR("omni_path_follower: planner has not been initialized");
      return false;
    }

    //get the current robot pose in the costmap
    tf::Stamped<tf::Pose> robot_pose;
    if(!costmap_ros_->getRobotPose(robot_pose))
    {
      cmd_vel = zero_vel;
      ROS_ERROR("path_executer: cannot get robot pose");
      return false;
    }

    Eigen::Vector2d vec_lastnext(next_waypoint_.position.x - last_waypoint_.position.x,
                                 next_waypoint_.position.y - last_waypoint_.position.y);
    Eigen::Vector2d vec_lastrob(robot_pose.getOrigin().getX() - last_waypoint_.position.x,
                                robot_pose.getOrigin().getY() - last_waypoint_.position.y);
    Eigen::Vector2d vec_nextrob(robot_pose.getOrigin().getX() - next_waypoint_.position.x,
                                robot_pose.getOrigin().getY() - next_waypoint_.position.y);

    double robot_angle = tf::getYaw(robot_pose.getRotation());
    double path_angle = atan2(vec_lastnext[1],vec_lastnext[0]);
    double delta_angle = angles::shortest_angular_distance(path_angle,robot_angle);

    double len_lastnext = vec_lastnext.norm();

    //shortest distance from robot to path
    double cross = vec_lastnext[0]*vec_lastrob[1] - vec_lastnext[1]*vec_lastrob[0];
    double to_path_dist = cross/len_lastnext;   //TODO norm = 0?!

    //distance along path between last and next waypoint
    double along_path_dist = len_lastnext - vec_lastnext.dot(vec_lastrob)/len_lastnext;

    double to_path_vel = - to_path_k_ * to_path_dist;
    double in_path_vel = in_path_vel_;

    //if we are in the last path segment, slow down
    if(path_index_ == (path_length_ - 2))
    {
      in_path_vel = in_path_vel_ / len_lastnext * along_path_dist;

      double path_dist = hypot(to_path_dist, along_path_dist);

      if(path_dist < goal_threshold_)
      {
        cmd_vel = zero_vel;
        goal_reached_ = true;
        return true;
      }
    }

    //rotate velocity into robot frame
    cmd_vel.linear.x = cos(delta_angle)*in_path_vel + sin(delta_angle)*to_path_vel;
    cmd_vel.linear.y = -sin(delta_angle)*in_path_vel + cos(delta_angle)*to_path_vel;
    cmd_vel.angular.z = -angle_k_ * delta_angle;

    //check if we need to switch to the next path segment
    double dot = (-1*vec_lastnext).dot(vec_nextrob);
    if(dot < 0 && path_index_ < (path_length_ - 2))
    {
      ROS_DEBUG("path_follower: next waypoint");
      path_index_ += 1;
      last_waypoint_ = global_plan_.at(path_index_).pose;
      next_waypoint_ = global_plan_.at(path_index_+1).pose;
    }

    return true;
  }

  bool PathFollower::isGoalReached()
  {
    return goal_reached_;
  }

  bool PathFollower::setPlan(const std::vector< geometry_msgs::PoseStamped > &plan)
  {
    if(!initialized_)
    {
      ROS_ERROR("path follower: planner has not been initialized");
      return false;
    }

    ROS_DEBUG("path follower: got plan");
    global_plan_  = plan;

    path_index_ = 0;
    path_length_ = global_plan_.size();
    last_waypoint_ = global_plan_.at(path_index_).pose;
    next_waypoint_ = global_plan_.at(path_index_ + 1).pose;

    goal_reached_ = false;
    return true;
  }
}
