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

  bool PathFollower::posesEqual(geometry_msgs::PoseStamped first, geometry_msgs::PoseStamped second)
  {
    //    first.pose.position - second.pose.position;
    if(fabs(first.pose.position.x - second.pose.position.x) < EPSILON &&
       fabs(first.pose.position.y - second.pose.position.y) < EPSILON &&
       fabs(first.pose.position.z - second.pose.position.z) < EPSILON
       )
      return true;

    return false;
  }

  void PathFollower::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
  {
    tfl_ = tf;
    costmap_ros_ = costmap_ros;

    in_path_vel_ = 0.4;
    to_path_k_ = 0.75;
    angle_k_ = 0.5;
    goal_threshold_ = 0.5;
    max_lin_vel_ = 0.75;
    max_ang_vel_ = 1.5;
    min_lin_vel_ = 0.01;
    min_ang_vel_ = 0.05;
    max_path_offset_ = 3.5;
    parking_scale_ = 0.5;

    rotate_to_path_ = true;
    rotate_at_start_ = true;
    rotating_ = false;
    path_index_offset_ = 8;

    //initialize empty global plan
    std::vector<geometry_msgs::PoseStamped> empty_plan;
    empty_plan.push_back(geometry_msgs::PoseStamped());
    global_plan_ = empty_plan;

    goal_reached_ = false;
    initialized_ = true;

    ros::NodeHandle nh;
    config_subscriber_ = nh.subscribe("omni_path_follower/config", 1, &PathFollower::config_callback, this);
    waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("omni_path_follower/current_waypoint", 1);
    return;
  }

  void PathFollower::config_callback(Config msg)
  {
    ROS_INFO("setting new configs");
    in_path_vel_ = msg.in_path_vel;
    rotate_vel_ = msg.rotate_vel;
    rotate_to_path_ = msg.rotate_to_path;
  }

  bool PathFollower::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    geometry_msgs::Twist zero_vel;
    if(path_length_ == 0)
    {
      ROS_INFO("omni path follower: path is empty");
      cmd_vel = zero_vel;
      return true;
    }

    if(goal_.header.frame_id.compare("map")>1)
    {
      ROS_ERROR("omni path follower can only process paths in map frame");
      ROS_ERROR_STREAM("goal frame: " << goal_.header.frame_id << std::endl);
      return false;
    }

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

    //transform robot pose in path frame id
    try
    {
      tfl_->waitForTransform(goal_.header.frame_id, robot_pose.frame_id_,
                             robot_pose.stamp_, ros::Duration(0.2));
      tfl_->transformPose(goal_.header.frame_id, robot_pose, robot_pose);
    }

    catch(tf::TransformException ex)
    {
      ROS_ERROR("path_follower: could not transform robot pose in goal frame, "
                "tf anwered: %s", ex.what());
      cmd_vel = zero_vel;
      return true;
    }

    /** calculate velocity commands **/

    int follow_idx = std::min(path_index_+path_index_offset_+1, path_length_-1);
    geometry_msgs::PoseStamped follow_waypoint = global_plan_.at(follow_idx);

    //publish follow waypoint for debugging
    waypoint_pub_.publish(follow_waypoint);

    Eigen::Vector2d vec_lastfollow(follow_waypoint.pose.position.x - last_waypoint_.position.x,
                                   follow_waypoint.pose.position.y - last_waypoint_.position.y);
    Eigen::Vector2d vec_lastrob(robot_pose.getOrigin().getX() - last_waypoint_.position.x,
                                robot_pose.getOrigin().getY() - last_waypoint_.position.y);

    double robot_angle = tf::getYaw(robot_pose.getRotation());
    double path_angle = atan2(vec_lastfollow[1],vec_lastfollow[0]);
    double delta_angle = angles::shortest_angular_distance(path_angle,robot_angle);

    double len_lastfollow = vec_lastfollow.norm();

    //shortest distance from robot to path
    double cross = vec_lastfollow[0]*vec_lastrob[1] - vec_lastfollow[1]*vec_lastrob[0];
    double to_path_dist = cross/len_lastfollow;   //TODO norm = 0?!

     if(fabs(to_path_dist) > max_path_offset_)
     {
       ROS_INFO("omni path follower: distance to path too big!");
       cmd_vel = zero_vel;
       return false;
     }

    //velocity controller
    double to_path_vel = - to_path_k_ * to_path_dist;
    double in_path_vel = in_path_vel_;
    double rotate_vel = -angle_k_ * delta_angle;

    //if we are close to the goal, slow down
    Eigen::Vector2d vec_goalrob(robot_pose.getOrigin().getX() - goal_.pose.position.x,
                                robot_pose.getOrigin().getY() - goal_.pose.position.y);
    double goal_dist = vec_goalrob.norm();

    //parking in to goal
    if(goal_dist < goal_threshold_ || path_index_ >= (path_length_ - 2))
    {
      ROS_DEBUG("parking move");
      //make sure we dont do out of parking move
      path_index_ = path_length_ - 1;

      //get goal in robot coordinate frame
      tf::Transform trafo_robot_in_world(robot_pose.getRotation(), robot_pose.getOrigin());
      tf::Transform trafo_world_in_robot = trafo_robot_in_world.inverse();

      tf::Point goal_in_world;
      tf::pointMsgToTF(goal_.pose.position, goal_in_world);
      tf::Point goal_in_robot = trafo_world_in_robot * goal_in_world;

      double goal_angle = tf::getYaw(goal_.pose.orientation);
      delta_angle = angles::shortest_angular_distance(goal_angle, robot_angle);

      if(goal_dist > goal_threshold_)
      {
        //if we went into parking because of the path index
        ROS_DEBUG("above threshold, scaling!");
        goal_in_robot.setX(goal_in_robot.x() * goal_threshold_ / goal_dist);
        goal_in_robot.setY(goal_in_robot.y() * goal_threshold_ / goal_dist);
      }

      cmd_vel.linear.x = parking_scale_ * in_path_vel * goal_in_robot.x() / goal_threshold_;
      cmd_vel.linear.y = parking_scale_ * in_path_vel * goal_in_robot.y() / goal_threshold_;
      cmd_vel.angular.z = parking_scale_ * -angle_k_ * delta_angle / goal_threshold_;

      //once velocities are under threshold, report goal reached
      if(fabs(cmd_vel.linear.x) < min_lin_vel_ &&
         fabs(cmd_vel.linear.y) < min_lin_vel_ &&
         fabs(cmd_vel.angular.z) < min_ang_vel_)
      {
        cmd_vel = zero_vel;
        goal_reached_ = true;
      }

      return true;
    }

    //rotate velocity into robot frame
    cmd_vel.linear.x = cos(delta_angle)*in_path_vel + sin(delta_angle)*to_path_vel;
    cmd_vel.linear.y = -sin(delta_angle)*in_path_vel + cos(delta_angle)*to_path_vel;

    if(rotate_to_path_)
      cmd_vel.angular.z = rotate_vel;
    else
      cmd_vel.angular.z = rotate_vel_;

    //limit velocities
    double abs_lin_vel = hypot(cmd_vel.linear.x, cmd_vel.linear.y);
    if(abs_lin_vel > max_lin_vel_)
    {
      cmd_vel.linear.x = cmd_vel.linear.x * max_lin_vel_ / abs_lin_vel;
      cmd_vel.linear.y = cmd_vel.linear.y * max_lin_vel_ / abs_lin_vel;
    }

    if(fabs(cmd_vel.angular.z) > max_ang_vel_)
    {
      cmd_vel.angular.z = cmd_vel.angular.z / fabs(cmd_vel.angular.z) * max_ang_vel_;
    }

    /** update path index **/

    Eigen::Vector2d vec_nextlast(last_waypoint_.position.x - next_waypoint_.position.x,
                                 last_waypoint_.position.y - next_waypoint_.position.y);
    Eigen::Vector2d vec_nextrob(robot_pose.getOrigin().getX() - next_waypoint_.position.x,
                                robot_pose.getOrigin().getY() - next_waypoint_.position.y);

    double dot = vec_nextlast.dot(vec_nextrob);
    while(dot < 0.0 && path_index_ < (path_length_ - 2))
    {
      ROS_DEBUG("path_follower: next waypoint");
      path_index_ += 1;

      last_waypoint_ = global_plan_.at(path_index_).pose;
      next_waypoint_ = global_plan_.at(path_index_+1).pose;

      vec_nextlast << last_waypoint_.position.x - next_waypoint_.position.x,
          last_waypoint_.position.y - next_waypoint_.position.y;
      vec_nextrob << robot_pose.getOrigin().getX() - next_waypoint_.position.x,
          robot_pose.getOrigin().getY() - next_waypoint_.position.y;

      dot = vec_nextlast.dot(vec_nextrob);
    }

    //rotate only at the start
    if(rotating_)
    {
      if(fabs(delta_angle) > 0.5)
      {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
      }
      else
        rotating_ = false;
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

    //only reset waypoints and path index if the start changed
    if(!posesEqual(global_plan_.front(), plan.front()))
    {
      ROS_DEBUG("reset path index");
      path_index_ = 0;
      path_length_ = plan.size();
      last_waypoint_ = plan.at(path_index_).pose;
      next_waypoint_ = plan.at(path_index_ + 1).pose;
    }

    global_plan_  = plan;
    goal_reached_ = false;
    goal_ = global_plan_.back();

    if(rotate_at_start_)
      rotating_ = true;

    return true;
  }
}
