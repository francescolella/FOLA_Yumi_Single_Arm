#include <string>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


class MoveGroupFunction
{
  
public:
  MoveGroupFunction();
  void Move_joint(std::vector<double> joint_group_positions);
  void Move_point(geometry_msgs::Pose target_pose);     
  void callback_desiderate_joint(const sensor_msgs::JointState::ConstPtr& msg);
  void callback_desiderate_position(const geometry_msgs::Pose::ConstPtr& msg);
  void callback_marker_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void run();
  void init();
  geometry_msgs::Pose pose_marker_rif;
  geometry_msgs::Pose pose_marker_obstacle;
  geometry_msgs::Pose pose_obstacle;
  moveit_msgs::CollisionObject collision_object;
  float dx_marker_rif, dy_marker_rif, dz_marker_rif, dx_marker_obstacle, dy_marker_obstacle, dz_marker_obstacle, reduce_x, reduce_y, reduce_z;
private:
ros::NodeHandle nh;
ros::Subscriber sub_desiderate_joint_, sub_desiderate_pose_, sub_marker_;
ros::Publisher pub_move_result;
};