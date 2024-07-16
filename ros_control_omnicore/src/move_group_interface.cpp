
/*
NODO per l'utilizzo di move_group.
In questo nodo si implementano due funzioni, una per la pianificazione ed esecuzione di una traiettoria ricevendo dal topic "desiderate_joint",
l'altra dal topic "desiderate_pose". I due casi sono diversi in quanto, nel primo caso si specifica la posa del robot che si vuole raggiungere
in termini di pose dei giunti, nel secondo invece viene indicata la posizione in termini cartesiani e di rotazioni rx, ry, rz, rw.
*/

#include "../include/move_group_function.hpp"
#include "geometric_shapes/shape_operations.h"


int main(int argc, char **argv){

  ros::init(argc, argv, "move_group_interface");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  MoveGroupFunction Function;
  Function.init();
  ros::Rate rate(100);
  while (ros::ok())
  {
  ros::spinOnce();
  Function.run();
  rate.sleep();
  }
  ros::shutdown();
  return 0;
}

MoveGroupFunction::MoveGroupFunction()
{
  //caricamento parametri di riferinento per l'ostacolo
  //parametri posizione del marker di riferimento rispetto alla terna di riferimento posta alla base del robot
  if (nh.hasParam("dx_marker_rif")){nh.getParam("dx_marker_rif", dx_marker_rif);ROS_INFO_STREAM("dx_marker_rif\t" << dx_marker_rif);}
  if (nh.hasParam("dy_marker_rif")){nh.getParam("dy_marker_rif", dy_marker_rif);ROS_INFO_STREAM("dy_marker_rif\t" << dy_marker_rif);}
  if (nh.hasParam("dz_marker_rif")){nh.getParam("dz_marker_rif", dz_marker_rif);ROS_INFO_STREAM("dz_marker_rif\t" << dz_marker_rif);}
  //parametri posizione del marker dell'ostacolo rispetto al centro dell'oggetto
  if (nh.hasParam("dx_marker_obstacle")){nh.getParam("dx_marker_obstacle", dx_marker_obstacle);ROS_INFO_STREAM("dx_marker_rif\t" << dx_marker_obstacle);}
  if (nh.hasParam("dy_marker_obstacle")){nh.getParam("dy_marker_obstacle", dy_marker_obstacle);ROS_INFO_STREAM("dy_marker_rif\t" << dy_marker_obstacle);}
  if (nh.hasParam("dz_marker_obstacle")){nh.getParam("dz_marker_obstacle", dz_marker_obstacle);ROS_INFO_STREAM("dz_marker_rif\t" << dz_marker_obstacle);}
  //parametri per dimensione ostacolo
  if (nh.hasParam("reduce_x")){nh.getParam("reduce_x", reduce_x);ROS_INFO_STREAM("reduce_x\t" << reduce_x);}
  if (nh.hasParam("reduce_y")){nh.getParam("reduce_y", reduce_y);ROS_INFO_STREAM("reduce_y\t" << reduce_y);}
  if (nh.hasParam("reduce_z")){nh.getParam("reduce_z", reduce_z);ROS_INFO_STREAM("reduce_z\t" << reduce_z);}
  //Subscriber
	sub_desiderate_joint_ = nh.subscribe("desiderate_joint", 1, &MoveGroupFunction::callback_desiderate_joint, this);
	sub_desiderate_pose_ = nh.subscribe("desiderate_pose", 1, &MoveGroupFunction::callback_desiderate_position, this);
  sub_marker_ = nh.subscribe("aruco_single/pose", 1, &MoveGroupFunction::callback_marker_pose, this);
  

  //Publisher
  pub_move_result = nh.advertise<std_msgs::Int8>("move_result", 1);

}

void MoveGroupFunction::Move_joint(std::vector<double> joint_group_positions)
{
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //inizio inserimento ostacolo
  Eigen::Vector3d b(reduce_x, reduce_y, reduce_z);
  moveit_msgs::CollisionObject collision_object;
  collision_object.id = "wall";
  shapes::Mesh* m = shapes::createMeshFromResource("file:///home/francesco/tesi3_ws/src/ABB_omnicore_ros_driver/ros_control_omnicore/src/box.stl", b);
  ROS_INFO("wall mesh loaded"); //debug
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  //dimensione oggetto e posa
  collision_object.meshes.resize(1);
  collision_object.mesh_poses.resize(1);
  collision_object.meshes[0] = mesh;
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.mesh_poses[0].position.x = pose_obstacle.position.x;
  collision_object.mesh_poses[0].position.y = pose_obstacle.position.y;
  collision_object.mesh_poses[0].position.z = pose_obstacle.position.z;
  collision_object.mesh_poses[0].orientation.x = 1.57;
  //caricamento oggetto nello spazio di lavoro e aggiuta agli oggetti di collisione
  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
  collision_object.operation = collision_object.ADD;
  std::vector<moveit_msgs::CollisionObject> collision_vector;
  collision_vector.push_back(collision_object);
  planning_scene_interface.applyCollisionObjects(collision_vector);
  ROS_INFO("Wall added into the world");

  const robot_state::JointModelGroup* joint_model_group;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = false;
  moveit::core::RobotStatePtr current_state;
  ROS_INFO("move_group_Id= %s", move_group.getPlannerId().c_str());
  ROS_INFO("move_group_Id= %s", move_group.getPlannerId().c_str());
  current_state = move_group.getCurrentState();
  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //carica la posizione attuale
  move_group.setJointValueTarget(joint_group_positions);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED"); //debug
  success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS); 
  ROS_INFO("Succes o no?: %d", success); //debug
  std_msgs::Int8 mes;
  mes.data =1;
  //creazione loop che tenta l'esecuzione del percorso pianificato finchè non ha raggiunto il punto desiderato
  while(!success)
  {
    success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Succes o no?: %d", success); //debug
  }
  pub_move_result.publish(mes); //pubblica sul topic move_result il caso in cui si è terminata l'esecuzione del movimento
}

void MoveGroupFunction::Move_point(geometry_msgs::Pose target_pose)
{
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //inizio inserimento object
  Eigen::Vector3d b(0.2, .2, .2);
  collision_object.id = "wall";
  shapes::Mesh* m = shapes::createMeshFromResource("file:///home/francesco/tesi3_ws/src/ABB_omnicore_ros_driver/ros_control_omnicore/src/box.stl", b);
  ROS_INFO("wall mesh loaded");
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  //dimensione oggetto e posa
  collision_object.meshes.resize(1);
  collision_object.mesh_poses.resize(1);
  collision_object.meshes[0] = mesh;
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.mesh_poses[0].position.x = pose_obstacle.position.x;
  collision_object.mesh_poses[0].position.y = pose_obstacle.position.y;
  collision_object.mesh_poses[0].position.z = pose_obstacle.position.z;
  collision_object.mesh_poses[0].orientation.x = 1.57;
  //caricamento oggetto nello spazio di lavoro e aggiuta agli oggetti di collisione
  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
  collision_object.operation = collision_object.ADD;
  std::vector<moveit_msgs::CollisionObject> collision_vector;
  collision_vector.push_back(collision_object);

  planning_scene_interface.applyCollisionObjects(collision_vector);
  ROS_INFO("Wall added into the world");
  bool success = false;
  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //carica la posizione attuale
  move_group.setPoseTarget(target_pose);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED"); //debug
  success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS); 
  ROS_INFO("Succes o no?: %d", success);  //debug
  std_msgs::Int8 mes;
  mes.data =1;
  //creazione loop che tenta l'esecuzione del percorso pianificato finchè non ha raggiunto il punto desiderato
  while(!success)
  {
    success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS); 
    ROS_INFO("Succes o no?: %d", success);//debug
  }
  pub_move_result.publish(mes); //pubblica sul topic move_result il caso in cui si è terminata l'esecuzione del movimento
}



void MoveGroupFunction::callback_desiderate_joint(const sensor_msgs::JointState::ConstPtr& msg)
{
  //ROS_INFO("E' arrivato un dato"); //debug
  std::vector<double> joint_group_positions;
  joint_group_positions.resize((unsigned)7);
  joint_group_positions[0]=msg->position[0];
  joint_group_positions[1]=msg->position[1];
  joint_group_positions[2]=msg->position[2];
  joint_group_positions[3]=msg->position[3];
  joint_group_positions[4]=msg->position[4];
  joint_group_positions[5]=msg->position[5];
  joint_group_positions[6]=msg->position[6];
  this->Move_joint(joint_group_positions);

}

void MoveGroupFunction::callback_desiderate_position(const geometry_msgs::Pose::ConstPtr& msg)
{
  geometry_msgs::Pose target_pose;
  target_pose.position.x = msg->position.x;
  target_pose.position.y = msg->position.y;
  target_pose.position.z = msg->position.z;
  target_pose.orientation.x = msg->orientation.x;
  target_pose.orientation.y = msg->orientation.y;
  target_pose.orientation.z = msg->orientation.z;
  target_pose.orientation.w = msg->orientation.w;
  this->Move_point(target_pose);
  
}

void MoveGroupFunction::callback_marker_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if(msg->header.frame_id == "26")
  {
    pose_marker_obstacle.position.x = msg->pose.position.x;
    pose_marker_obstacle.position.y = msg->pose.position.y;
    pose_marker_obstacle.position.z = msg->pose.position.z;
    pose_marker_obstacle.orientation.x = msg->pose.orientation.x;
    pose_marker_obstacle.orientation.y = msg->pose.orientation.y;
    pose_marker_obstacle.orientation.z = msg->pose.orientation.z;
    pose_marker_obstacle.orientation.w = msg->pose.orientation.w;
    //ROS_INFO("Ho trovato il 26"); //debug
  }
  else if(msg->header.frame_id == "582")
  {
    pose_marker_rif.position.x = msg->pose.position.x;
    pose_marker_rif.position.y = msg->pose.position.y;
    pose_marker_rif.position.z = msg->pose.position.z;
    pose_marker_rif.orientation.x = msg->pose.orientation.x;
    pose_marker_rif.orientation.y = msg->pose.orientation.y;
    pose_marker_rif.orientation.z = msg->pose.orientation.z;
    pose_marker_rif.orientation.w = msg->pose.orientation.w;
    //ROS_INFO("Ho trovato il 582"); //debug
  }
  
}

void MoveGroupFunction::run()
{

  pose_obstacle.position.x = - pose_marker_obstacle.position.x +pose_marker_rif.position.x + dx_marker_obstacle + dx_marker_rif;
  pose_obstacle.position.y = - pose_marker_obstacle.position.z +pose_marker_rif.position.z + dy_marker_obstacle + dy_marker_rif;
  pose_obstacle.position.z = - pose_marker_obstacle.position.y +pose_marker_rif.position.y + dz_marker_obstacle + dz_marker_rif;
  float distanza;
  distanza = sqrt(pow(pose_obstacle.position.x,2) + pow(pose_obstacle.position.y,2) +pow(pose_obstacle.position.z,2));
  // debug
  // ROS_INFO("La distanza attualmente lungo x è di: %f", pose_obstacle.position.x);
  // ROS_INFO("La distanza attualmente lungo y è di: %f", pose_obstacle.position.y);
  // ROS_INFO("La distanza attualmente lungo z è di: %f", pose_obstacle.position.z);
  // collision_object.mesh_poses[0].position.x = -0.1;
  // ROS_INFO("La distanza è: %f", distanza);
}

void MoveGroupFunction::init()
{
  //ROS_INFO("Sono in int"); //debug
  pose_marker_rif.position.x = 0;
  pose_marker_rif.position.y = 0;
  pose_marker_rif.position.z = 0;
  pose_marker_rif.orientation.x = 0;
  pose_marker_rif.orientation.y = 0;
  pose_marker_rif.orientation.z = 0;
  pose_marker_rif.orientation.w = 0;
  pose_marker_obstacle.position.x = 0;
  pose_marker_obstacle.position.y = 0;
  pose_marker_obstacle.position.z = 0;
  pose_marker_obstacle.orientation.x = 0;
  pose_marker_obstacle.orientation.y = 0;
  pose_marker_obstacle.orientation.z = 0;
  pose_marker_obstacle.orientation.w = 0;
}