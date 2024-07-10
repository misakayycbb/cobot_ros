/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, SIASUN Robotics
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */




#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>

#include "duco_driver/DucoRobotInterface.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gcr5_demo");
  ros::NodeHandle node_handle;
  duco_driver::DucoRobotInterface  duco_robot;
  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  duco_robot.powerOn();

  duco_robot.enable();


  // Define the planning group name
  static const std::string PLANNING_GROUP = "duco_gcr5";


  // Create a planning group interface object and set up a planning group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPoseReferenceFrame("base_link");


  // Create a planning scene interface object
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  // Create a robot model information object
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  // Create an object of the visualization class
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();


  // Load remote control tool
  visual_tools.loadRemoteControl();


  // Create text
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.2;
  visual_tools.publishText(text_pose, "Duco Demo", rvt::RED, rvt::XLARGE);
  // Text visualization takes effect
  visual_tools.trigger();


  // Get the coordinate system of the basic information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());


  // Get the end of the basic information
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Visual terminal prompt (blocking)
  visual_tools.prompt("Press 'next'0 in the RvizVisualToolsGui window to start the demo");
  //IO Set
  duco_robot.ioOut(duco_driver::IOType::kStandardDO, 5, true);

  usleep(100000);

  bool value = false;
  duco_robot.ioIn(duco_driver::IOType::kStandardDO, 5, value);
  std::cout <<"duco_robot.ioIn(5, value) value " << value << std::endl;

  duco_robot.ioOut(duco_driver::IOType::kToolDO, 1, true);

  usleep(100000);

  value = false;
  duco_robot.ioIn(duco_driver::IOType::kToolDO, 1, value);
  std::cout <<" duco_robot.toolIoIn(0, value) value " << value << std::endl;

  // Visual terminal prompt (blocking)
  visual_tools.prompt("Press 'next'1 in the RvizVisualToolsGui window to start the demo");
  
  //***************************************************************************************************  Home Position

  std::vector<double> home_position;
  home_position.push_back(-0.001255);
  home_position.push_back(-0.148822);
  home_position.push_back(-1.406503);
  home_position.push_back(0.311441);
  home_position.push_back(-1.571295);
  home_position.push_back(-0.002450);
  move_group.setJointValueTarget(home_position);
  move_group.move();


  visual_tools.prompt("Press 'next'2 in the RvizVisualToolsGui window to continue the demo");

  //**************************************************************************************************   First example: planning and moving to a target pose
  // Set the target pose , RPY mode (rotation around the reference axis X, Y, Z)
  tf::Quaternion q;
  q.setRPY(3.14,0,-1.57);       //radian

  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = -0.4;
  target_pose1.position.y = -0.3;
  target_pose1.position.z = 0.30;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();

  move_group.setPoseTarget(target_pose1);

  // Call the planner for planning calculations Note: This is just planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");


  // visual planning path in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Duco Pose Goal Example1", rvt::RED, rvt::XLARGE);
  // Parameter 1 (trajectory_): path information
  // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();


  // Perform planning actions
  move_group.execute(my_plan);
  


  visual_tools.prompt("Press 'next'3 in the RvizVisualToolsGui window to continue the demo");

// **************************************************************************************************    The Third example, plan circle.

std::vector<double> circle_center;
circle_center.push_back(-2.4096);
circle_center.push_back(-0.0726);
circle_center.push_back(-1.8542);
circle_center.push_back( 0.3569);
circle_center.push_back( 1.5691);
circle_center.push_back( -2.4112);

move_group.setJointValueTarget(circle_center);
move_group.move();

sleep(1);

visual_tools.prompt("Press 'next'4 move to circle center ");

      // 设置机器人终端的目标位置
geometry_msgs::Pose target_pose;//设定圆心的位置

q.setRPY(3.14,0,1.57);       //radian

target_pose.orientation.x =  q.x();
target_pose.orientation.y =  q.y();
target_pose.orientation.z =  q.z();
target_pose.orientation.w = q.w();

target_pose.position.x = 0.464692;
target_pose.position.y = 0.228434;
target_pose.position.z = 0.315347;

//move_group.setPoseTarget(target_pose);
//move_group.move();//首先运动到圆心的位置
sleep(1);

std::vector<geometry_msgs::Pose> waypoint2;
waypoint2.push_back(target_pose);

  double centerA = target_pose.position.y;
  double centerB = target_pose.position.z;
  double radius = 0.100;

  for(double th=0.0; th<3.14; th=th+0.01)
  {
      target_pose.position.y = centerA + radius * cos(th);
      target_pose.position.z = centerB + radius * sin(th);
      waypoint2.push_back(target_pose);
  }

moveit_msgs::RobotTrajectory trajectory1;
const double jump_threshold1 = 0.0;
const double eef_step1 = 0.01;
double fraction1 = 0.0;
int maxtries = 100;   //最大尝试规划次数
int attempts = 0;     //已经尝试规划次数

while(fraction1 < 1.0 && attempts < maxtries)
{
      fraction1 = move_group.computeCartesianPath(waypoint2, eef_step1, jump_threshold1, trajectory1);
      attempts++;
      
      if(attempts % 10 == 0)
          ROS_INFO("Still trying after %d attempts...", attempts);
}

  // Visualize the plan in RViz
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "SIASUN Joint Space Goal Example4", rvt::RED, rvt::XLARGE);
visual_tools.publishPath(waypoint2, rvt::LIME_GREEN, rvt::SMALL);
for (std::size_t i = 0; i < waypoint2.size(); ++i)
{
    visual_tools.publishAxisLabeled(waypoint2[i], "pt" + std::to_string(i), rvt::SMALL);
}
visual_tools.trigger();
  
if(fraction1 == 1)
{   
  ROS_INFO("Path computed successfully. Moving the arm.");

  // 生成机械臂的运动规划数据
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory1;

  // 执行运动
  move_group.execute(plan);
  sleep(1);
}
else
{
    ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction1, maxtries);
}

 visual_tools.prompt("Press 'next'5 Back to start_joint");

std::vector<double> start_joint1;
start_joint1.push_back( 1.8024);
start_joint1.push_back( 0.1257);
start_joint1.push_back(-2.2681);
start_joint1.push_back( 0.5866);
start_joint1.push_back( 1.5708);
start_joint1.push_back( -2.9100);


ROS_INFO_NAMED("tutorial", "Move to the home point position");
move_group.setJointValueTarget(start_joint1);
move_group.move();



visual_tools.prompt("Press 'next'5 in the RvizVisualToolsGui window to continue the demo");
  // **************************************************************************************************   Fourth example: planning and moving a Cartesian interpolation path

  q.setRPY(3.14,0,-1.57);
  geometry_msgs::Pose     start_pose2;
  start_pose2.position.x = -0.4;
  start_pose2.position.y = 0.05;
  start_pose2.position.z = 0.54;
  start_pose2.orientation.x = q.x();
  start_pose2.orientation.y = q.y();
  start_pose2.orientation.z = q.z();
  start_pose2.orientation.w = q.w();
  //  Add three waypoints
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.15;
  waypoints.push_back(target_pose3);  // right

  // target_pose3.position.z += 0.2;
  // target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left


  // Reduce the speed of the robot arm by the scaling factor of the maximum speed of each joint. Please note that this is not the speed of the final effector.
  move_group.setMaxVelocityScalingFactor(0.5);



  // We want the Cartesian path to be interpolated at a resolution of 1 cm.
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;           //(The jump threshold is set to 0.0)
  const double eef_step = 0.01;                //(interpolation step)



  // Calculate Cartesian interpolation path: return path score (0~1, -1 stands for error)
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan  (Cartesian path) (%.2f%% acheived)", fraction * 100.0);


  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "SIASUN Joint Space Goal Example4", rvt::RED, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  }
  visual_tools.trigger();


  my_plan.trajectory_= trajectory;
  move_group.execute(my_plan);
  
  sleep(2);

  visual_tools.prompt("Press 'next'6  Back To Home");

  // Move to the home point position
  move_group.setJointValueTarget(home_position);
  move_group.move();

  visual_tools.prompt("Press 'next'7  ADD a OBject in the RvizVisualToolsGui window to continue the demo");



  // **************************************************************************************************   Fifth example: adding obstacles

  // Define a collision object
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();


  // Set the ID of the object
  collision_object.id = "box1";


  // ********************************Defining a box (obstacle) added to the world
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.05;
  primitive.dimensions[2] = 0.4;

  // 设置盒子(障碍物)的位置和姿态
  // Set the box (obstacle) pose
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -0.3;
  box_pose.position.y = 0.2;
  box_pose.position.z = 0.54;


  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;


  // *********************************Defining a box (desktop) added in the world
  moveit_msgs::CollisionObject collision_object2;
  collision_object2.header.frame_id = move_group.getPlanningFrame();

  // Set the ID of the object
  collision_object2.id = "box2";


  // Set length, width and height
  shape_msgs::SolidPrimitive primitive2;
  primitive2.type = primitive.BOX;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[0] = 1.7;
  primitive2.dimensions[1] = 1.7;
  primitive2.dimensions[2] = 0.05;


  // Set the box (Desktop) pose
  geometry_msgs::Pose box_pose2;
  box_pose2.orientation.w = 1.0;
  box_pose2.position.x = 0.0;
  box_pose2.position.y = 0.0;
  box_pose2.position.z = 0.0;

  collision_object2.primitives.push_back(primitive2);
  collision_object2.primitive_poses.push_back(box_pose2);
  collision_object2.operation = collision_object2.ADD;


  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  collision_objects.push_back(collision_object2);

  planning_scene_interface.addCollisionObjects(collision_objects);


  // Show text in RViz of status
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Duco Add object Example5", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next'6 in the RvizVisualToolsGui window to once the collision object appears in RViz");



  // **************************************************************************************************   Sixth example: obstacle avoidance movement


  // Add a track to avoid obstacle movement
  q.setRPY(1.77,-0.59,-1.79 );                       // radian

  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.x = q.x();
  another_pose.orientation.y = q.y();
  another_pose.orientation.z = q.z();
  another_pose.orientation.w = q.w();
  another_pose.position.x = -0.37;
  another_pose.position.y = 0.6;
  another_pose.position.z = 0.4;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");


  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Duco Obstacle Goal Exalmple6", rvt::RED, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();


  // Perform planning actions
  move_group.execute(my_plan);


  // Move to the home point position
  #if 0
  move_group.setJointValueTarget(home_position);
  move_group.move();
  #endif 

  visual_tools.prompt("next step 7 attach the collision object to the robot");



  // **************************************************************************************************   Seventh example: Simulated collision

  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  visual_tools.publishText(text_pose, "Duco Object attached to robot Example7", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next'8 in the RvizVisualToolsGui window to once the collision object attaches to the "
                      "robot");



  // **************************************************************************************************   Eighth example: Simulated no collision

  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  visual_tools.publishText(text_pose, "Duco Object dettached from robot Example8", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next'9 in the RvizVisualToolsGui window to once the collision object detaches to the "
                      "robot");




  //**************************************************************************************************    Ninth example: removing obstacles


  // Remove obstacles
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Duco Object1 removed EXample9", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();



   //**************************************************************************************************    Tenth example: random motion (screened, users need to be able to open it themselves. Tip: random motion is uncertain, users need to hold the emergency stop to pay attention to safety)



  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Duco Random Move Exalmple 10", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  for(int i=0;i<30;i++)
  {

    move_group.setRandomTarget();

    move_group.move();

    ROS_INFO_NAMED("tutorial", "Random Moving %d:",i);

  }
  visual_tools.prompt("Press 'next'10 : Robot Random Moving");


   // Remove the desktop
   ROS_INFO_NAMED("tutorial", "Remove the desktop from the world");
   object_ids.push_back(collision_object2.id);
   planning_scene_interface.removeCollisionObjects(object_ids);
   // Show text in RViz of status
   visual_tools.deleteAllMarkers();
   visual_tools.publishText(text_pose, " Finish ", rvt::RED, rvt::XLARGE);
   visual_tools.trigger();

  duco_robot.powerOff();
  duco_robot.disable();

   // END_TUTORIAL
   ros::shutdown();
   return 0;
}
