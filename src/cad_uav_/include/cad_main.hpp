/* Authors: Yeong In Song */


#ifndef CAD_UAV_CONTROL_LOOP_H_
#define CAD_UAV_CONTROL_LOOP_H_


#include <vector>
#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/String.h>
#include <cmath>
#include <cstdio>
#include <chrono>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "nav_msgs/Odometry.h"

//----------FOR GROUND STATION---------------//
#include "FAC_MAV/ArmService.h" //ASDF
#include "FAC_MAV/KillService.h" //ASDF
#include "FAC_MAV/PosCtrlService.h" //ASDF
#include "FAC_MAV/HoverService.h" //ASDF
#include "FAC_MAV/FAC_HoverService.h" //ASDF

//-------------------------------------------//

class cad_agent_control_loop
{
 public:
  cad_agent_control_loop();
  ~cad_agent_control_loop();
  
  //-----changable data-----//
  void UpdateParameter();
  
  
private:


/*****************************************************************************
** ROS NodeHandle
*****************************************************************************/
ros::NodeHandle node_handle_;
ros::NodeHandle priv_node_handle_;


/*****************************************************************************
** Init Functions
*****************************************************************************/

void initSubscriber();
void initParameter();

/*****************************************************************************
** ROS Subscribers, Callback Functions and Relevant Functions
*****************************************************************************/

ros::Subscriber dynamixel_state;
ros::Subscriber att;
ros::Subscriber rc_in;
ros::Subscriber battery_checker;
ros::Subscriber t265_pos;
ros::Subscriber t265_rot;
ros::Subscriber t265_odom;

void jointstateCallback(const sensor_msgs::JointState& msg);
void imu_Callback(const sensor_msgs::Imu& msg);
sensor_msgs::JointState servo_msg_create(double desired_theta1, double desired_theta2, double desired_theta3, double desired_theta4);
void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array);
void batteryCallback(const std_msgs::Int16& msg);
void posCallback(const geometry_msgs::Vector3& msg);
void rotCallback(const geometry_msgs::Quaternion& msg);
void filterCallback(const sensor_msgs::Imu& msg);
void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

};

#endif
