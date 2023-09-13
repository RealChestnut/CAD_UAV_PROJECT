

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

#include "cad_uav_controller.hpp"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "cad_uav");
  ros::NodeHandle nh;

  //initialize ros node//
  //initSubscriber();
  
    /////////////////////////////////////////////////SUBSCFRIBER START//////////////////////////////////////////////////////
    dynamixel_state = nh.subscribe("joint_states",100,jointstate_Callback, ros::TransportHints().tcpNoDelay());
    att = nh.subscribe("/imu/data",1,imu_Callback,ros::TransportHints().tcpNoDelay());
    rc_in = nh.subscribe("/sbus",100,sbus_Callback,ros::TransportHints().tcpNoDelay());
    battery_checker = nh.subscribe("/battery",100,battery_Callback,ros::TransportHints().tcpNoDelay());
    t265_position=nh.subscribe("/t265_pos",100,t265_position_Callback,ros::TransportHints().tcpNoDelay());
    t265_odom=nh.subscribe("/rs_t265/odom/sample",100,t265_Odom_Callback,ros::TransportHints().tcpNoDelay());

    /////////////////////////////////////////////////PUBLISHER START//////////////////////////////////////////////////////
    PWMs = nh.advertise<std_msgs::Int16MultiArray>("PWMs", 1); 
    PWM_generator = nh.advertise<std_msgs::Int32MultiArray>("command",1);  // publish to pca9685
    desired_motor_thrust = nh.advertise<std_msgs::Float32MultiArray>("Forces",100);

    goal_dynamixel_position  = nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position",100); // desired theta1,2

    euler = nh.advertise<geometry_msgs::Vector3>("angle",1); 
    desired_angle = nh.advertise<geometry_msgs::Vector3>("desired_angle",100);


    desired_torque = nh.advertise<geometry_msgs::Vector3>("torque_d",100);

    linear_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel",100);
    desired_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel_d",100);

    angular_velocity = nh.advertise<geometry_msgs::Vector3>("gyro",100);

    desired_position = nh.advertise<geometry_msgs::Vector3>("pos_d",100);
    position = nh.advertise<geometry_msgs::Vector3>("pos",100);

    desired_force = nh.advertise<geometry_msgs::Vector3>("force_d",100);

    battery_voltage = nh.advertise<std_msgs::Float32>("battery_voltage",100);
    delta_time = nh.advertise<std_msgs::Float32>("delta_t",100);


 
  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    Clock();
    shape_detector();
    UpdateParameter();
    if(){
    Command_Generator();
    attitude_controller();
    position_controller();
    altitude_controller();

    Accelerometer_LPF();
    velocity_controller();
    
    setCM_Xc_p2();
    K_matrix();

    PWM_signal_Generator();
    }


    PublishData();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


