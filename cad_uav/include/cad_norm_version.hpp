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

#include "cad_util_function.hpp"

//----------FOR GROUND STATION-------------//
/*
#include "FAC_MAV/ArmService.h" //ASDF
#include "FAC_MAV/KillService.h" //ASDF
#include "FAC_MAV/PosCtrlService.h" //ASDF
#include "FAC_MAV/HoverService.h" //ASDF
#include "FAC_MAV/FAC_HoverService.h" //ASDF
*/
//-----------------------------------------//

class cad_Loop
{
public:
  cad_Loop();
  ~cad_Loop();
 

//////////////////////////////
// Parameter Initialization //
//////////////////////////////

double freq=200;//controller loop frequency

std::chrono::duration<double> delta_t;
int16_t Sbus[10];
int16_t PWM_d;
int16_t loop_time;
std_msgs::Int16MultiArray PWMs_command;
std_msgs::Int32MultiArray PWMs_value;
std_msgs::Float32MultiArray Force;
sensor_msgs::Imu imu;
geometry_msgs::Quaternion imu_quaternion;
geometry_msgs::Vector3 imu_rpy;
geometry_msgs::Vector3 imu_angular_velocity;
geometry_msgs::Vector3 imu_linear_acceleration;
geometry_msgs::Vector3 angle_d;

geometry_msgs::Vector3 angular_acceleration_desired;
Eigen::Vector3d rpy_ddot_desired; // angular acceleration desired vector for cal.

geometry_msgs::Vector3 position_from_t265; 
geometry_msgs::Vector3 linear_velocity_from_t265;
geometry_msgs::Vector3 angular_velocity_from_t265;
geometry_msgs::Vector3 lin_vel;
geometry_msgs::Vector3 prev_lin_vel;
geometry_msgs::Vector3 desired_lin_vel;

geometry_msgs::Vector3 ang_vel;
geometry_msgs::Quaternion t265_quat;
geometry_msgs::Quaternion desired_value;
geometry_msgs::Vector3 desired_pos;
geometry_msgs::Vector3 F_total;
geometry_msgs::Vector3 torque_d;
geometry_msgs::Vector3 force_d;
geometry_msgs::Vector3 t265_att;

geometry_msgs::Vector3 filtered_angular_rate;
geometry_msgs::Vector3 lin_acl;
std_msgs::Float32 altitude_d;
std_msgs::Float32 battery_voltage_msg;
std_msgs::Float32 battery_real_voltage;
std_msgs::Float32 dt;

Eigen::Vector3d X_c_p1;
Eigen::Vector3d X_c_p2;

///Time loop for control logic///
ros::Timer timer;
////////////////////////////////

bool servo_sw=false;
double theta1_command, theta2_command, theta3_command, theta4_command;
bool start_flag=false;
bool tilting_flag=false;

//Mode selection flag
bool attitude_mode = false;
bool velocity_mode = false;
bool position_mode = false;
bool kill_mode = true;
bool altitude_mode = false;
bool tilt_mode = false;

//module combined flag
bool mono_flight_mode;
bool combined_flight_mode = false;




//Thruster_cmd
double F1 = 0;//desired propeller 1 force
double F2 = 0;//desired propeller 2 force
double F3 = 0;//desired propeller 3 force
double F4 = 0;//desired propeller 4 force

//Global : XYZ  Body : xyz

double tau_r_d = 0;//roll  desired torque (N.m)
double tau_p_d = 0;//pitch desired torque(N.m)
double tau_y_d = 0;//yaw desired torque (N.m)
double tau_y_sin = 0; //yaw sine term torque (N.m)
double tau_y_d_non_sat=0;//yaw deried torque non-saturation (N.m)

double r_ddot_d = 0; //roll angular acceleration
double p_ddot_d = 0; //pitch angular acceleration
double y_ddot_d = 0; //yaw angular acceleration

double Thrust_d = 0;//altitude desired thrust(N)

double r_d = 0;//desired roll angle
double p_d = 0;//desired pitch angle
double y_d = 0;//desired yaw angle
double y_d_tangent = 0;//yaw increment tangent
double T_d = 0;//desired thrust

//Desired Global position
double X_d = 0;//desired X position
double Y_d = 0;//desired Y position 
double Z_d = 0;//desired altitude
double X_d_base = 0;//initial desired X position
double Y_d_base = 0;//initial desired Y position
double Z_d_base = 0;//initial desired Z position

//Global Desired Global velocity
double X_dot_d = 0;
double Y_dot_d = 0;
double Z_dot_d = 0;

//Global desired acceleration
double X_ddot_d = 0;
double Y_ddot_d = 0;
double Z_ddot_d = 0;

double alpha = 0;
double beta = 0;

//Body desired force
double F_xd = 0;
double F_yd = 0;
double F_zd = 0;

//Yaw safety
double yaw_prev = 0;
double yaw_now = 0;
double base_yaw = 0;
int yaw_rotate_count = 0;
//--------------------------------------------------------

//General dimensions

double r_arm = 0.3025;// m // diagonal length between thruster x2
double l_servo = 0.035;

double mass_system =0;
double mass = 5.6;//2.9;//3.8; 2.365;//(Kg)
double mass_sub1 = 0;
double mass_sub2 = 0;

double r2=sqrt(2);
double l_module = 0.50; //(m) module horizontal body length

//Propeller constants(DJI E800(3510 motors + 620S ESCs))
double xi = 0.01;//F_i=k*(omega_i)^2, M_i=b*(omega_i)^2
//--------------------------------------------------------

//General parameters======================================

double pi = 3.141592;//(rad)
double g = 9.80665;//(m/s^2)

double rp_limit = 0.25;//(rad)
double y_vel_limit = 0.01;//(rad/s)
double y_d_tangent_deadzone = (double)0.05 * y_vel_limit;//(rad/s)
double T_limit = 80;//(N)
double altitude_limit = 1;//(m)
double XY_limit = 1.0;
double XYZ_dot_limit=1;
double XYZ_ddot_limit=2;
double alpha_beta_limit=1;
double hardware_servo_limit=0.3;
double servo_command_limit = 0.3;
double tau_y_limit = 1.0;

//Body desired force limit
double F_xd_limit = mass*2.0;
double F_yd_limit = mass*2.0;

double x_c_hat=0.0;
double y_c_hat=0.0;
double z_c_hat=0.0;

//  Inertia Tensor elements


//Original inertia tensor diagonal
double J_xx = 0.01;
double J_yy = 0.01;
double J_zz = 0.1;
//Original inertia tensor off-diagonal
double J_xy = 0;
double J_xz = 0;

double J_yx = 0;
double J_yz = 0;

double J_zx = 0;
double J_zy = 0;


//Control gains===========================================

//integratior(PID) limitation
double integ_limit=10;
double z_integ_limit=100;
double position_integ_limit=10;
double velocity_integ_limit=10;

//Roll, Pitch PID gains
double Pa=3.5;
double Ia=0.4;
double Da=0.5;


//Yaw PID gains
double Py=2.0;
double Dy=0.1;

//Z Velocity PID gains
double Pz=16.0;
double Iz=5.0;
double Dz=15.0;

//XY Velocity PID gains
double Pv=5.0;
double Iv=0.1;
double Dv=5.0;

//Position PID gains
double Pp=3.0;
double Ip=0.1;
double Dp=5.0;

//Conventional Flight Mode Control Gains
double conv_Pa, conv_Ia, conv_Da;
double conv_Py, conv_Dy;
double conv_Pz, conv_Iz, conv_Dz;
double conv_Pv, conv_Iv, conv_Dv;
double conv_Pp, conv_Ip, conv_Dp;

//Tilt Flight Mode Control Gains
double tilt_Pa, tilt_Ia, tilt_Da;
double tilt_Py, tilt_Dy;
double tilt_Pz, tilt_Iz, tilt_Dz;
double tilt_Pv, tilt_Iv, tilt_Dv;
double tilt_Pp, tilt_Ip, tilt_Dp;

//error data for PID controller

double e_r_i = 0;//roll error integration
double e_p_i = 0;//pitch error integration
double e_y_i = 0;//yaw error integration
double e_X_i = 0;//X position error integration
double e_Y_i = 0;//Y position error integration
double e_Z_i = 0;//Z position error integration
double e_X_dot_i = 0;//X velocity error integration
double e_Y_dot_i = 0;//Y velocity error integration
double e_Z_dot_i = 0;//Z velocity error integration


//--------------------------------------------------------

//Servo angle=============================================
Eigen::VectorXd servo_theta;
//--------------------------------------------------------

//Voltage=================================================
double voltage=22.4;
double voltage_old=22.4;
//--------------------------------------------------------

template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//----------- for module docking ----------------//
int case_num = 0;
int module_num_count = 1; // number of modules
int shape_num=2;
int toggle_sub1 = 0; // setMoI toggle flag
int toggle_sub2 = 0;
//-----------------------------------------------//

////////////////////
// Init Functions //
////////////////////

void initParameter();
void initSubscriber();
void initPublisher();

//////////////////////////
// Controller Functions //
//////////////////////////
void command_generator();
void attitude_controller();
void position_controller();
void altitude_controller();
void velocity_controller();
void force_torque_distributor();



////////////////////
// Util Functions //
////////////////////
void shape_detector();
void UpdateParameter();
void attitude_DOB();
void K_matrix();
double Force_to_PWM(double F);
void callback(const ros::TimerEvent& event);


//------------------------------------------------------------//

private:

////////////////////
// ROS NodeHandle // 
////////////////////

ros::NodeHandle node_handle_;
ros::NodeHandle priv_node_handle_;

////////////////////
// Basic Function //
////////////////////

//Eigen::MatrixXd CM_Xc_p2(4,4); //Thrust Allocation
//Eigen::MatrixXd invCM_Xc_p2(4,4);



Eigen::MatrixXd setCM_Xc_p2(Eigen::VectorXd theta, Eigen::Vector3d CoM)
{

  Eigen::MatrixXd CM_Xc_p2(4,4); //Thrust Allocation
  Eigen::MatrixXd invCM_Xc_p2(4,4);
  double xc = CoM(0);
  double yc = CoM(1);
  double zc = CoM(2);
	
	CM_Xc_p2 << 
	// 1x1
	(yc+r_arm/r2)*cos(theta(0))+(-(l_servo-zc)+xi)*sin(theta(0))/r2,  
	// 1x2
	(yc+r_arm/r2)*cos(theta(1))+((l_servo-zc)-xi)*sin(theta(1))/r2,   
	// 1x3
	(yc-r_arm/r2)*cos(theta(2))+((l_servo-zc)-xi)*sin(theta(2))/r2,  
	// 1x4
	(yc-r_arm/r2)*cos(theta(3))+(-(l_servo-zc)+xi)*sin(theta(3))/r2,
	// 2x1
	-(xc-r_arm/r2)*cos(theta(0))+((l_servo-zc)+xi)*sin(theta(0))/r2, 
	// 2x2
	-(xc+r_arm/r2)*cos(theta(1))+((l_servo-zc)+xi)*sin(theta(1))/r2, 
	// 2x3
	-(xc+r_arm/r2)*cos(theta(2))+(-(l_servo-zc)-xi)*sin(theta(2))/r2, 
	// 2x4
	-(xc-r_arm/r2)*cos(theta(3))+(-(l_servo-zc)-xi)*sin(theta(3))/r2,
	// 3x1
	-xi*cos(theta(0))+(-(xc-yc)/r2)*sin(theta(0)),
	// 3x2
	xi*cos(theta(1))+((xc+yc)/r2)*sin(theta(1)),
	// 3x3
	-xi*cos(theta(2))+((xc-yc)/r2)*sin(theta(2)),
	// 3x4
	xi*cos(theta(3))+(-(xc+yc)/r2)*sin(theta(3)),
	// 4xn
	-cos(theta(0)), -cos(theta(1)), -cos(theta(2)), -cos(theta(3));
	invCM_Xc_p2 = CM_Xc_p2.inverse();
	return invCM_Xc_p2;
}




Eigen::MatrixXd setSA(Eigen::VectorXd F)
{
  	Eigen::MatrixXd SA(4,4);
  	Eigen::MatrixXd invSA(4,4);

	SA <<  
	// 1xn
	F(0)/r2,     F(1)/r2,     -(F(2)/r2),     -(F(3)/r2),
	// 2xn
	F(0)/r2,    -(F(1)/r2),   -(F(2)/r2),       F(3)/r2,
	// 3xn
	r_arm*F(0),  r_arm*F(1),   r_arm*F(2),   r_arm*F(3),
	// 4xn
	r_arm*F(0), -(r_arm*F(1)),   r_arm*F(2),  -(r_arm*F(3));

	invSA = SA.inverse();
	
	return invSA;
}

Eigen::Matrix3d W2B_rot;



////////////////////////////////////////
// ROS Publishers, Callback Functions //
////////////////////////////////////////

ros::Publisher PWMs; // PWM data logging
ros::Publisher PWM_generator; // To ros-pwm-generator node

ros::Publisher goal_dynamixel_position; // To dynamixel position && servo data logging

ros::Publisher desired_motor_thrust; 

ros::Publisher desired_force; 
ros::Publisher desired_torque; 

ros::Publisher angular_Acceleration; 
ros::Publisher linear_acceleration; 

ros::Publisher linear_velocity;
ros::Publisher desired_velocity;
ros::Publisher angular_velocity;

ros::Publisher desired_position;
ros::Publisher position;

ros::Publisher euler; // euler angle data logging
ros::Publisher desired_angle; // desired angle data logging

ros::Publisher battery_voltage;
ros::Publisher delta_time;
//ros::Publisher TosubdroneData :: if you are main drone

ros::Subscriber dynamixel_state; // servo angle data callback
ros::Subscriber att; // imu data callback
ros::Subscriber rc_in; //Sbus signal callback from Arduino
ros::Subscriber battery_checker; // battery level callback from Arduino 
ros::Subscriber t265_position; // position data callback from T265 
ros::Subscriber t265_rot; // angle data callback from T265
ros::Subscriber t265_odom; // odometry data (linear velocity) callback from T265 

void jointstate_Callback(const sensor_msgs::JointState& msg);
void imu_Callback(const sensor_msgs::Imu& msg);
sensor_msgs::JointState servo_msg_create(double desired_theta1, double desired_theta2, double desired_theta3, double desired_theta4);
void sbus_Callback(const std_msgs::Int16MultiArray::ConstPtr& array);
void battery_Callback(const std_msgs::Int16& msg);
void t265_position_Callback(const geometry_msgs::Vector3& msg);
void t265_angle_Callback(const geometry_msgs::Quaternion& msg);
void filter_Callback(const sensor_msgs::Imu& msg);
void t265_Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg);
//void serial data callback :: if you are serve drone
};

#endif