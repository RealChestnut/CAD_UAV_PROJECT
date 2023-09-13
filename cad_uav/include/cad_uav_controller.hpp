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


////ROS NODE HANDLE////

///////////////////////////////GENERAL PARAMETER START////////////////////////////////////////////

//Timer class :: c++11
std::chrono::high_resolution_clock::time_point end=std::chrono::high_resolution_clock::now();
std::chrono::high_resolution_clock::time_point start=std::chrono::high_resolution_clock::now();
std::chrono::duration<double> delta_t;
std_msgs::Float32 dt;

//Thruster_cmd

double F1=0;//desired propeller 1 force
double F2=0;//desired propeller 2 force
double F3=0;//desired propeller 3 force
double F4=0;//desired propeller 4 force

//////////Global : XYZ  Body : xyz///////////////////

Eigen::Vector3d tau_rpy_desired; // desired torque (N.m)

double tau_y_sin = 0; //yaw sine term torque (N.m)
double tau_y_d_non_sat=0;//yaw deried torque non-saturation (N.m)
double tau_y_th = 0; // yaw desired torque w.r.t. servo tilt (N.m)

Eigen::Vector3d rpy_ddot_desired; // angular acceleration

double Thrust_d = 0;//altitude desired thrust(N)

Eigen::Vector3d rpy_desired; // desired rpy angle

double y_d_tangent = 0;//yaw increment tangent
double T_d = 0;//desired thrust

Eigen::Vector3d XYZ_desired; //desired global position
Eigen::Vector3d XYZ_desired_base; // initial desired XYZ global position
Eigen::Vector3d XYZ_dot_desired; // desired global linear velocity
Eigen::Vector3d XYZ_ddot_desired; // desired global linear acceleration

Eigen::Vector3d F_xyzd; // desired body force

double r_arm = 0.3025;// m // diagonal length between thruster x2
double l_servo = 0.035; // length from servo motor to propeller

double mass_system =0; // system mass (kg) ex) M = main+sub1+sub2
double mass_main = 8.2; // main drone mass(kg) 
double mass_sub1 = 0; // sub1 drone mass (kg)
double mass_sub2 = 0; // sub2 drone mass (kg)

double r2=sqrt(2); // root(2)
double l_module = 0.50; //(m) module horizontal body length
double xi = 0.01;// aerodynamics constant value F_i=k*(omega_i)^2, M_i=b*(omega_i)^2
double pi = 3.141592;//(rad)
double g = 9.80665;// gravity acceleration (m/s^2)


//////// limitation value ////////
double rp_limit = 0.25;// roll pitch angle limit (rad)
double y_vel_limit = 0.01;// yaw angle velocity limit (rad/s)
double y_d_tangent_deadzone = (double)0.05 * y_vel_limit;//(rad/s)
double T_limit = 80;// thrust limit (N)
double altitude_limit = 1;// z direction limit (m)
double XY_limit = 1.0; // position limit
double XYZ_dot_limit=1; // linear velocity limit
double XYZ_ddot_limit=2; // linear acceleration limit
double hardware_servo_limit=0.3; // servo limit w.r.t. hardware
double servo_command_limit = 0.3; // servo limit w.r.t. command part
double tau_y_limit = 1.0; // yaw torque limit
double tau_y_th_limit = 0.5; // yaw torque w.r.t. servo tilt limit

double F_xd_limit = mass_system*2.0; // X direction force limit 
double F_yd_limit = mass_system*2.0; // Y direction force limit

Eigen::Vector3d CoM_hat; // center of mass

//////// CONTROL GAIN PARAMETER /////////

//integratior(PID) limitation
double integ_limit=10;
double z_integ_limit=100;
double position_integ_limit=10;
double velocity_integ_limit=10;

//Roll, Pitch PID gains
double Pa=3.5;
double Ia=0.4;
double Da=0.5;

//Roll PID gains
double Par=3.5;
double Iar=0.4;
double Dar=0.5;

//Pitch PID gains
double Pap=0.0;
double Iap=0.0;
double Dap=0.0;

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

//////////////////////// TOPIC MESSAGE START //////////////////////


//////////////////////// SUBSCRIBER START /////////////////////////

ros::Subscriber dynamixel_state; // servo angle data callback
ros::Subscriber att; // imu data callback
ros::Subscriber rc_in; //Sbus signal callback from Arduino
ros::Subscriber battery_checker; // battery level callback from Arduino 
ros::Subscriber t265_position; // position data callback from T265 
ros::Subscriber t265_rot; // angle data callback from T265
ros::Subscriber t265_odom; // odometry data (linear velocity) callback from T265


//////////////////////// PUBLISHER START /////////////////////////

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




///////////////////////////////CALLBACK FUNCTION START////////////////////////////////////////////

void Clock()
{
  end=std::chrono::high_resolution_clock::now();
  delta_t=end-start; 
  start=std::chrono::high_resolution_clock::now();
  dt.data=delta_t.count();
  //ROS_INFO_STREAM("Clock Time : " << dt.data);

}

//SERVO ANGLE CALLBACK//
Eigen::VectorXd servo_theta;
void jointstate_Callback(const sensor_msgs::JointState& msg)
{
    servo_theta(0)=msg.position[0];
    servo_theta(1)=msg.position[1];
    servo_theta(2)=msg.position[2];
    servo_theta(3)=msg.position[3];
}


  

//IMU DATA CALLBACK//
geometry_msgs::Quaternion imu_quaternion;
geometry_msgs::Vector3 imu_rpy;
geometry_msgs::Vector3 imu_angular_velocity;
geometry_msgs::Vector3 imu_linear_acceleration;
Eigen::Matrix3d W2B_rot;
double yaw_prev = 0;
double yaw_now = 0;
double base_yaw = 0;
int yaw_rotate_count = 0;
double t265_yaw_angle=0;
void imu_Callback(const sensor_msgs::Imu& msg)
{
    // TP attitude - Quaternion representation
    imu_quaternion=msg.orientation;
    imu_angular_velocity=msg.angular_velocity;
    imu_linear_acceleration=msg.linear_acceleration;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(imu_quaternion,quat);

    // TP attitude - Euler representation
    tf::Matrix3x3(quat).getRPY(imu_rpy.x,imu_rpy.y,imu_rpy.z);
    base_yaw = t265_yaw_angle;
    if(base_yaw - yaw_prev < -pi) yaw_rotate_count++;
    else if(base_yaw - yaw_prev > pi) yaw_rotate_count--;
	  yaw_now = base_yaw+2*pi*yaw_rotate_count;
	
    imu_rpy.z = yaw_now;
    yaw_prev = base_yaw;

    W2B_rot <<  
    cos(imu_rpy.y)*cos(imu_rpy.z), 
    cos(imu_rpy.y)*sin(imu_rpy.z), 
   -sin(imu_rpy.y), 
   -cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.y)*sin(imu_rpy.x), 
    cos(imu_rpy.x)*cos(imu_rpy.z)-sin(imu_rpy.y)*sin(imu_rpy.x)*sin(imu_rpy.z), 
   -cos(imu_rpy.y)*sin(imu_rpy.x), 
    cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y)-sin(imu_rpy.x)*sin(imu_rpy.z), 
    cos(imu_rpy.z)*sin(imu_rpy.x)+cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z), 
    cos(imu_rpy.y)*cos(imu_rpy.x);
	
}

//LINEAR VELOCITY DATA CALLBACK//
Eigen::Vector3d cam_v;
Eigen::Matrix3d R_v;
Eigen::Vector3d v;
geometry_msgs::Vector3 linear_velocity_from_t265;
geometry_msgs::Vector3 angular_velocity_from_t265;
geometry_msgs::Vector3 lin_vel;
geometry_msgs::Vector3 prev_lin_vel;
geometry_msgs::Vector3 t265_att;
geometry_msgs::Quaternion t265_quat;

void t265_Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    linear_velocity_from_t265=msg->twist.twist.linear;
    angular_velocity_from_t265=msg->twist.twist.angular;
    t265_quat=msg->pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(t265_quat,quat);
    tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);
    t265_yaw_angle = t265_att.z;
    cam_v << linear_velocity_from_t265.x, linear_velocity_from_t265.y, linear_velocity_from_t265.z;  

    //rotation matrix for aligning from camera axis to body axis
    R_v <<
            0, -r2/2, r2/2,
            0, -r2/2, -r2/2,
            1, 0, 0;

    //camera axis to body axis
    v = R_v*cam_v; // linear velocity

    // body axis to global axis :: linear velocity
    double global_X_dot = v(2)*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-v(1)*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.z)*cos(imu_rpy.y);
    double global_Y_dot = v(1)*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.z)*sin(imu_rpy.y))-v(2)*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.z)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.y)*sin(imu_rpy.z);
    double global_Z_dot = -v(0)*sin(imu_rpy.y)+v(2)*cos(imu_rpy.x)*cos(imu_rpy.y)+v(1)*cos(imu_rpy.y)*sin(imu_rpy.x);

    lin_vel.x=global_X_dot;
    lin_vel.y=global_Y_dot;
    lin_vel.z=global_Z_dot;
}


//Accelerometer LPF//
double x_ax_dot = 0;
double x_ay_dot = 0;
double x_az_dot = 0;
double x_ax = 0;
double x_ay = 0;
double x_az = 0;
double accel_cutoff_freq = 1.0;

geometry_msgs::Vector3 lin_acl;
void Accelerometer_LPF()
{
  x_ax_dot=-accel_cutoff_freq*x_ax+imu_linear_acceleration.x;
	x_ax+=x_ax_dot*delta_t.count();
	x_ay_dot=-accel_cutoff_freq*x_ay+imu_linear_acceleration.y;
	x_ay+=x_ay_dot*delta_t.count();
	x_az_dot=-accel_cutoff_freq*x_az+imu_linear_acceleration.z;
	x_az+=x_az_dot*delta_t.count();
	lin_acl.x=accel_cutoff_freq*x_ax;
	lin_acl.y=accel_cutoff_freq*x_ay;
	lin_acl.z=accel_cutoff_freq*x_az;
}


//SBUS DATA CALLBACK//
bool attitude_mode = false;
bool velocity_mode = false;
bool position_mode = false;
bool kill_mode = true;
bool altitude_mode = false;
bool tilt_mode = false;
int16_t Sbus[10];
template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void sbus_Callback(const std_msgs::Int16MultiArray::ConstPtr& array)
{
    for(int i=0;i<10;i++){
		Sbus[i]=map<int16_t>(array->data[i], 352, 1696, 1000, 2000);
	}
	
	if(Sbus[4]<1500) kill_mode=true;
	else kill_mode=false;
}

//POSITION DATA CALLBACK//
geometry_msgs::Vector3 position_from_t265; 
void t265_position_Callback(const geometry_msgs::Vector3& msg)
{
    position_from_t265.x=msg.x;
    position_from_t265.y=msg.y;
    position_from_t265.z=msg.z;
}
  


//BATTERY DATA CALLBACK//

double voltage=22.4;
double voltage_old=22.4;
std_msgs::Float32 battery_voltage_msg;
void battery_Callback(const std_msgs::Int16& msg)
{
    int16_t value=msg.data;
    voltage=value*5.0/(double)1024/(7440./(30000.+7440.)); //4096
    double kv=0.08;
    voltage=kv*voltage+(1-kv)*voltage_old;
    voltage_old=voltage;
    //ROS_INFO("%f",voltage);
    if(voltage>25.2) voltage=25.2;
    if(voltage<20.0) voltage=20.0;
    battery_voltage_msg.data=voltage;
}







///////////////////////////////CONSTROLLER START////////////////////////////////////////////


void shape_detector()
{

  //main인지 sub인지 플레그
  //아두이노로 부터 switch 데이터 수신, 일정
  //눌린 스위치의 위치에 따라서 결합 형태 판단.
  //일정 조건을 만족했을때 결합됐다는 신호 날림.
  //하나씩 하나씩 결합될것임.
  //결합됐다고 판단이 되면은 여기서 서보모터에 회전 신호 인풋
  //
  //Eigen3

}

Eigen::Vector3d X_c_p1;
Eigen::Vector3d X_c_p2;
void UpdateParameter()
{

  //shpe detector의 조건에 따라서 parameter 값 update
  //1. 전체 system mass
  //2. 전체 system COM
  //3. Xc_p1, Xc_p2 control logic에 들어갈 COM
  //4. 각종 limitation value :: ex) Fxyzd, thrust, lin_acl, lin_vel, torque_d, ect.

  // combination rule 
  // 1_ main drone must be located on the far left
  // 2_ main drone must be located at the bottom
  // 3_ sub1 drone must be battery exchanger	

  // setMoI value changed in here	
  int num = 2;
  if(num==1){
                    
                //p_c_main << x_c_hat,y_c_hat,z_c_hat;
                //p_c_sub1 << 0, 0, 0;
                //p_c_sub2 << 0, 0, 0;
                
                
                mass_main = 8.2;
                mass_sub1 = 0;
                mass_sub2 = 0;
                
                mass_system = mass_main;

                X_c_p1 << 0,0,0;
                X_c_p2 << CoM_hat(0),CoM_hat(1),CoM_hat(2);    

  }
  else if(num==2){
      

                //p_c_main << x_c_hat,y_c_hat,z_c_hat;
                //p_c_sub1 << x_c_hat,y_c_hat+2*l_module,z_c_hat;
                //p_c_sub2 << 0,0,0;
                
                mass_sub1=8.2;
                mass_sub2=0;
                mass_system = mass_main+mass_sub1;

                X_c_p1 << 0,CoM_hat(1),0;
                X_c_p2 << CoM_hat(0),0,CoM_hat(2);

  }
  else if(num==3){

                //p_c_main << x_c_hat,y_c_hat,z_c_hat;
                //p_c_sub1 << 0,0,0;
                //p_c_sub2 << x_c_hat-l_module,y_c_hat,z_c_hat;

                mass_sub1=8.2;
                mass_sub2=8.2;
                mass_system = mass_main+mass_sub1+mass+sub2;
                X_c_p1 << 0,CoM_hat(1),0;
                X_c_p2 << CoM_hat(0),0,CoM_hat(2);
  }  

}

void Command_Generator()
{


}


double e_r = 0;
double e_p = 0;
double e_y = 0;

void attitude_controller()
{
	rpy_desired(0) = 0.0;
  rpy_desired(1) = 0.0;

  y_d_tangent=y_vel_limit*(((double)Sbus[0]-(double)1500)/(double)500);
	if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
	rpy_desired(2)+=y_d_tangent;

  
  e_r = rpy_desired(0) - imu_rpy.x;
  e_p = rpy_desired(1) - imu_rpy.y;
  e_y = rpy_desired(2) - imu_rpy.z;

  e_r_i += e_r * delta_t.count();
	if (fabs(e_r_i) > integ_limit)	e_r_i = (e_r_i / fabs(e_r_i)) * integ_limit;
	e_p_i += e_p * delta_t.count();
	if (fabs(e_p_i) > integ_limit)	e_p_i = (e_p_i / fabs(e_p_i)) * integ_limit;
	

	tau_rpy_desired(0) = Par * e_r + Iar * e_r_i + Dar * (-imu_angular_velocity.x);//- (double)0.48; //Pa -> Par
	tau_rpy_desired(1) = Par * e_p + Iar * e_p_i + Dar * (-imu_angular_velocity.y);//+ (double)0.18; //Pa -> Par 
	tau_rpy_desired(2) = Py * e_y + Dy * (-imu_angular_velocity.z);
	
  tau_y_d_non_sat=tau_rpy_desired(2);
	if(fabs(tau_rpy_desired(2)) > tau_y_limit) tau_rpy_desired(2) = tau_rpy_desired(2)/fabs(tau_rpy_desired(2))*tau_y_limit;
  
}

geometry_msgs::Vector3 desired_lin_vel;

void position_controller()
{
  double e_X=0;
  double e_Y=0;

  // should change SBUS to Ground Station input
  XYZ_desired(0) = XYZ_desired_base(0) - XY_limit*(((double)Sbus[1]-(double)1500)/(double)500);
  XYZ_desired(1) = XYZ_desired_base(1) + XY_limit*(((double)Sbus[3]-(double)1500)/(double)500);

  e_X = XYZ_desired(0) - position_from_t265.x;
  e_Y = XYZ_desired(1) - position_from_t265.y;
  e_X_i += e_X * delta_t.count();
  if (fabs(e_X_i) > position_integ_limit) e_X_i = (e_X_i / fabs(e_X_i)) * position_integ_limit;
  e_Y_i += e_Y * delta_t.count();
  if (fabs(e_Y_i) > position_integ_limit) e_Y_i = (e_Y_i / fabs(e_Y_i)) * position_integ_limit;

  XYZ_dot_desired(0) = Pp * e_X + Ip * e_X_i - Dp * lin_vel.x;
  XYZ_dot_desired(1) = Pp * e_Y + Ip * e_Y_i - Dp * lin_vel.y;
  if(fabs(XYZ_dot_desired(0)) > XYZ_dot_limit) XYZ_dot_desired(0) = (XYZ_dot_desired(0)/fabs(XYZ_dot_desired(0)))*XYZ_dot_limit;
  if(fabs(XYZ_dot_desired(1)) > XYZ_dot_limit) XYZ_dot_desired(1) = (XYZ_dot_desired(1)/fabs(XYZ_dot_desired(1)))*XYZ_dot_limit;

  desired_lin_vel.x = XYZ_dot_desired(0);
  desired_lin_vel.y = XYZ_dot_desired(1);
}

void velocity_controller()
{
  double e_X_dot = 0;
  double e_Y_dot = 0;
  Eigen::Vector3d xy_acl_d;
  e_X_dot = XYZ_dot_desired(0) - lin_vel.x;
  e_Y_dot = XYZ_dot_desired(1) - lin_vel.y;
  e_X_dot_i += e_X_dot * delta_t.count();
  //limitation
  if (fabs(e_X_dot_i) > velocity_integ_limit) e_X_dot_i = (e_X_dot_i / fabs(e_X_dot_i)) * velocity_integ_limit;
  e_Y_dot_i += e_Y_dot * delta_t.count();
  if (fabs(e_Y_dot_i) > velocity_integ_limit) e_Y_dot_i = (e_Y_dot_i / fabs(e_Y_dot_i)) * velocity_integ_limit;

  XYZ_ddot_desired(0) = Pv * e_X_dot + Iv * e_X_dot_i - Dv * lin_acl.x;
  XYZ_ddot_desired(1) = Pv * e_Y_dot + Iv * e_Y_dot_i - Dv * lin_acl.y;
 
  //limitation
  if(fabs(XYZ_ddot_desired(0)) > XYZ_ddot_limit) XYZ_ddot_desired(0) = (XYZ_ddot_desired(0)/fabs(XYZ_ddot_desired(0)))*XYZ_ddot_limit;
  if(fabs(XYZ_ddot_desired(1)) > XYZ_ddot_limit) XYZ_ddot_desired(1) = (XYZ_ddot_desired(1)/fabs(XYZ_ddot_desired(1)))*XYZ_ddot_limit;

  F_xyzd(0) = mass_system*(W2B_rot(0,0)*XYZ_ddot_desired(0)
                   +W2B_rot(0,1)*XYZ_ddot_desired(1)
                   +W2B_rot(0,2)*XYZ_ddot_desired(2));
  
  F_xyzd(1) = mass_system*(W2B_rot(1,0)*XYZ_ddot_desired(0)
                   +W2B_rot(1,1)*XYZ_ddot_desired(1)
                   +W2B_rot(1,2)*XYZ_ddot_desired(2));
  //F_xd = mass*(X_ddot_d*cos(imu_rpy.z)*cos(imu_rpy.y)+Y_ddot_d*sin(imu_rpy.z)*cos(imu_rpy.y)-(Z_ddot_d)*sin(imu_rpy.y));
  //F_yd = mass*(-X_ddot_d*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+Y_ddot_d*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d)*cos(imu_rpy.y)*sin(imu_rpy.x));
  // Force limitation
  if(fabs(F_xyzd(0)) > F_xd_limit) F_xyzd(0) = (F_xyzd(0)/fabs(F_xyzd(0)))*F_xd_limit;
  if(fabs(F_xyzd(1)) > F_yd_limit) F_xyzd(1) = (F_xyzd(1)/fabs(F_xyzd(1)))*F_yd_limit;

  
  
}

void altitude_controller()
{
  
  double e_Z=0;

  e_Z = XYZ_desired(2) - position_from_t265.z;
  e_Z_i += e_Z * delta_t.count();	
  if (fabs(e_Z_i) > z_integ_limit) e_Z_i = (e_Z_i / fabs(e_Z_i)) * z_integ_limit;

  XYZ_ddot_desired(2) = Pz * e_Z + Iz * e_Z_i - Dz * lin_vel.z;
  desired_lin_vel.z = 0; // But this is desired acceleration
  
  // Global to Body ration
  if(!attitude_mode) F_xyzd(2) = mass_system*(W2B_rot(2,0)*XYZ_ddot_desired(0)
                                      +W2B_rot(2,1)*XYZ_ddot_desired(1)
                                      +W2B_rot(2,2)*XYZ_ddot_desired(2));
       
  else F_xyzd(2) = mass_system*(XYZ_ddot_desired(2));
  //F_zd = mass*(X_ddot_d*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-Y_ddot_d*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d)*cos(imu_rpy.x)*cos(imu_rpy.y));
}


void setCM_Xc_p2()
{

  Eigen::MatrixXd CM_Xc_p2(4,4); //Thrust Allocation
  Eigen::MatrixXd invCM_Xc_p2(4,4);
  double xc = CoM_hat(0);
  double yc = CoM_hat(1);
  double zc = CoM_hat(2);
  double th1 = servo_theta(0);
  double th2 = servo_theta(1);
  double th3 = servo_theta(2);
  double th4 = servo_theta(3);
	
	CM_Xc_p2 << 
	// 1x1
	(yc+r_arm/r2)*cos(th1)+(-(l_servo-zc)+xi)*sin(th1)/r2,  
	// 1x2
	(yc+r_arm/r2)*cos(th2)+((l_servo-zc)-xi)*sin(th2)/r2,   
	// 1x3
	(yc-r_arm/r2)*cos(th3)+((l_servo-zc)-xi)*sin(th3)/r2,  
	// 1x4
	(yc-r_arm/r2)*cos(th4)+(-(l_servo-zc)+xi)*sin(th4)/r2,
	// 2x1
	-(xc-r_arm/r2)*cos(th1)+((l_servo-zc)+xi)*sin(th1)/r2, 
	// 2x2
	-(xc+r_arm/r2)*cos(th2)+((l_servo-zc)+xi)*sin(th2)/r2, 
	// 2x3
	-(xc+r_arm/r2)*cos(th3)+(-(l_servo-zc)-xi)*sin(th3)/r2, 
	// 2x4
	-(xc-r_arm/r2)*cos(th4)+(-(l_servo-zc)-xi)*sin(th4)/r2,
	// 3x1
	-xi*cos(th1)+(-(xc-yc)/r2)*sin(th1),
	// 3x2
	xi*cos(th2)+((xc+yc)/r2)*sin(th2),
	// 3x3
	-xi*cos(th3)+((xc-yc)/r2)*sin(th3),
	// 3x4
	xi*cos(th4)+(-(xc+yc)/r2)*sin(th4),
	// 4xn
	-cos(th1), -cos(th2), -cos(th3), -cos(th4);
	invCM_Xc_p2 = CM_Xc_p2.inverse();
	
}


Eigen::VectorXd allocation_factor(2);
Eigen::MatrixXd K_M(4,2);
Eigen::MatrixXd invK_M(2,4);
Eigen::VectorXd Dump(4);
void K_matrix()
{
    double F_xd=F_xyzd(0);
    double F_yd=F_xyzd(1);
    double F_zd=F_xyzd(2);
    double xc = X_c_p1(0);
    double yc = X_c_p1(1);
    double zc = X_c_p1(2);
    Dump << 0,0,0,1;
    K_M << 
    //1x1
    F_yd*zc-F_zd*yc, 
    //1x2
    F_yd*zc-F_zd*(yc+2*l_module),
    //2x1
    -(F_xd*zc-F_zd*xc), 
    //2x2
    -(F_xd*zc-F_zd*xc),
    //3x1
    F_xd*yc-F_yd*xc, 
    //3x2
    F_xd*(yc+2*l_module)-F_yd*xc,
    //4xn
    1,      1;

    invK_M = K_M.completeOrthogonalDecomposition().pseudoInverse();

    allocation_factor = invK_M*Dump;
       
}

void PWM_signal_Generator()
{

}

void PublishData()
{

}