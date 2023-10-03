
#include "cad_uav_controller_2.hpp"

void publisherSET();
void Clock();
void jointstate_Callback(const sensor_msgs::JointState& msg);
void imu_Callback(const sensor_msgs::Imu& msg);
void t265_Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg);
void Accelerometer_LPF();
void sbus_Callback(const std_msgs::Int16MultiArray::ConstPtr& array);
void t265_position_Callback(const geometry_msgs::Vector3& msg);
void battery_Callback(const std_msgs::Int16& msg);
void shape_detector();
void setMoI();
void pid_Gain_Setting();
void UpdateParameter();
void Command_Generator();
void attitude_controller();
void position_controller();
void velocity_controller();
void altitude_controller();
void K_matrix();
void wrench_allocation();
void yaw_torque_distribute();
void setCM_Xc_p2();
void setSA();
void PWM_signal_Generator();
std::string data_2_string();
void PublishData();
void reset_data();


//직접 계산에 사용하는 변수는 이름을 축약해서
//퍼를리쉬 하기 위해 선언한 변수는 길게

int main(int argc, char **argv)
{

  ros::init(argc, argv, "cad_uav_2");
  ros::NodeHandle nh;

  
  //integratior(PID) limitation
                integ_limit=nh.param<double>("attitude_integ_limit",10);
                integ_yaw_limit=nh.param<double>("attitude_y_integ_limit",10);
                z_integ_limit=nh.param<double>("altitude_integ_limit",100);
                position_integ_limit=nh.param<double>("position_integ_limit",10);

                CoM_hat.x = nh.param<double>("x_center_of_mass",1.0);
                CoM_hat.y = nh.param<double>("y_center_of_mass",1.0);
                CoM_hat.z = nh.param<double>("z_center_of_mass",1.0);

                        //Roll, Pitch PID gains

                        tilt_Par=nh.param<double>("tilt_attitude_r_P_gain",3.5);
                        tilt_Iar=nh.param<double>("tilt_attitude_r_I_gain",3.5);
                        tilt_Dar=nh.param<double>("tilt_attitude_r_D_gain",3.5);

                        tilt_Pap=nh.param<double>("tilt_attitude_p_P_gain",3.5);
                        tilt_Iap=nh.param<double>("tilt_attitude_p_I_gain",3.5);
                        tilt_Dap=nh.param<double>("tilt_attitude_p_D_gain",3.5);

                        //Yaw PID gains
                        tilt_Py=nh.param<double>("tilt_attitude_y_P_gain",5.0);
                        tilt_Iy=nh.param<double>("tilt_attitude_y_I_gain",5.0);
                        tilt_Dy=nh.param<double>("tilt_attitude_y_D_gain",0.3);

                        //Altitude PID gains
                        tilt_Pz=nh.param<double>("tilt_altitude_P_gain",15.0);
                        tilt_Iz=nh.param<double>("tilt_altitude_I_gain",5.0);
                        tilt_Dz=nh.param<double>("tilt_altitude_D_gain",10.0);

                        //Velocity PID gains
                        tilt_Pv=nh.param<double>("tilt_velocity_P_gain",5.0);
                        tilt_Iv=nh.param<double>("tilt_velocity_I_gain",0.1);
                        tilt_Dv=nh.param<double>("tilt_velocity_D_gain",5.0);

                        //Position PID gains
                        tilt_Pp=nh.param<double>("tilt_position_P_gain",3.0);
                        tilt_Ip=nh.param<double>("tilt_position_I_gain",0.1);
                        tilt_Dp=nh.param<double>("tilt_position_D_gain",5.0);

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

    angular_velocity = nh.advertise<geometry_msgs::Vector3>("ang_vel",100);
    desired_angular_velocity = nh.advertise<geometry_msgs::Vector3>("ang_vel_d",100); //REVISE

    desired_position = nh.advertise<geometry_msgs::Vector3>("position_d",100);
    position = nh.advertise<geometry_msgs::Vector3>("position",100);

    desired_force = nh.advertise<geometry_msgs::Vector3>("force_d",100);

    battery_voltage = nh.advertise<std_msgs::Float32>("battery_voltage",100);
    delta_time = nh.advertise<std_msgs::Float32>("delta_t",100);

    ToSubAgent = nh.advertise<std_msgs::String>("ToSubData",1);
    
    
    ros::Timer timerPublish = nh.createTimer(ros::Duration(1.0/200.0),std::bind(publisherSET));
    ros::spin();
    return 0;
}
 
  
void publisherSET(){
    Clock();
    if(true/*!kill_mode*/)
    {
    shape_detector();
    UpdateParameter(); 
    //setMoI,pid_Gain_Setting, etc.

    //if(){
    Command_Generator();
    attitude_controller();
    position_controller();
    altitude_controller();

    Accelerometer_LPF();
    velocity_controller();
    K_matrix();
    wrench_allocation();
    yaw_torque_distribute();

    PWM_signal_Generator(); 
    //contain :: setCM,setSA, etc
    //}
    }
    else
    {
      
      reset_data();
      pwm_Kill();
      
    }

    PublishData();
  }
  
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
Eigen::VectorXd servo_theta(4);
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
geometry_msgs::Vector3 imu_ang_vel;
geometry_msgs::Vector3 imu_lin_acl;
geometry_msgs::Vector3 prev_ang_vel;
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
    imu_ang_vel=msg.angular_velocity;
    imu_lin_acl=msg.linear_acceleration;

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
    sin(imu_rpy.y), 
    cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.y)*sin(imu_rpy.x), 
    cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.y)*sin(imu_rpy.x)*sin(imu_rpy.z), 
    cos(imu_rpy.y)*sin(imu_rpy.x), 
    cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y)+sin(imu_rpy.x)*sin(imu_rpy.z), 
    cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z), 
    cos(imu_rpy.y)*cos(imu_rpy.x);
	
}

//LINEAR VELOCITY DATA CALLBACK//
Eigen::Vector3d cam_v;
Eigen::Matrix3d R_v;
Eigen::Vector3d v;
geometry_msgs::Vector3 lin_vel_from_t265;
geometry_msgs::Vector3 ang_vel_from_t265;
geometry_msgs::Vector3 lin_vel;
geometry_msgs::Vector3 prev_lin_vel;
geometry_msgs::Vector3 t265_att;
geometry_msgs::Quaternion t265_quat;

void t265_Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    lin_vel_from_t265=msg->twist.twist.linear;
    ang_vel_from_t265=msg->twist.twist.angular;
    t265_quat=msg->pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(t265_quat,quat);
    tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);
    t265_yaw_angle = t265_att.z;
    cam_v << lin_vel_from_t265.x, lin_vel_from_t265.y, lin_vel_from_t265.z;  

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
  x_ax_dot=-accel_cutoff_freq*x_ax+imu_lin_acl.x;
	x_ax+=x_ax_dot*delta_t.count();
	x_ay_dot=-accel_cutoff_freq*x_ay+imu_lin_acl.y;
	x_ay+=x_ay_dot*delta_t.count();
	x_az_dot=-accel_cutoff_freq*x_az+imu_lin_acl.z;
	x_az+=x_az_dot*delta_t.count();
	lin_acl.x=accel_cutoff_freq*x_ax;
	lin_acl.y=accel_cutoff_freq*x_ay;
	lin_acl.z=accel_cutoff_freq*x_az;
}


//SBUS DATA CALLBACK//

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

int module_num=1;
bool mono_flight = true;

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

  module_num=1;

}

Eigen::Matrix3d origin_MoI;
Eigen::Matrix3d hat_CoM_x_main;
Eigen::Matrix3d hat_CoM_x_sub1;
Eigen::Matrix3d hat_CoM_x_sub2;
Eigen::Matrix3d hat_MoI;
int toggle_sub1=0;
int toggle_sub2=0;

void setMoI(){
	
	origin_MoI << 
  Jxx,   0,    0,
  0,   Jyy,    0,
  0,      0, Jzz;

	hat_CoM_x_main << 
           0,  -CoM_hat.z,    CoM_hat.y, 
   CoM_hat.z,           0,   -CoM_hat.x,   
  -CoM_hat.y,   CoM_hat.x,            0;

  hat_CoM_x_sub1 << 
           0,  -CoM_hat.z,    CoM_hat.y+2*l_module, 
   CoM_hat.z,           0,              -CoM_hat.x,   
  -(CoM_hat.y+2*l_module),   CoM_hat.x,          0;

  hat_CoM_x_sub2 << 
           0,  -CoM_hat.z,                CoM_hat.y, 
   CoM_hat.z,           0,  -(CoM_hat.x+2*l_module),   
  -CoM_hat.y,    CoM_hat.x-2*l_module,            0;



	
	hat_MoI = (origin_MoI  - mass_main*hat_CoM_x_main*hat_CoM_x_main)+
                        toggle_sub1*(origin_MoI  - mass_sub1*hat_CoM_x_sub1*hat_CoM_x_sub1)+
                        toggle_sub2*(origin_MoI  - mass_sub2*hat_CoM_x_sub2*hat_CoM_x_sub2);
        /*	
	ROS_INFO("%f|%f|%f, %f|%f|%f, %f|%f|%f",
	hat_MoI(0,0),hat_MoI(0,1),hat_MoI(0,2),
	hat_MoI(1,0),hat_MoI(1,1),hat_MoI(1,2),
	hat_MoI(2,0),hat_MoI(2,1),hat_MoI(2,2));*/
}

void pid_Gain_Setting()
{
	Par = tilt_Par;
	Iar = tilt_Iar;
	Dar = tilt_Dar;

	Pap = tilt_Pap;
        Iap = tilt_Iap;
        Dap = tilt_Dap;
	
	Py = tilt_Py;
	Iy = tilt_Iy;
	Dy = tilt_Dy;
	
	Pz = tilt_Pz;
	Iz = tilt_Iz;
	Dz = tilt_Dz;

	Pv = tilt_Pv;
	Iv = tilt_Iv;
	Dv = tilt_Dv;

	Pp = tilt_Pp;
	Ip = tilt_Ip;
	Dp = tilt_Dp;

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
  // 전역에서 다른 전역변수와 연산으로 정의한 변수는
  // 업데이트 하고자 할 경우 함수 루프에서 한번더 돌려야함.
 

  int num = 1;

  if(num==1){
                    
                //p_c_main << x_c_hat,y_c_hat,z_c_hat;
                //p_c_sub1 << 0, 0, 0;
                //p_c_sub2 << 0, 0, 0;

                mass_system = mass_main;

                X_c_p1 << 0,0,0;
                X_c_p2 << CoM_hat.x,CoM_hat.y,CoM_hat.z;
                toggle_sub1=0;
                toggle_sub2=0;               

  }
  else if(num==2){
      

                //p_c_main << x_c_hat,y_c_hat,z_c_hat;
                //p_c_sub1 << x_c_hat,y_c_hat+2*l_module,z_c_hat;
                //p_c_sub2 << 0,0,0;

                mass_system = mass_main+mass_sub1;

                X_c_p1 << 0, CoM_hat.y, 0;
                X_c_p2 << CoM_hat.x, 0, CoM_hat.z;
                toggle_sub1=1;
                toggle_sub2=0;

  }
  else if(num==3){

                //p_c_main << x_c_hat,y_c_hat,z_c_hat;
                //p_c_sub1 << 0,0,0;
                //p_c_sub2 << x_c_hat-l_module,y_c_hat,z_c_hat;

                mass_system = mass_main+mass_sub1+mass_sub2;
                X_c_p1 << CoM_hat.x,CoM_hat.y,CoM_hat.z;
                X_c_p2 << 0,0,0;
                toggle_sub1=1;
                toggle_sub2=1;
  }
  // Data (combinated with other data)
  F_xd_limit=mass_system*2;
  F_yd_limit=mass_system*2;
  //// system MoI Initialize ////
  pid_Gain_Setting();
  setMoI();  
  
}

//-----------------------GUI control parameter---------------------------//
double freq=200;//controller loop frequency

double Landing_time = 2.0; //put landing time (sec)
double Hovering_time = 1.0; //put Hovering time (sec)
double X_Speed = 0.2; //desired value change speed of x_axis
double Y_Speed = 0.2; //desired value change speed of y_axis
double yaw_Speed = 0.2; //desired value change speed of yaw
double Z_Speed = 0.2; //desired value change speed of z_axis
//---------------------------------------------------------//
double Landing_Inc = 1 / (delta_t.count() * Landing_time);
double Hovering_Inc = 1 / (delta_t.count() * Hovering_time);
double X_Inc = X_Speed / delta_t.count();
double Y_Inc = Y_Speed / delta_t.count();
double yaw_Inc = yaw_Speed / delta_t.count();
double z_Inc = Z_Speed / delta_t.count();
double X_Goal = 0;
double Y_Goal = 0;
double y_Goal = 0;
double z_Goal = 0;
//--------------------------------------------------------

//////Ground Station Boolian ////////
bool isKill = false; //ASDF
bool isArm = false;  //ASDF
bool isHover = false; //ASDF
bool isHovering = false; //ASDF
bool isLanding = false; //ASDF
bool isTilt = false;
int Hover_Checker = 0;
int Land_Checker = 0;

void Command_Generator()
{

  

  if(true)
  {
    /////////// angle command /////////////
    rpy_desired.x = 0.0;
    rpy_desired.y = 0.0;

    y_d_tangent=y_vel_limit*(((double)Sbus[0]-(double)1500)/(double)500);
    if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
    rpy_desired.z+=y_d_tangent;


    /////////// position command /////////////

    XYZ_desired.x = XYZ_desired_base.x - XY_limit*(((double)Sbus[1]-(double)1500)/(double)500);
    XYZ_desired.y = XYZ_desired_base.y + XY_limit*(((double)Sbus[3]-(double)1500)/(double)500);
    
    if(Sbus[2]>1800){
			XYZ_desired.z-=0.0005;
		}
		else if(Sbus[2]<1200){
			XYZ_desired.z+=0.0005;
		}
			
		if(XYZ_desired.z <-0.7) XYZ_desired.z=-0.7;
		if(XYZ_desired.z > 0) XYZ_desired.z=0;
  }
  
  if(false/*ground station*/)
  {
    ///////// //angle command ////////////
    
    rpy_desired.x = 0.0;
    rpy_desired.y = 0.0;

    ///////// Ground Station /////////////
    if(isLanding){
	    if(XYZ_desired.z>-0.1) XYZ_desired.z -= Landing_Inc; // 2.5초간 하강 ASDF
	    else{
		    if(position_from_t265.x<0.15){
			    Land_Checker ++;
			    if(Land_Checker>=100) //0.5초간 유지 ASDF
			    {
				    isLanding = false;
				    isHover = false; //ASDF
				    isArm = false; //ASDF
              //Service.request.FAC_isLanding = false;
              //Service.request.FAC_isHover = false;
              //Service.request.FAC_isHovering = false;
              //HoverClient.call(Service);
	      }
        }
        else Land_Checker = 0;		
      }
    }

    if(isHovering){
	    if(XYZ_desired.z<=1) XYZ_desired.z += Hovering_Inc; // 2.5초간 상승 ASDF
	    else{
		    //if(pos.z>0.9 && pos.z<1.1)  //1미터 +- 10cm 범위내에 도달하면
		    ////{
		    ////Hover_Checker ++;
		    ////if(Hover_Checker >= 600)
		    ////{
            	isHovering = false;
            	//Service.request.FAC_isHovering = false;
            	//Service.request.FAC_isHover = true;
            	//Service.request.FAC_isLanding = false;
            	//HoverClient.call(Service);
            	z_Goal = XYZ_desired.z;
          	//}
        	//}
        	//else Hover_Checker = 0;
		}
    }

    if (!isHovering || !isLanding )
    {
      if(X_Goal - XYZ_desired.x >= X_Inc ) XYZ_desired.x +=X_Inc;
      if(X_Goal - XYZ_desired.x <= -X_Inc ) XYZ_desired.x -=X_Inc;

      if(Y_Goal - XYZ_desired.y >= Y_Inc ) XYZ_desired.y +=Y_Inc;
      if(Y_Goal - XYZ_desired.y <= -Y_Inc) XYZ_desired.y -=Y_Inc;   

      if(y_Goal - rpy_desired.z >= yaw_Inc ) rpy_desired.z +=yaw_Inc; //ASDF
      if(y_Goal - rpy_desired.z <= -yaw_Inc) rpy_desired.z -=yaw_Inc;

      if(z_Goal - XYZ_desired.z >= z_Inc ) XYZ_desired.z +=z_Inc;
      if(z_Goal - XYZ_desired.z <= -z_Inc) XYZ_desired.z -=z_Inc;
    }

  }

}


double e_r = 0;
double e_p = 0;
double e_y = 0;

double e_r_dot = 0; //REVISE
double e_p_dot = 0;
double e_y_dot = 0;//REVISE

geometry_msgs::Vector3 ang_acl;
geometry_msgs::Vector3 rpy_ddot_d;

Eigen::Vector3d tau_cmd(3);
Eigen::Vector3d rpy_ddot_cmd(3);
double omega = 200;
void attitude_controller()
{
  
  e_r = rpy_desired.x - imu_rpy.x;
  e_p = rpy_desired.y - imu_rpy.y;
  e_y = rpy_desired.z - imu_rpy.z;

  /*
  ang_acl.x = (imu_ang_vel.x-prev_ang_vel.x)/delta_t.count();//REVISE
	ang_acl.y = (imu_ang_vel.y-prev_ang_vel.y)/delta_t.count();
	ang_acl.z = (imu_ang_vel.z-prev_ang_vel.z)/delta_t.count();//REVISE
  */

  e_r_i += e_r * delta_t.count();
	if (fabs(e_r_i) > integ_limit)	e_r_i = (e_r_i / fabs(e_r_i)) * integ_limit;
	e_p_i += e_p * delta_t.count();
	if (fabs(e_p_i) > integ_limit)	e_p_i = (e_p_i / fabs(e_p_i)) * integ_limit;
  e_y_i += e_p * delta_t.count();
	if (fabs(e_y_i) > integ_yaw_limit)	e_y_i = (e_y_i / fabs(e_y_i)) * integ_yaw_limit;

  /*
  rpy_dot_desired.x = Par* e_r + Iar * e_r_i + Dar * (-imu_ang_vel.x); //REVISE
	rpy_dot_desired.y = Par * e_p + Iar * e_p_dot_i + Dar * (-imu_ang_vel.y); 
	rpy_dot_desired.z = Py * e_y + Dy * (-imu_ang_vel.z); //REVISE
	*/
  /*
  e_r_dot_i += e_r_dot * delta_t.count();
	if (fabs(e_r_dot_i) > integ_limit)	e_r_dot_i = (e_r_dot_i / fabs(e_r_dot_i)) * integ_limit; //REVISE
	e_p_dot_i += e_p_dot * delta_t.count();
	if (fabs(e_p_dot_i) > integ_limit)	e_p_dot_i = (e_p_dot_i / fabs(e_p_dot_i)) * integ_limit;
  e_y_dot_i += e_p_dot * delta_t.count();
	if (fabs(e_y_dot_i) > integ_yaw_limit)	e_y_dot_i = (e_y_dot_i / fabs(e_y_dot_i)) * integ_yaw_limit;//REVISE
  */

  rpy_ddot_d.x = Par* e_r + Iar * e_r_i + Dar * (-imu_ang_vel.x); //REVISE
	rpy_ddot_d.y = Par * e_p + Iar * e_p_i + Dar * (-imu_ang_vel.y); 
	rpy_ddot_d.z = Py * e_y + Iy*e_y_i + Dy * (-imu_ang_vel.z); //REVISE
	
  rpy_ddot_cmd << rpy_ddot_d.x, rpy_ddot_d.y, rpy_ddot_d.z;

  tau_cmd = hat_MoI*rpy_ddot_cmd; // Calculate tau rpy

	tau_rpy_desired.x = tau_cmd(0);
	tau_rpy_desired.y = tau_cmd(1);
	tau_rpy_desired.z = tau_cmd(2);
  
  

}

geometry_msgs::Vector3 lin_vel_desired;
void position_controller()
{
  double e_X=0;
  double e_Y=0;

  // should change SBUS to Ground Station input

  e_X = XYZ_desired.x - position_from_t265.x;
  e_Y = XYZ_desired.y - position_from_t265.y;
  e_X_i += e_X * delta_t.count();
  if (fabs(e_X_i) > position_integ_limit) e_X_i = (e_X_i / fabs(e_X_i)) * position_integ_limit;
  e_Y_i += e_Y * delta_t.count();
  if (fabs(e_Y_i) > position_integ_limit) e_Y_i = (e_Y_i / fabs(e_Y_i)) * position_integ_limit;

  XYZ_dot_desired.x = Pp * e_X + Ip * e_X_i - Dp * lin_vel.x;
  XYZ_dot_desired.y = Pp * e_Y + Ip * e_Y_i - Dp * lin_vel.y;
  if(fabs(XYZ_dot_desired.x) > XYZ_dot_limit) XYZ_dot_desired.x = (XYZ_dot_desired.x/fabs(XYZ_dot_desired.x))*XYZ_dot_limit;
  if(fabs(XYZ_dot_desired.y) > XYZ_dot_limit) XYZ_dot_desired.y = (XYZ_dot_desired.y/fabs(XYZ_dot_desired.y))*XYZ_dot_limit;

  lin_vel_desired.x = XYZ_dot_desired.x;
  lin_vel_desired.y = XYZ_dot_desired.y;
}

double X_ddot_d=0;
double Y_ddot_d=0;
double Z_ddot_d=0;
void velocity_controller()
{
  double e_X_dot = 0;
  double e_Y_dot = 0;

  e_X_dot = XYZ_dot_desired.x - lin_vel.x;
  e_Y_dot = XYZ_dot_desired.y - lin_vel.y;
  e_X_dot_i += e_X_dot * delta_t.count();
  //limitation
  if (fabs(e_X_dot_i) > velocity_integ_limit) e_X_dot_i = (e_X_dot_i / fabs(e_X_dot_i)) * velocity_integ_limit;
  e_Y_dot_i += e_Y_dot * delta_t.count();
  if (fabs(e_Y_dot_i) > velocity_integ_limit) e_Y_dot_i = (e_Y_dot_i / fabs(e_Y_dot_i)) * velocity_integ_limit;
  XYZ_ddot_desired.x = Pv * e_X_dot + Iv * e_X_dot_i - Dv * lin_acl.x;
  XYZ_ddot_desired.y = Pv * e_Y_dot + Iv * e_Y_dot_i - Dv * lin_acl.y;
 
  //limitation
  if(fabs(XYZ_ddot_desired.x) > XYZ_ddot_limit) XYZ_ddot_desired.x = (XYZ_ddot_desired.x/fabs(XYZ_ddot_desired.x))*XYZ_ddot_limit;
  if(fabs(XYZ_ddot_desired.y) > XYZ_ddot_limit) XYZ_ddot_desired.y = (XYZ_ddot_desired.y/fabs(XYZ_ddot_desired.y))*XYZ_ddot_limit;
X_ddot_d=XYZ_ddot_desired.x;
Y_ddot_d=XYZ_ddot_desired.y;
/*  F_xyzd.x = mass_system*(W2B_rot(0,0)*XYZ_ddot_desired.x
                   +W2B_rot(0,1)*XYZ_ddot_desired.y
                   +W2B_rot(0,2)*(-XYZ_ddot_desired.z));
  
  F_xyzd.y = mass_system*(W2B_rot(1,0)*(-XYZ_ddot_desired.x)
                   +W2B_rot(1,1)*XYZ_ddot_desired.y
                   +W2B_rot(1,2)*XYZ_ddot_desired.z);*/
  F_xyzd.x = mass_system*(X_ddot_d*cos(imu_rpy.z)*cos(imu_rpy.y)+Y_ddot_d*sin(imu_rpy.z)*cos(imu_rpy.y)-(Z_ddot_d)*sin(imu_rpy.y));
  F_xyzd.y = mass_system*(-X_ddot_d*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+Y_ddot_d*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d)*cos(imu_rpy.y)*sin(imu_rpy.x));
  // Force limitation
  if(fabs(F_xyzd.x) > F_xd_limit) F_xyzd.x = (F_xyzd.x/fabs(F_xyzd.x))*F_xd_limit;
  if(fabs(F_xyzd.y) > F_yd_limit) F_xyzd.y = (F_xyzd.y/fabs(F_xyzd.y))*F_yd_limit;
}

void altitude_controller()
{
  
  double e_Z=0;

  e_Z = XYZ_desired.z - position_from_t265.z;
  e_Z_i += e_Z * delta_t.count();	
  if (fabs(e_Z_i) > z_integ_limit) e_Z_i = (e_Z_i / fabs(e_Z_i)) * z_integ_limit;

  XYZ_ddot_desired.z = Pz * e_Z + Iz * e_Z_i - Dz * lin_vel.z;
  Z_ddot_d = XYZ_ddot_desired.z;
  lin_vel_desired.z = 0; // But this is desired acceleration
  
  // Global to Body ration
	 /* F_xyzd.z = mass_system*(W2B_rot(2,0)*XYZ_ddot_desired.x
                                      +W2B_rot(2,1)*(-XYZ_ddot_desired.y)
                                      +W2B_rot(2,2)*XYZ_ddot_desired.z);*/
	  F_xyzd.z = mass_system*(X_ddot_d*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-Y_ddot_d*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d)*cos(imu_rpy.x)*cos(imu_rpy.y));
  
       
  //else F_xyzd.z = mass_system*(XYZ_ddot_desired.z);
  if(F_xyzd.z > -0.5*mass_system*g) F_xyzd.z = -0.5*mass_system*g;
  if(F_xyzd.z <= -1.7*mass_system*g) F_xyzd.z = -1.7*mass_system*g;


  //F_zd = mass*(X_ddot_d*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-Y_ddot_d*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d)*cos(imu_rpy.x)*cos(imu_rpy.y));
}

Eigen::VectorXd allocation_factor(2);
Eigen::MatrixXd K_M(4,2);
Eigen::MatrixXd invK_M(2,4);
Eigen::VectorXd Dump(4);
void K_matrix()
{
  double F_xd=F_xyzd.x;
  double F_yd=F_xyzd.y;
  double F_zd=F_xyzd.z;
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


void wrench_allocation()
{
  //////////// torque distribute ////////////
  tau_rpy_desired.x = tau_rpy_desired.x/module_num;
  tau_rpy_desired.y = tau_rpy_desired.y/module_num;
  tau_rpy_desired.z = tau_rpy_desired.z/module_num;
  
  //////////// force distribute ////////////
  if(!mono_flight){
  F_xyzd_sub1.x = F_xyzd.x*allocation_factor(1);
  F_xyzd_sub1.y = F_xyzd.y*allocation_factor(1);
  F_xyzd_sub1.z = F_xyzd.z*allocation_factor(1);

  F_xyzd.x = F_xyzd.x*allocation_factor(0);
  F_xyzd.y = F_xyzd.y*allocation_factor(0);
  F_xyzd.z = F_xyzd.z*allocation_factor(0);
  }
}

void yaw_torque_distribute()
{
  tau_y_d_non_sat=tau_rpy_desired.z;
  if(fabs(tau_rpy_desired.z) > tau_y_limit)
  {
     tau_rpy_desired.z = tau_rpy_desired.z/fabs(tau_rpy_desired.z)*tau_y_limit;
  }

  if((abs(tau_rpy_desired.z)-tau_y_limit)==0)
  {
	tau_y_th = tau_y_d_non_sat-tau_rpy_desired.z;
	if(fabs(tau_y_th) > tau_y_th_limit) tau_y_th = (tau_y_th/fabs(tau_y_th))*tau_y_th_limit;//2023.08.17 update
  }
}


Eigen::MatrixXd CM_Xc_p2(4,4); //Thrust Allocation
Eigen::MatrixXd invCM_Xc_p2(4,4);
void setCM_Xc_p2()
{ 

  double xc = X_c_p2(0);
  double yc = X_c_p2(1);
  double zc = X_c_p2(2);
  
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
	-cos(th1), 
	-cos(th2), 
	-cos(th3), 
	-cos(th4);
	invCM_Xc_p2 = CM_Xc_p2.inverse();
	
}

double F1=0; // desired motor1 thrust command
double F2=0; // desired motor2 thrust command
double F3=0; // desired motor3 thrust command
double F4=0; // desired motor4 thrust command
Eigen::MatrixXd SA(4,4);
Eigen::MatrixXd invSA(4,4);
void setSA()
{
  SA <<  F1/r2,     F2/r2,     -(F3/r2),     -(F4/r2),
	       F1/r2,    -(F2/r2),   -(F3/r2),       F4/r2,
	      r_arm*F1,  r_arm*F2,   r_arm*F3,   r_arm*F4,
	      r_arm*F1, -(r_arm*F2),   r_arm*F3,  -(r_arm*F4);
  
  invSA=SA.inverse();
}
//////////////////////////////////////

Eigen::VectorXd U(4);
Eigen::VectorXd desired_prop_force(4); // desired motor thrust command
Eigen::VectorXd control_by_theta(4);
Eigen::VectorXd sine_theta_command(4);
std_msgs::Float32MultiArray Force_prop;
double theta1_command=0;
double theta2_command=0;
double theta3_command=0;
double theta4_command=0;
void PWM_signal_Generator()
{

  
  setCM_Xc_p2();
  U << tau_rpy_desired.x, 
       tau_rpy_desired.y, 
       tau_rpy_desired.z, 
       F_xyzd.z;
  //U <<0,0,0,150; //F1234가 이상하게 나오는 것은 U에 값이 정확하게 들어가지 않아서임.23.09.25
  desired_prop_force=invCM_Xc_p2*U;

  F1=desired_prop_force(0);
  F2=desired_prop_force(1);
  F3=desired_prop_force(2);
  F4=desired_prop_force(3);
  
  setSA();
  
  control_by_theta << F_xyzd.x, F_xyzd.y, tau_y_th, 0;
  sine_theta_command = invSA*control_by_theta;

  theta1_command = asin(sine_theta_command(0));
  theta2_command = asin(sine_theta_command(1));
  theta3_command = asin(sine_theta_command(2));
  theta4_command = asin(sine_theta_command(3));
  if(fabs(theta1_command)>hardware_servo_limit) theta1_command = (theta1_command/fabs(theta1_command))*hardware_servo_limit;
  if(fabs(theta2_command)>hardware_servo_limit) theta2_command = (theta2_command/fabs(theta2_command))*hardware_servo_limit;
  if(fabs(theta3_command)>hardware_servo_limit) theta3_command = (theta3_command/fabs(theta3_command))*hardware_servo_limit;
  if(fabs(theta4_command)>hardware_servo_limit) theta4_command = (theta4_command/fabs(theta4_command))*hardware_servo_limit;
  

  pwm_Kill();
  //pwm_Command(Force_to_PWM(F1),Force_to_PWM(F2), Force_to_PWM(F3), Force_to_PWM(F4));
  
  Force_prop.data.resize(4);
  Force_prop.data[0]=F1;
  Force_prop.data[1]=F2;
  Force_prop.data[2]=F3;
  Force_prop.data[3]=F4;

}

///////// MAIN TO SUB DATA ////////////
std_msgs::String send_data_for_sub;
std::string serial_buffer;
double data1,data2,data3,data4,data5,data6;
bool data7;
std::string data_2_string()
{
  data1=F_xyzd_sub1.x;
  data2=F_xyzd_sub1.y;
  data3=F_xyzd_sub1.z;
  data4=tau_rpy_desired.x;
  data5=tau_rpy_desired.y;
  data6=tau_rpy_desired.z;
  data7=kill_mode;
  

  serial_buffer ="<"+std::to_string(data1)+"PE"
                    +std::to_string(data2)+"TE"
                    +std::to_string(data3)+"KE"
                    +std::to_string(data4)+"XE"
                    +std::to_string(data5)+"YE"
                    +std::to_string(data6)+"ZE"
                    +std::to_string(data7)+">";
  return serial_buffer;
}






void PublishData()
{
  send_data_for_sub.data = data_2_string();
  ToSubAgent.publish(send_data_for_sub); // send for sub agent data
  
  
  goal_dynamixel_position.publish(servo_msg_create(theta1_command,theta2_command, theta3_command, theta4_command)); // desired theta

  PWM_generator.publish(PWMs_val); // To ros-pca9685-node
  PWMs.publish(PWMs_cmd);// PWMs_d value
  desired_motor_thrust.publish(Force_prop);// desired force to motor
  
  euler.publish(imu_rpy);//rpy_act value
  desired_angle.publish(rpy_desired);//rpy_d value
  desired_torque.publish(tau_rpy_desired); // torque desired
	
  position.publish(position_from_t265); // actual position
  desired_position.publish(XYZ_desired);//desired position 

  linear_velocity.publish(lin_vel); // actual linear velocity 
  desired_velocity.publish(lin_vel_desired); // desired linear velocity   
  
  desired_force.publish(F_xyzd); // desired force it need only tilt mode 	
  battery_voltage.publish(battery_voltage_msg);
  delta_time.publish(dt);
    
  ///////////////////////Previous Data Shifting///////////////////////////
	
  prev_ang_vel = imu_ang_vel;
  prev_lin_vel = lin_vel;


}

void reset_data()
{

  rpy_desired.z=t265_att.z;	//[J]This line ensures that yaw desired right after disabling the kill switch becomes current yaw attitude
  //initial_z=position_from_t265.x;
  e_r_i = 0;
  e_p_i = 0;
  e_y_i = 0;
  
  e_r_dot_i = 0;
  e_p_dot_i = 0;
  e_y_dot_i = 0;

  e_Z_i = 0;
  e_X_i=0;
  
}