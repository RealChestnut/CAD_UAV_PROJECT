#include <cad_norm_version.hpp>

//void cad_Loop::ControlLoopSet(); // For Control LOOP Fuction w.r.t. Time( && freqency )
void launchparam(); // data that wrote on launch file 
void publishset(); // publish set for data sending( log, serial communication, send servo motion)

cad_Loop::cad_Loop()
: node_handle_(""),
  priv_node_handle_("~")
  //cad_Loop::cad_Loop(ros::NodeHandle* nodehandle):node_handle_(*nodehandle)
  {

    // initialize Subscriber, Publisher //

    initSubscriber();
    initPublisher();

    // servo theta<<0000; 여기서 이렇게 초기화 하면 에러가남 와이?
    servo_theta.resize(4);
    servo_theta << 0,0,0,0;
    
    //World to Body frame rotation matrix
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

  // boolian initialize
  mono_flight_mode = true;
  combined_flight_mode = false;

    



    
    
  }
  
  void cad_Loop::initSubscriber()
  {

    dynamixel_state = node_handle_.subscribe("joint_states",100,&cad_Loop::jointstate_Callback, this, ros::TransportHints().tcpNoDelay());
   	att = node_handle_.subscribe("/imu/data",1,&cad_Loop::imu_Callback,this,ros::TransportHints().tcpNoDelay());
    rc_in = node_handle_.subscribe("/sbus",100,&cad_Loop::sbus_Callback,this,ros::TransportHints().tcpNoDelay());
    battery_checker = node_handle_.subscribe("/battery",100,&cad_Loop::battery_Callback,this,ros::TransportHints().tcpNoDelay());
    t265_position=node_handle_.subscribe("/t265_pos",100,&cad_Loop::t265_position_Callback,this,ros::TransportHints().tcpNoDelay());
    t265_odom=node_handle_.subscribe("/rs_t265/odom/sample",100,&cad_Loop::t265_Odom_Callback,this,ros::TransportHints().tcpNoDelay());

  }

  void cad_Loop::initPublisher()
  {
    PWMs = node_handle_.advertise<std_msgs::Int16MultiArray>("PWMs", 1); 
    PWM_generator = node_handle_.advertise<std_msgs::Int32MultiArray>("command",1);  // publish to pca9685
    desired_motor_thrust = node_handle_.advertise<std_msgs::Float32MultiArray>("Forces",100);
    
    goal_dynamixel_position  = node_handle_.advertise<sensor_msgs::JointState>("goal_dynamixel_position",100); // desired theta1,2
	
    euler = node_handle_.advertise<geometry_msgs::Vector3>("angle",1); 
	  desired_angle = node_handle_.advertise<geometry_msgs::Vector3>("desired_angle",100);
	
    
    desired_torque = node_handle_.advertise<geometry_msgs::Vector3>("torque_d",100);
    
    linear_velocity = node_handle_.advertise<geometry_msgs::Vector3>("lin_vel",100);
    desired_velocity = node_handle_.advertise<geometry_msgs::Vector3>("lin_vel_d",100);

    angular_velocity = node_handle_.advertise<geometry_msgs::Vector3>("gyro",100);
	
    desired_position = node_handle_.advertise<geometry_msgs::Vector3>("pos_d",100);
	  position = node_handle_.advertise<geometry_msgs::Vector3>("pos",100);
	
    desired_force = node_handle_.advertise<geometry_msgs::Vector3>("force_d",100);
	
    battery_voltage = node_handle_.advertise<std_msgs::Float32>("battery_voltage",100);
	  delta_time = node_handle_.advertise<std_msgs::Float32>("delta_t",100);

  }

  void cad_Loop::jointstate_Callback(const sensor_msgs::JointState& msg)
  {
    servo_theta(0)=msg.position[0];
    servo_theta(1)=msg.position[1];
    servo_theta(2)=msg.position[2];
    servo_theta(3)=msg.position[3];
  }

  void cad_Loop::imu_Callback(const sensor_msgs::Imu& msg)
  {
    // TP attitude - Quaternion representation
    imu_quaternion=msg.orientation;
    imu_angular_velocity=msg.angular_velocity;
    imu_linear_acceleration=msg.linear_acceleration;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(imu_quaternion,quat);

    // TP attitude - Euler representation
    tf::Matrix3x3(quat).getRPY(imu_rpy.x,imu_rpy.y,imu_rpy.z);
    base_yaw = t265_att.z;
    if(base_yaw - yaw_prev < -pi) yaw_rotate_count++;
    else if(base_yaw - yaw_prev > pi) yaw_rotate_count--;
	  yaw_now = base_yaw+2*pi*yaw_rotate_count;
	
    imu_rpy.z = yaw_now;
    yaw_prev = base_yaw;
	
  }

  void cad_Loop::sbus_Callback(const std_msgs::Int16MultiArray::ConstPtr& array)
  {
    for(int i=0;i<10;i++){
		Sbus[i]=map<int16_t>(array->data[i], 352, 1696, 1000, 2000);
	}
	
	if(Sbus[4]<1500) kill_mode=true;
	else kill_mode=false;
  }

  void cad_Loop::battery_Callback(const std_msgs::Int16& msg)
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

  void cad_Loop::t265_position_Callback(const geometry_msgs::Vector3& msg)
  {
    position_from_t265.x=msg.x;
    position_from_t265.y=msg.y;
    position_from_t265.z=msg.z;
  }
  

  Eigen::Vector3d cam_v;
  Eigen::Matrix3d R_v;
  Eigen::Vector3d v;

  void cad_Loop::t265_Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    linear_velocity_from_t265=msg->twist.twist.linear;
    angular_velocity_from_t265=msg->twist.twist.angular;
    t265_quat=msg->pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(t265_quat,quat);
    tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);
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

void cad_Loop::shape_detector()
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


void cad_Loop::UpdateParameter()
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

                mass_sub1 = 0;
                mass_sub2 = 0;
                
                X_c_p1 << 0,0,0;
                X_c_p2 << x_c_hat,y_c_hat,z_c_hat;    

  }
  else if(num==2){
      

                //p_c_main << x_c_hat,y_c_hat,z_c_hat;
                //p_c_sub1 << x_c_hat,y_c_hat+2*l_module,z_c_hat;
                //p_c_sub2 << 0,0,0;
    
                mass_sub1=5.6;
                mass_sub2=0;

                X_c_p1 << 0,y_c_hat,0;
                X_c_p2 << x_c_hat,0,z_c_hat;

  }
  else if(num==3){

                //p_c_main << x_c_hat,y_c_hat,z_c_hat;
                //p_c_sub1 << 0,0,0;
                //p_c_sub2 << x_c_hat-l_module,y_c_hat,z_c_hat;

                mass_sub1=0;
                mass_sub2=5.6;
                X_c_p1 << 0,y_c_hat,0;
                X_c_p2 << x_c_hat,0,z_c_hat;
  }  

}

Eigen::VectorXd allocation_factor(2);
Eigen::MatrixXd K_M(4,2);
Eigen::MatrixXd invK_M(2,4);
Eigen::VectorXd Dump(4);
void cad_Loop::K_matrix(){


        Dump << 0,0,0,1;
        K_M << F_yd*X_c_p1(2)-F_zd*X_c_p1(1), F_yd*X_c_p1(2)-F_zd*(X_c_p1(1)+2*l_module),
                -(F_xd*X_c_p1(2)-F_zd*X_c_p1(0)), -(F_xd*X_c_p1(2)-F_zd*X_c_p1(0)),
                F_xd*X_c_p1(1)-F_yd*X_c_p1(0), F_xd*(X_c_p1(1)+2*l_module)-F_yd*X_c_p1(0),
                1,      1;

        invK_M = K_M.completeOrthogonalDecomposition().pseudoInverse();

        allocation_factor = invK_M*Dump;
       
}



  void cad_Loop::attitude_controller()
{

		r_d = 0.0;
		p_d = 0.0;
      
    //Translational Force :: world to body frame
    

}

void cad_Loop::position_controller()
{
  double e_X=0;
  double e_Y=0;

  // should change SBUS to Ground Station input
  X_d = X_d_base - XY_limit*(((double)Sbus[1]-(double)1500)/(double)500);
  Y_d = Y_d_base + XY_limit*(((double)Sbus[3]-(double)1500)/(double)500);

  e_X = X_d - position_from_t265.x;
  e_Y = Y_d - position_from_t265.y;
  e_X_i += e_X * delta_t.count();
  if (fabs(e_X_i) > position_integ_limit) e_X_i = (e_X_i / fabs(e_X_i)) * position_integ_limit;
  e_Y_i += e_Y * delta_t.count();
  if (fabs(e_Y_i) > position_integ_limit) e_Y_i = (e_Y_i / fabs(e_Y_i)) * position_integ_limit;

  X_dot_d = Pp * e_X + Ip * e_X_i - Dp * lin_vel.x;
  Y_dot_d = Pp * e_Y + Ip * e_Y_i - Dp * lin_vel.y;
  if(fabs(X_dot_d) > XYZ_dot_limit) X_dot_d = (X_dot_d/fabs(X_dot_d))*XYZ_dot_limit;
  if(fabs(Y_dot_d) > XYZ_dot_limit) Y_dot_d = (Y_dot_d/fabs(Y_dot_d))*XYZ_dot_limit;

  desired_lin_vel.x = X_dot_d;
	desired_lin_vel.y = Y_dot_d;
}

void cad_Loop::velocity_controller()
{
  double e_X_dot = 0;
	double e_Y_dot = 0;
  Eigen::Vector3d xy_acl_d;
  e_X_dot = X_dot_d - lin_vel.x;
  e_Y_dot = Y_dot_d - lin_vel.y;
  e_X_dot_i += e_X_dot * delta_t.count();
  //limitation
  if (fabs(e_X_dot_i) > velocity_integ_limit) e_X_dot_i = (e_X_dot_i / fabs(e_X_dot_i)) * velocity_integ_limit;
  e_Y_dot_i += e_Y_dot * delta_t.count();
  if (fabs(e_Y_dot_i) > velocity_integ_limit) e_Y_dot_i = (e_Y_dot_i / fabs(e_Y_dot_i)) * velocity_integ_limit;

  X_ddot_d = Pv * e_X_dot + Iv * e_X_dot_i - Dv * lin_acl.x;
  Y_ddot_d = Pv * e_Y_dot + Iv * e_Y_dot_i - Dv * lin_acl.y;
 
  //limitation
  if(fabs(X_ddot_d) > XYZ_ddot_limit) X_ddot_d = (X_ddot_d/fabs(X_ddot_d))*XYZ_ddot_limit;
  if(fabs(Y_ddot_d) > XYZ_ddot_limit) Y_ddot_d = (Y_ddot_d/fabs(Y_ddot_d))*XYZ_ddot_limit;

  F_xd = mass*(W2B_rot(0,0)*X_ddot_d+W2B_rot(0,1)*Y_ddot_d+W2B_rot(0,2)*Z_ddot_d);
  F_yd = mass*(W2B_rot(1,0)*X_ddot_d+W2B_rot(1,1)*Y_ddot_d+W2B_rot(1,2)*Z_ddot_d);
  //F_xd = mass*(X_ddot_d*cos(imu_rpy.z)*cos(imu_rpy.y)+Y_ddot_d*sin(imu_rpy.z)*cos(imu_rpy.y)-(Z_ddot_d)*sin(imu_rpy.y));
  //F_yd = mass*(-X_ddot_d*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+Y_ddot_d*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d)*cos(imu_rpy.y)*sin(imu_rpy.x));
  // Force limitation
  if(fabs(F_xd) > F_xd_limit) F_xd = (F_xd/fabs(F_xd))*F_xd_limit;
  if(fabs(F_yd) > F_yd_limit) F_yd = (F_yd/fabs(F_yd))*F_yd_limit;

  
  
}

void cad_Loop::altitude_controller()
{
  
  double e_Z=0;

  e_Z = Z_d - position_from_t265.z;
  e_Z_i += e_Z * delta_t.count();	
	if (fabs(e_Z_i) > z_integ_limit) e_Z_i = (e_Z_i / fabs(e_Z_i)) * z_integ_limit;

  Z_ddot_d = Pz * e_Z + Iz * e_Z_i - Dz * lin_vel.z;
  desired_lin_vel.z = 0; // But this is desired acceleration
  
  // Global to Body ration
  if(!attitude_mode) F_zd = mass*(W2B_rot(2,0)*X_ddot_d+W2B_rot(2,1)*Y_ddot_d+W2B_rot(2,2)*Z_ddot_d);
  else F_zd = mass*(Z_ddot_d);
  //F_zd = mass*(X_ddot_d*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-Y_ddot_d*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d)*cos(imu_rpy.x)*cos(imu_rpy.y));
}


/*
void cad_Loop::ControlLoopSet()
{


    //여기에서 모든 동작들이 수행됨.

    //shape detector();

    if(true){
    //UpdateParameter(); :: module convert
    //command_generator :: futaba command OR Ground station
    //attitude_controller 
    //attitude_DOB
    //position_controller
    //velocity_controller
    //force_torque_distributor
    }

    //if(combined flight.flag){
    //UpdateParameter(); :: module convert
    //command generator :: futaba command OR Ground station
    //attitude_controller 
    //attittude_DOB
    //position_controller
    //velocity_controller
    //force_torque_allocation
    //}
    
    //publishset();
    

}*/
void cad_Loop::callback(const ros::TimerEvent& event)
{
  ROS_INFO("hello");
}


cad_Loop::~cad_Loop()
{
  ros::shutdown();
}

// ----------------MAIN CONTROL LOOP------------------------//
int main(int argc, char **argv)
{

  //initialize ros node//
  ros::init(argc, argv, "cad_uav");
  cad_Loop CAD_Loop;
  
  ros::NodeHandle nh;  
  ros::Timer timerPublish = nh.createTimer(ros::Duration(1.0/200.0), &cad_Loop::callback, &CAD_Loop);
  ros::spin();
  return 0;
}



void launchparam()
{
    /*
    //Center of Mass
	x_c_hat=nh.param<double>("x_center_of_mass",1.0);
	y_c_hat=nh.param<double>("y_center_of_mass",1.0);
	z_c_hat=nh.param<double>("z_center_of_mass",1.0);

	//Conventional Flight Mode Control Gains
	//Roll, Pitch PID gains
	conv_Par=nh.param<double>("conv_attitude_r_P_gain",3.5);
	conv_Iar=nh.param<double>("conv_attitude_r_I_gain",0.4);
	conv_Dar=nh.param<double>("conv_attitude_r_D_gain",0.5);

	conv_Pap=nh.param<double>("conv_attitude_p_P_gain",3.5);
    conv_Iap=nh.param<double>("conv_attitude_p_I_gain",0.4);
    conv_Dap=nh.param<double>("conv_attitude_p_D_gain",0.5);

    //Yaw PID gains
    conv_Py=nh.param<double>("conv_attitude_y_P_gain",2.0);
    conv_Dy=nh.param<double>("conv_attitude_y_D_gain",0.1);

    //Altitude PID gains
    conv_Pz=nh.param<double>("conv_altitude_P_gain",16.0);
    conv_Iz=nh.param<double>("conv_altitude_I_gain",5.0);
    conv_Dz=nh.param<double>("conv_altitude_D_gain",15.0);

    //Velocity PID gains
    conv_Pv=nh.param<double>("conv_velocity_P_gain",5.0);
    conv_Iv=nh.param<double>("conv_velocity_I_gain",1.0);
    conv_Dv=nh.param<double>("conv_velocity_D_gain",5.0);

    //Position PID gains
    conv_Pp=nh.param<double>("conv_position_P_gain",3.0);
    conv_Ip=nh.param<double>("conv_position_I_gain",0.1);
    conv_Dp=nh.param<double>("conv_position_D_gain",5.0);

//Tilt Flight Mode Control Gains
    //Roll, Pitch PID gains
    
    tilt_Par=nh.param<double>("tilt_attitude_r_P_gain",3.5);
                tilt_Iar=nh.param<double>("tilt_attitude_r_I_gain",3.5);
                tilt_Dar=nh.param<double>("tilt_attitude_r_D_gain",3.5);

    tilt_Pap=nh.param<double>("tilt_attitude_p_P_gain",3.5);
                tilt_Iap=nh.param<double>("tilt_attitude_p_I_gain",3.5);
                tilt_Dap=nh.param<double>("tilt_attitude_p_D_gain",3.5);

    //Yaw PID gains
    tilt_Py=nh.param<double>("tilt_attitude_y_P_gain",5.0);
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
    */
}

void publishset()
{

}