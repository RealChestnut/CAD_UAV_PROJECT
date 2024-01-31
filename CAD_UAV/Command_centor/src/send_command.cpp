
#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <sys/ioctl.h>
#include "rqt_mypkg/ArmService.h"
#include "rqt_mypkg/KillService.h"
#include "rqt_mypkg/PosCtrlService.h"
#include "rqt_mypkg/DockService.h"
#include "rqt_mypkg/HoverService.h"
#include "rqt_mypkg/ApproachService.h"
#include "rqt_mypkg/ManipulatorService.h"
#include "rqt_mypkg/DockSafetyService.h"
#include <chrono>

using namespace std;

serial::Serial ser_master;



std_msgs::String ToSub;
std_msgs::String push_data;

std_msgs::String send_data_for_sub;



std::string serial_buffer;
std::string data_2_string(bool Approach, bool Dock, bool Manipulate)
{


  serial_buffer ="<"+std::to_string(Approach)+","
                    +std::to_string(Dock)+","
                    +std::to_string(Manipulate)+
                   ">";

  // kill_mode는 end단에서 선언하지 않으면 변환된 데이터가 들어가지 않음
 ROS_INFO_STREAM(serial_buffer);
 return serial_buffer;
}

/////////////////////////////////////////////////////////////////




bool isDock=false;
bool isAppraoch=false;
bool isManipul = false;

bool Docking_Callback(rqt_mypkg::DockService::Request &req, rqt_mypkg::DockService::Response &res){
	if(req.Dock_Do){
        isDock = true;
		ROS_INFO_STREAM("Docking :: "<< isDock);
	}
	else{
        isDock = false;
		ROS_INFO_STREAM("Docking :: "<< isDock);
	}
	return true;
}

bool Approching_Callback(rqt_mypkg::ApproachService::Request &req, rqt_mypkg::ApproachService::Response &res){
	if(req.isApproach){
        isAppraoch = true;
		ROS_INFO_STREAM("Approach :: "<< isAppraoch);
	}
	else{
        isAppraoch = false;
		ROS_INFO_STREAM("Approach :: "<< isAppraoch);
	}
	return true;
}

bool Manipulating_Callback(rqt_mypkg::ManipulatorService::Request &req, rqt_mypkg::ManipulatorService::Response &res){
	if(req.Manipul){
        isManipul = true;
		ROS_INFO_STREAM("Manipulate :: "<< isManipul);
	}
	else{
        isManipul = false;
		ROS_INFO_STREAM("Manipulate :: "<< isManipul);
	}
	return true;
}

bool safety_process = false;
int cnt = 0;
void DockSafetyProcess()
{
    rqt_mypkg::DockSafetyService Service;
    
    Service.request.Safety_Do = safety_process;


}

bool ConvertStringToBoolian(const std::string& str)
{
    int dumi = 0;
    dumi = std::stoi(str);
    return static_cast<bool>(dumi);
}



int main (int argc, char** argv){
    ros::init(argc, argv, "send_command_message");
    ros::NodeHandle nh;
    
   
    std_msgs::UInt16 cmd_msg;
	ros::ServiceServer DockServer = nh.advertiseService("/DockService", Docking_Callback); //ASDF
    ros::ServiceServer ManipulatorServer = nh.advertiseService("/ManipulatorService",Manipulating_Callback);
    ros::ServiceServer ApproachServer = nh.advertiseService("/ApproachService",Approching_Callback);
    ros::Publisher DockSafety_pub = nh.advertise<std_msgs::Bool>("/DockSafetyPub",1);

    std::chrono::high_resolution_clock::time_point end=std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point start=std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> delta_t;

    try
    {
        ser_master.setPort("/dev/ttyZIG_MASTER");
        ser_master.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_master.setTimeout(to);
	    serial::flowcontrol_t flow_state = serial::flowcontrol_t::flowcontrol_none;
        ser_master.setFlowcontrol(flow_state);
        ser_master.open();
	    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser_master.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    
    }else{
        return -1;
    }
    

    ros::Rate loop_rate(100);
    std_msgs::Bool safety_process_;
    string message_safety="";
    string command_message="";
    bool cnt_down_flag = false;
    double delta_buff=0.0;
    double time_sat = 1;
    while(ros::ok()){
         
    ros::spinOnce();

    end=chrono::high_resolution_clock::now();
    delta_t=end-start;
    start=chrono::high_resolution_clock::now();

	/*
    if((cnt<301)&&cnt_down_flag==false){if(cnt==300){cnt_down_flag=true;safety_process_.data=false;DockSafety_pub.publish(safety_process_);}cnt++;}
    
    if(cnt_down_flag){cnt--; if(cnt==0){cnt_down_flag=false;safety_process_.data=true;DockSafety_pub.publish(safety_process_);}}
    ROS_INFO_STREAM(cnt);
    */
    
	if(ser_master.available()){
		message_safety=ser_master.read();
        safety_process_.data = ConvertStringToBoolian(message_safety);

        if(safety_process_.data ==true)
        {
           delta_buff+=delta_t.count();
           if(delta_buff>=time_sat){DockSafety_pub.publish(safety_process_);}

        }
        else if(safety_process_.data ==false)
        {
            DockSafety_pub.publish(safety_process_);
            delta_buff =0;
        }
        ROS_INFO_STREAM(delta_buff);

        
		ROS_INFO_STREAM("receive_serve   : " << message_safety);
    }
	if(!ser_master.available()){message_safety.clear();}
	command_message= data_2_string(isAppraoch, isDock, isManipul);

	//ROS_INFO_STREAM(command_message);
      	
	ser_master.write(command_message);

    loop_rate.sleep();

    
    }
}

