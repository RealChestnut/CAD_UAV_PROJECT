
#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <sys/ioctl.h>
#include "rqt_mypkg/ArmService.h"
#include "rqt_mypkg/KillService.h"
#include "rqt_mypkg/PosCtrlService.h"
#include "rqt_mypkg/DockService.h"
#include "rqt_mypkg/HoverService.h"
#include "rqt_mypkg/ApproachService.h"
#include "rqt_mypkg/ManipulatorService.h"


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
bool isHover=false;
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

bool Hovering_Callback(rqt_mypkg::HoverService::Request &req, rqt_mypkg::HoverService::Response &res){
	if(req.isHover){
        isHover = true;
		ROS_INFO_STREAM("Hovering :: "<< isHover);
	}
	else{
        isHover = false;
		ROS_INFO_STREAM("Hovering :: "<< isHover);
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



int main (int argc, char** argv){
    ros::init(argc, argv, "send_command_message");
    ros::NodeHandle nh;
    
    //rqt_mypkg::DockService Service;

    
    std_msgs::UInt16 cmd_msg;
	ros::ServiceServer DockServer = nh.advertiseService("/DockService", Docking_Callback); //ASDF
    //ros::ServiceServer LandServer = nh.advertiseService("/LandService",Hovering_Callback);
    ros::ServiceServer ManipulatorServer = nh.advertiseService("/ManipulatorService",Manipulating_Callback);
    //ros::ServiceServer HoverServer = nh.advertiseService("/HoverService",Hovering_Callback);
    ros::ServiceServer ApproachServer = nh.advertiseService("/ApproachService",Approching_Callback);

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

    string message_safety="";
    
    string command_message=""; 
    while(ros::ok()){
         
    ros::spinOnce();
	
	if(ser_master.available()){
		message_safety=ser_master.read();
		//ROS_INFO_STREAM("receive_serve   : " << message_safety);}
    }

	
	command_message= data_2_string(isAppraoch, isDock, isManipul);

	ROS_INFO_STREAM(command_message);
      	
	ser_master.write(command_message);

    loop_rate.sleep();

    
    }
}

