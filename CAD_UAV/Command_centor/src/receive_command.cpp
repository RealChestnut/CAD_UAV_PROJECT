
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

using namespace std;

serial::Serial ser_serve1;

int topic_num = 5;
string temp_result;
bool temp_result_on=false;
string buffer="";
string temp_1;
bool temp_1_on=false;
string temp_2;
bool temp_2_on=false;
string temp_3;
bool temp_3_on=false;
bool reconfig_port_flag=false;

std_msgs::String send_data_for_sub;

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_node_main");
    ros::NodeHandle nh;
    
    ros::Publisher read_from_command = nh.advertise<std_msgs::String>("read_from_command", 1);

    

    try
    {
        ser_serve1.setPort("/dev//dev/ttyZIG_SERVE2");
        ser_serve1.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_serve1.setTimeout(to);
	    serial::flowcontrol_t flow_state = serial::flowcontrol_t::flowcontrol_none;
        ser_serve1.setFlowcontrol(flow_state);
        ser_serve1.open();
	    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser_serve1.isOpen()){    
        ROS_INFO_STREAM("Serial Port initialized");
    
    }else{
        return -1;
    }
    
    ros::Rate loop_rate(50);
    
    while(ros::ok()){
         
    ros::spinOnce();
	
	if(ser_serve1.available()){
        buffer.clear();
	ser.flush();


	    //To avoid undefined data for software blocking safety
            //reconfigure_port();
	    serial_open_safety();
	    
	    //ROS_INFO_STREAM(buffer);

	    //receive_data_test(buffer);

	    std_msgs::Float32MultiArray result;

    	result.data.resize(topic_num);
	    
	    receive_data(buffer);
	 ////////////// parsing from complete data to std_msgs   /////////////////////
	    
	//	if(last_dump.size()==topic_num){
		
			string dumi="";
			ROS_INFO_STREAM(last_dump.size());
			for(int i =0;i<last_dump.size();i++)
			{
				dumi = last_dump.at(i);
				ROS_INFO_STREAM("IM_DUMI :: " << dumi);
				//result.data[i]=strtof(dumi.c_str(),nullptr); // error code last dump의 크기보다 receive_data의 크기가 작아 발생한 애러
				ROS_INFO_STREAM(last_dump[i]);
				
			}
			read_command_from_main.publish(result);
	//	}
	/////////////////////////////////////////////////////////////////////// 		
	    last_dump.clear();

	loop_rate.sleep();
	
	



        loop_rate.sleep();

        }
    }
}


void parseData(const string& str, vector<string>& values,string& delimiter){
	string msg;
	msg.assign(str);
	
	if((msg.find('<')!=string::npos) && (msg.find('|')!=string::npos))
	{
		msg.erase(std::find(msg.begin(),msg.end(),'<'));
		msg.erase(std::find(msg.begin(),msg.end(),'|'));
	}

		
	string::size_type Fpos = msg.find_first_not_of(delimiter,0);
	string::size_type Lpos = msg.find_first_of(delimiter, Fpos);
	while (string::npos != Fpos || string::npos != Lpos)
	{
		
	
		values.push_back(msg.substr(Fpos, Lpos - Fpos));
		
		

		Fpos = msg.find_first_not_of(delimiter, Lpos);
		Lpos = msg.find_first_of(delimiter, Fpos);
		
	}

}

void receive_data(const string& str){
	    //1. receive data from buffer 
	    // we dont know per buffer has markers ensurely 
	    //2. check start marker end, end marker
	    //first check that buffer has end marker
	    //if has end marker, put data from end marker to string head position
	    //if has start marker, put data from < to the rest
	    //3. and take a loop again until buffer has > end marker
	    //if has end marker, 


	    int i = 0;
	    bool start_flag=false;
	    while(i<str.size())
	    {
		    if((str[i] == '>' ) && (str[i] == '|' )&& temp_2.empty())
		    {
			    temp_1=str;
			    temp_1.clear();
		    }
		    if(str[i]=='<')
		    {
			    start_flag=true;
			    if(temp_2.empty() && !temp_2_on){temp_2_on=true;}
		    }

		    if(temp_2_on)
		    {
			    temp_2.push_back(str[i]);
			    if(str[i]=='|')
			    {
				    cnt_start++;
				    parseData(temp_2,last_dump ,dot);
			   	    cnt_end++;
				    //ROS_INFO_STREAM(temp_2);
				    temp_2.clear();
				    temp_2_on=false;


			    }
		    }

		    i++;
	    }


}
