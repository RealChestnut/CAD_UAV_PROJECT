/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string.h>
#include <string>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <algorithm>
#include <iterator>
#include <bits/stdc++.h>

using namespace std;
serial::Serial ser;

static char sSTX() { return static_cast<char>(0x02);}
static char sETX() { return static_cast<char>(0x03);}
static char sACK() { return static_cast<char>(0x06);}
static char sNAK() { return static_cast<char>(0x15);}

int topic_num = 7;
string temp_result;
bool temp_result_on=false;
string buffer="";
string temp_1;
bool temp_1_on=false;
string temp_2;
bool temp_2_on=false;
string temp_3;
bool temp_3_on=false;

string dot = ",";
string startMarker = "<";
string endMarker = ">";
string blink ="";
string receTemp;


string::size_type Start;
string::size_type Tp_end;
string::size_type Tt_end;
string::size_type Tk_end;
string::size_type Fx_end;
string::size_type Fy_end;
string::size_type Fz_end;
string::size_type End;


vector<double> DoubleVec;
vector<string> last_dump(7);
void receive_data(const string& str);
void parseData(const string& str, vector<string>& values, string& delimiter);
void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}



int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1);

    try
    {
        ser.setPort("/dev/ttyS0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(500);
    while(ros::ok()){

        ros::spinOnce();

	buffer.clear();
	
	if(ser.available()){
        
		
		ROS_INFO_STREAM("Reading from serial port");
            
 	    std_msgs::Float32MultiArray result;


	    buffer= ser.read(ser.available());


	    ROS_INFO_STREAM("BUFFER : " << buffer);

	    

	    receive_data(buffer);


		    
	    if(last_dump.size()==topic_num)
	    {

		   transform(last_dump.begin(), last_dump.end(), std::back_inserter(DoubleVec),
				   [&](string s){
				   return stod(s);
				   });
		/*		   
		for(int i =0; i<DoubleVec.size(); i++)
		{
		//	ROS_INFO_STREAM(" TEST : " << DoubleVec[i]);
		}	
		*/   	
		   result.layout.dim.push_back(std_msgs::MultiArrayDimension());
		   result.layout.dim[0].size = last_dump.size();
		   result.layout.dim[0].stride = 1;
		   result.data.clear();
		   result.data.insert(result.data.end(), floatVec.begin(), floatVec.end());
		   
	    last_dump.clear(); 
	    DoubleVec.clear();
		
	    
	}
	loop_rate.sleep();
    }
    	
}


void parseData(const string& str, vector<string>& values, string& delimiter){
	string msg;
	msg.assign(str);
	
	string::size_type Fpos = msg.find_first_not_of(delimiter,0);
	string::size_type Lpos = msg.find_first_of(delimiter, Fpos);
	while (string::npos != Fpos || string::npos != Lpos)
	{
		
	
		values.push_back(msg.substr(Fpos, Lpos - Fpos));
		
		

		Fpos = msg.find_first_not_of(delimiter, Lpos);
		Lpos = msg.find_first_of(delimiter, Fpos);
		
		//ROS_INFO_STREAM("TEST : " << last_dump.size());
		//ROS_INFO_STREAM("VALUE[0] : " << values[0]);
		//ROS_INFO_STREAM("test :: " << Fpos << "&&" << Lpos);
	}


	/*
	Start=msg.find('<');
	Tp_end=msg.find("PE");
	Tt_end=msg.find("TE");
	Tk_end=msg.find("KE");
	Fx_end=msg.find("XE");
	Fy_end=msg.find("YE");
	Fz_end=msg.find("ZE");
	End=msg.find('>');
	if(Start!=string::npos && Tp_end !=string::npos)
	{values.push_back(msg.substr(Start, Tp_end - Start));}
	if(Tp_end !=string::npos && Tt_end !=string::npos)
	{values.push_back(msg.substr(Tp_end, Tt_end - Tp_end));}
	if(Tt_end !=string::npos && Tt_end !=string::npos)
        {values.push_back(msg.substr(Start, Tp_end - Start));}
	if(Tk_end !=string::npos && Fx_end !=string::npos)
        {values.push_back(msg.substr(Start, Tp_end - Start));}
	if(Fx_end !=string::npos && Fy_end !=string::npos)
        {values.push_back(msg.substr(Start, Tp_end - Start));}
	if(Fy_end !=string::npos && Fz_end !=string::npos)
        {values.push_back(msg.substr(Start, Tp_end - Start));}
	if(Fz_end !=string::npos && End !=string::npos)
        {values.push_back(msg.substr(Start, Tp_end - Start));}
	

	ROS_INFO_STREAM(" CHECK : "<< start << " | " << Tp_end);
	*/

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
	    string st= "";
	    bool start_flag=false;
	    while(i<str.size())
	    {
		    if((str[i] == '>')&& temp_2.empty())
		    {
			    temp_1=st;
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
			    if(str[i]=='>')
			    {
				    temp_2_on=false;
				    parseData(temp_2, last_dump ,dot);
				    ROS_INFO_STREAM(temp_2);
				    temp_2.clear();


			    }
		    }

		    st[i]=str[i];
		    i++;
	    }

}
