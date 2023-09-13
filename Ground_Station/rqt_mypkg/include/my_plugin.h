/*
  Copyright 2018
*/
#ifndef RQT_MYPKG_CPP_MY_PLUGIN_H
#define RQT_MYPKG_CPP_MY_PLUGIN_H

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <iostream>
#include <turtlesim/Pose.h>
#include "rqt_mypkg/ui_my_plugin.h"
#include "rqt_mypkg/ArmService.h"
#include "rqt_mypkg/KillService.h"
#include "rqt_mypkg/PosCtrlService.h"
#include "rqt_mypkg/DockService.h"
#include "rqt_mypkg/HoverService.h"
#include "rqt_mypkg/TiltService.h"
#include "rqt_mypkg/ManipulatorService.h"
#include <std_msgs/UInt16.h>

//#include "rqt_mypkg/FAC_HoverService.h"
//#include <QKeyEvent> 
namespace rqt_mypkg_cpp
{

class MyPlugin : public rqt_gui_cpp::Plugin
{    
    Q_OBJECT
public:
    MyPlugin();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();

private slots:

    void Arm_Callback(bool val);    
    void Kill_Callback(bool val);
    void Tilt_Callback(bool val);  
    void Manipulator_Callback(bool val);  
    void Hover_Callback(bool val);
    void Land_Callback(bool val);
    void PosCtrl_Callback(bool val);
    void Dock_Callback(bool val);
    void Switch_callback(const std_msgs::UInt16 &str_msg);
    void run();

 //   bool FAC_Hover_Callback(rqt_mypkg::FAC_HoverService::Request &req, rqt_mypkg::FAC_HoverService::Response &res);
 //   void keyPressEvent(QKeyEvent *event); 
    
    private:
    Ui::MyPluginWidget ui_;
    QWidget* widget_;
    ros::Publisher publisher;         //이건 GUI Shutdown 용이라서 건들면 안 됨.
    ros::ServiceClient ArmClient;
    ros::ServiceClient KillClient;
    ros::ServiceClient PosCtrlClient;
    ros::ServiceClient DockClient;   
    ros::ServiceClient HoverClient;  
    ros::ServiceClient TiltClient;
    ros::ServiceClient ManipulatorClient;
    ros::Subscriber SwitchSubscriber;
    //    ros::ServiceServer HoverServer;
};

}  // namespace rqt_mypkg_cpp

#endif  // RQT_MYPKG_CPP_MY_PLUGIN_H
