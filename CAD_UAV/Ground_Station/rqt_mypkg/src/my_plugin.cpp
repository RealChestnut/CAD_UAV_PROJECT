/*
  Copyright 2018
*/

#include "my_plugin.h"
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>

double PI = 3.141592;
bool isArm = false;
bool isHover = false;
bool isLanding = false;
bool isApproaching = false;

namespace rqt_mypkg_cpp
{



MyPlugin::MyPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    setObjectName("C++PluginT");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);
////////////////////////////////////////////////////////////////////////////////////////
  ros::start();
  ros::NodeHandle n;

  QObject::connect(ui_.btn_Arm, SIGNAL(clicked(bool)), this, SLOT(Arm_Callback(bool))); // Arm Button (Toggle)
//  QObject::connect(ui_.btn_Kill, SIGNAL(toggled(bool)), this, SLOT(Kill_Callback(bool))); // Kill Button (Toggle)
  QObject::connect(ui_.btn_Hover, SIGNAL(clicked(bool)), this, SLOT(Hover_Callback(bool))); // Hover Button (Click)
  QObject::connect(ui_.btn_Land, SIGNAL(clicked(bool)), this, SLOT(Land_Callback(bool))); // Land Button (Normal)
  QObject::connect(ui_.btn_PosCtrl, SIGNAL(clicked(bool)), this, SLOT(PosCtrl_Callback(bool))); // Pos Ctrl Button (Normal)
  QObject::connect(ui_.btn_Dock, SIGNAL(toggled(bool)), this, SLOT(Dock_Callback(bool)));  // Docking Button (Normal)
  QObject::connect(ui_.btn_Appr, SIGNAL(toggled(bool)), this, SLOT(Approach_Callback(bool))); // Approach Button (Normal)
  QObject::connect(ui_.btn_Manipulator, SIGNAL(toggled(bool)), this, SLOT(Manipulator_Callback(bool))); // Manipulator button (Click)

  ArmClient = n.serviceClient<rqt_mypkg::ArmService>("/ArmService"); // Arm ServiceClient
  //KillClient = n.serviceClient<rqt_mypkg::KillService>("/KillService"); // Kill ServiceClient
  PosCtrlClient = n.serviceClient<rqt_mypkg::PosCtrlService>("/PosCtrlService"); // PosCtrl ServiceClient
  DockClient = n.serviceClient<rqt_mypkg::DockService>("/DockService");   // Docking ServiceClient
  HoverClient = n.serviceClient<rqt_mypkg::HoverService>("/HoverService"); //Landing, Hovering, Hovered ServiceClient
  ApproachClient = n.serviceClient<rqt_mypkg::ApproachService>("/ApproachService"); // Approach Mode Change ServiceClient
  ManipulatorClient = n.serviceClient<rqt_mypkg::ManipulatorService>("/ManipulatorService"); // Battery change with Manipulator ServiceClient
  //HoverServer = n.advertiseService("/FAC_HoverService", &MyPlugin::FAC_Hover_Callback, this); // Get state from Drone to GUI

  SwitchSubscriber = n.subscribe("message", 100, &MyPlugin::Switch_callback, this); // Switch Subscriber callback

}




void MyPlugin::shutdownPlugin()
{
    // unregister all publishers here
    publisher.shutdown();
}


void MyPlugin::Arm_Callback(bool val)
{
    rqt_mypkg::ArmService Service;
    if(!isArm)
    {
      Service.request.Arm_isChecked = true;
      
        if (ArmClient.call(Service)) 
        {
        isArm = true;
        ui_.btn_Arm->setText("Armed");
        ui_.btn_Arm->setStyleSheet("background-color:Green");
        }
        else
        {
          ui_.btn_Arm->setText("Arm Failed");
          ui_.btn_Arm->setStyleSheet("background-color:red");
        }

      ArmClient.call(Service);
      ros::spinOnce();
      return ;
    }
    
    if(!isHover && isArm)
    {
      isArm = false;
      Service.request.Arm_isChecked = false;
      ui_.btn_Arm->setText("Arm");
      ui_.btn_Arm->setStyleSheet("background-color:none");      
    
      ArmClient.call(Service);
      ros::spinOnce();
      return ;
    
    }
}


void MyPlugin::Hover_Callback(bool val)
{   
  if(isArm)
  {
      rqt_mypkg::HoverService Service;
      ui_.edi_Alti->setText("100");
      
      Service.request.isHover = true;
      Service.request.isHovering = true;
      Service.request.isLanding = false;

          if(HoverClient.call(Service))
          {
          isHover = true;
          ui_.btn_Hover->setText("Hovering");
          ui_.btn_Hover->setStyleSheet("background-color:Yellow");
          ui_.btn_Land->setText("Land");
          ui_.btn_Land->setStyleSheet("background-color:None");
          }
          else
          {
          ui_.btn_Hover->setText("Failed Hover");
          ui_.btn_Hover->setStyleSheet("background-color:Red");
          }



      HoverClient.call(Service);
      ros::spinOnce();
  }
}


void MyPlugin::Land_Callback(bool val)
{
  if(isHover)
  {
      rqt_mypkg::HoverService Service;
      ui_.edi_Alti->setText("0");
      Service.request.isHover = true;
      Service.request.isLanding = true;
      Service.request.isHovering = false;

          if(HoverClient.call(Service))
          {
          isHover = false;
          isArm = false;
          ui_.btn_Hover->setText("Hover");
          ui_.btn_Hover->setStyleSheet("background-color:None");
          ui_.btn_Arm->setText("Arm");
          ui_.btn_Arm->setStyleSheet("background-color:None");
          }
          else
          {
          ui_.btn_Land->setText("Failed Land");
          ui_.btn_Land->setStyleSheet("background-color:Red");
          }


      HoverClient.call(Service);
      ros::spinOnce();
  }
}


void MyPlugin::PosCtrl_Callback(bool val)
{
  if(isHover)
  {
  rqt_mypkg::PosCtrlService Service;


  Service.request.desired_X = ui_.edi_X->text().toDouble() / 100;
  Service.request.desired_Y = ui_.edi_Y->text().toDouble() / 100;
  Service.request.desired_Yaw = ui_.edi_Yaw->text().toDouble() * PI / 180.0; 
  Service.request.desired_Alti = ui_.edi_Alti->text().toDouble() / 100;


      if(PosCtrlClient.call(Service))
      {
      ui_.btn_PosCtrl->setText("Position Ctrl");
      ui_.btn_PosCtrl->setStyleSheet("background-color:none");
      }
      else
      {
      ui_.btn_PosCtrl->setText("Failed Ctrl");
      ui_.btn_PosCtrl->setStyleSheet("background-color:Red");
      }

  PosCtrlClient.call(Service);
  ros::spinOnce();
  }
}

bool is_Appr=false;
bool is_Dock=false;
bool is_Mani=false;



//------------------------Docking-----------------------//



void MyPlugin::Dock_Callback(bool val)
{
    rqt_mypkg::DockService Service;
      

          if(ui_.btn_Dock->isChecked()) //ui에서 정의된 btn_Dock 맴버변수의 isChecked()<bool data>
          {
          Service.request.Dock_Do = true;
          is_Dock = true;
  
          ui_.btn_Dock->setText("Docking");
          ui_.btn_Dock->setStyleSheet("background-color:Green");
          }
          else
          { 
          Service.request.Dock_Do=false;
          is_Dock=false;
          ui_.btn_Dock->setText("Dock");
          ui_.btn_Dock->setStyleSheet("background-color:None");
          }

      if(!is_Dock && is_Mani){
      is_Mani=false;
      ui_.btn_Manipulator->setStyleSheet("background-color:None");
      ui_.btn_Manipulator->toggle();}

      if(!is_Dock && is_Appr){
      is_Appr=false;
      ui_.btn_Appr->setText("Approach");
      ui_.btn_Appr->setStyleSheet("background-color:none");
      ui_.btn_Appr->toggle();}

      if(is_Dock && is_Appr){
      is_Appr=false;
      ui_.btn_Appr->setText("Approach");
      ui_.btn_Appr->setStyleSheet("background-color:none");
      ui_.btn_Appr->toggle();}

      

      DockClient.call(Service);
      ros::spinOnce();
}

void MyPlugin::Approach_Callback(bool val)
{
  
    rqt_mypkg::ApproachService Service;
  if(!is_Dock){
    if(ui_.btn_Appr->isChecked())
    {
      is_Appr=true;
      Service.request.Approach_isChecked = is_Appr;
      Service.request.isApproach = is_Appr;

      ui_.btn_Appr->setText("Approaching");
      ui_.btn_Appr->setStyleSheet("background-color:Green");     
      
    }
    else 
    {
      is_Appr=false;
      Service.request.Approach_isChecked = is_Appr;
      Service.request.isApproach = is_Appr;
      ui_.btn_Appr->setText("Approach");
      ui_.btn_Appr->setStyleSheet("background-color:none");
    }
  }
 
    ApproachClient.call(Service);
    ros::spinOnce();

}

//-----------------------Battery Change with Manipulator---------------------------//
void MyPlugin::Manipulator_Callback(bool val)
{
    rqt_mypkg::ManipulatorService Service;
    //Service.request.Manipul = true;
  if(is_Dock){
          if(ui_.btn_Manipulator->isChecked()) //ui에서 정의된 btn_Dock 맴버변수의 isChecked()<bool data>
          {
          is_Mani =true;
          Service.request.Manipul = is_Mani;
          ui_.btn_Manipulator->setText("Manipulating");
          ui_.btn_Manipulator->setStyleSheet("background-color:Green");
          }
          else
          { 
          is_Mani =false;  
          Service.request.Manipul=is_Mani;
          ui_.btn_Manipulator->setText("Manipulate");
          ui_.btn_Manipulator->setStyleSheet("background-color:None");
          }
    }

    

    ManipulatorClient.call(Service);
    ros::spinOnce();
    return ;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

void MyPlugin::Switch_callback(const std_msgs::UInt16 &isdock)
{

  if(isdock.data == 0) ui_.lbl_Dock->setText("Docking");
  else ui_.lbl_Dock->setText("UnDocking");

}












void MyPlugin::run() {
  ros::Rate loop_rate(10); //일단주파수는 10정도로 해놓자
	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();
	}
}


//------------------------------Drone to GUI callback. --------------------//
/*
bool MyPlugin::FAC_Hover_Callback(rqt_mypkg::FAC_HoverService::Request &req, 
                                  rqt_mypkg::FAC_HoverService::Response &res){  //ASDF
isHover = req.FAC_isHover;
isHovering = req.FAC_isHovering;
isLanding = req.FAC_isLanding;

  if(!req.FAC_isHovering && req.FAC_isHover)  // 드론에서 Hover 했다고 신호가 옴
  {
    ui_.btn_Hover->setText("Hovered");
    ui_.btn_Hover->setStyleSheet("background-color:Green");
    ui_.btn_PosCtrl->setStyleSheet("background-color:Green");
  }

  if(!req.FAC_isLanding && !req.FAC_isHover)  //드론에서 Land 했다고 신호가 옴
  {
    ui_.btn_Land->setText("Land");
    ui_.btn_Land->setStyleSheet("background-color:None");

    isArm = false;
    isHovering = false;
    ui_.btn_Arm->setText("Arm");
    ui_.btn_Arm->setStyleSheet("background-color:None");
  }
return true;
}

//------------------------------스페이스바를 누르면 Kill 상태로 보낼 수 있게 만들려고 했는데 일단 실패함 --------------------//
void MyPlugin::keyPressEvent(QKeyEvent *event)
{
  if(event->key() == Qt::Key_Space)
  { 
  ui_.btn_Kill->isChecked();
  }
}


//------------------------------아마도 군집 조종하기 위해 사용할 것 같음.... --------------------//
void MyPlugin::gotogether(bool val)
{
  secondturtles::turtlesrv server;

    if(ui_.chk_t1->isChecked())
    {
    server.request.a = ui_.lbl_a->text().toDouble();
    server.request.b = ui_.lbl_b->text().toDouble(); 
    servicecaller.call(server);   
    ros::spinOnce();    
    }

    if(ui_.chk_t2->isChecked())
    {
    server.request.c = ui_.lbl_c->text().toDouble();
    server.request.d = ui_.lbl_d->text().toDouble();    
    secondservicecaller.call(server);   
    ros::spinOnce();
    }

}


*/

}  // namespace rqt_mypkg_cpp









// #define PLUGINLIB_DECLARE_CLASS(pkg, class_name, class_type, base_class_type)
PLUGINLIB_EXPORT_CLASS(rqt_mypkg_cpp::MyPlugin, rqt_gui_cpp::Plugin)


