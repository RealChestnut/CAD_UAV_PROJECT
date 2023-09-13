#include "cad_norm_version_class.cpp"

void publisherSET();

int main(int argc, char **argv)
{

  //initialize ros node//
  ros::init(argc, argv, "cad_uav");
  cad_Loop CAD_Loop;
  

  ros::Rate loop_rate(200);
  
  while (ros::ok())
  {

    //end =std::chrono::high_resolution_clock::now();
    //delta_t=end-start;
    //CAD_Loop.dt.data=delta_t.count();
    //start=std::chrono::high_resolution_clock::now();
    CAD_Loop.Clock();
    CAD_Loop.attitude_controller();
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



    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
