#include <cad_main.hpp>

cad_agent_control_loop::cad_agent_control_loop()
: node_handle_(""),
  priv_node_handle_("~")
  {
    /************************************************************
    ** Initialize variables
    ************************************************************/

    initParameter();
    initSubscriber();    
    initPublisher();

    

  }