#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_client.hh>
#include "grasscutter.pb.h"

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  gazebo::client::setup(_argc, _argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init("grasscutter");
  std::cout << ">" << std::endl;
  
  gr_simulation_msgs::msgs::GrassCutterRequest request;
  gazebo::transport::PublisherPtr pub = 
      node->Advertise< gr_simulation_msgs::msgs::GrassCutterRequest>("/gazebo/grasscutter/test");///grassrow/event"); 
  std::cout << "waiting"; 
  pub->WaitForConnection();
  sleep(1);
  pub->Publish(request);
  sleep(1);

  gazebo::client::shutdown();
  return 1;
}
