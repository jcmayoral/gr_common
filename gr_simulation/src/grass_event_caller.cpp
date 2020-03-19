#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_client.hh>
#include "grasscutter.pb.h"

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  gr_simulation_msgs::msgs::GrassCutterRequest request;
  // Load gazebo as a client
  std::cout << ">" << std::endl;
  gazebo::transport::init();
  gazebo::transport::run();
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init("default");
  gazebo::transport::PublisherPtr pub = node->Advertise< gr_simulation_msgs::msgs::GrassCutterRequest>("/test");///grassrow/event"); 
  std::cout << "waiting"; 
  pub->WaitForConnection();
  sleep(1);
  pub->Publish(request);
  sleep(1);

  gazebo::transport::fini();
  return 1;
}
