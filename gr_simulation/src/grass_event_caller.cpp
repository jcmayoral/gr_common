#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_client.hh>
#include "grasscutter.pb.h"

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo as a client
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Empty>("/test");///grassrow/event");
///grassrow::link_0:/event");
    //"/grassrow_0/event");
  std::cout << pub->GetTopic() << std::endl;
  // Wait for a subscriber to connect to this publisher
  pub->WaitForConnection();
  gr_simulation_msgs::msgs::GrassCutterRequest msg;

  // Set the velocity in the x-component
  //gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]), std::atof(_argv[2]), std::atof(_argv[3])));

  // Send the message
  for (int i=0; i<10; i++){
    pub->Publish(msg);
    sleep(3);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();

  return 1;
}
