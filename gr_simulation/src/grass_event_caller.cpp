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
  node->Init();
  std::cout << ">" << std::endl;
  std::string topic = _argv[1];
  gr_simulation_msgs::msgs::GrassCutterRequest request;
  gazebo::transport::PublisherPtr pub = 
      node->Advertise< gr_simulation_msgs::msgs::GrassCutterRequest>("/"+topic);///grassrow/event"); 
  std::cout << "waiting" << pub->GetTopic() << pub->GetMsgType()<< std::endl; 
  request.set_row(1);
  request.set_column(1);
  //argument to set if cut or not
  request.set_cut(std::atoi(_argv[2]));

  pub->WaitForConnection();
  while (!pub->HasConnections());

  sleep(1);
  pub->Publish(request);
  sleep(1);

  gazebo::client::shutdown();
  return 1;
}
