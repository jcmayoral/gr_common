#include <gazebo/msgs/msgs.hh>


class MotionModel{

    void UpdateWorldPose(ignition::Math::Vector3d npose);
    void UpdateRelative3DVelocity(ignition::Math::Vector3d npose)
  // Create a a vector3 message
  gazebo::msgs::Vector3d msg;

  // Set the velocity in the x-component
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]), std::atof(_argv[2]), std::atof(_argv[3])));

    private:
        double accx;
        double accx;
        double posx;
        double posy;
}