#include<ros/ros_interface.h>
using namespace gr_simulation;


ROSInterface::ROSInterface(): tfBuffer(ros::Duration(5)),
                                tf2_listener(tfBuffer){
    ac_client_ = boost::make_shared<actionlib::SimpleActionClient<gr_action_msgs::SimMotionPlannerAction>>("SimMotionPlanner/my_person",true);
    ROS_INFO("WAIT FOR SERVER");
    ac_client_->waitForServer();
    ROS_INFO("SERVER FOUND");
    dyn_server_cb_ = boost::bind(&ROSInterface::dyn_reconfigureCB, this, _1, _2);
    dyn_server_.setCallback(dyn_server_cb_);
    ros::spin();
}


ROSInterface::~ROSInterface(){
    
}


void ROSInterface::dyn_reconfigureCB(gr_simulation::PersonMotionConfig &config, uint32_t level){
    std::cout << "BEFORE " << std::endl;
    gr_action_msgs::SimMotionPlannerGoal goal;

    geometry_msgs::TransformStamped base_link_to_odom;
    geometry_msgs::PoseStamped p;

    //orientation of current odometry to map
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "base_link";
    p.pose.position.x = config.start_offset_x;
    p.pose.position.y = config.start_offset_y;
    p.pose.orientation.w = 1.0;
    base_link_to_odom = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(1.0) );
    tf2::doTransform(p, p, base_link_to_odom);

    tf2::Quaternion myquaternion;
    myquaternion.setRPY(0,0,config.start_yaw);
    //tf2::convert(p.pose.orientation , myquaternion);
    p.pose.orientation = tf2::toMsg(myquaternion);
    ROS_INFO_STREAM(p);


    goal.startpose = p;
    goal.linearspeed = config.linearvel * cos(config.start_yaw);
    goal.linearspeedy = config.linearvel * sin(config.start_yaw);
    ROS_INFO_STREAM("vel "<< goal.linearspeed);
    goal.is_motion = true;
    goal.setstart = true;
    ac_client_->sendGoal(goal);
    bool res = ac_client_->waitForResult(ros::Duration(2.0));
    std::cout << "Available again " << res << std::endl;
}