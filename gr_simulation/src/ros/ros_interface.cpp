#include<ros/ros_interface.h>
using namespace gr_simulation;


ROSInterface::ROSInterface(std::string action_id): tfBuffer(ros::Duration(5)),
                                tf2_listener(tfBuffer){
    ac_client_ = boost::make_shared<actionlib::SimpleActionClient<gr_action_msgs::SimMotionPlannerAction>>(action_id,true);
    ROS_INFO("WAIT FOR SERVER");
    ac_client_->waitForServer();
    ROS_INFO("SERVER FOUND");
    dyn_server_cb_ = boost::bind(&ROSInterface::dyn_reconfigureCB, this, _1, _2);
    dyn_server_.setCallback(dyn_server_cb_);
    ros::spin();
}

void ROSInterface::dyn_reconfigureCB(gr_simulation::PersonMotionConfig &config, uint32_t level){
    std::cout << "BEFORE " << std::endl;
    gr_action_msgs::SimMotionPlannerGoal goal;

    geometry_msgs::TransformStamped base_link_to_odom;
    geometry_msgs::PoseStamped p;

    //orientation of current odometry to map
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "odom";
    p.pose.position.x = config.start_offset_x;
    p.pose.position.y = config.start_offset_y;
    p.pose.orientation.w = 1.0;
    tf2::Quaternion myquaternion;
    myquaternion.setRPY(0,0,config.start_yaw);
    //tf2::convert(p.pose.orientation , myquaternion);
    p.pose.orientation = tf2::toMsg(myquaternion);
    //base_link_to_odom = tfBuffer.lookupTransform("odom", "base_link", ros::Time::now(), ros::Duration(1.0) );
    //tf2::doTransform(p, p, base_link_to_odom);

    //tf2::Quaternion myquaternion;
    //myquaternion.setRPY(0,0,config.start_yaw);
    //tf2::convert(p.pose.orientation , myquaternion);
    p.pose.orientation = tf2::toMsg(myquaternion);

    ROS_INFO_STREAM("START POSE " <<p);

    //ROS_INFO_STREAM(p);
    //ROS_INFO("Calculate goal");
    geometry_msgs::PoseStamped p2;
    //orientation of current odometry to map
    p2.header.stamp = ros::Time::now();
    p2.header.frame_id = "odom";
    p2.pose.position.x = config.start_offset_x + config.distanceToMove*cos(config.start_yaw);
    p2.pose.position.y = config.start_offset_y + config.distanceToMove*sin(config.start_yaw);;
    //p2.pose.orientation.w = 1.0;
    myquaternion.setRPY(0,0,config.start_yaw);
    //tf2::convert(p.pose.orientation , myquaternion);
    p2.pose.orientation = tf2::toMsg(myquaternion);

    //tf2::doTransform(p2, p2, base_link_to_odom);

    goal.original_distance = sqrt(pow(p.pose.position.x - p2.pose.position.x,2)+pow(p.pose.position.y - p2.pose.position.y,2))*1.2;
    ROS_INFO_STREAM("original_distance  "<<goal.original_distance);

    goal.startpose = p;
    goal.goalPose = p2;
    goal.dist2collision = config.collision_distance;

    goal.linearspeed = config.linearvel * cos(config.start_yaw);
    goal.linearspeedy = config.linearvel * sin(config.start_yaw);
    ROS_INFO_STREAM("vel "<< goal.linearspeed);
    goal.is_motion = true;
    goal.setstart = true;
    ac_client_->sendGoal(goal);
    bool res = ac_client_->waitForResult(ros::Duration(2.0));
    std::cout << "Available again " << res << std::endl;
}