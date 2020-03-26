#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

//Based originally from the rosbag c++ implementation
//https://github.com/ros/ros_comm/tree/noetic-devel/tools/rosbag/src
//used as tutorial to get into the definition of a message.
namespace gr_safety_gridmap{
    template<class T>
        class LayerSubscriber{
            public: 
                void updateRateTopicTime(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event){
                    boost::shared_ptr<topic_tools::ShapeShifter const> const &ssmsg = msg_event.getConstMessage();
                    std::string def = ssmsg->getMessageDefinition();
                    size_t length = ros::serialization::serializationLength(*ssmsg);

                    // Check the message definition.
                    std::istringstream f(def);
                    std::string s;
                    bool flag = false;
                    while(std::getline(f, s, '\n')) {
                        if (!s.empty() && s.find("#") != 0) {
                            // Does not start with #, is not a comment.
                            if (s.find("Header ") == 0) {
                                flag = true;
                            }
                            break;
                        }
                    }

                    // If the header is not the first element in the message according to the definition, throw an error.
                    if (!flag) {
                        std::cout << std::endl << "WARNING: Rate control topic is bad, header is not first. MSG may be malformed." << std::endl;
                        return;
                    }

                    std::vector<uint8_t> buffer(length);
                    ros::serialization::OStream ostream(&buffer[0], length);
                    ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, *ssmsg);
                    
                    // Assuming that the header is the first several bytes of the message.
                    //uint32_t header_sequence_id   = buffer[0] | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2] << 16 | (uint32_t)buffer[3] << 24;
                    int32_t header_timestamp_sec  = buffer[4] | (uint32_t)buffer[5] << 8 | (uint32_t)buffer[6] << 16 | (uint32_t)buffer[7] << 24;
                    int32_t header_timestamp_nsec = buffer[8] | (uint32_t)buffer[9] << 8 | (uint32_t)buffer[10] << 16 | (uint32_t)buffer[11] << 24;

                    //last_rate_control_ = ros::Time(header_timestamp_sec, header_timestamp_nsec);
            };

                LayerSubscriber(): sub_(boost::make_shared<ros::Subscriber>()){
                    ros::SubscribeOptions ops;
                    ops.topic ="/test";//options_.rate_control_topic;
                    ops.queue_size = 1;
                    ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
                    ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
                    ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
                         const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
                                    boost::bind(&LayerSubscriber::updateRateTopicTime, this, _1));

                    rsub_ = nh_.subscribe(ops);

        std::cout << " done." << std::endl;
                }
            private:
                boost::shared_ptr<ros::Subscriber> sub_;
                ros::NodeHandle nh_;  
                ros::Subscriber rsub_;  
    
    };
}