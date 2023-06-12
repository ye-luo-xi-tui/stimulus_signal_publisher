#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "stimulus_signal_publisher/SetPublisherStatus.h"

int status = stimulus_signal_publisher::SetPublisherStatus::Request::RESET;
ros::Time start_time;

bool setPublisherStatus(stimulus_signal_publisher::SetPublisherStatus::Request& req,stimulus_signal_publisher::SetPublisherStatus::Response& res)
{
    if(status == stimulus_signal_publisher::SetPublisherStatus::Request::RESET && req.status == stimulus_signal_publisher::SetPublisherStatus::Request::START)
        start_time = ros::Time::now();
    status = req.status;
    res.is_success = true;
    ROS_INFO("Switch to status: %d",status);
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"stimulus_signal_publisher");
    ros::NodeHandle nh("~");

    ros::ServiceServer service = nh.advertiseService("set_publisher_status",setPublisherStatus);
    ROS_INFO("Ready to set status.");

    std::string topic_name;
    nh.getParam("command_topic",topic_name);
    ros::Publisher command_publisher = nh.advertise<std_msgs::Float64>(topic_name,10);

    std::vector<double> frequency_points;
    XmlRpc::XmlRpcValue frequency_points_config;
    nh.getParam("frequency_points",frequency_points_config);
    ROS_ASSERT(frequency_points_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
    frequency_points.reserve(frequency_points_config.size());
    for(int i = 0; i < frequency_points_config.size(); i++)
        frequency_points.push_back(frequency_points_config[i]);
    auto current_point_it = frequency_points.begin();
    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        ros::spinOnce();
        std_msgs::Float64 command;
        if(status == stimulus_signal_publisher::SetPublisherStatusRequest::START)
        {
          if((ros::Time::now() - start_time).toSec() >= 20 * 1 / *current_point_it)
          {
            if((current_point_it + 1) == frequency_points.end())
            {
              status =
                  stimulus_signal_publisher::SetPublisherStatusRequest::RESET;
              current_point_it = frequency_points.begin();
            }
            else
            {
              start_time = start_time + ros::Duration(20 * 1 / *current_point_it);
              current_point_it++;
            }
          }
          if(status == stimulus_signal_publisher::SetPublisherStatusRequest::START)
            command.data = std::sin(2 * M_PI * *current_point_it * (ros::Time::now() - start_time).toSec());
        }
        if(status == stimulus_signal_publisher::SetPublisherStatusRequest::RESET)
          command.data = 0.;
        command_publisher.publish(command);
        loop_rate.sleep();
    }
    return 0;
}