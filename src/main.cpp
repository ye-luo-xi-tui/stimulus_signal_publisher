#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "stimulus_signal_publisher/signal_producer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stimulus_signal_publisher");
  ros::NodeHandle nh("~");

  std::string topic_name;
  nh.getParam("command_topic", topic_name);
  ros::Publisher command_publisher = nh.advertise<std_msgs::Float64>(topic_name, 10);

  std::string signal;
  nh.getParam("signal", signal);

  stimulus_signal_publisher::SignalProducerBase* signal_producer;
  if (signal.find("sin_wave") != std::string::npos)
  {
    double magnitude = 1.0;
    nh.getParam("magnitude", magnitude);
    XmlRpc::XmlRpcValue frequency_points_config;
    nh.getParam("frequency_points", frequency_points_config);
    signal_producer = new stimulus_signal_publisher::SinWave(magnitude, frequency_points_config, nh);
  }
  else if (signal.find("constant_velocity") != std::string::npos)
  {
    double start_point = 0;
    nh.getParam("start_point", start_point);
    double end_point = 0;
    nh.getParam("end_point", end_point);
    XmlRpc::XmlRpcValue velocity_config;
    nh.getParam("velocity", velocity_config);
    signal_producer = new stimulus_signal_publisher::ConstantVelocity(start_point, end_point, velocity_config, nh);
  }
  else
  {
    return 1;
  }
  ros::Rate loop_rate(500);
  while (ros::ok())
  {
    ros::spinOnce();
    std_msgs::Float64 command;
    command.data = signal_producer->output();
    command_publisher.publish(command);
    loop_rate.sleep();
  }
  delete signal_producer;
  return 0;
}
