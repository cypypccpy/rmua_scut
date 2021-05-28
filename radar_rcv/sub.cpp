/*#include "ros/ros.h"
#include "radar/CarCoordinate.h"

void chatterCallback(const radar::CarCoordinate::ConstPtr& msg)//这里msg前面是自定义的那一套
{
  ROS_INFO("I heard: [%f] [%f]", msg->red2x,msg->red2y);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mysub");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("carcoordinate", 1000, chatterCallback);//新建一个sub，mymsg要一致，chatterCallback是回调函数名

  ros::spin();

  return 0;
}*/

#include <iostream>
#include "Socket/include/Socket.h"
#include <ros/ros.h>
#include "roborts_msgs/CarCoordinate.h"
using namespace std;

//RegParam param = RegParam();

int main(int argc, char *argv[])
{
  carposition cp;
  CLIENT serv;
  ros::init(argc, argv, "radar_rcv");
  ros::NodeHandle ros_nh_;
  ros::Publisher pub_ = ros_nh_.advertise<roborts_msgs::CarCoordinate>("carcoordinate", 1);
  roborts_msgs::CarCoordinate coord_;

  serv.clientinit("192.168.1.105", 9002);
  while (ros::ok())
  {
    serv.clientreceive();
    coord_.blue1x = static_cast<double>(serv.result.blue1.x);
    coord_.blue1y = static_cast<double>(serv.result.blue1.y);
    coord_.blue2x = static_cast<double>(serv.result.blue2.x);
    coord_.blue2y = static_cast<double>(serv.result.blue2.y);
    coord_.red1x = static_cast<double>(serv.result.red1.x);
    coord_.red1y = static_cast<double>(serv.result.red1.y);
    coord_.red2x = static_cast<double>(serv.result.red2.x);
    coord_.red2y = static_cast<double>(serv.result.red2.y);
    pub_.publish(coord_);

  }
}
