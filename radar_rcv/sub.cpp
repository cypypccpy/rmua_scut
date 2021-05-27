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
//#include "radar/CarCoordinate.h"
using namespace std;

//RegParam param = RegParam();

int main(int argc, char *argv[])
{
  carposition cp;
  CLIENT serv;
  serv.clientinit("192.168.1.103", 8888);
  while (1)
  {
    serv.clientreceive();
  }
}
