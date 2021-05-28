#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <errno.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
struct carposition
{
    Point2d blue1 = Point(0.0, 0.0);
    Point2d blue2 = Point(0.0, 0.0);
    Point2d red1 = Point(0.0, 0.0);
    Point2d red2 = Point(0.0, 0.0);
};

class SERVICE
{
public:
    ~SERVICE()
    {
        close(servfd); //关闭套接字
        close(clitfd);
    };
    int serverinit(const char *SIP, uint16_t SPORT);

    void serversend(carposition CP);
    int servfd, clitfd;                      //创建两个文件描述符，servfd为监听套接字，clitfd用于数据传输
    struct sockaddr_in serv_addr, clit_addr; //创建地址结构体，分别用来存放服务端和客户端的地址信息
};

class CLIENT
{
public:
    ~CLIENT()
    {
        close(clitfd);
    };
    int clientinit(const char *CIP, uint16_t CPORT);
    void clientreceive();
    int clitfd;                   //文件描述符
    struct sockaddr_in serv_addr; //目的服务端地址结构体
    carposition result;
};
