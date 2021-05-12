#include "include/Socket.h"

int SERVICE::serverinit(const char *SIP, uint16_t SPORT)
{

    memset(&serv_addr, 0, sizeof(serv_addr)); //初始化
    memset(&clit_addr, 0, sizeof(clit_addr)); //初始化

    if ((servfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) //创建套接字
    {
        cout << "creat socket failed : " << strerror(errno) << endl; //如果出错则打印错误
        return 0;
    }

    //给服务端的地址结构体赋值
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(SPORT);          //将主机上的小端字节序转换为网络传输的大端字节序（如果主机本身就是大端字节序就不用转换了）
    serv_addr.sin_addr.s_addr = inet_addr(SIP); //将字符串形式的ip地址转换为点分十进制格式的ip地址

    //绑定地址信息到监听套接字上，第二个参数强转是因为形参类型为sockaddr ，而实参类型是sockaddr_in 型的
    if (bind(servfd, (sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
    {
        cout << "bind failed : " << strerror(errno) << endl;
        return 0;
    }

    //将servfd套接字置为监听状态
    if (listen(servfd, 1024) == -1)
    {
        cout << "listen failed : " << strerror(errno) << endl;
        return 0;
    }

    cout << "Init Success ! " << endl;
    cout << "ip : " << inet_ntoa(serv_addr.sin_addr) << "  port : " << ntohs(serv_addr.sin_port) << endl;
    cout << "Waiting for connecting ... " << endl;

    socklen_t clit_size = 0; //用于accept函数中保存客户端的地址结构体大小

    //accept成功后，clitfd则指向了这条服务端与客户端成功连接的”通路“
    if ((clitfd = accept(servfd, (sockaddr *)&clit_addr, &clit_size)) == -1)
    {
        cout << "accept failed : " << strerror(errno) << endl;
        return 0;
    }

    cout << "Client access : " << inet_ntoa(clit_addr.sin_addr) << "  " << ntohs(clit_addr.sin_port) << endl;
}

void SERVICE::serversend(carposition CP)
{
    write(clitfd, (char *)&CP, (sizeof(CP) + 2)); //发回客户端
}

int CLIENT::clientinit(const char *CIP, uint16_t CPORT)
{
    memset(&serv_addr, 0, sizeof(serv_addr));

    if ((clitfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) //创建套接字
    {
        cout << "creat socket failed : " << strerror(errno) << endl;
        return 0;
    }
    //将目的服务端的地址信息赋值给地址结构体
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(CPORT);
    serv_addr.sin_addr.s_addr = inet_addr(CIP);

    cout << "try to connect ... " << endl;

    //通过套接字发起连接请求，成功后clitfd套接字则表示此次成功的连接
    if (connect(clitfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
    {
        cout << "connet failed : " << strerror(errno) << endl;
        return 0;
    }

    cout << "connect success !" << endl;
}

void CLIENT::clientreceive()
{

    int rdcnt = read(clitfd, (char *)&result, (sizeof(result) + 2));
    if (rdcnt == -1)
    {
        perror(NULL);
        return;
    }

    if (rdcnt)
    {
        cout << "(Client)recv : " << result.blue1 << endl;
    }
    else
    {
        cout << "Server has closed ! " << endl;
        cout << "Client will close..." << endl;
        return;
    }
}