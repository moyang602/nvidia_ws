#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "map"
#include <math.h>

// ===========================================
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <ifaddrs.h>
// =============== 网络变量 ====================
#define MAX_LENGTH 254

int port = 0;
std::string hostIP;
struct sockaddr_in addrCli;
struct sockaddr_in addrSer;
int sockCli;
socklen_t addrlen;
char sendbuf[MAX_LENGTH];

// ================ 常量 ======================
# define DEADZONE 20
# define rDEADZONE 20

# define DEG2RAD 3.1415926/180.0
# define RAD2DEG 180.0/3.1415926

// ================ 串口变量 ===================
serial::Serial ser; //声明串口对象

std::string param_port_path_;
int param_baudrate_;


int param_loop_rate_; // 循环周期

// ================= 接收变量 ==================
#define CHANNEL 11
int i = 0;
int j = 0;
uint8_t sum = 0;
int rec_right[CHANNEL] = {0};
int lastrec[CHANNEL] = {0};

double beta_cmd;
double rightarmbeta;
double leftarmbeta;

std::vector<double> recf[CHANNEL];  // 滤波器通道

double fillter(double rec, int index, int times)
{
    double sum = 0.0;
    int i;
    while (recf[index].size() <= times)
    {
        recf[index].push_back(rec);
    }

    while (recf[index].size() > times)
    {
        recf[index].erase(recf[index].begin());

        for (i = 0, sum = 0; i< recf[index].size(); i++)
        {
            sum += recf[index].at(i);
        }

        sum /= (double)(recf[index].size());
    }

    return sum;
}

void UDP_send(char *ch)
{
    sendto(sockCli, ch, strlen(ch)+1, 0, (struct sockaddr*)&addrSer, addrlen);
    ch[strlen(ch)] == '\0';
    ROS_INFO("%s",ch);
}

double map(double in, double down, double up, double downlim, double uplim)
{
    double inup = (down> up? down:up);
    double indown = (down> up? up: down);
    if (in > inup)
        in = inup;
    if (in < indown)
        in = indown;

    if (down == up){
        return downlim;
    }
    else{
        return (in - down) /(up - down) * (uplim - downlim) + downlim;
    }  
}

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "arm_manipulator"); 

    //声明节点句柄 
    ros::NodeHandle nh; 

    uint8_t rec[2000] = {'\0'}; 
    int intool = 0;
    double armx;
    double army;
    double armz;
    double armRx;
    double armRy;
    double armRz;
    double carx;
    double carz;

    // 发布各轴速度
    double speedx = 0;
    double speedy = 0;
    double speedz = 0;
    double speedRx = 0;
    double speedRy = 0;
    double speedRz = 0;

    // 单次使能标志位
    int left_once = 0;
    int right_once = 0;
    int first_lock = 1;

    // 控制通道
    double chooseCH;
    double xCH;
    double yCH;
    double zCH;
    double rxCH;
    double ryCH;
    double rzCH;
    double betaCH;
    double frameCH;
    double enableCH;

    // 取得launch中的参数
	nh.param<std::string>("arm_manipulator/port", param_port_path_, "/dev/remote_USB");
	nh.param<int>("arm_manipulator/baudrate", param_baudrate_, 9600);
	nh.param<int>("arm_manipulator/loop_rate", param_loop_rate_, 20);
	nh.param<double>("arm_manipulator/armx", armx, 2);
	nh.param<double>("arm_manipulator/army", army, 2);
	nh.param<double>("arm_manipulator/armz", armz, 2);
	nh.param<double>("arm_manipulator/armRx", armRx, 3);
	nh.param<double>("arm_manipulator/armRy", armRy, 3);
	nh.param<double>("arm_manipulator/armRz", armRz, 3);
    nh.param<double>("arm_manipulator/rightarmbeta", rightarmbeta, 1.0);
    nh.param<double>("arm_manipulator/leftarmbeta", leftarmbeta, 1.0);

    nh.param<int>("robot_port",port, 0);
	nh.param<std::string>("robot_ip",hostIP, "127.0.0.1");


    sockCli = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockCli == -1)
    {
        perror("socket");
        exit(1);
    }

    addrSer.sin_family = AF_INET;
    addrSer.sin_port = htons(port);
    addrSer.sin_addr.s_addr = inet_addr(hostIP.c_str());
    
    addrlen = sizeof(struct sockaddr);

    ROS_INFO("udp init ok,IP:%s,port %d", hostIP.c_str(), port);

    // 打开串口
    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort(param_port_path_); 
        ser.setBaudrate(param_baudrate_); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port"<<param_port_path_<<" initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    //指定循环的频率 
    ros::Rate loop_rate(param_loop_rate_); 
    while(ros::ok()) 
    { 
        // while(ser.available() < 10);
        if(ser.available()){
            
            ser.read(rec,1);
            // ROS_INFO("%d, %x", ser.available(), rec[0]);

            if (rec[0] == 0xaa)
            {
                ser.read(rec+1, 2 * 9);
                sum = 0;
                for(j = 0; j < 8; ++j){
                    rec_right[j] = ( rec[2*j+1] <<8 ) + rec[2*j+2];
                    sum += rec[2*j+1];
                    sum += rec[2*j+2];
                }
                rec_right[8] = (rec[17] >> 4) & 0x01;
                rec_right[9] = (rec[17] >> 3) & 0x01;
                rec_right[10] = (rec[17] & 0x07);
                sum += rec[17];
                sum += 0xaa;
                // ser.flushInput(); 

                // ROS_INFO("sum=%x;rec[18]=%x\n",sum,rec[18]);

                // 校验通过
                if (rec[18] == sum){

                    // 通道处理
                    for (i = 0; i< 8; i++)
                    {
                        rec_right[i] -= 512;
                    }
                    
                    for (i= 0; i < 8 ;i++){
                        if(i == 3){      // 需要滤波的通道（绝对通道）
                            if(rec_right[i] > 500) rec_right[i] = 500;
                            if(rec_right[i] < -500) rec_right[i] = -500;
                            
                            // 中值滤波
                            rec_right[i] = (int)(fillter((double)rec_right[i], i, 20));
                            if (abs(rec_right[i] - lastrec[i]) < 2 )
                            {
                                rec_right[i] = lastrec[i];
                            }
                            lastrec[i] = rec_right[i];

                            // 死区补偿
                            if (abs(rec_right[i])<rDEADZONE) rec_right[i] = 0;
                            else{
                                if (rec_right[i] > 0) rec_right[i] -= rDEADZONE;
                                if (rec_right[i] < 0) rec_right[i] += rDEADZONE;
                            }
                        }
                        else{
                            if(rec_right[i] > 500) rec_right[i] = 500;
                            if(rec_right[i] < -500) rec_right[i] = -500;

                            // 死区补偿
                            if (abs(rec_right[i])<DEADZONE) rec_right[i] = 0;
                            else{
                                if (rec_right[i] > 0) rec_right[i] -= DEADZONE;
                                if (rec_right[i] < 0) rec_right[i] += DEADZONE;
                            }
                        }
                    }

                    // ROS_INFO("1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d 8:%d 9:%d 10:%d 11:%d  ",rec_right[0],rec_right[1],rec_right[2],rec_right[3],rec_right[4],rec_right[5],rec_right[6],rec_right[7],rec_right[8],rec_right[9],rec_right[10]);


                    // 得到遥控器的通道数据，进行通道映射
                    
                    // 进行通道映射
                    chooseCH = rec_right[10];
                    enableCH = rec_right[8];
                    frameCH = rec_right[9];
                    xCH = map(rec_right[5], -480, 480, -armx, armx);
                    yCH = map(rec_right[4], -480, 480, -army, army);
                    zCH = map(rec_right[2], 480, -480, -armz, armz);
                    rxCH = map(rec_right[7], -480, 480, -armRx, armRx);
                    ryCH = map(rec_right[6], -480, 480, -armRy, armRy);
                    rzCH = map(rec_right[1], -480, 480, -armRz, armRz);
                    
                    if (chooseCH == 4)        // 右手拨杆向前
                    {
                        beta_cmd = map(rec_right[3], -480, 480, -leftarmbeta, leftarmbeta);
                        if (leftarmbeta > 0){
                            if (beta_cmd < 0){
                                beta_cmd = 0;
                            }
                        }
                        else if (leftarmbeta < 0){
                            if (beta_cmd > 0){
                                beta_cmd = 0;
                            }
                        }

                        intool = frameCH;
                        
                        if(enableCH == 1)
                        {
                            left_once = 0;
                            sprintf(sendbuf,"speedL(0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d)\n", xCH, yCH, zCH, rxCH, ryCH, rzCH, beta_cmd, intool);
                            UDP_send(sendbuf);
                        }else if (enableCH == 0)
                        {
                            if (left_once == 0)
                            {
                                left_once = 1;
                                sprintf(sendbuf,"stopMove(0)\n");
                                UDP_send(sendbuf);
                            }
                        }
                    }
                    else if (chooseCH == 1)       // 右手拨杆向后
                    {
                        beta_cmd = map(rec_right[3], -480, 480, -rightarmbeta, rightarmbeta);
                        if (rightarmbeta > 0){
                            if (beta_cmd < 0){
                                beta_cmd = 0;
                            }
                        }
                        else if (rightarmbeta < 0){
                            if (beta_cmd > 0){
                                beta_cmd = 0;
                            }
                        }

                        intool = frameCH;
                        
                        if(enableCH == 1)
                        {
                            right_once = 0;
                            sprintf(sendbuf,"speedL(1,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d)\n", xCH, yCH, zCH, rxCH, ryCH, rzCH, beta_cmd, intool);
                            UDP_send(sendbuf);
                        }else if (enableCH == 0)
                        {
                            if (right_once == 0)
                            {
                                right_once = 1;
                                sprintf(sendbuf,"stopMove(1)\n");
                                UDP_send(sendbuf);
                            }
                        }
                    }
                    else if (chooseCH == 2)       // 右手拨杆向后
                    {
                        printf("head\n");
                    }
                }
            }
            else {
                ser.flushInput(); 
            }
        }
        ROS_INFO("in");
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } //
    ser.close();
    close(sockCli);
}
