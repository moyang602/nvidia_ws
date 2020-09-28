#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 



#include<geometry_msgs/Twist.h>

// ===========================================
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <ifaddrs.h>
#define MAX_LENGTH 254

int port = 0;
std::string hostIP;

struct sockaddr_in addrCli;
struct sockaddr_in addrSer;
int sockCli;
socklen_t addrlen;

char sendbuf[MAX_LENGTH];

// ===========================================
#define DEADZONE 50
# define rDEADZONE 20
#define MAX_x 1.0
#define MAX_y 1.0
#define MAX_z 1.0

# define max_step_angle 10.0

#define DEG2RAD 3.1415926/180.0

serial::Serial ser; //声明串口对象

/* serial */
std::string param_port_path_;
int param_baudrate_;
int param_loop_rate_;
serial::parity_t param_patity_;

// ===========================================
#define CHANNEL 7
int i = 0;
int j = 0;
uint8_t sum = 0;
int rec_right[CHANNEL] = {0};
int lastrec[CHANNEL] = {0};
uint8_t op_flag_1 = 0;
uint8_t op_flag_2 = 0;
uint8_t op_flag_3 = 0;

uint32_t watchdog = 0;

std::vector<double> recf[CHANNEL];

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


int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "ardu_remoter_pub"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
	geometry_msgs::Twist cmd;

    uint8_t rec[2000] = {'\0'}; 

    //发布主题 
    ros::Publisher remo_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000); 
    
	nh.param<std::string>("ardu_remoter_pub/port", param_port_path_, "/dev/remote_USB");
	nh.param<int>("ardu_remoter_pub/baudrate", param_baudrate_, 9600);
	nh.param<int>("ardu_remoter_pub/loop_rate", param_loop_rate_, 20);

    nh.param<int>("robot_port",port, 0);
	nh.param<std::string>("robot_ip",hostIP, "127.0.0.1");

    sockCli = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockCli == -1)
    {
        perror("socket");
        exit(1);
    }

    addrSer.sin_family = AF_INET;
    addrSer.sin_port = htons(port);//????
    addrSer.sin_addr.s_addr = inet_addr(hostIP.c_str());//????????? 0?127.0.0.1?? 1?????ip
 
    addrlen = sizeof(struct sockaddr);

    ROS_INFO("udp init ok,IP:%s,port %d", hostIP.c_str(), port);
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
    int middle_cali = 0;
    i=0;
    uint8_t rec1=0;
    uint8_t index = 0;
    double angle1 = 0.0;
    double angle2 = 0.0;
    double angle3 = 0.0;

    double last_angle1 = 0.0;
    double last_angle2 = 0.0;
    double last_angle3 = 0.0;

    int slow;
    
    while(ros::ok()) 
    { 
        // while(ser.available() < 10);

            // ROS_INFO("num:%d \n",ser.available());
        if(ser.available()){ 
            watchdog = 0;
            ser.read(rec,1);
            if (rec[0] == 0xaa)
            {
                ser.read(rec+1,CHANNEL * 2 +1);
                for(j = 0; j < CHANNEL ; ++j){
                    rec_right[j] = ( rec[2*j+1] <<8 ) + rec[2*j+2];
                    sum += rec[2*j+1];
                    sum += rec[2*j+2];
                }
                sum += 0xaa;
                //ROS_INFO("sum=%x;rec[9]=%x\n",sum,rec[9]);
                if (rec[CHANNEL * 2 +1] == sum ){
                    //ROS_INFO("ok\n");
                    rec_right[0] -= 1500;
                    rec_right[1] -= 1500;
                    rec_right[2] -= 1000;
                    rec_right[3] -= 1500;
                    rec_right[4] -= 1500;
                    rec_right[5] -= 1500;
                    rec_right[6] -= 1500;
                    
                    for (i= 0; i <7 ;i++){
                        if (i == 2){
                            if(rec_right[i] > 1000) rec_right[i] = 1000;
                            if(rec_right[i] < 0) rec_right[i] = 0;
                            if (rec_right[i]<100) rec_right[i] = 0;
                            else rec_right[i] -=100;
                        }
                        else if(i >= 4){
                            if(rec_right[i] > 500) rec_right[i] = 500;
                            if(rec_right[i] < -500) rec_right[i] = -500;
                            if (abs(rec_right[i])<rDEADZONE) rec_right[i] = 0;
                            
                            rec_right[i] = (int)(fillter((double)rec_right[i], i, 20));
                            if (abs(rec_right[i] - lastrec[i]) <2 )
                            {
                                rec_right[i] = lastrec[i];
                            }
                            lastrec[i] = rec_right[i];

                        }
                        else{
                            if(rec_right[i] > 500) rec_right[i] = 500;
                            if(rec_right[i] < -500) rec_right[i] = -500;
                            if (abs(rec_right[i])<DEADZONE) rec_right[i] = 0;
                        }
                    }

                    ROS_INFO("1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d  ",rec_right[0],rec_right[1],rec_right[2],rec_right[3],rec_right[4],rec_right[5],rec_right[6]);

                    cmd.linear.x  = float( rec_right[2] * rec_right[1] ) / 450000.0 * MAX_x;
                    cmd.linear.y  = 0.0; //float( rec_right[2] ) * float( rec_right[0] ) / 450000.0 * MAX_y ;
                    cmd.linear.z  = 0;
                    cmd.angular.x = 0;
                    cmd.angular.y = 0;
                    cmd.angular.z = float( rec_right[2] * rec_right[3] ) / 450000.0 * MAX_z ;

                    remo_pub.publish(cmd);

                    if(abs(rec_right[4]) < rDEADZONE)
                    {
                        op_flag_1 = 1;
                    }
                    if(abs(rec_right[5]) < rDEADZONE)
                    {
                        op_flag_2 = 1;
                    }
                    if(abs(rec_right[6]) < rDEADZONE)
                    {
                        op_flag_3 = 1;
                    }

                    if (op_flag_1 == 1)
                    {
                        angle1 = (double)rec_right[4] /500.0 * 180.0;
                    }
                    else {
                        angle1 = 0.0;
                    }
                    if (op_flag_2 == 1)
                    {
                        angle2 = (double)rec_right[5] /500.0 * 60.0;
                    }
                    else {
                        angle2 = 0.0;
                    }
                    if (op_flag_3 == 1)
                    {
                        angle3 = (double)rec_right[6] /500.0 * 180.0;
                    }
                    else {
                        angle3 = 0.0;
                    }

                    if (op_flag_1 && op_flag_2  && op_flag_3)
                    {
                        if (angle1 - last_angle1 > max_step_angle)
                        {
                            angle1 = last_angle1 + max_step_angle;
                        }
                        else if (angle1 - last_angle1 < -max_step_angle)
                        {
                            angle1 = last_angle1 - max_step_angle;
                        }
                        last_angle1 = angle1;

                        if (angle2 - last_angle2 > max_step_angle)
                        {
                            angle2 = last_angle2 + max_step_angle;
                        }
                        else if (angle2 - last_angle2 < -max_step_angle)
                        {
                            angle2 = last_angle2 - max_step_angle;
                        }
                        last_angle2 = angle2;

                        if (angle3 - last_angle3 > max_step_angle)
                        {
                            angle3 = last_angle3 + max_step_angle;
                        }
                        else if (angle3 - last_angle3 < -max_step_angle)
                        {
                            angle3 = last_angle3 - max_step_angle;
                        }
                        last_angle3 = angle3;
                        sprintf(sendbuf,"moveFollow(3,%.3f,%.3f,%.3f,%.3f,%.3f)\n", angle3 * DEG2RAD, angle1 * DEG2RAD, -angle2 * DEG2RAD, angle2 * DEG2RAD, angle1 * DEG2RAD);
                        
                        sendto(sockCli, sendbuf, strlen(sendbuf)+1, 0, (struct sockaddr*)&addrSer, addrlen);
                        sendbuf[strlen(sendbuf)] == '\0';
                        ROS_INFO("%s",sendbuf);
                    }

                }
                sum = 0;
                ser.flushInput(); 
                
            }
        } 
        else{
            watchdog++;
            if (watchdog > param_loop_rate_ * 3){  // 3秒检测不到则认为已经加锁
                watchdog = 0;
                op_flag_1 = 0;
                op_flag_2 = 0;
                op_flag_3 = 0;
            }
        }    
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } //
    ser.close();
    close(sockCli);
}
