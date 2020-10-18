#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
// #include <serial_dev_msgs/Paw.h>
# include <serial_dev_msgs/headpan.h>

// ===========================================

# define DEG2RAD 3.1415926/180.0
# define RAD2DEG 180.0/3.1415926

serial::Serial headser; //声明串口对象

/* serial */
std::string param_port_path_;
int param_baudrate_;
int param_loop_rate_;

double pitchuplim;
double pitchdownlim;
double yawuplim;
double yawdownlim;

void write_callback_pawcmd(const serial_dev_msgs::headpan & pancmd) 
{
    uint8_t cmd[8] = {'\0'};
    uint16_t cmd_pos;
    uint16_t pitchcmd;
    uint16_t yawcmd;
    int i;

    cmd[0] = 0xa0;
    ROS_INFO("Set head to : pitch %f, yaw %f", pancmd.pitch, pancmd.yaw); 
    if (pancmd.pitch > pitchuplim){
        pitchcmd = uint16_t((pitchuplim/180.0 - 0.5)* 1000 + 1500);
    }
    else if (pancmd.pitch < pitchdownlim){
        pitchcmd = uint16_t((pitchdownlim /180.0 - 0.5)* 1000 + 1500);
    }
    else{
        pitchcmd = uint16_t((pancmd.pitch /180.0 - 0.5)* 1000 + 1500);
    }

    if (pancmd.yaw > yawuplim){
        yawcmd = uint16_t((yawuplim /180.0 - 0.5)* 1000 + 1500);
    }
    else if (pancmd.yaw < yawdownlim){
        yawcmd = uint16_t((yawdownlim /180.0 - 0.5)* 1000 + 1500);
    }
    else{
        yawcmd = uint16_t((pancmd.yaw /180.0 - 0.5)* 1000 + 1500);
    }

    cmd[1] = yawcmd >> 8;
    cmd[2] = yawcmd & 0xff;
    cmd[3] = pitchcmd >> 8;
    cmd[4] = pitchcmd & 0xff;
    cmd[5] = 0x00;
    for (i = 0; i<5; i++){
        cmd[5] += cmd[i];
    }
    headser.write(cmd,6);
    //ser.write(msg->data);   //发送串口数据 
} 

// ===========================================
int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "head_pan"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
    uint8_t rec[2000] = {'\0'}; 

    //发布主题 
    ros::Subscriber paw_sub = nh.subscribe("/head_pan", 10, write_callback_pawcmd);
    
	nh.param<std::string>("head_pan/port", param_port_path_, "/dev/head_pan");
	nh.param<int>("head_pan/baudrate", param_baudrate_, 9600);
	nh.param<int>("head_pan/loop_rate", param_loop_rate_, 20);
	nh.param<double>("head_pan/pitchuplim", pitchuplim, 180.0);
	nh.param<double>("head_pan/pitchdownlim", pitchdownlim, 0.0);
	nh.param<double>("head_pan/yawuplim", yawuplim, 180.0);
	nh.param<double>("head_pan/yawdownlim", yawdownlim, 0.0);

    try 
    { 
        //设置串口属性，并打开串口 
        headser.setPort(param_port_path_); 
        headser.setBaudrate(param_baudrate_); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        headser.setTimeout(to); 
        headser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "<<param_port_path_); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(headser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port "<<param_port_path_<<" initialized"); 
    } 
    else 
    { 
        return -1; 
    }

    //指定循环的频率 
    ros::Rate loop_rate(param_loop_rate_); 
    
    while(ros::ok()) 
    { 
        if(headser.available()){
            // headser.read(rec,1);
            // if (rec[0] == 0xaa)
            // {
            //     headser.read(rec+1, );
            //     sum += 0xaa;
            //     if (rec[CHANNEL * 2 +1] == sum ){

            //     }
            //     sum = 0;
            //     headser.flushInput(); 
            // }
        } 
        

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } //
    headser.close();
}
