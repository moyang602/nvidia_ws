#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <serial_dev_msgs/Paw.h>

// ===========================================

# define DEG2RAD 3.1415926/180.0
# define RAD2DEG 180.0/3.1415926

serial::Serial leftser; //声明串口对象
serial::Serial rightser; //声明串口对象

/* serial */
std::string param_leftport_path_;
int param_leftbaudrate_;
int param_leftuse_ = 0;
std::string param_rightport_path_;
int param_rightbaudrate_;
int param_rightuse_ = 0;

int param_loop_rate_;

void write_callback_pawcmd(const serial_dev_msgs::Paw & pawcmd) 
{
    uint8_t cmd[8] = {'\0'};
    uint16_t cmd_pos;
    if (param_leftuse_){
        cmd[0] = 0xaa;
        ROS_INFO("Set left Paw to : %f", pawcmd.leftPawPosition); 
        if ( pawcmd.leftPawPosition >= 0 && pawcmd.leftPawPosition <= 90.0){
            cmd_pos = uint16_t(pawcmd.leftPawPosition * 13.5 + 800);
            cmd[1] = cmd_pos >> 8;
            cmd[2] = cmd_pos & 0xff;
        }
        cmd[3] = 0x55;
        leftser.write(cmd,4);
    }

    if (param_rightuse_){
        cmd[0] = 0xaa;
        ROS_INFO("Set right Paw to : %f", pawcmd.rightPawPosition); 
        if ( pawcmd.rightPawPosition >= 0 && pawcmd.rightPawPosition <= 90.0){
            cmd_pos = uint16_t(pawcmd.rightPawPosition * 13.5 + 800);
            cmd[1] = cmd_pos >> 8;
            cmd[2] = cmd_pos & 0xff;
        }
        cmd[3] = 0x55;
        rightser.write(cmd,4);
    }
    //ser.write(msg->data);   //发送串口数据 
} 

// ===========================================
int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "paw_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
    uint8_t rec[2000] = {'\0'}; 

    //发布主题 
    ros::Subscriber paw_sub = nh.subscribe("/paw_cmd", 10, write_callback_pawcmd);
    
	nh.param<std::string>("paw/leftpaw_port", param_leftport_path_, "/dev/leftpaw");
	nh.param<int>("paw/leftpaw_baudrate", param_leftbaudrate_, 9600);
	nh.param<int>("paw/leftpaw_use", param_leftuse_, 0);

    nh.param<std::string>("paw/rightpaw_port", param_rightport_path_, "/dev/rightpaw");
	nh.param<int>("paw/rightpaw_baudrate", param_rightbaudrate_, 9600);
	nh.param<int>("paw/rightpaw_use", param_rightuse_, 0);

	nh.param<int>("paw/loop_rate", param_loop_rate_, 20);

/* ==================================== left ==================================== */
    if(param_leftuse_){
        ROS_INFO_STREAM("use left paw");
        try 
        { 
            //设置串口属性，并打开串口 
            leftser.setPort(param_leftport_path_); 
            leftser.setBaudrate(param_leftbaudrate_); 
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
            leftser.setTimeout(to); 
            leftser.open(); 
        } 
        catch (serial::IOException& e) 
        { 
            ROS_ERROR_STREAM("Unable to open port "<<param_leftport_path_); 
            return -1; 
        } 

        //检测串口是否已经打开，并给出提示信息 
        if(leftser.isOpen()) 
        { 
            ROS_INFO_STREAM("Serial Port "<<param_leftport_path_<<" initialized"); 
        } 
        else 
        { 
            return -1; 
        }
    }

/* ==================================== right ==================================== */
    if(param_rightuse_){
        ROS_INFO_STREAM("use right paw");
        try 
        { 
            //设置串口属性，并打开串口 
            rightser.setPort(param_rightport_path_); 
            rightser.setBaudrate(param_rightbaudrate_); 
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
            rightser.setTimeout(to); 
            rightser.open(); 
        } 
        catch (serial::IOException& e) 
        { 
            ROS_ERROR_STREAM("Unable to open port "<<param_rightport_path_); 
            return -1; 
        } 

        //检测串口是否已经打开，并给出提示信息 
        if(rightser.isOpen()) 
        { 
            ROS_INFO_STREAM("Serial Port "<<param_rightport_path_<<" initialized"); 
        } 
        else 
        { 
            return -1; 
        }
    }
/* ================================================================================= */

    //指定循环的频率 
    ros::Rate loop_rate(param_loop_rate_); 
    
    while(ros::ok()) 
    { 
        if (param_leftuse_){
            if(leftser.available()){
                // leftser.read(rec,1);
                // if (rec[0] == 0xaa)
                // {
                //     leftser.read(rec+1, );
                //     sum += 0xaa;
                //     if (rec[CHANNEL * 2 +1] == sum ){

                //     }
                //     sum = 0;
                //     leftser.flushInput(); 
                // }
            } 
        }
        

        if (param_leftuse_){
            if(rightser.available()){
                // rightser.read(rec,1);
                // if (rec[0] == 0xaa)
                // {
                //     rightser.read(rec+1, );
                //     sum += 0xaa;
                //     if (rec[CHANNEL * 2 +1] == sum ){

                //     }
                //     sum = 0;
                //     rightser.flushInput(); 
                // }
            } 
        }
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } //
    if (param_leftuse_)
        leftser.close();
    if (param_leftuse_)
        rightser.close();
}
