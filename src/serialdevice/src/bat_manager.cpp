#include <iostream>
#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 

#include<serial_dev_msgs/batteryInfo.h>
#include<serial_dev_msgs/SetHeight.h>

int fillter_period = 10;;

int i, j;
serial::Serial ser; //声明串口对象

serial_dev_msgs::batteryInfo batstate;
serial_dev_msgs::SetHeight st;

/* serial */
std::string param_port_path_;
int param_baudrate_;
std::vector<double> recdata[4];
double resultdata[4];

void write_callback_bat(const serial_dev_msgs::batteryInfo & a) 
{
    uint8_t cmd[8] = {0x00};
    cmd[0] = 0xaa;
    cmd[1] = 0x55;
    if (a.delay == 1)
    {
        cmd[2] = 0x01;
        batstate.delay = 1;
    }
    else if (a.delay == 0)
    {
        cmd[2] = 0x00;
        batstate.delay = 0;
    }
    ser.write(cmd,3);
    ROS_INFO("Sent");
}
 
int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "bat_manager"); 
    //声明节点句柄 
    ros::NodeHandle nh; 

    uint8_t rec[8] = {'\0'}; 

    //订阅主题，并配置回调函数 
    ros::Subscriber bat_sub = nh.subscribe("battery_cmd", 1000, write_callback_bat); 
    //发布主题 
    ros::Publisher bat_pub = nh.advertise<serial_dev_msgs::batteryInfo>("battery_state", 1000); 
    
    //nh.param<bool>("debug_imu", param_use_debug, false);
	nh.param<std::string>("bat_manager/port", param_port_path_, "/dev/ttyUSB1");
	nh.param<int>("bat_manager/baudrate", param_baudrate_, 9600);
	nh.param<int>("bat_manager/fillter_period", fillter_period, 10);

    ROS_INFO_STREAM("bat_manager port is " << param_port_path_<< ",baud rate:" <<param_baudrate_<< ",fillter:"<<fillter_period);

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
        ROS_INFO_STREAM("Serial Port:"<<param_port_path_<<" initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    //指定循环的频率 
    ros::Rate loop_rate(1000); 
    while(ros::ok()) 
    { 
        try{
            if(ser.available()){ 
                ser.read(rec,ser.available()/8);
                ser.flushInput();

                if (rec[0] == 0xff)
                {
                    for ( i =0; i < 4; i++)
                    {
                        resultdata[i] = 0;

                        while (recdata[i].size() <= fillter_period)
                        {
                            recdata[i].push_back(double(rec[i * 2 + 1] << 8 | rec[i * 2 + 2]));
                        }

                        if (recdata[i].size() > fillter_period)
                        {
                            recdata[i].erase(recdata[i].begin());
                            for (j = 0; j< recdata[i].size(); j++)
                            {
                                resultdata[i] += recdata[i].at(j) / fillter_period;
                            }
                        }
                    }

                    batstate.i1 = resultdata[0] / 10.0;
                    batstate.i2 = resultdata[1] / 10.0;
                    batstate.v_bat = resultdata[2] / 10.0;
                    batstate.v_test = resultdata[3] / 10.0;

                    ROS_INFO_STREAM(batstate.i1<<"A,"<<batstate.i2<<"A,"<<batstate.v_bat<<"V,"<<batstate.v_test<<"V\n");  
                    
                    bat_pub.publish(batstate);
                }
            } 
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
            ser.close();
        }
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 

    }
    ser.close();
}