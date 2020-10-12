#include "ros/ros.h"
#include "std_msgs/String.h"
#include<geometry_msgs/Twist.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <sstream>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
 

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <ifaddrs.h>

#define MAX_LENGTH 254
# define MAX_IP 10

int port = 0;
std::string hostIP;
ros::Subscriber sub;

struct sockaddr_in addrCli;
struct sockaddr_in addrSer;
int sockCli;
socklen_t addrlen;

char sendbuf[MAX_LENGTH];

void callback(const geometry_msgs::Twist& cmd_vel)
{
	double v = cmd_vel.linear.x;
	double w = cmd_vel.angular.z;
	sprintf(sendbuf, "carMove(%.3f,%.3f)\n", v, w);
	sendto(sockCli, sendbuf, strlen(sendbuf)+1, 0, (struct sockaddr*)&addrSer, addrlen);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "veltrans");
	ros::NodeHandle n;
 
	n.param<int>("robot_port",port, 0);
	n.param<std::string>("robot_ip",hostIP, "127.0.0.1");

	sub = n.subscribe("cmd_vel", 10,  callback);

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

    ROS_INFO("veltrans udp init ok,IP:%s,port %d", hostIP.c_str(), port);

	ros::Rate loop_rate(1000);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	close(sockCli);

	return 0;
}
