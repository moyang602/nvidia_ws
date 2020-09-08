#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

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


#include <stdio.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <cstring>

#define MAX_LENGTH 254
# define MAX_IP 10

int port = 0;
int id = 3;
char recvbuf[1024];


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "rob_feedback");
	ros::NodeHandle n;
	struct sockaddr_in addrCli;
	struct sockaddr_in addrSer;
 
	n.param<int>("rob_feedback/port",port, 0);
	n.param<int>("rob_feedback/ip_index",id, 3);
	ROS_INFO("%d,%d\n",port,id);

	int sockSer = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockSer == -1)
    {
        perror("socket");
        exit(1);
    }
 
	char ipAddr[MAX_LENGTH];
    ipAddr[0] = '\0';
    struct ifaddrs * ifAddrStruct = NULL;
    void * tmpAddrPtr = NULL;
	char addressBuffer[MAX_IP][INET_ADDRSTRLEN];

    if (getifaddrs(&ifAddrStruct) != 0)
    {
        //if wrong, go out!
        printf("Somting is Wrong!\n");
        return -1;
    }
    struct ifaddrs * iter = ifAddrStruct;
	int cnt = 0;
    while (iter != NULL && cnt <= MAX_IP) {
        if (iter->ifa_addr->sa_family == AF_INET) { //if ip4
            // is a valid IP4 Address
            tmpAddrPtr = &((struct sockaddr_in *)iter->ifa_addr)->sin_addr;
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer[cnt], INET_ADDRSTRLEN);
            if (strlen(ipAddr) + strlen(addressBuffer[cnt]) < MAX_LENGTH - 1)
            {
                if (strlen(ipAddr) > 0)
                {
                     strcat(ipAddr, ";");
                }
                strcat(ipAddr, addressBuffer[cnt]);
            }
            else
            {
                printf("Too many ips!\n");
                break;
            }
        }
        //else if (ifaddrstruct->ifa_addr->sa_family == af_inet6) { // check it is ip6

        /* deal ip6 addr */
        //    tmpaddrptr = &((struct sockaddr_in *)ifaddrstruct->ifa_addr)->sin_addr;
        //    char addressbuffer[inet6_addrstrlen];
        //    inet_ntop(af_inet6, tmpaddrptr, addressbuffer, inet6_addrstrlen);

        //}
        iter = iter->ifa_next;
		cnt ++;
    }
    
	strcat(addressBuffer[id], "\0");
    printf("The binded ips: %s, port is %d\n", addressBuffer[id], port);

    addrSer.sin_family = AF_INET;
    addrSer.sin_port = htons(port);//端口号
    addrSer.sin_addr.s_addr = inet_addr("10.1.76.121");//服务器地址 0为127.0.0.1， 1为默认的ip
 
    socklen_t addrlen = sizeof(struct sockaddr);
    int ret = bind(sockSer, (struct sockaddr*)&addrSer, addrlen);
    if(ret == -1)
    {
        perror("bind failed");
        exit(1);
    }
 
	// Jointstates things
	sensor_msgs::JointState js;
	js.name.resize(22);
	js.name = {"leftarm_joint1","leftarm_joint2","leftarm_joint3","leftarm_joint4","leftarm_joint5","leftarm_joint6","leftarm_joint7",
				"rightarm_joint1","rightarm_joint2","rightarm_joint3","rightarm_joint4","rightarm_joint5","rightarm_joint6","rightarm_joint7",
				"dt_joint","pt_joint","yt_joint","yao_joint", 
				"leftleg_joint1","leftleg_joint2","rightleg_joint1","rightleg_joint2"};
	js.position.resize(js.name.size());

	ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
	ros::Rate loop_rate(1000);

	int rec_len = 0;
	std::string s;
	std::vector<std::string> vStr;
	double joint[14];
	
	while (ros::ok())
	{
	
		rec_len = recvfrom(sockSer, recvbuf, sizeof(recvbuf), MSG_DONTWAIT, (struct sockaddr*)&addrCli, &addrlen);
		if (rec_len>0)
		{
			// ROS_INFO("Cli:>%s\n", recvbuf);
			js.header.stamp = ros::Time::now();

			s = recvbuf;
			boost::split( vStr, s, boost::is_any_of( "," ), boost::token_compress_on );
			for( int i = 0; i < 17; i++)
			{
				js.position.at(i) = atof(vStr.at(i).c_str());
			}

			js.position.at(17) = 0.0;		// yao_joint
			
			ROS_INFO_STREAM("in:" << js.position.at(0) << " " << vStr.at(vStr.size()-2) << vStr.at(1));
			js_pub.publish(js);
		}
        
		ros::spinOnce();
		loop_rate.sleep();
	}
	freeifaddrs(ifAddrStruct);
	close(sockSer);

	return 0;
}