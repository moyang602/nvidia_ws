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

#include <ifaddrs.h>

#define MAX_LENGTH 254
# define MAX_IP 10

int port = 0;
std::string hostIP;
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
	n.param<std::string>("rob_feedback/hostIP",hostIP, "127.0.0.1");
	

	int sockSer = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockSer == -1)
    {
        perror("socket");
        exit(1);
    }

    addrSer.sin_family = AF_INET;
    addrSer.sin_port = htons(port);//????
    addrSer.sin_addr.s_addr = inet_addr(hostIP.c_str());//????????? 0?127.0.0.1?? 1?????ip
 
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
	int pub_cnt = 0;
	
	while (ros::ok())
	{
	
		rec_len = recvfrom(sockSer, recvbuf, sizeof(recvbuf), MSG_DONTWAIT, (struct sockaddr*)&addrCli, &addrlen);
		if (rec_len>0)
		{
			//ROS_INFO("Cli:>%s\n", recvbuf);
			js.header.stamp = ros::Time::now();

			s = recvbuf;
			try{

				boost::split( vStr, s, boost::is_any_of( "," ), boost::token_compress_on );
				for( int i = 0; i < 17; i++)
					{
						js.position.at(i) = atof(vStr.at(i).c_str());
					}

					js.position.at(17) = 0.0;		// yao_joint
					
					ROS_INFO_STREAM("in:" << js.position.at(0) << " " << vStr.at(vStr.size()-2) << vStr.at(1));
					if (pub_cnt > 10)
					{
						pub_cnt = 0;
						js_pub.publish(js);
					}
					pub_cnt ++;
			}
			catch(std::exception e1){
				ROS_WARN("failed");
			}	
}
				
		
        
		ros::spinOnce();
		loop_rate.sleep();
	}
	close(sockSer);

	return 0;
}
