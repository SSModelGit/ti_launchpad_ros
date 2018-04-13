/*
    ROS udp client to Texas Instruments CC3200 Launchpad board
*/
#include<stdint.h>
#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<iomanip>
#include<sstream>
#include<string>
#include<arpa/inet.h>
#include<sys/socket.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#define BUFLEN 8  //Max length of buffer
 
void error(char *s)
{
    perror(s);
    exit(1);
}

unsigned char message[BUFLEN];
struct sockaddr_in si_other;
int s, i, slen=sizeof(si_other);

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	int j;
	uint8_t joyVal[BUFLEN];
	joyVal[0] = (int) (127.5 * (joy->axes[0] + 1.0));
	joyVal[1] = (int) (127.5 * (joy->axes[1] + 1.0));
	joyVal[2] = (int) (127.5 * (-1.0 * joy->axes[2] + 1.0));
	joyVal[3] = (int) (127.5 * (joy->axes[3] + 1.0));
	joyVal[4] = (int) (127.5 * (joy->axes[4] + 1.0));
	joyVal[5] = (int) (127.5 * (-1.0 * joy->axes[5] + 1.0));
	joyVal[6] = (int) (127.5 * (joy->axes[6] + 1.0));
	joyVal[7] = (int) (127.5 * (joy->axes[7] + 1.0));

	printf("Joystick values: %d | %d | %d | %d | ", joyVal[0], joyVal[1], joyVal[2], joyVal[3]);
	printf("%d | %d | %d | %d \n", joyVal[4], joyVal[5], joyVal[6], joyVal[7]);
	for(j = 0; j < BUFLEN; j++) {
		message[j] = joyVal[j];
	}
	printf("Message: %d | %d | %d | %d | ", message[0], message[1], message[2], message[3]);
	printf("%d | %d | %d | %d \n", message[4], message[5], message[6], message[7]);
	if (sendto(s, message, sizeof(message) , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            error("ERROR writing to socket");
        }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cc3200_joy"); // connection between Modelica and ROS

    ros::NodeHandle nh;
	ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    ros::NodeHandle nh_param("~");
	int port;
	std::string server_ip;
	nh_param.param<int>("port", port, 2391);
	nh_param.param<std::string>("server_ip", server_ip, "192.168.1.114");
 
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        error("ERROR opening socket");
    }
 
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(port);
     
    if (inet_aton(server_ip.c_str() , &si_other.sin_addr) == 0) 
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }

 	ros::spin();

    while(ros::ok())
    {
        
    }
 
    close(s);

    return 0;
}
