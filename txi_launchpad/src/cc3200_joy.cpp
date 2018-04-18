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

#define BUFLEN 12  //Max length of buffer
#define TANK 1
#define JOY 2
 
void error(char *s)
{
    perror(s);
    exit(1);
}

float speedScale = 1.0;
int mode = TANK;
unsigned char message[BUFLEN];
struct sockaddr_in si_other;
int s, i, slen=sizeof(si_other);

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	int j;
	uint8_t joyVal[BUFLEN];
    
    if(joy->buttons[0] == 1 && joy->buttons[1] == 0) {
        speedScale = 2.0; // full speed if the B button was pressed - default state
    }
    else if(joy->buttons[0] == 0 && joy->buttons[1] == 1) {
        speedScale = 1.0; // half speed if the A button was pressed
    }
    else if(joy->buttons[0] == 1 && joy->buttons[1] == 1) {
        speedScale = 2.0; // mashing buttons will result in half speed
    }

    if(joy->buttons[2] == 1 && joy->buttons[3] == 0) {
        mode = TANK; // X button runs TANK mode
    }
    else if(joy->buttons[2] == 0 && joy->buttons[3] == 1) {
        mode = JOY; // Y button runs JOY mode
    }
    else if(joy->buttons[2] == 1 && joy->buttons[3] == 1) {
        mode = TANK; // mashing both buttons will give way to TANK mode
    }

	joyVal[0] = (int) (127.5 * (joy->axes[0] / speedScale + 1.0)); // Left joystick L-R axes
	joyVal[1] = (int) (127.5 * (joy->axes[1] / speedScale + 1.0)); // Left joystick U-D axes
	joyVal[2] = (int) ((-1.0 * joy->axes[2] + 1.0) / speedScale); // LT
	joyVal[3] = (int) (127.5 * (joy->axes[3] / speedScale + 1.0)); // Right joystick L-R axes
	joyVal[4] = (int) (127.5 * (joy->axes[4] / speedScale + 1.0)); // Right joystick U-D axes
	joyVal[5] = (int) ((-1.0 * joy->axes[5] + 1.0) / speedScale); // RT
	joyVal[6] = (int) (127.5 * (joy->axes[6] / speedScale + 1.0)); // crosspad L-R
	joyVal[7] = (int) (127.5 * (joy->axes[7] / speedScale + 1.0)); // crosspad U-D
    joyVal[8] = (int) (2 * joy->buttons[4] / (int) speedScale); // LB
    joyVal[9] = (int) (2 * joy->buttons[5] / (int) speedScale); // RB
    joyVal[10] = (int) speedScale; // speed scale value
    joyVal[11] = mode; // wheels operation mode

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
