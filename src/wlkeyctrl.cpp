/*
*  Software License Agreement (BSD License)
*  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
*  All rights reserved.
*  Author:¡­¡­
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Intelligent Robotics Lab nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/

#include <sys/types.h>
#include <sys/socket.h> 
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h> 
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "ros/ros.h"
#include <signal.h>
#include <signal.h>
#include <termios.h>
#include "time.h"
#include <sys/file.h>
#include <pthread.h>
#include <std_msgs/Bool.h>
#include <termios.h>
#include <ros/ros.h>

#define  CLIENT_PORT 3333
#define  IP_ADDRESS "192.168.1.241"
#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20
#define KEYCODE_0 0x30
#define KEYCODE_ENTER 0x0A

int main(int argc, char** argv)
{
  //ros::init(argc, argv, "teleop_key");
  char buff[5];
  char c;
  int sockfd;
  int g_kfd=0;
  struct termios g_cooked, g_raw;
  struct sockaddr_in server_sockaddr;
  sockfd=socket(AF_INET,SOCK_STREAM,0);
  if(sockfd<0)
  {
    ROS_INFO("socket create failed\n");
    return -1;
  }
  ROS_INFO("socket success!,sockfd = %d\n",sockfd);

  memset(server_sockaddr.sin_zero, 0x00, 8);
  server_sockaddr.sin_family = AF_INET;
  server_sockaddr.sin_port = htons(CLIENT_PORT);
  server_sockaddr.sin_addr.s_addr = inet_addr(IP_ADDRESS);

  ROS_INFO("connecting...\n");

  if(connect(sockfd,(struct sockaddr *)&server_sockaddr,sizeof(server_sockaddr))<0)
  {
    ROS_INFO("connect error");
    return -1;
  }
  // get the console in g_raw mode                                                              
  tcgetattr(g_kfd, &g_cooked);
  memcpy(&g_raw, &g_cooked, sizeof(struct termios));
  g_raw.c_lflag &= ~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  g_raw.c_cc[VEOL] = 1;
  g_raw.c_cc[VEOF] = 2;
  tcsetattr(g_kfd, TCSANOW, &g_raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");
  for(;;)
  {
    // get the next event from the keyboard  
    if(read(g_kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    
  switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        send(sockfd,"L",1,0);
        sleep(0.5);
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        send(sockfd,"R",1,0);
        sleep(0.5);
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        send(sockfd,"F",1,0);
 sleep(0.5);
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        send(sockfd,"B",1,0);
 sleep(0.5);
	break;
      case KEYCODE_SPACE:
	ROS_DEBUG("STOP");
	send(sockfd,"S",1,0);
        break;
     /* case KEYCODE_0:
	ROS_DEBUG("BEGIN");
	send(sockfd,L,1,0);
        break;
      case KEYCODE_ENTER:
	ROS_DEBUG("END");
	send(sockfd,L,1,0);
        break;
      default :
	break;
    */	
    }
  }
  return(0);
}

    
  




  
