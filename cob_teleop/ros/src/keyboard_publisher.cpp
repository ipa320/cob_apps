/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_apps
 * \note
 *   ROS package name: cob_teleop
 *
 * \author
 *   Author: Felix Messmer, email:felix.messmer@ipa.fhg.de
 *
 * \date Date of creation: January 2011
 *
 * \brief
 *   Implementation of teleoperation node for keyboard.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/




#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include <joy/Joy.h>




#define KEYCODE_N 0x6E	//run
#define KEYCODE_M 0x6D	//mode
#define KEYCODE_H 0x68	//help


//platform
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65


//arm+tray
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38

//#define KEYCODE_Q 0x61
//#define KEYCODE_W 0x61
//#define KEYCODE_E 0x61
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_Z 0x7A
#define KEYCODE_U 0x75
#define KEYCODE_I 0x69


//torso+head
//#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67

#define KEYCODE_Y 0x79
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63
#define KEYCODE_V 0x76
#define KEYCODE_B 0x62



const int PUBLISH_FREQ = 5.0;

int kfd = 0;
struct termios cooked, raw;

bool fast_toggle,mode;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}


void composeJoyMessage(joy::Joy &msg, char c)
{
	msg.buttons[5]=0;	
	
	if(fast_toggle)
		msg.buttons[7]=1;

	if(mode)	//joint_mode
	{
		switch(c)
		{
		//positive
		case KEYCODE_1:	//arm
		  msg.buttons[0]=1;
		  msg.axes[4]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_2:
		  msg.buttons[0]=1;
		  msg.axes[5]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_3:
		  msg.buttons[1]=1;
		  msg.axes[4]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_4:
		  msg.buttons[1]=1;
		  msg.axes[5]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_5:
		  msg.buttons[2]=1;
		  msg.axes[4]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_6:
		  msg.buttons[2]=1;
		  msg.axes[5]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_7:
		  msg.buttons[3]=1;
		  msg.axes[4]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_8:	//tray
		  msg.buttons[3]=1;
		  msg.axes[5]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_Y:	//torso
		  msg.buttons[6]=1;
		  msg.axes[4]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_S:
		  msg.buttons[6]=1;
		  msg.axes[5]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_C:
		  msg.buttons[4]=1;
		  msg.axes[4]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_F:
		  msg.buttons[4]=1;
		  msg.axes[5]=1.0;
		  msg.buttons[5]=1;
		  break;
		//case KEYCODE_G:	//head
		//  msg.buttons[2]=1;
		//  msg.axes[4]=1.0;
		//  msg.buttons[5]=1;
		//  break;		  
		
		//negative  
		case KEYCODE_Q:	//arm
		  msg.buttons[0]=1;
		  msg.axes[4]=-1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_W:
		  msg.buttons[0]=1;
		  msg.axes[5]=-1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_E:
		  msg.buttons[1]=1;
		  msg.axes[4]=-1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_R:
		  msg.buttons[1]=1;
		  msg.axes[5]=-1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_T:
		  msg.buttons[2]=1;
		  msg.axes[4]=-1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_Z:
		  msg.buttons[2]=1;
		  msg.axes[5]=-1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_U:
		  msg.buttons[3]=1;
		  msg.axes[4]=-1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_I:	//tray
		  msg.buttons[3]=1;
		  msg.axes[5]=-1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_A:	//torso
		  msg.buttons[6]=1;
		  msg.axes[4]=-1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_X:
		  msg.buttons[6]=1;
		  msg.axes[5]=-1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_D:
		  msg.buttons[4]=1;
		  msg.axes[4]=-1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_V:
		  msg.buttons[4]=1;
		  msg.axes[5]=-1.0;
		  msg.buttons[5]=1;
		  break;
		//case KEYCODE_B:	//head
		//  msg.buttons[2]=1;
		//  msg.axes[4]=-1.0;
		//  msg.buttons[5]=1;
		//  break;			  		  		  
		}
	}
	else		//platform_mode
	{
		switch(c)
		{
		case KEYCODE_W:
		  msg.axes[1]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_S:
		  msg.axes[1]=-1.0;
		  msg.buttons[5]=1;    
		  break;
		case KEYCODE_A:
		  msg.axes[0]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_D:
		  msg.axes[0]=-1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_Q:
		  msg.axes[2]=1.0;
		  msg.buttons[5]=1;
		  break;
		case KEYCODE_E:
		  msg.axes[2]=-1.0;
		  msg.buttons[5]=1;
		  break;
		}
	}
}


void showHelp()
{
    puts(""); 
    puts("Reading from keyboard");
  	puts("---------------------------");
  	puts("Use 'm' to toggle modes (joint/platform)");
  	puts("Use 'n' to toggle run");
  	puts("---------------------------");
  	puts("In platform_mode");
  	puts("Use 'wasd' to translate");
  	puts("Use 'qe' to yaw");
  	puts("---------------------------");
  	puts("In joint_mode");
  	puts("Use '1'-'7' and 'q'-'u' for arm");
  	puts("Use '8' and 'i' for tray");
  	puts("Use 'a'-'f' and 'y'-'v' for torso");
  	puts("---------------------------");
  	puts("Use 'h' to show this help");
  	puts("Hit 'SPACE' to stop movement");
    puts("");   	
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_publisher");
  ros::NodeHandle n;
  
    puts(""); 
    puts("Reading from keyboard");
  	puts("---------------------------");
  	puts("Use 'm' to toggle modes (joint/platform)");
  	puts("Use 'n' to toggle run");
  	puts("---------------------------");
  	puts("In platform_mode");
  	puts("Use 'wasd' to translate");
  	puts("Use 'qe' to yaw");
  	puts("---------------------------");
  	puts("In joint_mode");
  	puts("Use '1'-'7' and 'q'-'u' for arm");
  	puts("Use '8' and 'i' for tray");
  	puts("Use 'a'-'f' and 'y'-'v' for torso");
  	puts("---------------------------");
  	puts("Use 'h' to show this help");
  	puts("Hit 'SPACE' to stop movement");  	
    puts("");   	

  signal(SIGINT,quit);

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  //raw.c_cc[VMIN] = 0;
  //raw.c_cc[VTIME] = 0;
  tcsetattr(kfd, TCSANOW, &raw);

  
  fast_toggle=false;
  mode=false;

  ros::Publisher keyboard_pub = n.advertise<joy::Joy>("joy", 1);
  char c;

  while (ros::ok())
  {
    ros::spinOnce();  
  
  	joy::Joy msg;
	msg.axes.resize(6);
	msg.buttons.resize(12);
  
  	// get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

	//ROS_INFO("I got key %d",c);
	
	switch(c)
    {
      // Fast_Toggle
    case KEYCODE_N:
	  if(fast_toggle)
	  {
	  	fast_toggle=false;
	  	ROS_INFO("fast_mode: OFF");
	  }
	  else
	  {
	  	fast_toggle=true;
	  	ROS_INFO("fast_mode: ON");
	  }
      break;
      // Mode_Toggle
    case KEYCODE_M:
      if(mode)
      {
      	mode=false;
      	ROS_INFO("Mode: Platform_Mode");
      }
      else
      {
        mode=true;
        ROS_INFO("Mode: Joint_Mode");
      } 
	  break;
	case KEYCODE_H:
	  showHelp();
	  break;
	}            
      
	
	composeJoyMessage(msg, c);

    keyboard_pub.publish(msg);
  }


  return 0;
}

