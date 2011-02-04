#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include <joy/Joy.h>




#define KEYCODE_N 0x6E	//run
#define KEYCODE_M 0x6D	//mode


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
		msg.buttons[5]=1;	//deadman
		switch(c)
		{
		//positive
		case KEYCODE_1:	//arm
		  msg.buttons[0]=1;
		  msg.axes[4]=1.0;
		  break;
		case KEYCODE_2:
		  msg.buttons[0]=1;
		  msg.axes[5]=1.0;
		  break;
		case KEYCODE_3:
		  msg.buttons[1]=1;
		  msg.axes[4]=1.0;
		  break;
		case KEYCODE_4:
		  msg.buttons[1]=1;
		  msg.axes[5]=1.0;
		  break;
		case KEYCODE_5:
		  msg.buttons[2]=1;
		  msg.axes[4]=1.0;
		  break;
		case KEYCODE_6:
		  msg.buttons[2]=1;
		  msg.axes[5]=1.0;
		  break;
		case KEYCODE_7:
		  msg.buttons[3]=1;
		  msg.axes[4]=1.0;
		  break;
		case KEYCODE_8:	//tray
		  msg.buttons[3]=1;
		  msg.axes[5]=1.0;
		  break;
		case KEYCODE_Y:	//torso
		  msg.buttons[6]=1;
		  msg.axes[4]=1.0;
		  break;
		case KEYCODE_S:
		  msg.buttons[6]=1;
		  msg.axes[5]=1.0;
		  break;
		case KEYCODE_C:
		  msg.buttons[4]=1;
		  msg.axes[4]=1.0;
		  break;
		case KEYCODE_F:
		  msg.buttons[4]=1;
		  msg.axes[5]=1.0;
		  break;
		//case KEYCODE_G:	//head
		//  msg.buttons[2]=1;
		//  msg.axes[4]=1.0;
		//  break;		  
		
		//negative  
		case KEYCODE_Q:	//arm
		  msg.buttons[0]=1;
		  msg.axes[4]=-1.0;
		  break;
		case KEYCODE_W:
		  msg.buttons[0]=1;
		  msg.axes[5]=-1.0;
		  break;
		case KEYCODE_E:
		  msg.buttons[1]=1;
		  msg.axes[4]=-1.0;
		  break;
		case KEYCODE_R:
		  msg.buttons[1]=1;
		  msg.axes[5]=-1.0;
		  break;
		case KEYCODE_T:
		  msg.buttons[2]=1;
		  msg.axes[4]=-1.0;
		  break;
		case KEYCODE_Z:
		  msg.buttons[2]=1;
		  msg.axes[5]=-1.0;
		  break;
		case KEYCODE_U:
		  msg.buttons[3]=1;
		  msg.axes[4]=-1.0;
		  break;
		case KEYCODE_I:	//tray
		  msg.buttons[3]=1;
		  msg.axes[5]=-1.0;
		  break;
		case KEYCODE_A:	//torso
		  msg.buttons[6]=1;
		  msg.axes[4]=-1.0;
		  break;
		case KEYCODE_X:
		  msg.buttons[6]=1;
		  msg.axes[5]=-1.0;
		  break;
		case KEYCODE_D:
		  msg.buttons[4]=1;
		  msg.axes[4]=-1.0;
		  break;
		case KEYCODE_V:
		  msg.buttons[4]=1;
		  msg.axes[5]=-1.0;
		  break;
		//case KEYCODE_B:	//head
		//  msg.buttons[2]=1;
		//  msg.axes[4]=-1.0;
		//  break;			  		  		  
		}
	}
	else		//platform_mode
	{
		msg.buttons[5]=1;	//deadman
		switch(c)
		{
		case KEYCODE_W:
		  msg.axes[1]=1.0;
		  break;
		case KEYCODE_S:
		  msg.axes[1]=-1.0;    
		  break;
		case KEYCODE_A:
		  msg.axes[0]=1.0;
		  break;
		case KEYCODE_D:
		  msg.axes[0]=-1.0;
		  break;
		case KEYCODE_Q:
		  msg.axes[2]=1.0;
		  break;
		case KEYCODE_E:
		  msg.axes[2]=-1.0;
		  break;
		}
	}


}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_publisher");
  ros::NodeHandle n;

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
	}            
      
	
	composeJoyMessage(msg, c);

    keyboard_pub.publish(msg);

    ros::spinOnce();
  }


  return 0;
}

