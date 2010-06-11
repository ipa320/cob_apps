
/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_apps
 * ROS package name: cob_simpletrajectories
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: June 2010
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

// ROS includes
#include <ros/ros.h>
#include <cob_simpletrajectories/MoveRelLin.h>
#include <cob_simpletrajectories/MoveDoorHandle.h>
//#include <cob_simpletrajectories/TrajectoryCmd.h>
#include <visualization_msgs/Marker.h>

// KDL includes
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>

#include <kdl/path_line.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/frames.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>




class SimpleTrajectories
{
public:
  SimpleTrajectories();
  ros::NodeHandle nh_;
private:


  ros::ServiceServer srvServer_MoveRelLin_;
  ros::ServiceServer srvServer_MoveDoorHandle_;
  ros::ServiceClient srvClient_MoveTrajectory;
  ros::Publisher marker_pub;

  std::vector<geometry_msgs::Point> TrajPoints;


  KDL::Trajectory_Segment * m_SyncMM_Trajectory ;

  int moveRelLin(double x, double y, double z, double timeS);
  int moveDoorHandle(double radius, double movingAngle, double timeS, bool dir, double delay);

  bool srvCallback_MoveRelLin(cob_simpletrajectories::MoveRelLin::Request &req, cob_simpletrajectories::MoveRelLin::Response &res );
  bool srvCallback_MoveDoorHandle(cob_simpletrajectories::MoveDoorHandle::Request &req, cob_simpletrajectories::MoveDoorHandle::Response &res );

  KDL::Frame GlobalToKDL(KDL::Frame in);
  KDL::Frame KDLToGlobal(KDL::Frame in);
  
};

SimpleTrajectories::SimpleTrajectories()
{
  srvServer_MoveRelLin_ = nh_.advertiseService("MoveRelLin", &SimpleTrajectories::srvCallback_MoveRelLin, this);
  srvServer_MoveDoorHandle_ = nh_.advertiseService("MoveDoorHandle", &SimpleTrajectories::srvCallback_MoveDoorHandle, this);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

}

bool SimpleTrajectories::srvCallback_MoveRelLin(cob_simpletrajectories::MoveRelLin::Request &req,
						cob_simpletrajectories::MoveRelLin::Response &res )
{
  moveRelLin(req.x, req.y, req.z, req.time);

  visualization_msgs::Marker points;
  points.header.frame_id = "/base_link";
  points.header.stamp = ros::Time::now();
  points.ns = "Trajectory";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.01;
  points.scale.y = 0.01;
  points.color.g = 1.0f;
  points.color.a = 1.0;
  
  points.points = TrajPoints;
  marker_pub.publish(points);

  return true;
}

bool SimpleTrajectories::srvCallback_MoveDoorHandle(cob_simpletrajectories::MoveDoorHandle::Request &req, cob_simpletrajectories::MoveDoorHandle::Response &res )
{
  moveDoorHandle(req.radius, req.movingAngle, req.time, req.direction, req.delay);

  visualization_msgs::Marker points;
  points.header.frame_id = "/base_link";
  points.header.stamp = ros::Time::now();
  points.ns = "Trajectory";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.01;
  points.scale.y = 0.01;
  points.color.g = 1.0f;
  points.color.a = 1.0;

  points.points = TrajPoints;
  marker_pub.publish(points);

  return true;
}

int SimpleTrajectories::moveRelLin(double x, double y, double z, double timeS)
{
  double r,p,yaw;

  //dhParameterArm
  KDL::Chain chain;
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0,  -0.250    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0,  -0.408    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0, -0.316    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI,  -0.327    , 0.0     )));


  KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);

  // read current position of Jointsvel
  KDL::Frame cartpos;
  unsigned int nj = chain.getNrOfJoints();
  KDL::JntArray jointpositions = KDL::JntArray(nj);
  
  //TODO: jointpositions = getJointPos(nj);
  for(int i = 0; i<7; i++)
    jointpositions(i) = 0.0;
 

  fksolver.JntToCart(jointpositions,cartpos);

  KDL::Vector vRelPos(x, y, z);
  KDL::Frame frameStart = KDLToGlobal(cartpos);
  KDL::Frame frameEnd = GlobalToKDL(KDL::Frame(frameStart.M, frameStart.p + vRelPos));

  KDL::RotationalInterpolation_SingleAxis rotInt_sa;
  KDL::Path_Line * myPath = new KDL::Path_Line(cartpos, frameEnd, &rotInt_sa, 1);
  KDL::VelocityProfile_Trap * myVelProfile = new KDL::VelocityProfile_Trap(150,70);// Parameter: maxvel, maxacc


  double m_SyncMM_Trajectory_TIME = timeS;
  m_SyncMM_Trajectory = new KDL::Trajectory_Segment(myPath, myVelProfile, m_SyncMM_Trajectory_TIME); //duration
  
  double resolution = 0.5;
  for (double i = 0.0; i < m_SyncMM_Trajectory_TIME; i+=resolution)
    {
      KDL::Frame pathpos = KDLToGlobal(m_SyncMM_Trajectory->Pos(i));
      geometry_msgs::Point p;
      p.x = pathpos.p.x();
      p.y = pathpos.p.y();
      p.z = pathpos.p.z();
      TrajPoints.push_back(p);
    }


  ROS_INFO("Trajectory generated");
  ROS_INFO("Length: %f   Duration: %f s", myPath->PathLength(), m_SyncMM_Trajectory->Duration());
  return 0;
}

int SimpleTrajectories::moveDoorHandle(double radius, double movingAngle, double timeS, bool dir, double delay)
{
  double m_dDelay = delay;

  KDL::Chain chain;
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0,  -0.250    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0,  -0.408    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0, -0.316    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI,  -0.327    , 0.0     )));

  KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);

  // read current position of Jointsvel
  KDL::Frame cartpos;
  unsigned int nj = chain.getNrOfJoints();
  KDL::JntArray jointpositions = KDL::JntArray(nj);

  //TODO: jointpositions = getJointPos(nj);
  for(int i = 0; i<7; i++)
    jointpositions(i) = 0.0;

  fksolver.JntToCart(jointpositions,cartpos);

  KDL::Frame frameStart = KDLToGlobal(cartpos);


  double kreisteil = 0.4;
  //calculate Startpos
  KDL::Frame newStartPos(cartpos.M, KDL::Vector(cartpos.p.x(),cartpos.p.y(),cartpos.p.z()));

  KDL::Rotation endPos;
  double phi = movingAngle*3.1;
  double R[3][3];
  double achse_x = 0;
  double achse_y = 0;
  if (dir)
    {
      double Rx[3][3] = { {1, 0, 0}, {0, cos(phi), -sin(phi)}, {0, sin(phi), cos(phi)} };
      for (int i=0;  i<3; i++)
	{
	  for(int j=0; j<3; j++)
	    {
	      R[i][j] = Rx[i][j];
	    }
	}
      achse_x = movingAngle/fabs(movingAngle)*radius*2;
      achse_y = 0;
    }
  else
    {
      double Ry[3][3] = { {cos(phi), 0, sin(phi)}, {0, 1, 0}, {-sin(phi), 0, cos(phi)} };
      for (int i=0;  i<3; i++)
	{
	  for(int j=0; j<3; j++)
	    {
	      R[i][j] = Ry[i][j];
	    }
	}
      achse_y = movingAngle/fabs(movingAngle)*radius*2;
      achse_x = 0;
    }

  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
	{
	  endPos.data[i*3+j] = 0;
	  for (int k=0; k<3; k++)
	    {
	      endPos.data[i*3+j] += R[i][k] * cartpos.M.data[k*3+j];
	    }
	}
    }

  KDL::RotationalInterpolation_SingleAxis rotInt_sa;
  KDL::Path_Circle * myCircPath = new KDL::Path_Circle(newStartPos, GlobalToKDL(KDL::Frame(frameStart.M, frameStart.p + KDL::Vector(0,0,-radius))).p, GlobalToKDL(KDL::Frame(frameStart.M, frameStart.p + KDL::Vector( achse_x, achse_y, 0))).p, endPos, M_PI*fabs(movingAngle), &rotInt_sa, 1);
  KDL::VelocityProfile_Trap * myVelProfile = new KDL::VelocityProfile_Trap(80,70);// Parameter: maxvel, maxacc

  double m_SyncMM_Trajectory_TIME = timeS;
  m_SyncMM_Trajectory = new KDL::Trajectory_Segment(myCircPath, myVelProfile, m_SyncMM_Trajectory_TIME); //duration
 
 double resolution = 0.5;
  for (double i = 0.0; i < m_SyncMM_Trajectory_TIME; i+=resolution)
    {
      KDL::Frame pathpos = KDLToGlobal(m_SyncMM_Trajectory->Pos(i));
      geometry_msgs::Point p;
      p.x = pathpos.p.x();
      p.y = pathpos.p.y();
      p.z = pathpos.p.z();
      TrajPoints.push_back(p);
    }


  ROS_INFO("Trajectory generated");
  ROS_INFO("Length: %f   Duration: %f s", myCircPath->PathLength(), m_SyncMM_Trajectory->Duration());
  return 0;
}



KDL::Frame SimpleTrajectories::KDLToGlobal(KDL::Frame in)
{
  // Create Transformation Matrix from ArmBase to Robot Base
  /*  KDL::Rotation T_Rot_AB_RB(M4_ArmBase[0][0],M4_ArmBase[1][0],M4_ArmBase[2][0],
          M4_ArmBase[0][1],M4_ArmBase[1][1],M4_ArmBase[2][1],
	  M4_ArmBase[0][2],M4_ArmBase[1][2],M4_ArmBase[2][2]);*/
  /*KDL::Rotation T_Rot_AB_RB(-0.683013,0.258819,-0.683013,
        0.183013,0.965926,0.183013,
	0.707107,0.00000,-0.707107);*/
  KDL::Rotation T_Rot_AB_RB(-0.683013,0.258819,-0.683013,
			    0.183013,0.965926,0.183013,
			    0.707107,0.00000,-0.707107);
  // Turn T_Rot 180° around x-axis to adapt KDL to COb specification:
  //T_Rot_AB_RB = T_Rot_AB_RB * KDL::Rotation::EulerZYX(0, 0, M_PI);
  // translational part of transformation
  //  KDL::Vector T_Trans(M4_ArmBase[3][0],M4_ArmBase[3][1],M4_ArmBase[3][2]);
  //  KDL::Vector T_Trans(0,-0.115,0.76);
  KDL::Vector T_Trans(0.045,-0.005,0.83637);
  // KDL::Vector T_Trans(0,-0.115,0.63);
  // create transformation frame
  KDL::Frame T_AB_RB(T_Rot_AB_RB, T_Trans);
  return T_AB_RB * in;
}

KDL::Frame SimpleTrajectories::GlobalToKDL(KDL::Frame in)
{

  // Create Transformation Matrix from ArmBase to Robot Base
  /*  KDL::Rotation T_Rot_AB_RB(M4_ArmBase[0][0],M4_ArmBase[1][0],M4_ArmBase[2][0],
          M4_ArmBase[0][1],M4_ArmBase[1][1],M4_ArmBase[2][1],
	  M4_ArmBase[0][2],M4_ArmBase[1][2],M4_ArmBase[2][2]);*/
  /*
  KDL::Rotation T_Rot_AB_RB(-0.683013,0.258819,-0.683013,
      0.183013,0.965926,0.183013,
      0.707107,0.00000,-0.707107);*/
  KDL::Rotation T_Rot_AB_RB(-0.683013,0.258819,-0.683013,
			    0.183013,0.965926,0.183013,
			    0.707107,0.00000,-0.707107);
  // Turn T_Rot 180° around x-axis to adapt KDL to COb specification:
  //T_Rot_AB_RB = T_Rot_AB_RB * KDL::Rotation::EulerZYX(0, 0, M_PI);
  // translational part of transformation
  //KDL::Vector T_Trans(0,0,0.83637);
  //  KDL::Vector T_Trans(0,-0.115,0.76);
  KDL::Vector T_Trans(0.045,-0.005,0.83637);
  // create transformation frame
  //  T_Trans.ReverseSign();
  KDL::Frame T_AB_RB(T_Rot_AB_RB, T_Trans);
  return T_AB_RB.Inverse() * in;
}


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "cob_simpletrajectory");
  SimpleTrajectories st;
  ros::Rate loop_rate(5); // Hz
  while(st.nh_.ok())
    {
      ros::spinOnce();
      usleep(200000);
    }
  
  return 0;
}



