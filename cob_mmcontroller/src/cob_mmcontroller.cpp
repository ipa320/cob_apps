


#include <ros/ros.h>
#include <cob_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
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
#include <kdl/trajectory_segment.hpp>

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>


class cob_mmcontroller
{
private:
  ros::Subscriber topicSub_cartvel_;
  ros::Subscriber topicSub_ControllerState_;
  ros::Publisher topicPub_armcmd_;


  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_Pause_;
  ros::ServiceServer srvServer_Stop_;
  ros::ServiceServer srvServer_Trajectoy_;

  
  KDL::Chain m_chain;
  KDL::ChainFkSolverPos_recursive * fksolver;;
  KDL::ChainIkSolverVel_pinv * iksolver1v; //generalize pseudo inverse
  KDL::ChainIkSolverPos_NR * iksolver1;
  std::vector<double> m_CurrentConfig;
  std::vector<double> m_CurrentVels;

  KDL::Twist cmdTwist;
  bool m_bNewTwist;

  tf::TransformListener tflistener;
  tf::StampedTransform transform_arm_base;
public: 
  ros::NodeHandle nh_;


  cob_mmcontroller();
  void resetTwist();
  void updateArmCommands();

  void topicCallback_CartVel(const geometry_msgs::Twist::ConstPtr& msg);
  void topicCallback_ControllerState(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg);
  bool srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res );
  bool srvCallback_Pause(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res );
  bool srvCallback_Stop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res );
 // bool srvCallback_Trajectory(cob_mmcontroller::MoveTrajectory::Request &req, cob_mmcontroller::MoveTrajectory::Response &res );
    
  bool m_bInitialized;
  
};

cob_mmcontroller::cob_mmcontroller()
{
  topicPub_armcmd_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
  topicSub_cartvel_ = nh_.subscribe("cartvel_command", 1, &cob_mmcontroller::topicCallback_CartVel, this);
  topicSub_ControllerState_ = nh_.subscribe("controller_state", 1, &cob_mmcontroller::topicCallback_ControllerState, this);
 // srvServer_Trajectoy_ = nh_.advertiseService("MMController/MoveTrajectory", &cob_mmcontroller::srvCallback_Trajectory, this);
  srvServer_Init_ = nh_.advertiseService("MMController/Init", &cob_mmcontroller::srvCallback_Init, this);
  srvServer_Pause_ = nh_.advertiseService("MMController/Pause", &cob_mmcontroller::srvCallback_Pause, this);
  srvServer_Stop_ = nh_.advertiseService("MMController/Stop", &cob_mmcontroller::srvCallback_Stop, this);
  m_bInitialized = false;
  m_bNewTwist = false;
}

void cob_mmcontroller::resetTwist()
{
	cmdTwist = KDL::Twist();
	m_bNewTwist = false;
}
void cob_mmcontroller::updateArmCommands()
{
  if(m_bInitialized && m_bNewTwist)
    {
      unsigned int nj = m_chain.getNrOfJoints();
      KDL::JntArray jointpositions = KDL::JntArray(nj);
      KDL::JntArray qdot_out = KDL::JntArray(nj);
      for(unsigned int i = 0; i < nj; i++)
		jointpositions(i) = m_CurrentConfig[i];
      iksolver1v->CartToJnt(jointpositions, cmdTwist, qdot_out);
      trajectory_msgs::JointTrajectory msg;
      msg.header.stamp = ros::Time::now();
      msg.points.resize(1);
      msg.points[0].velocities.resize(nj);
      //std::cerr << "JointPositions: " << jointpositions(0) << " " <<  jointpositions(1) << " " <<  jointpositions(2) << " " <<  jointpositions(3) << " " <<  jointpositions(4) << " " << jointpositions(5) << " " <<  jointpositions(6) << "\n";
      //std::cerr << "Twist " << cmdTwist.vel.x() << " " << cmdTwist.vel.y() << " " << cmdTwist.vel.z() << "\n";
	  //std::cerr << "Vels: " << qdot_out(0) << " " <<  qdot_out(1) << " " <<  qdot_out(2) << " " <<  qdot_out(3) << " " <<  qdot_out(4) << " " <<  qdot_out(5) << " " <<  qdot_out(6) << "\n";
      for(unsigned int i = 0; i < nj; i++)
		msg.points[0].velocities[i] = qdot_out(i);
      topicPub_armcmd_.publish(msg);
      resetTwist();
    }
		       
}

void cob_mmcontroller::topicCallback_CartVel(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_DEBUG("Received Twist");

	try{
		tflistener.lookupTransform("arm_0_link", "base_link", ros::Time(0), transform_arm_base);
    }
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	KDL::Twist t;
	tf::TwistMsgToKDL(*msg, t);
	KDL::Frame frm_arm_base;
	tf::TransformTFToKDL(transform_arm_base, frm_arm_base);
	KDL::Twist t_base(frm_arm_base * t.vel, frm_arm_base * t.rot);
	cmdTwist += t_base/1000;
	m_bNewTwist = true;
  
}

void cob_mmcontroller::topicCallback_ControllerState(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
  m_CurrentConfig = msg->actual.positions;
  m_CurrentVels = msg->actual.velocities;

}

bool cob_mmcontroller::srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res )
{
  if(!m_bInitialized)
	{
	  ROS_INFO("Initializing MMController");
	  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0,  -0.250    , 0.0     )));
	  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
	  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0,  -0.408    , 0.0     )));
	  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
	  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0, -0.316    , 0.0     )));
	  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
	  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI,  -0.327    , 0.0     )));

	  m_CurrentConfig.resize(m_chain.getNrOfJoints());
	  m_CurrentVels.resize(m_chain.getNrOfJoints());

	  ROS_INFO("Creating Kinematic Solvers");
	  fksolver = new KDL::ChainFkSolverPos_recursive(m_chain);
	  iksolver1v = new  KDL::ChainIkSolverVel_pinv(m_chain,1e-6,200); //generalize pseudo inverse


	  m_bInitialized = true;
	}
  else
	{	
	  ROS_WARN("Allready initialized");
	}
  return true;
}

bool cob_mmcontroller::srvCallback_Pause(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res )
{
  return true;
}

bool cob_mmcontroller::srvCallback_Stop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res )
{
  return true;
}
/*
bool cob_mmcontroller::srvCallback_Trajectory(cob_mmcontroller::MoveTrajectory::Request &req, cob_mmcontroller::MoveTrajectory::Response &res )
{
	return true;
}
*/
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "cob_mmcontroller");
  cob_mmcontroller mmctrl;
  ros::Rate loop_rate(10); // Hz

  while(mmctrl.nh_.ok())
    {
	  //mmctrl.resetTwist();
      mmctrl.updateArmCommands();
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
