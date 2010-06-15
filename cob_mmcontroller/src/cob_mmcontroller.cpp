


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


class cob_mmcontroller
{
private:
  ros::Subscriber topicSub_cartvel_;
  ros::Subscriber topicSub_ControllerState_;
  ros::Publisher topicPub_armcmd_;


  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_Pause_;
  ros::ServiceServer srvServer_Stop_;
  //  ros::ServiceServer srvServer_Trajectoy_;

  
  KDL::Chain m_chain;
  KDL::ChainFkSolverPos_recursive * fksolver;;
  KDL::ChainIkSolverVel_pinv * iksolver1v; //generalize pseudo inverse
  KDL::ChainIkSolverPos_NR * iksolver1;
  std::vector<double> m_CurrentConfig;
  std::vector<double> m_CurrentVels;
public:
  ros::NodeHandle nh_;


  cob_mmcontroller();
  void getExternalTwist();
  void getTrajectoryTwist();
  void updateArmCommands();

  void topicCallback_CartVel(const geometry_msgs::Twist::ConstPtr& msg);
  void topicCallback_ControllerState(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg);
  bool srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res );
  bool srvCallback_Pause(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res );
  bool srvCallback_Stop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res );
  //  bool srvCallback_Trajectory(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res );
    
  bool m_bInitialized;
  
};

cob_mmcontroller::cob_mmcontroller()
{
  topicPub_armcmd_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
  topicSub_cartvel_ = nh_.subscribe("cartvel_command", 1, &cob_mmcontroller::topicCallback_CartVel, this);
  topicSub_ControllerState_ = nh_.subscribe("controller_state", 1, &cob_mmcontroller::topicCallback_ControllerState, this);
  srvServer_Init_ = nh_.advertiseService("MMController/Init", &cob_mmcontroller::srvCallback_Init, this);
  srvServer_Pause_ = nh_.advertiseService("MMController/Pause", &cob_mmcontroller::srvCallback_Pause, this);
  srvServer_Stop_ = nh_.advertiseService("MMController/Stop", &cob_mmcontroller::srvCallback_Stop, this);
  m_bInitialized = false;
}

void cob_mmcontroller::getExternalTwist()
{
}

void cob_mmcontroller::getTrajectoryTwist()
{
}

void cob_mmcontroller::updateArmCommands()
{
  if(m_bInitialized)
    {
      unsigned int nj = m_chain.getNrOfJoints();
      KDL::JntArray jointpositions = KDL::JntArray(nj);
      KDL::JntArray qdot_out = KDL::JntArray(nj);
      KDL::Twist myTwist;
      for(int i = 0; i < nj; i++)
	jointpositions(i) = m_CurrentConfig[i];
      iksolver1v->CartToJnt(jointpositions, myTwist, qdot_out);
      trajectory_msgs::JointTrajectory msg;
      msg.header.stamp = ros::Time::now();
      msg.points.resize(0);
      msg.points[0].velocities.resize(nj);
      for(int i = 0; i < nj; i++)
	msg.points[0].velocities[i] = qdot_out(i);
      topicPub_armcmd_.publish(msg);     
    }
		       
}

void cob_mmcontroller::topicCallback_CartVel(const geometry_msgs::Twist::ConstPtr& msg)
{
  
}

void cob_mmcontroller::topicCallback_ControllerState(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
  m_CurrentConfig = msg->actual.positions;
  m_CurrentVels = msg->actual.velocities;

}

bool cob_mmcontroller::srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res )
{
  
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0,  -0.250    , 0.0     )));
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0,  -0.408    , 0.0     )));
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0, -0.316    , 0.0     )));
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI,  -0.327    , 0.0     )));

  fksolver = new KDL::ChainFkSolverPos_recursive(m_chain);
  iksolver1v = new  KDL::ChainIkSolverVel_pinv(m_chain,1e-6,200); //generalize pseudo inverse


  m_bInitialized = true;
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


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "cob_mmcontroller");
  cob_mmcontroller mmctrl;
  ros::Rate loop_rate(50); // Hz

  while(mmctrl.nh_.ok())
    {
      mmctrl.getExternalTwist();
      mmctrl.getTrajectoryTwist();
      mmctrl.updateArmCommands();
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
