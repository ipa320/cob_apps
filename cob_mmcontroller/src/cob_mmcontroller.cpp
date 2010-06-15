


#include <ros/ros.h>
#include <cob_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <pr2_controller_msgs/JointTrajectoryControllerState.h>



class cob_mmcontroller
{
private:
  ros::Subscriber topicSub_cartvel_;
  ros::Subscriber topicSub__ControllerState_;
  ros::Publisher topicPub_armcmd_;


  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_Pause_;
  ros::ServiceServer srvServer_Stop_;
  //  ros::ServiceServer srvServer_Trajectoy_;

  KDL::Chain m_chain;


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
}

void cob_mmcontroller::topicCallback_CartVel(const geometry_msgs::Twist::ConstPtr& msg)
{
}

bool cob_mmcontroller::srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res )
{
  m_bInitialized = true;
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0,  -0.250    , 0.0     )));
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0,  -0.408    , 0.0     )));
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  -M_PI/2.0, -0.316    , 0.0     )));
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI/2.0,  0.0    , 0.0     )));
  m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0,  M_PI,  -0.327    , 0.0     )));
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

  while(nodeClass.n_.ok())
    {
      mmctrl.getExternalTwist();
      mmctrl.getTrajectoryTwist();
      mmctrl.updateArmCommands();
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
