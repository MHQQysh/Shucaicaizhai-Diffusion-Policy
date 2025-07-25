#ifndef __BARRETT_CONTROLLERS_CALIBRATION_CONTROLLER_H
#define __BARRETT_CONTROLLERS_CALIBRATION_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>
#include <control_toolbox/pid.h>
#include <kdl/velocityprofile_trap.hpp>
#include <hardware_interface/joint_command_interface.h>

namespace bmirobot_control 
{

  class IK_Controller : public controller_interface::Controller<hardware_interface::PositionJointInterface>
  {
  public:

    IK_Controller();
    
    ~IK_Controller();

    bool init(hardware_interface::PositionJointInterface* hw,ros::NodeHandle &nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    void stopping(const ros::Time& time);
  private:
    ros::Subscriber MT_cmdsub;


  };

}

#endif // ifndef __BARRETT_CONTROLLERS_CALIBRATION_CONTROLLER_H
