///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * Author: Wim Meeussen
 */


//#include <bmirobot_controllers/IK_controller.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <kdl/frames.hpp>
#include <message_filters/subscriber.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


namespace bmirobot_control
{
  class IK_controller : public controller_interface::Controller<hardware_interface::PositionJointInterface>
  {
  private:
    ros::Subscriber MT_cmdsub;
    ros::NodeHandle node;
    std::vector<hardware_interface::JointHandle>  joints;

  public:
    IK_controller()
    {
      ROS_INFO("setp1 data");
    }
    
    ~IK_controller()
    {

    }

    bool init(hardware_interface::PositionJointInterface* hw,ros::NodeHandle &nh)
    {
      node=nh;
  	  MT_cmdsub = node.subscribe("command",1,&IK_controller::FdctrmsgCB,this);

      ROS_INFO("init data");

      return true;
    }
    void starting(const ros::Time& time)
    {


    }
    void update(const ros::Time& time, const ros::Duration& period)
    {
     ROS_INFO("hello");

    }
    void stopping(const ros::Time& time)
    {


    }
    
    void FdctrmsgCB(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
    {
        ROS_INFO("new data here");


    }   



  };

};


PLUGINLIB_EXPORT_CLASS(
    bmirobot_control::IK_controller,
    controller_interface::ControllerBase
)

