/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
 *  All rights reserved.
 *
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Tests the grasp generator filter
*/
 
// ROS 
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Grasp
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_filter.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Baxter specific properties
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_simple_grasps/custom_environment2.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>

// moveit specific properties
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>

#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include "bmirobot_visual_servo/VisualServoStatus.h"

#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#define USE_SERVO 0
   std::vector<moveit_msgs::Grasp> possible_grasps_filtered;

   std::string targetKey;

	double joint_states[9];
	int firstJointF=0;
	int waitCount=0;
	int maxCount = 10;
	bool clearF = false;
	bool exitF = false;
	int tryCount = 0;
namespace moveit_simple_grasps
{
    // Table dimensions
    static const double TABLE_HEIGHT = .92;
    static const double TABLE_WIDTH = .85;
    static const double TABLE_DEPTH = .47;
    static const double TABLE_X = 0.66;
    static const double TABLE_Y = 0;
    static const double TABLE_Z = -0.9/2+0.01;

    static const double BLOCK_SIZE = 0.04;

    class GraspGeneratorTest
    {
    	private:
  	// A shared node handle
      	ros::NodeHandle nh_;

  	// Grasp generator
      	moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  	// Tool for visualizing things in Rviz
      	moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  	// Grasp filter
      	moveit_simple_grasps::GraspFilterPtr grasp_filter_;

  	// data for generating grasps
      	moveit_simple_grasps::GraspData grasp_data_;

  	// Shared planning scene (load once for everything)
      	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  	// which baxter arm are we using
      	std::string arm_;
      	std::string ee_group_name_;
      	std::string planning_group_name_;
    public:

		// Constructor
		GraspGeneratorTest()
		:nh_("~")
		{
		// Get arm info from param server
			nh_.param("arm", arm_, std::string("right"));
			nh_.param("ee_group_name", ee_group_name_, std::string(arm_ + "_hand"));
			planning_group_name_ = "arm1";

			ROS_INFO_STREAM_NAMED("test","Arm: " << arm_);
			ROS_INFO_STREAM_NAMED("test","End Effector: " << ee_group_name_);
			ROS_INFO_STREAM_NAMED("test","Planning Group: " << planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load grasp datapu
        if (!grasp_data_.loadRobotGraspData(nh_, ee_group_name_))
        {
          	ros::shutdown();
          	ROS_INFO_STREAM_NAMED("test","error");
        }

        ROS_INFO_STREAM_NAMED("temp","load planning scene");
    	// ---------------------------------------------------------------------------------------------
    	// Load planning scene to share
        planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

        ROS_INFO_STREAM_NAMED("temp","get the planning name:="<<planning_scene_monitor_->getName ());
    	//
//---------------------------------------------------------------------------------------------
    	// Load the Robot Viz Tools for publishing to Rviz
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(grasp_data_.base_link_, "/rviz_visual_tools", planning_scene_monitor_));
		//ROS_ERROR_STREAM_NAMED("temp","joint list type is not type array???1");
        const robot_model::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup(grasp_data_.ee_group_);

		//visual_tools_->publishTests();
        visual_tools_->setLifetime(40.0);
        visual_tools_->loadEEMarker(ee_jmg);
        //visual_tools_->setFloorToBaseHeight(0);

    	// Clear out old collision objects just because
        visual_tools_->removeAllCollisionObjects();
		//ROS_ERROR_STREAM_NAMED("temp","joint list type is not type array???2");
    	// Create a collision table for fun
        visual_tools_->publishCollisionTable(TABLE_X, TABLE_Y, 0, TABLE_WIDTH, TABLE_HEIGHT, TABLE_DEPTH, "table");
		//ROS_ERROR_STREAM_NAMED("temp","joint list type is not type array???3");
    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
        simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) );

    // ---------------------------------------------------------------------------------------------
    // Load grasp filter
        robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
        grasp_filter_.reset(new moveit_simple_grasps::GraspFilter(robot_state, visual_tools_) );


 		ROS_INFO_STREAM_NAMED("test","2");
    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects
    	}

		void generateGrasps(geometry_msgs::Pose object_pose)
		{
			// Clear out old collision objects just because
			visual_tools_->removeAllCollisionObjects();
			//ROS_ERROR_STREAM_NAMED("temp","joint list type is not type array???2");
			// Create a collision table for fun
			visual_tools_->publishCollisionTable(TABLE_X, TABLE_Y, 0, TABLE_WIDTH, TABLE_HEIGHT, TABLE_DEPTH, "table");

			std::vector<moveit_msgs::Grasp> possible_grasps;
			std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions; // save each grasps ik solution for visualization
			possible_grasps_filtered.clear();
			std::vector<moveit_msgs::Grasp>().swap(possible_grasps_filtered);
			// Remove randomness when we are only running one test
			//if (num_tests == 1)
			//  generateTestObject(object_pose);
			//else
			//generateRandomObject(object_pose);

			// Show the block
			//visual_tools_->publishBlock(object_pose, rviz_visual_tools::BLUE, BLOCK_SIZE);
			// visual_tools_->publishCollisionBlock(object_pose,"target object", BLOCK_SIZE, rviz_visual_tools::BLUE);
			possible_grasps.clear();
			std::vector<moveit_msgs::Grasp>().swap(possible_grasps);
			ik_solutions.clear();
			std::vector<trajectory_msgs::JointTrajectoryPoint>().swap(ik_solutions);
			ROS_INFO("simple_grasps_: ");

			// Generate set of grasps for one object
			simple_grasps_->generateBlockGrasps( object_pose, grasp_data_, possible_grasps);
			ROS_INFO("simple_grasps_2: ");      
			const robot_model::JointModelGroup* ee_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(grasp_data_.ee_group_);
			//visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg);

			// Filter the grasp for only the ones that are reachable
			bool filter_pregrasps = true;
			grasp_filter_->filterGrasps(possible_grasps, ik_solutions, filter_pregrasps, grasp_data_.ee_parent_link_, planning_group_name_);

      		// Visualize them

			possible_grasps_filtered.push_back(possible_grasps[0]);
			visual_tools_->publishAnimatedGrasps(possible_grasps_filtered, ee_jmg);
			//const robot_model::JointModelGroup* arm_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(planning_group_name_);
			//visual_tools_->publishIKSolutions(ik_solutions, arm_jmg, 0.25);
		}
	}; // end of class
} // namespace



moveit_simple_grasps::GraspGeneratorTest* tester;
int count;
geometry_msgs::Pose object_pose;
int runF ;
ros::Publisher vis_pub,vis_pub_place,vis_pub_place2,grasp_joint_pub,arm_joint_pub;
float curJoints[7];
void grasper_control(double a)
{
	std_msgs::Float64MultiArray msg;
  msg.data.push_back(a);
  msg.data.push_back(-a);
  //msg.data[1] = 0.5;

  while(grasp_joint_pub.getNumSubscribers() == 0)
  {
    ROS_INFO("wait for grasper joint sub");
    sleep(1.0);
  }

  grasp_joint_pub.publish(msg);

  double err = 1000;
  while(err>0.2||firstJointF==0)
  {
	err= fabs(joint_states[0]-a)+fabs(joint_states[1]+a);	
    sleep(0.1);
    ROS_INFO("grasp joint err%f\n",err);
  }
  sleep(1.0);
}


void open_grasp() 
{
	grasper_control(0.50);
	ROS_INFO("open_grasp ok");
}


void close_grasp()
{
	grasper_control(0.2);
	ROS_INFO("close_grasp ok");
}

void move_arm(float jointS[7])
{
	std_msgs::Float64MultiArray msg;
	for(int i = 0;i< 7;i++)
	{
		msg.data.push_back(jointS[i]);
	}
	
	//msg.data.push_back(0.30);
	//msg.data[1] = 0.5;

		while(arm_joint_pub.getNumSubscribers() == 0)
	{
		ROS_INFO("wait for mover joint sub");
		sleep(1.0);

	}
	arm_joint_pub.publish(msg);

	double err = 1000;
	while(err>0.11||firstJointF==0)
	{
		err = 0;
		for(int i = 0;i<7;i++)
		{
			err+= fabs(joint_states[i+2]-jointS[i]);
		}
		//ros::spinOnce();
		sleep(0.1);
		ROS_INFO("arm joint err%f\n",err);
	}
	sleep(1.0);
}

void home_arm()
{
  float joints[]={0,0,0,0,0,0,0};
  move_arm(joints);
}


int load_controller_fun(ros::ServiceClient& CM_load_client,char * loadC)
{
    controller_manager_msgs::LoadController LC_srv;
    LC_srv.request.name = loadC;
    if(CM_load_client.call(LC_srv))
    {
      if((bool)LC_srv.response.ok == false)
      {
        ROS_ERROR("Failed to load cart_controller");
    //return 1;
      }
    }
    else
    {
      ROS_ERROR("Failed to call service load_controller");
    return 1;
    }
}


int switch_controller_fun(ros::ServiceClient& CM_switch_client,char * startC,char* stopC)
{
	ROS_INFO("Visualizing plan 3 (pose goal)");

    //std::cout << "sssssswitch";

    controller_manager_msgs::SwitchController SC_srv;
    SC_srv.request.start_controllers = {startC};
    SC_srv.request.stop_controllers = {stopC};
    SC_srv.request.strictness=1;
    if(CM_switch_client.call(SC_srv))
    {
      	if((bool)SC_srv.response.ok == false)
      	{
        	ROS_ERROR("Failed to load_trajectory_controller");
    		return 1;
      	}
      	else
		{
			ROS_ERROR("ok to load_trajectory_controller");
		}
    }
    else
    {
      	ROS_ERROR("ok to call service load_controller");
    	return 1;
    }
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(int i= 0;i< 7;i++)
	{
		curJoints[i] = msg->position[2+i];
	}
	for(int i = 0;i < 9;i++)
		joint_states[i] = msg->position[i];
	//ROS_INFO("receive joint states...");
	firstJointF = 1;
}

void objectFindCallback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
{
  	tf::TransformListener listener;
  	//tf::Transform transform;
  	//ros::Duration(1.0).sleep();
  	tf::StampedTransform transform;
	//ROS_INFO("get new object pose");
	try{
		if(0==count)
		{
			ros::Duration(3.0).sleep(); 
			listener.lookupTransform("/odom_combined", "/camera_depth_optical_frame",  
			ros::Time(0), transform);
			bool findF = false;
			for(int i = 0;i < msg->objects.size();i++)
			{
				ROS_INFO("seek object id ::%d %d,%s",waitCount,i,msg->objects[i].type.key.c_str());
				if(msg->objects[i].type.key.c_str()!=targetKey)
					continue;
				findF = true;
			}
			if(findF == false)
			{
				waitCount ++;
			}
			else
			{
				waitCount = 0;
				clearF = false;
			}
			for(int i = 0;i < msg->objects.size();i++)
			{
				ROS_INFO("%d,%s",i,msg->objects[i].type.key.c_str());
				tf::Vector3 pin;
				pin[0] = msg->objects[i].pose.pose.pose.position.x;
				pin[1] = msg->objects[i].pose.pose.pose.position.y;
				pin[2] = msg->objects[i].pose.pose.pose.position.z;

				if(sqrt(pin[0]*pin[0]+pin[1]*pin[1]+pin[2]*pin[2]) > 1.2)
					continue;
				if((msg->objects[i].type.key.c_str()!=targetKey)&&(findF==true))
					continue;
				if(findF == false&&waitCount<maxCount)
					continue;
				if(clearF==true&&waitCount>=maxCount)
				{
					exitF = true;
					count++;
					break;
				}
				//if(findF == false)
					//clearF = true;
				//findF = true;
				//tf::Vector3 pin;
				//pin[0] = msg->objects[i].pose.pose.pose.position.x;
				//pin[1] = msg->objects[i].pose.pose.pose.position.y;
				//pin[2] = msg->objects[i].pose.pose.pose.position.z;
				tf::Vector3 pout;
				pout = transform(pin);
				tf::Transform object_rgbd2,object_odom,realobject_object,realobject_odom;
				object_rgbd2.getOrigin ().setValue(pin[0],pin[1],pin[2]); 
				tf::Quaternion temp(msg->objects[i].pose.pose.pose.orientation.x,msg->objects[i].pose.pose.pose.orientation.y,msg->objects[i].pose.pose.pose.orientation.z,msg->objects[i].pose.pose.pose.orientation.w);
				object_rgbd2.setRotation (temp);
				object_odom = transform*object_rgbd2;
				if(msg->objects[i].type.key=="e6e639a3801de7fd755368fe5d003a47")
					realobject_object.getOrigin ().setValue(0.0335,0.0335,0.042); 
				else if (msg->objects[i].type.key=="17decf071d3a100d375b0bf25f000706")
					realobject_object.getOrigin ().setValue(0,0,0.042); 
				else if(msg->objects[i].type.key=="49e13f9cb60968b7c16b63c98000099b")
					realobject_object.getOrigin ().setValue(0,0,0.042);

				//realobject_object.getOrigin ().setValue(0.0335,0.0335,0.042); 
				realobject_odom = object_odom*realobject_object;

				object_pose.position.x = realobject_odom.getOrigin ().x();
				object_pose.position.y = realobject_odom.getOrigin ().y();
				object_pose.position.z = realobject_odom.getOrigin ().z();

				object_pose.orientation.x = 0;
				object_pose.orientation.y = 0;
				object_pose.orientation.z = 0;
				object_pose.orientation.w = 1;
				ROS_INFO("I heard: [%s %f %f %f]", msg->objects[i].pose.header.frame_id.c_str(),object_pose.position.x,object_pose.position.y,object_pose.position.z);
			
				visualization_msgs::Marker marker;
				marker.header.frame_id = "odom_combined";
				marker.header.stamp = ros::Time();
				marker.ns = "my_namespace";
				marker.id = 0;
				marker.type = visualization_msgs::Marker::CYLINDER;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x =  object_pose.position.x;
				marker.pose.position.y = object_pose.position.y;
				marker.pose.position.z = object_pose.position.z;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = 0.35;
				marker.scale.y = 0.35;
				marker.scale.z = 0.01;
				marker.color.a = 0.3; // Don't forget to set the alpha!
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
				//only if using a MESH_RESOURCE marker type:
				//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
				vis_pub.publish( marker );

				marker.header.frame_id = "odom_combined";
				marker.header.stamp = ros::Time();
				marker.ns = "my_namespace";
				marker.id = 0;
				marker.type = visualization_msgs::Marker::CYLINDER;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x =  0.5;
				marker.pose.position.y = -0.1;
				marker.pose.position.z = object_pose.position.z;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = 0.35;
				marker.scale.y = 0.35;
				marker.scale.z = 0.01;
				marker.color.a = 0.3; // Don't forget to set the alpha!
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
				//only if using a MESH_RESOURCE marker type:
				//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
				vis_pub_place.publish( marker );

				marker.header.frame_id = "odom_combined";
				marker.header.stamp = ros::Time();
				marker.ns = "my_namespace";
				marker.id = 0;
				marker.type = visualization_msgs::Marker::CYLINDER;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x =  0.5;
				marker.pose.position.y = 0.2;
				marker.pose.position.z = object_pose.position.z;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = 0.35;
				marker.scale.y = 0.35;
				marker.scale.z = 0.01;
				marker.color.a = 0.3; // Don't forget to set the alpha!
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
				//only if using a MESH_RESOURCE marker type:
				//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
				vis_pub_place2.publish( marker );
				runF++;
				count = 1;
				//ros::Duration(10.0).sleep(); 
				break;
			}
		
		}
	}
	catch (tf::TransformException ex){
		ROS_ERROR("aaaaa%s",ex.what());
		//ros::Duration(1.0).sleep();
	}


}
class collisionObjectAdder
    {
    protected:
         ros::NodeHandle nh;
          ros::Publisher planning_scene_diff_publisher; 
          ros::ServiceClient planning_scene_diff_client;
         //ros::Publisher planning_scene_diff_pub;

    public:
       collisionObjectAdder() 
       {
            //add_collision_object_pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1000);
        	planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("move_group/monitored_planning_scene", 1);
       	 	planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
       	 	planning_scene_diff_client.waitForExistence();
        	while(planning_scene_diff_publisher.getNumSubscribers() < 1)
        	{
          		ROS_INFO("Wait for Subscriber.");
          		ros::WallDuration sleep_t(0.5);
          		sleep_t.sleep();
        	}
       }
       void addCollisionObject()
       {
		    moveit_msgs::AttachedCollisionObject attached_object;
			attached_object.link_name = "link8";
			/* The header must contain a valid TF frame*/
			attached_object.object.header.frame_id = "odom_combined";
			/* The id of the object */
			attached_object.object.id = "table";

			/* A default pose */
			geometry_msgs::Pose pose;
			pose.orientation.w = 1.0;
			pose.position.x = 0.90;
			pose.position.y = 0.18;
			pose.position.z = 0.02;
			/* Define a box to be attached */
			shape_msgs::SolidPrimitive primitive;
			primitive.type = primitive.BOX;
			primitive.dimensions.resize(3);
			primitive.dimensions[0] = 1;
			primitive.dimensions[1] = 0.8;
			primitive.dimensions[2] = 0.01;

			attached_object.object.primitives.push_back(primitive);
			attached_object.object.primitive_poses.push_back(pose);
			ROS_INFO("Adding the object into the world at the location of the right wrist.");
			attached_object.object.operation = attached_object.object.ADD;
			moveit_msgs::PlanningScene planning_scene;
			planning_scene.world.collision_objects.push_back(attached_object.object);

			planning_scene.is_diff = true;
			//planning_scene_diff_publisher.publish(planning_scene);
			moveit_msgs::ApplyPlanningScene srv;
			srv.request.scene = planning_scene;
			planning_scene_diff_client.call(srv);

			ROS_INFO("Collision object published");
       }
    };

void place_object(int dir)
{
	float joints[][8]={  {	0,		0,		0,		0,		0,		0,		0,		0}
						,{	0.00,	-0.00,	0.00,	-0.1,	0.00,	0.00,	-0.00,	-0.00}
						,{	0.00,	-0.00,	0.00,	-0.2,	0.00,	-0.05,	-0.00,	-0.00}
						,{	0.00,	-0.00,	0.00,	0.30,	0.00,	-0.16,	-0.00,	-0.00}
						};
	int num = 4;
	for(int i = 2;i<num;i++)
	{
		move_arm(joints[dir==1?i:num-i-1]);
		sleep(1);
	}
}

int main(int argc, char *argv[])
{
    int num_tests = 1;
    runF =0;
    count =0;
    ros::init(argc, argv, "grasp_generator_test");

    // Allow the action server to recieve and send ros messages
    ros::AsyncSpinner spinner(2);
    spinner.start();
        ros::Publisher visual_target_pub_;
    // Check for verbose flag
    bool verbose = true;
    if (argc > 1)
    {
        
    }

    ros::NodeHandle n;

    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    vis_pub_place = n.advertise<visualization_msgs::Marker>( "visualization_marker_place", 0 ); 
    vis_pub_place2 = n.advertise<visualization_msgs::Marker>( "visualization_marker_place2", 0 ); 
    grasp_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "bmirobot/group_hand_controller/command", 0 ); 
    arm_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "bmirobot/group_controller/command", 0 ); 

    ros::spinOnce();
    ros::Subscriber sub = n.subscribe("recognized_objects", 1000, objectFindCallback);
    ros::Subscriber sub2 = n.subscribe("bmirobot/joint_states", 1000, jointStateCallback);
    tester = new moveit_simple_grasps::GraspGeneratorTest();
    //ros::spin();
    ros::Rate loop_rate(10);
    visual_target_pub_ = n.advertise<geometry_msgs::PoseStamped>("visual_servo_target_pose", 1);

    moveit::planning_interface::MoveGroup group("arm1");
    group.setPlannerId("RRTConnectkConfigDefault");
    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // (Optional) Create a publisher for visualizing plans in Rviz.
    ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ros::ServiceClient CM_load_client = n.serviceClient<controller_manager_msgs::LoadController>("/bmirobot/controller_manager/load_controller");
    ros::ServiceClient CM_switch_client = n.serviceClient<controller_manager_msgs::SwitchController>("/bmirobot/controller_manager/switch_controller");
    ros::ServiceClient CM_list_client = n.serviceClient<controller_manager_msgs::ListControllers>("/bmirobot/controller_manager/list_controller");

    ros::ServiceClient visual_servo_client = n.serviceClient<bmirobot_visual_servo::VisualServoStatus>("/visual_servo_status");
 

    load_controller_fun(CM_load_client,"group_controller");
    load_controller_fun(CM_load_client,"cart_controller");
    load_controller_fun(CM_load_client,"trajectory_controller");


    while (ros::ok())
    {
        ros::spinOnce();
        if(runF==1)
        {
        open_grasp();

        tester->generateGrasps(object_pose);
        geometry_msgs::PoseStamped pregrasp_pose,grasp_pose;
        //find higest quality grasp
        float maxscore = 0;
        int maxIndex=0;
        for(int i =0;i<possible_grasps_filtered.size();i++)
        {
            if(possible_grasps_filtered[i].grasp_quality>maxscore)
            {
                maxscore = possible_grasps_filtered[i].grasp_quality;
                maxIndex=i;
            }
        }
        ROS_INFO("Visualizing plan 1 (pose goal)");
        grasp_pose =possible_grasps_filtered[maxIndex].grasp_pose;
        const std::string ee = "link8";

    	pregrasp_pose =  moveit_simple_grasps::SimpleGrasps::getPreGraspPose(possible_grasps_filtered[maxIndex],ee);
		//pregrasp_pose = grasp_pose;
		geometry_msgs::Pose target_pose1;
		target_pose1.orientation.x = pregrasp_pose.pose.orientation.x;
		target_pose1.orientation.y = pregrasp_pose.pose.orientation.y;
		target_pose1.orientation.z = pregrasp_pose.pose.orientation.z;
		target_pose1.orientation.w = pregrasp_pose.pose.orientation.w;
		target_pose1.position.x = pregrasp_pose.pose.position.x;
		target_pose1.position.y =  pregrasp_pose.pose.position.y;
		target_pose1.position.z =  pregrasp_pose.pose.position.z;
		
		group.setPoseTarget(target_pose1);


    	moveit::planning_interface::MoveGroup::Plan my_plan;
    	bool success = static_cast<bool>(group.plan(my_plan));

    	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
		/* Sleep to give Rviz time to visualize the plan. */
		//sleep(5.0);

		// Visualizing plans
		// ^^^^^^^^^^^^^^^^^
		// Now that we have a plan we can visualize it in Rviz.  This is not
		// necessary because the group.plan() call we made above did this
		// automatically.  But explicitly publishing plans is useful in cases that we
		// want to visualize a previously created plan.
		if (1)
		{
		  ROS_INFO("Visualizing plan 1 (again)");    
		  display_trajectory.trajectory_start = my_plan.start_state_;
		  display_trajectory.trajectory.push_back(my_plan.trajectory_);
		  display_publisher.publish(display_trajectory);
		  /* Sleep to give Rviz time to visualize the plan. */
		  //sleep(3.0);
		}

        switch_controller_fun(CM_switch_client,"trajectory_controller","cart_controller");
        switch_controller_fun(CM_switch_client,"trajectory_controller","group_controller");
        group.move();
        sleep(1.0);

        target_pose1.orientation.x = grasp_pose.pose.orientation.x;
        target_pose1.orientation.y = grasp_pose.pose.orientation.y;
        target_pose1.orientation.z = grasp_pose.pose.orientation.z;
        target_pose1.orientation.w = grasp_pose.pose.orientation.w;
        target_pose1.position.x    = grasp_pose.pose.position.x;
        target_pose1.position.y    = grasp_pose.pose.position.y;
        target_pose1.position.z    = grasp_pose.pose.position.z;
        group.setPoseTarget(target_pose1);


        group.plan(my_plan);

        ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    

        if (0)
        {
        ROS_INFO("Visualizing plan 1 (again)");    
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(3.0);
        }

        switch_controller_fun(CM_switch_client,"trajectory_controller","cart_controller");
        switch_controller_fun(CM_switch_client,"trajectory_controller","group_controller");
        group.move();
        sleep(1.0);

        close_grasp();

        //sleep(5.0);
        if(clearF == false)
        {		

            target_pose1.orientation.x = -0.016;
            target_pose1.orientation.y = 0.016;
            target_pose1.orientation.z = 0.072;
            target_pose1.orientation.w = 0.997;
            target_pose1.position.x =  0.204;
            target_pose1.position.y =  -0.254;
            target_pose1.position.z =  0.123;
            
            group.setPoseTarget(target_pose1);
            group.plan(my_plan);
            switch_controller_fun(CM_switch_client,"trajectory_controller","cart_controller");
            switch_controller_fun(CM_switch_client,"trajectory_controller","group_controller");
            ros::spinOnce();
            group.move();
            sleep(4.0);

            //switch_controller_fun(CM_switch_client,"group_controller","trajectory_controller");
            //switch_controller_fun(CM_switch_client,"group_controller","cart_controller");
            //place_object(1);
            //sleep(7.0);
            //go to the put down place.vision servo or just the joint control
        
        // open the grasp
            open_grasp();
            
            runF++;
            exit(0);

        }
        else
        {
            float joints3[]={0,0,0.4,0,0,0,0};
            move_arm(joints3);
            open_grasp();
            runF = 0;
            count = 0;
        }
        loop_rate.sleep();
    }

    return 0;
}
