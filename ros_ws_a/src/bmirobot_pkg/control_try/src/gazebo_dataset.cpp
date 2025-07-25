// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Moveit
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// moveit specific property
#include <moveit/move_group_interface/move_group_interface.h>
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

#include <chrono>
#include <iostream>

// bmirobot_msg
#include <bmirobot_msg/Robot_jointfd.h>
#include <control_try/robot.h>

std::vector<double> curJoints={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<double> curSpeed={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float seconds;    // time

double rhome_pos[9]={0.00, -0.0, 0, 0.2, 0.1 , 0, 0, 0, 0};   // home joint
double pose_angle[3] = {0.0,0.0,0.0};
Eigen::Matrix<double, 9, 9> rM2J;        // transform matrix
Eigen::Matrix<double, 9, 1> rJ_pst, rJ_vect, joint_states,joint_speed;
Eigen::VectorXd x_dot;
Eigen::Matrix<double, 3, 1> x;
// joint

int count = 0;

const robot_state::JointModelGroup* joint_model_group;

void matrixToEuler(Eigen::Matrix<double, 3, 3> rotationMatrix){
	double r11 = rotationMatrix(0,0);
	double r12 = rotationMatrix(0,1);
	double r13 = rotationMatrix(0,2);
    	double r21 = rotationMatrix(1,0);
    	double r22 = rotationMatrix(1,1);
    	double r23 = rotationMatrix(1,2);
    	double r31 = rotationMatrix(2,0);
   	double r32 = rotationMatrix(2,1);
    	double r33 = rotationMatrix(2,2);
    	// pitch
    	pose_angle[0] = asin(-r31);
    	// yawl roll
    	if (cos(pose_angle[0]) != 0) {
        pose_angle[1] = atan2(r21 / cos(pose_angle[0]), r11 / cos(pose_angle[0]));
        pose_angle[2] = atan2(r32 / cos(pose_angle[0]), r33 / cos(pose_angle[0]));
    } else {
        pose_angle[1] = 0;
        pose_angle[2] = 0;
    }
}

// eff_pose
void eff_pose(robot_state::RobotStatePtr kinematic_state){
	const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("right_link7");
	Eigen::Matrix<double, 3, 3> rotation = end_effector_state.rotation();
	x = end_effector_state.translation();
	matrixToEuler(rotation);
	ROS_INFO_STREAM("Translation:" << end_effector_state.translation() << "\n");
	ROS_INFO_STREAM("Rotation:" << pose_angle[0] << pose_angle[1]<<pose_angle[2]<<"\n");

}

// eff_jacobian
Eigen::MatrixXd eff_jacobian(const robot_state::JointModelGroup* joint_model_group, robot_state::RobotStatePtr kinematic_state){
	Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
	Eigen::MatrixXd jacobian;
 	kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
	ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
	return jacobian;
}

// eff_speed
void eff_speed(Eigen::MatrixXd J){
	  x_dot = J * Eigen::VectorXd::Map(curSpeed.data(),curSpeed.size());
	  ROS_INFO_STREAM("End effector velocity:" << x_dot.transpose());
	  
}


void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(int i= 0;i< 7;i++)
	{
		curJoints[i] = msg->position[i+11];
		curSpeed[i] = msg->velocity[i+11];
	}
	for(int i = 0;i < 9;i++){
		rJ_pst(i) = msg->position[i+9];
		rJ_vect(i) = msg->velocity[i+9];
	}
	//ROS_INFO("receive joint states...");
	auto now = std::chrono::system_clock::now();
	auto duration = now.time_since_epoch();
	seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();

	ROS_INFO_STREAM("joint_states1: \n" << joint_states << "\n");
	ROS_INFO_STREAM("joint_speed1: \n" << joint_speed << "\n");

	count = 1;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "moveit_data_generator");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::NodeHandle n;
	
	ros::spinOnce();
	
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model;
	kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	joint_model_group = kinematic_model->getJointModelGroup("right_arm");
	kinematic_state->setToDefaultValues();
	
	
    	ros::Subscriber sub = n.subscribe("/joint_states", 1000, jointStateCallback);
    	ros::Publisher pub = n.advertise<control_try::robot>("/robot_data",1000);
    	control_try::robot p;
    	ros::Rate rate(1);
    	while(ros::ok()){
    	if (count==1){
    	kinematic_state->setJointGroupPositions(joint_model_group, curJoints);
	eff_pose(kinematic_state);
    	Eigen::MatrixXd J=eff_jacobian(joint_model_group,kinematic_state);
    	eff_speed(J);
    	count = 0;
    	for(int i= 0;i< 9;i++)
	{
		p.joint_pos[i] = rJ_pst(i);
		p.joint_spd[i] = rJ_vect(i);
	}
	for(int i= 0;i< 3;i++)
	{
		p.eff_pose[i] = x(i);
		p.eff_pose[i+3] = pose_angle[i];
	}
	for(int i= 0;i< 6;i++)
	{
		p.eff_spd[i] = x_dot(i);
	}
	p.timestamp=seconds;
	pub.publish(p);
    	}
    	//rate.sleep();
    	ros::spinOnce();
    	}
}

