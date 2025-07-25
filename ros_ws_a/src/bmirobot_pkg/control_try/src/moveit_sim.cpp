// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// moveit
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

#include <unistd.h>
#include<sstream>
#include<vector>
#include<stdio.h>
#include<termio.h>
// IK joint_state

int count=0;  // arm_joint
int hand_count = 0; // hand

std::vector<double> joint_positions = {0.0, -M_PI/4, 0.0, -M_PI/2, 0.0, M_PI/3, 0.0};
std::vector<double> hand_positions = {0.0, -M_PI/4};

void jointStateCallback(const std_msgs::Float64MultiArray& msg)
{
	for(int i=0;i<7;i++)
	{
	joint_positions[i] = msg.data.at(i);
	}
	count = 1;
}

// hand
void handStateCallback(const std_msgs::Float64MultiArray& msg)
{
	for(int i=0;i<2;i++)
	{
	hand_positions[i] = msg.data.at(i);
	}
	hand_count = 1;
}

// keyboard
char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return buf;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "moveit_sim");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::NodeHandle n;
	
	ros::spinOnce();
	
	// joint sim  current joint
	static const std::string PLANNING_GROUP = "right_arm";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::MoveGroupInterface move_group1("right_hand");
	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	moveit::core::RobotStatePtr current_state1 = move_group1.getCurrentState();
	std::vector<double> joint_group_positions;
	std::vector<double> joint_group_positions1;
	current_state->copyJointGroupPositions(move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP), joint_group_positions);
	current_state1->copyJointGroupPositions(move_group1.getCurrentState()->getJointModelGroup("right_hand"), joint_group_positions1);
        
        // ros
    	ros::Subscriber sub = n.subscribe("/bmirobot/right_group_controller/command", 1000, jointStateCallback);
    	ros::Subscriber sub1 = n.subscribe("/bmirobot/right_group_hand_controller/command", 1000, handStateCallback);
    	ros::Rate rate(1);
    	while(ros::ok()){
    	char c = getch();
    	if (hand_count==1 && c==106){   // j
    	// set target joint angle
    	for (int i=0;i<2;i++){
    	ROS_INFO("Joint_hand: %f", hand_positions[i]);
    	}
	move_group1.setJointValueTarget(hand_positions);
	
	// plan execute
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group1.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (success) {
        move_group1.execute(my_plan);
        }
    	hand_count = 0;
    	//rate.sleep();
    	ros::spinOnce();
    	}
    	
    	if (count==1 && c==106){
    	// set target joint angle
	move_group.setJointValueTarget(joint_positions);
	
	// plan execute
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (success) {
        move_group.execute(my_plan);
        }
    	count = 0;
    	//rate.sleep();
    	ros::spinOnce();
    	}
}
}

