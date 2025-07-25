// inverse motion
#include <string>
#include <math.h>
// ROS
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Eigen>
//Robot_msg
#include <bmirobot_msg/Robot_jointfd.h>
//key
#include <unistd.h>
#include<sstream>
#include<vector>
#include<stdio.h>
#include<termio.h>

//moveit
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#define pi 3.14
// inverse motion and key
using namespace std;
using std::string;

float num_groups[6] = {0.24,-0.341,0.2,0.0,0.0,0.0};

float translate[3] = {0.0,0.0,0.0};      //  applied for IK solution
//float joint_angle[7] = [0.0,0.0,0.0,0.0,0.0,0.0,0.0];   // joint_angle
Eigen::Matrix3d rotationMatrix;        // transform matrix

float rad=0.017;   // change rate
int num = 5;

// judge which command
int count_d = 0;

robot_state::JointModelGroup* joint_model_group;
Eigen::Isometry3d end_effector_state;
double timeout = 0.1;

std::vector<double> joint_values={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<double> hand_values={0.0,0.0};

// ros msg
vector<double> data = {0, 0, 0, 0, 0, 0, 0};
vector<double> hand_data = {0, 0};
vector<double> eff_data = {0, 0, 0, 0, 0, 0};

void key_con(char c)   // key -- connection
{
	if (c==119)   // up:w
    	{
    	  num_groups[0]=num_groups[0]+0.01;
    	  count_d =1;
    	}
    	else if(c==115)  // down:s
    	{
    	  num_groups[0]=num_groups[0]-0.01;
    	  count_d = 1;
    	}
    	else if(c==97)    // left:a
    	{
    	  num_groups[1]=num_groups[1]+0.01;
    	  count_d = 1;
    	}
    	else if(c==100)   // right:d
    	{
    	  num_groups[1]=num_groups[1]-0.01;
    	  count_d = 1;
    	}
    	else if(c==113)   // forward:q
    	{
    	  num_groups[2]=num_groups[2]+0.01;
    	  count_d = 1;
    	}
    	else if(c==101)   // backward:e
    	{
    	 num_groups[2]=num_groups[2]-0.01;
    	 count_d = 1;
    	}
    	else if(c==106){
    	 //num_groups[3]=num_groups[3]+rad*num;
    	 joint_values[4]=joint_values[4]+rad*num;
    	 count_d = 2;
    	}   // Rx:j
    	else if(c==108){
    	 //num_groups[3]=num_groups[3]-rad*num;
    	 joint_values[4]=joint_values[4]-rad*num;
    	 count_d = 2;
    	}   // Rx:l
    	else if(c==105){
    	 //num_groups[4]=num_groups[4]+rad*num;
    	 joint_values[5]=joint_values[5]+rad*num;
    	 count_d = 2;
    	}   // Ry:i  up down
    	else if(c==107){
    	 //num_groups[4]=num_groups[4]-rad*num;
    	 joint_values[5]=joint_values[5]-rad*num;
    	 count_d = 2;
    	}   // Ry:k
    	else if(c==117){
    	 //num_groups[5]=num_groups[5]+rad*num;
    	 joint_values[6]=joint_values[6]+rad*num;
    	 count_d = 2;
    	}   // Rz:u   left right
    	else if(c==111){
    	 //num_groups[5]=num_groups[5]-rad*num;
    	 joint_values[6]=joint_values[6]-rad*num;
    	 count_d = 2;
    	}   // Rz:o
    	else if(c==109){
    	 //num_groups[5]=num_groups[5]-rad*num;
    	 hand_values[0]=hand_values[0]+rad*num;
    	 hand_values[1]=hand_values[1]-rad*num;
    	 ROS_INFO("Joint_hand: %f", hand_values[0]);  //open
    	}   // hand:open, m
    	else if(c==110){
    	 //num_groups[5]=num_groups[5]-rad*num;
    	 hand_values[0]=hand_values[0]-rad*num/5*4;
    	 hand_values[1]=hand_values[1]+rad*num/5*4;
    	 ROS_INFO("Joint_hand: %f", hand_values[0]);  //close
    	}   // hand:open, n
}

// matrix
void matrix_tran(float num_groups[6]){
	float roll = num_groups[3];
	float pitch = num_groups[4];
	float yaw = num_groups[5];	
	double Rx[3][3] = {
        {1, 0, 0},
        {0, cos(roll), -sin(roll)},
        {0, sin(roll), cos(roll)}
    };      //roll
    	double Ry[3][3] = {
        {cos(pitch), 0, sin(pitch)},
        {0, 1, 0},
        {-sin(pitch), 0, cos(pitch)}
    };     //pitch
    	double Rz[3][3] = {
        {cos(yaw), -sin(yaw), 0},
        {sin(yaw), cos(yaw), 0},
        {0, 0, 1}
    };       // yaw
    	double temp[3][3];
    	for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                temp[i][j] += Rz[i][k] * Ry[k][j];
            }
        }
    }
    	for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rotationMatrix(i,j) = 0;
            for (int k = 0; k < 3; k++) {
                rotationMatrix(i,j) += temp[i][k] * Rx[k][j];
            }
        }
    }           // rotation_matrix
    	for(int i=0;i<3;i++){
    	translate[i] = num_groups[i];
    	}         // translation
}

std::array<double, 3> rotation_matrix_to_euler(const Eigen::Matrix3d R) {
    double R11 = R(0,0), R12 = R(0,1), R13 = R(0,2);
    double R21 = R(1,0), R22 = R(1,1), R23 = R(1,2);
    double R31 = R(2,0), R32 = R(2,1), R33 = R(2,2);
    std::array<double, 3> num = {0, 0, 0};
    
    double pitch = std::asin(-R31);

    double roll = std::atan2(R32, R33);

    double yaw = std::atan2(R21, R11);

    num[0] = roll * 180.0 / M_PI;   //roll
    num[1] = pitch * 180.0 / M_PI; // pitch
    num[2] = yaw * 180.0 / M_PI;     // yaw
    return num;
}


// IK solution moveit
void moveit_IK(robot_state::RobotStatePtr kinematic_state){
	Eigen::Vector3d translation(translate[0], translate[1], translate[2]);
	for (int i = 0; i < 3; i++)
  		{
    			ROS_INFO("translate: %f", translate[i]);
  		}
	end_effector_state.translation() = translation;
	end_effector_state.linear() = rotationMatrix;
	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);
	if (found_ik)
	{
  		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  		for (int i = 0; i < 7; i++)
  		{
    			ROS_INFO("Joint: %f", joint_values[i]);
  		}
	}
	else
	{
  	ROS_INFO("Did not find IK solution");
	}
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

int main(int argc, char **argv){
	ros::init(argc, argv, "key_demo");
	ros::NodeHandle n("bmirobot");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Publisher key_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/key",0);   // six numbers in a groups:[x,y,z,rx,ry,rz]
	ros::Publisher rightarm_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/bmirobot/right_group_controller/command", 0 );
	ros::Publisher righthand_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/bmirobot/right_group_hand_controller/command", 0 ); // hand
	std_msgs::Float64MultiArray msg, msg_hand;  // key
	std_msgs::Float64MultiArray msg_h;  // robot
	
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");       //robot_description
	robot_model::RobotModelPtr kinematic_model;
	kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	joint_model_group = kinematic_model->getJointModelGroup("right_arm");
	kinematic_state->setToDefaultValues();
	while(ros::ok()){
	char c = getch();
	key_con(c);
	if (count_d==1){
	matrix_tran(num_groups);
	moveit_IK(kinematic_state);
	count_d = 0;
	}
	// robot
	for(int k=0;k<7;k++){
    	  data[k]=joint_values[k];
    	}
    	msg.data = data;
    	rightarm_joint_pub.publish(msg);
    	
    	// hand
    	for(int k=0;k<2;k++){
    	  hand_data[k]=hand_values[k];
    	}
    	msg_hand.data = hand_data;
    	righthand_joint_pub.publish(msg_hand);
    	
    	// kinematic
	kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
	const Eigen::Isometry3d& end_state = kinematic_state->getGlobalLinkTransform("right_link7");
	std::array<double, 3> num = rotation_matrix_to_euler(end_state.rotation());  // robot state
	
    	// update:translation rotation
    	if(count_d==2){
	for(int i=0;i<3;i++){
    	num_groups[i] = end_state.translation()[i];   // moveit solver
    	}         // translation
	count_d = 0;
	for (int i = 0; i < 3; i++)
  		{
    			ROS_INFO("translate: %f", num_groups[i]);
  		}
	for (int i = 0; i < 7; i++)
  		{
    			ROS_INFO("Joint: %f", joint_values[i]);
  		}
	}
	// key
    	for(int k=0;k<3;k++){
    	  eff_data[k]=end_state.translation()[k];
    	  eff_data[k+3] = num[k];
    	}
    	msg_h.data = eff_data;
    	key_joint_pub.publish(msg_h);    // future pose
}
}

