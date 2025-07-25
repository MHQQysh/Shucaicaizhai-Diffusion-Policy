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
//
#include<fcntl.h>
#include<sensor_msgs/Joy.h>

#define pi 3.14
// inverse motion and key
using namespace std;
using std::string;


vector<double> data_h = {0, 0};

float ljoint_limition[7][2], rjoint_limition[7][2];
double right_joint[7]={0};  // vector
double left_joint[7]={0};
double now_right[7];   // now_right:joint
double lastrightjoints[7];  //
double interrightjoints[7];
double a0[7],a2[7],a3[7];
// float Px=0,Py=0,Pz=0,Rx=0,Ry=0,Rz=0; //(Rx,Ry,Rz):angle; (Px,Py,Pz):length




void joy_cb(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[0]==1)
	{
		data_h[0] += 0.5;
	}
	else if(msg->buttons[1]==1)
	{
		data_h[0] -= 0.5;
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_fk_demo");
    ros::NodeHandle n("bmirobot");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //readlimitjoint();    // read the limition of the joint
    //homeEular();
    int k;
    //ros::Subscriber sub1 = n.subscribe("/bmirobot/joint_states", 1000, jointStateCallback);  // state of the joint
    //ros::Publisher rightarm_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/bmirobot/right_group_controller/command", 0 );
    ros::Publisher chatter_pub3 = n.advertise<std_msgs::Float64MultiArray>("/bmirobot/right_group_hand_controller/command", 1000);
    //ros::Subscriber sub2 = n.subscribe("/angle", 1000, data_read);  // state of the joint
    ros::Subscriber sub2 = n.subscribe("/joy", 100, joy_cb);  // state of the joint
    std_msgs::Float64MultiArray msg;
    std_msgs::Float64MultiArray msg_h;
    
    while(ros::ok()) {
    printf("msg_h: %f \n",data_h[0]);
	//
	bool gripper_updated=true;
        if(gripper_updated) {
            msg_h.data = data_h;
            chatter_pub3.publish(msg_h);
        }

}
    }
