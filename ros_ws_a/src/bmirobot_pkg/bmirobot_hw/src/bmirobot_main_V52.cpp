#include <bmirobot_hw/bmirobot_V5.h>
//#include <bmirobot_hw/bmirobot_com.h>
#include <serial/serial.h>
#include <vector>
#include <iomanip> 
#include <ros/console.h>

#include <urdf/model.h>
#include <bmirobot_msg/Robot_ctr.h>
#include <bmirobot_msg/Robot_fdctr.h>
#include <bmirobot_msg/Robot_fdstatus.h>
#include <bmirobot_msg/Robot_mpu.h>
#include <bmirobot_msg/Robot_jointfd.h>
#include <stdlib.h>
#include <controller_manager/controller_manager.h>
#include <bmirobot_hw/kalman_filter.h>
#include <bmirobot_tools/PointsPR.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <Eigen/Eigen>

#define RAD2DEG(x) (x/3.1415926*180)
#define DEG2RAD(x) (x/180.0*3.1415926)
#define pi         3.1415926
using namespace std;

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

#define USE_ODOMETER  0

using namespace std;

char handJN_left[][50]={"left_hand_joint1","left_hand_joint2"};
char handJN_right[][50]={"right_hand_joint1","right_hand_joint2"};
char leftjoint[][50]={"left_joint1","left_joint2","left_joint3","left_joint4","left_joint5","left_joint6","left_joint7"};
char rightjoint[][50]={"right_joint1","right_joint2","right_joint3","right_joint4","right_joint5","right_joint6","right_joint7"};

int32_t sysstatus=0x0;
int leftarm1,rightarm2,point_ctr=0;

//ros::NodeHandle n;

BMIRobot::~BMIRobot() 
{

}
BMIRobot::BMIRobot() 
{
	ros::NodeHandle node;
   //read robot limitions
   std::string robot_desc_string;
   node.param("robot_description", robot_desc_string, std::string());
   urdf::Model model;
   int index=0;

   //if (!model.initFile("/home/bmi/Documents/ws_moveit/src/bmirobot_pkg/bmirobot_description/urdf/bmirobot.xacro")){
   if (!model.initString(robot_desc_string)){
     	ROS_ERROR("Failed to parse urdf file");
    return;
   }
	std::cout<< "parse urdf file ok"<<std::endl;
	std::shared_ptr<const urdf::Link> link = model.getLink("left_link1");
	std::cout<< link->child_joints[0]->name << " \n";
	ljoint_limition[0][0] = link->child_joints[0]->limits->lower;
	ljoint_limition[0][1] = link->child_joints[0]->limits->upper;
	for(int i = 0;i<7;i++)
	{
		link = link->child_links[0];
		int index = 0;
		if(i == 6)
			index = 0;
		std::cout<< link->child_joints[index]->name << "   \n";
		ljoint_limition[i+1][0] = link->child_joints[index]->limits->lower;
		ljoint_limition[i+1][1] = link->child_joints[index]->limits->upper;
	}
	ljoint_limition[8][0] = -1*ljoint_limition[7][1];
	ljoint_limition[8][1] = -1*ljoint_limition[7][0];
	
	link = model.getLink("right_link1");
	for(int i = 0;i<=7;i++)
	{
		if(i==7)
			index=1;
		else
			index=0;
		std::cout<< link->child_joints[index]->name << " \n";

		rjoint_limition[i][0] = link->child_joints[index]->limits->lower;
		rjoint_limition[i][1] = link->child_joints[index]->limits->upper;

		link = link->child_links[index];
	}
	rjoint_limition[8][0] = -1*rjoint_limition[7][1];
	rjoint_limition[8][1] = -1*rjoint_limition[7][0];


	//exit(-1);

    // connect and register the joint state interface
    for(int i=0;i < JOINT_NUM+HAND_JOINT_NUM;i++)
    {
      	left_pos[i] = left_cmd[i] =0;
        left_vel[i] = 0;
        left_eff[i] = 0;
	      	
		right_pos[i] = right_cmd[i] =0;
        right_vel[i] = 0;
        right_eff[i] = 0;	
    }


    for(int i =0;i < 7;i++)
    {
     	hardware_interface::JointStateHandle temp(leftjoint[i], &left_pos[i], &left_vel[i], &left_eff[i]);
     	jnt_state_interface.registerHandle(temp);
    }
    for(int i =0;i < 2;i++)
    {
     	hardware_interface::JointStateHandle temp(handJN_left[i], &left_pos[i+7], &left_vel[i+7], &left_eff[i+7]);
     	jnt_state_interface.registerHandle(temp);
    }

    for(int i =0;i < 7;i++)
    {
     	hardware_interface::JointStateHandle temp(rightjoint[i], &right_pos[i], &right_vel[i], &right_eff[i]);
     	jnt_state_interface.registerHandle(temp);
    }
    for(int i =0;i < 2;i++)
    {
     	hardware_interface::JointStateHandle temp(handJN_right[i], &right_pos[i+7], &right_vel[i+7], &right_eff[i+7]);
     	jnt_state_interface.registerHandle(temp);
    }

    registerInterface(&jnt_state_interface);

    for(int i =0;i < 7;i++)
    {
     	hardware_interface::JointHandle temp(jnt_state_interface.getHandle(leftjoint[i]), &left_cmd[i]);
     	jnt_pos_interface.registerHandle(temp);
    }
    for(int i =0;i < 2;i++)
    {
     	hardware_interface::JointHandle temp(jnt_state_interface.getHandle(handJN_left[i]), &left_cmd[i+7]);
     	jnt_pos_interface.registerHandle(temp);
    }

    for(int i =0;i < 7;i++)
    {
     	hardware_interface::JointHandle temp(jnt_state_interface.getHandle(rightjoint[i]), &right_cmd[i]);
     	jnt_pos_interface.registerHandle(temp);
    }
    for(int i =0;i < 2;i++)
    {
     	hardware_interface::JointHandle temp(jnt_state_interface.getHandle(handJN_right[i]), &right_cmd[i+7]);
     	jnt_pos_interface.registerHandle(temp);
    }
    registerInterface(&jnt_pos_interface);
    last_time = ros::Time::now();
    initrobot();
	init_left_robot();
	init_right_robot();


	//exit(-1);
}

void BMIRobot::initrobot()
{
    ROS_INFO("finish initrobot---------------------");
}

void BMIRobot::read()
{
	if(leftarm1==1)
	{
		updata_left_recevie();
		lFdctrmsgCB();
		left_read();
	}
	if(rightarm2==1)
	{
		updata_right_recevie();
		rFdctrmsgCB();
		right_read();
	}
	if(point_ctr==1)
	{
		updata_points_recevie();
	}

	com_save_data();
}

void BMIRobot::write()
{
	if(leftarm1==1)
	{
		left_write();
		updata_left_send();
	}
	if(rightarm2==1)
	{
		right_write();
		updata_right_send();
	}
	if(point_ctr==1)
	{
		updata_points_send();
	}

}

ros::Time BMIRobot::get_time(){
    return ros::Time::now();
}

ros::Duration BMIRobot::get_period(){
    ros::Time current_time = ros::Time::now();
    ros::Duration period = current_time - last_time;
    last_time = current_time;
    return period;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "bmirobot_hw_V4");

	ros::NodeHandle n("bmirobot");
	ros::Duration period(1.0);

	ros::AsyncSpinner spinner(2);
	spinner.start();
	//ros::Rate loop_rate(1000);

	lMT_ctrpub = n.advertise<bmirobot_msg::Robot_ctr>("lMT_ctr", 1000);
	lJoint_fdpub = n.advertise<bmirobot_msg::Robot_jointfd>("lMT_Jointfd", 1000);

	rMT_ctrpub = n.advertise<bmirobot_msg::Robot_ctr>("rMT_ctr", 1000);
	rJoint_fdpub = n.advertise<bmirobot_msg::Robot_jointfd>("rMT_Jointfd", 1000);

	lMt_fdctrpub 	= n.advertise<bmirobot_msg::Robot_fdctr>("lMT_fdctr", 100);
	lMt_fdstatuspub = n.advertise<bmirobot_msg::Robot_fdstatus>("lMT_fdstatus", 100);
	lMt_mpupub 		= n.advertise<bmirobot_msg::Robot_mpu>("lMT_fdmpu", 1000);

	rMt_fdctrpub 	= n.advertise<bmirobot_msg::Robot_fdctr>("rMT_fdctr", 100);
	rMt_fdstatuspub = n.advertise<bmirobot_msg::Robot_fdstatus>("rMT_fdstatus", 100);
	rMt_mpupub 		= n.advertise<bmirobot_msg::Robot_mpu>("rMT_fdmpu", 1000);


	Pointspub = n.advertise<bmirobot_tools::PointsPR>("PointsPR", 1000);

	BMIRobot robot;
	controller_manager::ControllerManager cm(&robot,n);
	usleep(300);
 	std::cout << argc << argv[1]<<'\n';

  	string saveflag;
  	n.getParam("/savedata", saveflag);
 	n.getParam("/leftarm", leftarm1);
 	n.getParam("/rightarm", rightarm2);
	
	ROS_ERROR("test,%d",point_ctr);
	n.getParam("/points", point_ctr);


	int tmpflag1, tmpflag2;

	init_com();

	//exit(1);

	ROS_INFO("start ---------------------");
	while (ros::ok())
	{
		ROS_ERROR("sysstatus,%d",sysstatus);
		//ros::spinOnce();
		switch(sysstatus)
		{
			case 0:
				if(leftarm1==1)			
					tmpflag1=robot.leftstep1();
				else
					tmpflag1=1;
				if(rightarm2==1)
					tmpflag2=robot.rightstep1();
				else
					tmpflag2=1;

				if((tmpflag1==1)&&(tmpflag2==1))
				{
					tmpflag1=0;tmpflag2=0;
					sysstatus=1;
					//sleep(1);
				}
				ROS_ERROR("start motor,%d,%d",tmpflag1,tmpflag2);
				robot.read();
				robot.write();
			break;

			case 1:
				if(leftarm1==1)			
					tmpflag1=robot.leftstep2();
				if(rightarm2==1)			
					tmpflag2=robot.rightstep2();

				if((tmpflag1==1)||(tmpflag2==1))
				{
					tmpflag1=0;tmpflag2=0;
					sysstatus=2;
				}
				robot.read();
				robot.write();
			break;

			case 2:
				robot.read();
				if(leftarm1==1)			
					tmpflag1=robot.leftstep3();
				else
					tmpflag1=1;
				if(rightarm2==1)
					tmpflag2=robot.rightstep3();
				else
					tmpflag2=1;
				if((tmpflag1==1)&&(tmpflag2==1))
				{
					tmpflag1=0;tmpflag2=0;
					sysstatus=3;
					ROS_ERROR("start work");
				}
				ROS_ERROR("home flag,%d,%d",tmpflag1,tmpflag2);
				robot.write();
			break;
			
			case 3:
				ros::spinOnce();
				robot.read();
				ros::spinOnce();
				cm.update(robot.get_time(), robot.get_period());
				ros::spinOnce();
				robot.write();
			break;
		}
		//ROS_ERROR("test,%d",point_ctr);
	//usleep(500);
  }
  com_stop();
  spinner.stop();
}

