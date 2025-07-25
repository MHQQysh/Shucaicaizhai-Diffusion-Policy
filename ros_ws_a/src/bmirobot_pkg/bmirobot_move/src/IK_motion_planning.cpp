
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<errno.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include <math.h>
#include <arpa/inet.h>
#include <random>
#include <algorithm>
#include <cmath>
// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Eigen>
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

#include <bmirobot_tools/PointsPR.h>
#include <bmirobot_msg/Robot_jointfd.h>
#include <time.h>
#include <stdio.h>

#include <iostream>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#define MAXLINE 4096

#define pi   3.141592653589793

using namespace std;
using namespace Eigen;
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::atan;




bmirobot_tools::PointsPR pointmsg;

ros::Publisher arm_joint_pub,grasp_joint_pub,rightarm_joint_pub,leftarm_joint_pub;
ros::Publisher marker_pub;

std::default_random_engine eng; //引擎

int16_t initcount=1000;
float Px[4],Py[4],Pz[4],Rx[4],Ry[4],Rz[4];

float storePx[4],storePy[4],storePz[4],storeRx[4],storeRy[4],storeRz[4];
float maxPx[4],maxPy[4],maxPz[4],maxRx[4],maxRy[4],maxRz[4];
float minPx[4],minPy[4],minPz[4],minRx[4],minRy[4],minRz[4];
float diffPoint[4][6];

float homePoint[4][6];

float arm_joints[7];
float hand_joints[2];

#define particle_num 50
static double praticle[4][particle_num][3];
static double bestp[4][3];


int receivedata=0;

Eigen::Matrix3d MRhome[4],MRnow[4];
Eigen::Vector3d MPhome[4],MPnow[4];
Eigen::Matrix4d left_Mat1,left_Mat2,right_Mat1,right_Mat2;
Eigen::Matrix4d left_initMat1,left_initMat2,right_initMat1,right_initMat2;
Eigen::Matrix4d left1_initMat,left2_initMat,right1_initMat,right2_initMat;
Eigen::Matrix3d	inithome;
Eigen::Matrix3d Mi2j[4];
Eigen::Vector3d Eulari2j[4];

Eigen::Matrix3d	lM0to1,lM1to2,rM0to1,rM1to2;

Eigen::Matrix4d left1_init,left2_init,right1_init,right2_init;
Eigen::Matrix4d left1_init1,left1_init2,left2_init1,left2_init2;
Eigen::Matrix4d right1_init1,right1_init2,right2_init1,right2_init2;


Eigen::Matrix3d lhomeMat0,lhomeMat1,rhomeMat0,rhomeMat1;

Eigen::Vector3d nowEular0to1,nowEular1to2,nowEular2to3,nowEular3to4;
Eigen::Vector3d homeEular0to1,homeEular1to2,homeEular2to3,homeEular3to4;

Eigen::Vector3d store0,store1,store2,store3;

double rightjointangle[7]={0},	leftjointangle[7],	handangle;
double nowrightjoints[7], nowleftrightjoints[7];
double lastrightjoints[7];
float homeangle[8];
float joint_limition[7][2];

//Eigen::Matrix<double, 3, 3> J_vect, M_vect,M_cmd;
extern void getmatrix();


int16_t msgcount=0;
void PointsPRCallback(const bmirobot_tools::PointsPR::ConstPtr& msg)
{
	int i;
	for(i=0;i<4;i++)
	{
		Px[i]=msg->Px[i]/100.0;
		Py[i]=msg->Py[i]/100.0;
		Pz[i]=msg->Pz[i]/100.0;
		Rx[i]=msg->Rx[i]*pi/180.0;
		Ry[i]=msg->Ry[i]*pi/180.0;
		Rz[i]=msg->Rz[i]*pi/180.0;
	}
	if(receivedata==0)
	{
		receivedata=1;
	}
	msgcount++;
}


void move_rightarm(double jointS[7])
{
	std_msgs::Float64MultiArray msg;
	for(int i = 0;i< 7;i++)
	{
		msg.data.push_back(jointS[i]);
	}

	rightarm_joint_pub.publish(msg);
    arm_joint_pub.publish(msg);
}

void move_leftarm(double jointS[7])
{
	std_msgs::Float64MultiArray msg;
	for(int i = 0;i< 7;i++)
	{
		msg.data.push_back(jointS[i]);
	}

	leftarm_joint_pub.publish(msg);
}

void grasper_control(double a)
{
	std_msgs::Float64MultiArray msg;
  	msg.data.push_back(a);
  	msg.data.push_back(-a);

  	grasp_joint_pub.publish(msg);

}

int countAverage(void)
{
	static int count=0;
		
	count++;
	printf("count:%d \n",count);

	if(count<initcount)
	{
		for(int i=0;i<4;i++)
		{
			storePx[i]+=Px[i];
			storePy[i]+=Py[i];
			storePz[i]+=Pz[i];
			storeRx[i]+=Rx[i];
			storeRy[i]+=Ry[i];
			storeRz[i]+=Rz[i];
			if(count==101)
			{
				maxPx[i]=Px[i];			minPx[i]=Px[i];
				maxPy[i]=Py[i];			minPy[i]=Py[i];
				maxPz[i]=Pz[i];			minPz[i]=Pz[i];
				maxRx[i]=Rx[i];			minRx[i]=Rx[i];
				maxRy[i]=Ry[i];			minRy[i]=Ry[i];
				maxRz[i]=Rz[i];			minRz[i]=Rz[i];
			}

			if(maxPx[i]<Px[i])		{maxPx[i]=Px[i];}
			if(minPx[i]>Px[i])		{minPx[i]=Px[i];}
			if(maxPy[i]<Py[i])		{maxPy[i]=Py[i];}
			if(minPy[i]>Py[i])		{minPy[i]=Py[i];}
			if(maxPz[i]<Pz[i])		{maxPz[i]=Pz[i];}
			if(minPz[i]>Pz[i])		{minPz[i]=Pz[i];}
			if(maxRx[i]<Rx[i])		{maxRx[i]=Rx[i];}
			if(minRx[i]>Rx[i])		{minRx[i]=Rx[i];}
			if(maxRy[i]<Ry[i])		{maxRy[i]=Ry[i];}
			if(minRy[i]>Ry[i])		{minRy[i]=Ry[i];}
			if(maxRz[i]<Rz[i])		{maxRz[i]=Rz[i];}
			if(minRz[i]>Rz[i])		{minRz[i]=Rz[i];}
		}
	}
	if(count>=initcount)
	{
		for(int i=0; i<4;i++)
		{
			diffPoint[i][0]=maxPx[i]-minPx[i];
			diffPoint[i][1]=maxPy[i]-minPy[i];
			diffPoint[i][2]=maxPz[i]-minPz[i];
			diffPoint[i][3]=maxRx[i]-minRx[i];
			diffPoint[i][4]=maxRy[i]-minRy[i];
			diffPoint[i][5]=maxRz[i]-minRz[i];
			//std::cout<<"num:"<<i<<"    maxpy:"<<maxPx[i]<<"    minpy:"<<minPx[i]<<endl;
			//std::cout<<"num:"<<i<<"    maxpy:"<<maxPy[i]<<"    minpy:"<<minPy[i]<<endl;
			//std::cout<<"num:"<<i<<"    maxpy:"<<maxPz[i]<<"    minpy:"<<minPz[i]<<endl;

			std::cout<<"num:"<<i<<"    dpx:"<<diffPoint[i][0]<<"    dpy:"<<diffPoint[i][1]<<"    dpz:"<<diffPoint[i][2]<<endl;
			std::cout<<"num:"<<i<<"    drx:"<<diffPoint[i][3]<<"    dry:"<<diffPoint[i][4]<<"    drz:"<<diffPoint[i][5]<<endl;
			//if((diffPoint[i][0]>5)&(diffPoint[i][1]>5)&(diffPoint[i][2]>5))	
			//exit(0);
			//if((diffPoint[i][3]>0.01)&(diffPoint[i][4]>0.01)&(diffPoint[i][5]>0.01))	
			//exit(0);
		}

		for(int i=0;i<4;i++)
		{
			homePoint[i][0]=storePx[i]/initcount;
			homePoint[i][1]=storePy[i]/initcount;
			homePoint[i][2]=storePz[i]/initcount;
			homePoint[i][3]=storeRx[i]/initcount;
			homePoint[i][4]=storeRy[i]/initcount;
			homePoint[i][5]=storeRz[i]/initcount;
			printf("homeposture:%d,%f,%f,%f,%f,%f,%f \n",i,homePoint[i][0],homePoint[i][1],homePoint[i][2],homePoint[i][3],homePoint[i][4],homePoint[i][5]);
		}
		return 1;
	}else
		return 0;
}


void limitjoint()
{
   ros::NodeHandle node;
   std::string robot_desc_string;
   node.param("robot_description", robot_desc_string, std::string());

   urdf::Model model;
   if (!model.initString(robot_desc_string)){
     	ROS_ERROR("Failed to parse urdf file");
    return;
   }
   std::cout<< "parse urdf file ok"<<std::endl;
   boost::shared_ptr<const urdf::Link> link = model.getLink("base_link");
   std::cout<< link->child_joints[0]->name << " \n";
   joint_limition[0][0] = link->child_joints[0]->limits->lower;
   joint_limition[0][1] = link->child_joints[0]->limits->upper;

   for(int i = 0;i<7;i++)
   {
		if(i==0)
     		link = link->child_links[0];
		else
			link = link->child_links[0];

        int index = 0;
    	if(i == 6)
		index = 1;

    	std::cout<< link->child_joints[index]->name << "   \n";
        joint_limition[i+1][0] = link->child_joints[index]->limits->lower;
        joint_limition[i+1][1] = link->child_joints[index]->limits->upper;
  }
  joint_limition[8][0] = -1*joint_limition[7][1];
  joint_limition[8][1] = -1*joint_limition[7][0];
}



void homeEular()
{
	Eigen::Matrix4d  Mat4d[4];


	left1_init1<<	0, 		1, 		0, 		0.121,
					0, 		0, 		-1, 	0,
					-1,		0,		0,		-0.0555,
					0,		0,		0,		1.0;
	left1_init2<<	1, 		0, 		0, 		0,
					0, 		0, 		-1, 	0,
					0,		1,		0,		0,
					0,		0,		0,		1.0;
	left1_init=left1_init1*left1_init2;

	left2_init1<<	0, 		1, 		0, 		0.121,
					-1, 	0, 		0, 		-0.24,
					0,		0,		1,		-0.25,
					0,		0,		0,		1.0;
	left2_init2<<	1, 		0, 		0, 		0.07,
					0, 		0, 		-1, 	0,
					0,		1,		0,		0,
					0,		0,		0,		1.0;
	left2_init=left2_init1*left2_init2;

	right1_init1<<	0, 		-1, 	0, 		0.121,
					0, 		0, 		1, 		0,
					-1,		0,		0,		-0.0555,
					0,		0,		0,		1.0;
	right1_init2<<	1, 		0, 		0, 		0,
					0, 		0, 		1, 		0,
					0,		-1,		0,		0,
					0,		0,		0,		1.0;
	right1_init=right1_init1*right1_init2;

	right2_init1<<	0, 		-1, 	0, 		0.121,
					1, 		0, 		0, 		0.24,
					0,		0,		1,		-0.25,
					0,		0,		0,		1.0;
	right2_init2<<	1, 		0, 		0, 		0.07,
					0, 		0, 		1, 		0,
					0,		-1,		0,		0,
					0,		0,		0,		1.0;
	right2_init=right2_init1*right2_init2;
	
	left_initMat1=	Matrix4d::Identity(4,4);
	left_initMat2=	Matrix4d::Identity(4,4);
	right_initMat1=	Matrix4d::Identity(4,4);
	right_initMat2=	Matrix4d::Identity(4,4);

	std::cout<<"left_initMat1"<<endl<<left_initMat1<<endl;

	for(int i=0;i<4;i++)
	{
		MRhome[i]= Eigen::AngleAxisd(homePoint[i][5], Eigen::Vector3d(0, 0, 1))  
        		 * Eigen::AngleAxisd(homePoint[i][4], Eigen::Vector3d(0, 1, 0))  
        		 * Eigen::AngleAxisd(homePoint[i][3], Eigen::Vector3d(1, 0, 0)); 			

		MPhome[i](0)=homePoint[i][0];
		MPhome[i](1)=homePoint[i][1];
		MPhome[i](2)=homePoint[i][2];

		Mat4d[i]=Matrix4d::Identity(4,4);
		Mat4d[i].block<3,3>(0,0)=MRhome[i];
		Mat4d[i].block<3,1>(0,3)=MPhome[i];
	}

	left_initMat1=Mat4d[2]*left1_init.inverse();
	left_initMat2=Mat4d[3]*left2_init.inverse();

	right_initMat1=Mat4d[1]*right1_init.inverse();
	right_initMat2=Mat4d[0]*right2_init.inverse();

	//std::cout<<"right_initMat1"<<endl<<right_initMat1<<endl;
	//std::cout<<"right_initMat2"<<endl<<right_initMat2<<endl;

//	exit(0);
}


void getmatrix()
{
	Eigen::Matrix4d  Matnow4d[4];

	for(int i=0;i<4;i++)
	{
		MRnow[i]= Eigen::AngleAxisd(Rz[i], Eigen::Vector3d(0, 0, 1))  
				* Eigen::AngleAxisd(Ry[i], Eigen::Vector3d(0, 1, 0))  
				* Eigen::AngleAxisd(Rx[i], Eigen::Vector3d(1, 0, 0));

		MPnow[i](0)=Px[i];
		MPnow[i](1)=Py[i];
		MPnow[i](2)=Pz[i];

		Matnow4d[i]=Matrix4d::Identity(4,4);
		Matnow4d[i].block<3,3>(0,0)=MRnow[i];
		Matnow4d[i].block<3,1>(0,3)=MPnow[i];
		//ROS_INFO("new angle ,%f,%f,%f",Rz[i],Ry[i],Rx[i]);
	}
	//std::cout<<"Matnow4d"<<endl<<Matnow4d[0]<<endl;

	left_Mat1=left_initMat1.inverse()*Matnow4d[2]*left1_init2.inverse();
	left_Mat2=left_initMat2.inverse()*Matnow4d[3]*left2_init2.inverse();

	right_Mat1=right_initMat1.inverse()*Matnow4d[1]*right1_init2.inverse();
	right_Mat2=right_initMat2.inverse()*Matnow4d[0]*right2_init2.inverse();

	//std::cout<<"right_Mat1"<<endl<<right_Mat1<<endl;
	//std::cout<<"right_Mat2"<<endl<<right_Mat2<<endl;
}

double atan21(double d1,double d2)
{
	double rst=0;
	if (d2>0)
	{
		if(d1>0)
		{
      		rst=atan(d1/d2);
		}else
		{
			rst=atan(d1/d2);
			rst=rst;
		}

	}else
	{
		if(d1>0)
		{
			rst=atan(d1/d2);
			rst=rst+pi;
		}else
		{
			rst=atan(d1/d2);
			rst=rst-pi;
		}
	}	

}

double atan22(double d1,double d2)
{
	double rst=0;
	if (d2>0)
	{
		if(d1>0)
		{
      		rst=atan(d1/d2);
		}else
		{
			rst=atan(d1/d2);
			rst=rst;
		}
	}else
	{
		if(d1>0)
		{
			rst=atan(d1/d2);
			rst=rst+pi;
		}else
		{
			rst=atan(d1/d2);
			rst=rst-pi;
		}
	}	

}

void IK_right_control(double *mtnow,  double *mtgoal)
{
	double ja1[3],ja2[3],ja3[3],ja4[3];
	double right[7]={0};
	double left[7]={0};
	double a1, a2, a3, a4, a5 ,a6, a7;

	Eigen::Matrix<double, 3, 3> tmat1;
	Eigen::Matrix<double, 3, 1> tmat2;
	Eigen::Matrix<double, 3, 1> tmat3;
	Eigen::Matrix<double, 4, 4>	T04;
	Eigen::Matrix<double, 4, 4> T47;

	int index=0;
	double x2=right_Mat2(0,3);
	double y2=right_Mat2(1,3);
	double z2=right_Mat2(2,3);

	double sqrt1,ta2,sa2;
	double temp1,temp2;

	//double na1=mtnow[0],na2=mtnow[1],na3=mtnow[2],na4=mtnow[3],na5=mtnow[4],na6=mtnow[5],na7=mtnow[6];
	double na1=lastrightjoints[0],na2=lastrightjoints[1],na3=lastrightjoints[2],na4=lastrightjoints[3],na5=lastrightjoints[4],na6=lastrightjoints[5],na7=lastrightjoints[6];
	double nowerr=abs(na1)+abs(na2)+abs(na3)+abs(na4)+abs(na5)+abs(na6)+abs(na7);
	double goals[100][7],goalerr[100];

	//std::cout<<"right_Mat2="<<endl<<right_Mat2<<endl;

	for(int i=0;i<100;i++)
	{
		a2=na2+0.005*(i-50);

		double cos1=z2*cos(a2)/2;
		double sin1=-y2*cos(a2)/2;
		double const1=(x2+sin(a2)/4-0.121)*(x2+sin(a2)/4-0.121)-sin(a2)*sin(a2)/16+y2*y2+z2*z2+0.0049;
		sqrt1=sqrt(cos1*cos1+sin1*sin1);
		if(sin1>0)
		{
			ta2=asin(-const1/sqrt1);
			sa2=atan2(cos1/sqrt1,sin1/sqrt1);
		}else
		{
			ta2=asin(const1/sqrt1);
			sa2=atan2(-cos1/sqrt1,-sin1/sqrt1);
		}
		a1= ta2-sa2;

		tmat1  <<	0.0, 				cos(a2)*6/25, 			sin(a2)*-6/25,			//  cos(a3)*cos(a4)     cos(a4)*sin(a3)     sin(a4) 
					cos(a1)*6/25,	sin(a1)*sin(a2)*6/25,	cos(a2)*sin(a1)*6/25,
					sin(a1)*6/25,	cos(a1)*sin(a2)*-6/25,	cos(a1)*cos(a2)*-6/25;

		tmat2<<  	(sin(a2)/4)-(121.0/1000)	+x2, 
					cos(a2)*sin(a1)/-4	+y2,
					cos(a1)*cos(a2)/4	+z2;

		tmat3=tmat1.inverse()*tmat2;
		a4=asin(tmat3(2));
		a3=atan2(tmat3(1)/cos(a4),tmat3(0)/cos(a4));

		T04 <<
											cos(a2)*cos(a4)*sin(a3) - sin(a2)*sin(a4),                          -cos(a2)*cos(a3),                                     cos(a4)*sin(a2) + cos(a2)*sin(a3)*sin(a4), 121/1000 - sin(a2)/4,
		cos(a4)*(cos(a1)*cos(a3) + sin(a1)*sin(a2)*sin(a3)) + cos(a2)*sin(a1)*sin(a4), cos(a1)*sin(a3) - cos(a3)*sin(a1)*sin(a2), sin(a4)*(cos(a1)*cos(a3) + sin(a1)*sin(a2)*sin(a3)) - cos(a2)*cos(a4)*sin(a1),  (cos(a2)*sin(a1))/4,
		cos(a4)*(cos(a3)*sin(a1) - cos(a1)*sin(a2)*sin(a3)) - cos(a1)*cos(a2)*sin(a4), sin(a1)*sin(a3) + cos(a1)*cos(a3)*sin(a2), sin(a4)*(cos(a3)*sin(a1) - cos(a1)*sin(a2)*sin(a3)) + cos(a1)*cos(a2)*cos(a4), -(cos(a1)*cos(a2))/4,
																				0,                                         0,                                                                             0,                    1;

		T47=T04.inverse()*right_Mat2;

		a6=asin(T47(0,2));
		a5=atan2(-T47(1,2)/cos(a6),T47(2,2)/cos(a6));
		a7=atan2(-T47(0,1)/cos(a6),T47(0,0)/cos(a6));

		if(!isnan(a1))
		if(!isnan(a2))
		if(!isnan(a3))
		if(!isnan(a4))
		if(!isnan(a5))
		if(!isnan(a6))
		if(!isnan(a7))
		{
			goals[i][0]=a1;
			goals[i][1]=a2;
			goals[i][2]=a3;
			goals[i][3]=a4;
			goals[i][4]=a5;
			goals[i][5]=a6;
			goals[i][6]=a7;
		}
		goalerr[i]=abs(a1-na1)+abs(a2-na2)+abs(a3-na3)+abs(a4-na4)+abs(a5-na5)+abs(a6-na6)+abs(a7-na7);
       // printf("err: %12d,%12f,%12f,%12f,%12f,%12f,%12f,%12f\n",i,goalerr[i],a1,a2,a3,a4,a5,a6,a7);
	}

	index=0;
    double low=1000000;
	for(int i=0;i<100;i++)
	{
		if(low>goalerr[i])
		{
			low=goalerr[i];
			index=i;
		}
	}
    if(!isnan(goalerr[index]))
    {
        mtgoal[0]=goals[index][0];
        mtgoal[1]=goals[index][1];
        mtgoal[2]=goals[index][2];
        mtgoal[3]=goals[index][3];
        mtgoal[4]=goals[index][4];
        mtgoal[5]=goals[index][5];
        mtgoal[6]=goals[index][6];
    }
	for(int i=0;i<7;i++)
		lastrightjoints[i]=mtgoal[i];

    ROS_INFO("rightnow: %12d, %12f,%12f,%12f,%12f,%12f,%12f,%12f \n",index,mtnow[0],mtnow[1],mtnow[2],mtnow[3],mtnow[4],mtnow[5],mtnow[6]);
	ROS_INFO("right: %12f, %12f,%12f,%12f,%12f,%12f,%12f,%12f \n",goalerr[index],mtgoal[0],mtgoal[1],mtgoal[2],mtgoal[3],mtgoal[4],mtgoal[5],mtgoal[6]);
	//printf("out: %12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f \n",rightjointangle[0],jointangle[1],jointangle[2],jointangle[3],jointangle[4],jointangle[5],jointangle[6],jointangle[7]);
   
	//move_rightarm(rightjointangle);
	//grasper_control(handangle);
	//exit(0);
}


void IK_left_control()
{
	double ja1[3],ja2[3],ja3[3],ja4[3];	double right[7]={0};
	double left[7]={0};
	double a1, a2, a3, a4, a5 ,a6, a7;
	double x2=left_Mat2(0,3);
	double y2=left_Mat2(1,3);
	double z2=left_Mat2(2,3);	

	Eigen::Matrix<double, 3, 3> tmat1;
	Eigen::Matrix<double, 3, 1> tmat2;
	Eigen::Matrix<double, 3, 1> tmat3;
	Eigen::Matrix<double, 4, 4>	T04;
	Eigen::Matrix<double, 4, 4> T47;

	//double temp1=14.902159224408750615035129535356*left_Mat1(1,0) - 12.493694723044117558501184097714*left_Mat1(2,0) - 14.934454506877525785661855949229;
	a2=asin(-left_Mat1(0,0));

	double cos1=z2*cos(a2)/2;
	double sin1=-y2*cos(a2)/2;
	double const1=(x2+sin(a2)/4-0.121)*(x2+sin(a2)/4-0.121)-sin(a2)*sin(a2)/16+y2*y2+z2*z2+0.0049;
	double sqrt1=sqrt(cos1*cos1+sin1*sin1);
	double ta2,sa2;
	if(sin1>0)
	{
	 	ta2=asin(-const1/sqrt1);
	 	sa2=atan2(cos1/sqrt1,sin1/sqrt1);
	}else
	{
	 	ta2=asin(const1/sqrt1);
	 	sa2=atan2(-cos1/sqrt1,-sin1/sqrt1);
	}
	a1= ta2-sa2;
	std::cout<<"sqrt1="<<sqrt1<<endl;
	std::cout<<"ta2="<<ta2<<endl;
	std::cout<<"sa2="<<sa2<<endl;
	std::cout<<"sin1="<<sin1<<endl;
	std::cout<<"cos1="<<cos1<<endl;
	std::cout<<"const1="<<const1<<endl;

	std::cout<<"a1="<<endl<<a1<<endl;
	std::cout<<"a2="<<endl<<a2<<endl;
	 //a2=asin(temp1);

	//a1=atan2(left_Mat1(1,0)/cos(a2),-left_Mat1(2,0)/cos(a2));

	 tmat1  <<	0.0, 			cos(a2)*-6/25, 			sin(a2)*-6/25,			//  cos(a3)*cos(a4)     cos(a4)*sin(a3)     sin(a4) 
				cos(a1)*-6/25,	sin(a1)*sin(a2)*-6/25,	cos(a2)*sin(a1)*6/25,
				sin(a1)*-6/25,	cos(a1)*sin(a2)*6/25,	cos(a1)*cos(a2)*-6/25;

	 tmat2<<  	(sin(a2)/4)-(121.0/1000)	+x2, 
				cos(a2)*sin(a1)/-4	+y2,
				cos(a1)*cos(a2)/4	+z2;

	tmat3=tmat1.inverse()*tmat2;

	 a4=asin(tmat3(2));
	 a3=atan2(tmat3(1)/cos(a4),tmat3(0)/cos(a4));

	 T04 <<
                                        -cos(a2)*cos(a4)*sin(a3) - sin(a2)*sin(a4),                            cos(a2)*cos(a3),                                      cos(a4)*sin(a2) - cos(a2)*sin(a3)*sin(a4), 121/1000 - sin(a2)/4,
    -cos(a4)*(cos(a1)*cos(a3) + sin(a1)*sin(a2)*sin(a3)) + cos(a2)*sin(a1)*sin(a4), -cos(a1)*sin(a3) + cos(a3)*sin(a1)*sin(a2), -sin(a4)*(cos(a1)*cos(a3) + sin(a1)*sin(a2)*sin(a3)) - cos(a2)*cos(a4)*sin(a1),  (cos(a2)*sin(a1))/4,
    -cos(a4)*(cos(a3)*sin(a1) - cos(a1)*sin(a2)*sin(a3)) - cos(a1)*cos(a2)*sin(a4), -sin(a1)*sin(a3) - cos(a1)*cos(a3)*sin(a2), -sin(a4)*(cos(a3)*sin(a1) - cos(a1)*sin(a2)*sin(a3)) + cos(a1)*cos(a2)*cos(a4), -(cos(a1)*cos(a2))/4,
                                                                                 0,                                          0,                                                                              0,                    1;

	T47=T04.inverse()*left_Mat2;

	a6=asin(T47(0,2));
	a5=atan2(-T47(1,2)/cos(a6),T47(2,2)/cos(a6));
	a7=atan2(-T47(0,1)/cos(a6),T47(0,0)/cos(a6));

	if(!isnan(a1))
		leftjointangle[0]=a1;
	
	if(!isnan(a2))
		leftjointangle[1]=a2;

	if(!isnan(a3))
		leftjointangle[2]=a3;

	if(!isnan(a4))
		leftjointangle[3]=a4;	

	if(!isnan(a5))
		leftjointangle[4]=a5;

	if(!isnan(a6))
		leftjointangle[5]=a6;

	if(!isnan(a7))
		leftjointangle[6]=a7;
	
	//jointangle[7]=ja4[0]-homeangle[7];
    //printf("home: %12f,%12f,%12f \n",homeangle[0],homeangle[1],homeangle[2]);
	//printf("angle: %12f,%12f \n",
	//nowEular1to2[1]*180/pi,	homeEular1to2[1]*180/pi);

	//printf("angle: %12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f \n",jointangle[0],jointangle[1],jointangle[2],jointangle[3],jointangle[4],jointangle[5],jointangle[6],jointangle[7]);
   	ROS_INFO("left: %12f,%12f,%12f,%12f,%12f,%12f,%12f \n",a1,a2,a3,a4,a5,a6,a7);
	//move_leftarm(leftjointangle);
	//grasper_control(handangle);
	//testEular();
	//exit(0);
}

void pulishtf(void)
{
	static tf::TransformBroadcaster br1,br2,br3,br4,br5,br6;

	tf::Matrix3x3 mat1;
	tf::Vector3 vect1;
	Eigen::Matrix3d basehome;
	Eigen::Matrix3d tempmat;

	basehome= Eigen::AngleAxisd(0, 	Eigen::Vector3d(0, 0, 1))  
		    * Eigen::AngleAxisd(0, 		Eigen::Vector3d(0, 1, 0))
		    * Eigen::AngleAxisd(0, 	Eigen::Vector3d(1, 0, 0));

	ros::spinOnce();

	tf::matrixEigenToTF(basehome, mat1);
    vect1=tf::Vector3(0.3, 0.0, 0);
	//tf::vectorEigenToTF(nowEular0to1, tra);
    br1.sendTransform(tf::StampedTransform(tf::Transform(mat1, vect1), ros::Time::now(), "base_link", "Polhemuslink"));

    tempmat=Mi2j[0];
	tf::matrixEigenToTF(tempmat, mat1);
	vect1=tf::Vector3(0.1, 0, 0);
	br2.sendTransform(tf::StampedTransform(tf::Transform(mat1, vect1), ros::Time::now(), "Polhemuslink","testlink1"));

    tempmat=Mi2j[1];
	tf::matrixEigenToTF(tempmat, mat1);
	vect1=tf::Vector3(0.2, 0.0, 0);
	br3.sendTransform(tf::StampedTransform(tf::Transform(mat1, vect1), ros::Time::now(), "testlink1","testlink2"));

    tempmat=Mi2j[2];
	tf::matrixEigenToTF(tempmat, mat1);
	vect1=tf::Vector3(0.2, 0.0, 0);
	br4.sendTransform(tf::StampedTransform(tf::Transform(mat1,vect1), ros::Time::now(), "testlink2","testlink3"));

    tempmat=Mi2j[3];
	tf::matrixEigenToTF(tempmat, mat1);
	vect1=tf::Vector3(0, 0.0, 0);
	br5.sendTransform(tf::StampedTransform(tf::Transform(mat1, vect1), ros::Time::now(), "testlink3","testlink4"));

	ros::spinOnce();
}



void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(int i = 0;i < 2;i++)
		hand_joints[i] = msg->position[i];

	for(int i = 2;i < 9;i++)
		nowrightjoints[i-2] = msg->position[i];
}

void jointsCallback(const bmirobot_msg::Robot_jointfd::ConstPtr& msg)
{
	for(int i = 0;i < 7;i++)
		nowrightjoints[i] = msg->Joint_fdpst[i];
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "bmirobot_Polhemus_control");

	ros::NodeHandle n("bmirobot");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//ros::Subscriber sub = n.subscribe("/bmirobot/PointsPR", 1000, PointsPRCallback);
    ros::Subscriber sub2 = n.subscribe("/bmirobot/MT_Jointfd", 1000, jointsCallback);

  	leftarm_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/bmirobot/left_group_controller/command", 0 );
  	rightarm_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/bmirobot/right_group_controller/command", 0 );
  	arm_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/bmirobot/group_controller/command", 0 );
   	grasp_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/bmirobot/group_hand_controller/command", 0 );

 	ros::Subscriber sub1 = n.subscribe("/bmirobot/joint_states", 1000, jointStateCallback);

    time_t t = time(0);

    char tmp[100];
	struct tm *info;
	info = localtime(&t);
	sprintf(tmp,"/home/bmi/Dropbox/ws_moveit/data/new-PolhemusControl.csv");
    std::cout<<tmp<<endl;

	std::ofstream out(tmp);
    out<<"count,time,m1pst,m1spd,m1tq,m1pst,m1spd,m1ctr,m2pst,m2spd,m2tq,m2pst,m2spd,m2ctr"<<endl;

	int step=0;
	int count=0;
	//testEular();


    ros::Time begininit= ros::Time::now();
	double secsinit = begininit.toSec();

    limitjoint();

	while(ros::ok())
	{
		switch(step)
		{
			case 0:
				//printf("first step\n");
                move_rightarm(rightjointangle);
				if(receivedata==1)
				if(countAverage()==1)
				{
					step=1;	
				}
			break;

			case 1:
				printf("second step\n");
				homeEular();
				step=2;
			break;

			case 2:
				getmatrix();
				IK_right_control(nowrightjoints,rightjointangle);
	            move_rightarm(rightjointangle);


	            //grasper_control(handangle);
				//IK_left_control();
				//pulishtf();
				//IK_control();
				//angleupdata();
				//particle_filter();
			break;
		}
		ros::spinOnce();

		ros::Time begin = ros::Time::now();
      	double secs = begin.toSec()-secsinit;

		out <<begin<<",";
		for(int i=0;i<4;i++)
		{
			out <<Px[i]<<","<<Py[i]<<","<<Pz[i]<<","<<Rx[i]<<","<<Ry[i]<<","<<Rz[i]<<",";
		}
		out <<endl;
		usleep(1000);
	}
}




