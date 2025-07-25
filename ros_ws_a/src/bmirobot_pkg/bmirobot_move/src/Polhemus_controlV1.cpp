
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

#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>

#include <bmirobot_tools/PointsPR.h>

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
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::atan;




bmirobot_tools::PointsPR pointmsg;

ros::Publisher grasp_joint_pub,arm_joint_pub;
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

Eigen::Matrix<double, 3, 3> PointM[4];
Eigen::Matrix3d MRhome[4];
Eigen::Matrix3d	MRnow[4];
Eigen::Matrix3d	inithome;
Eigen::Matrix3d Mi2j[4];
Eigen::Vector3d Eulari2j[4];

Eigen::Matrix3d	M0to1,M1to2,M2to3,M3to4;
Eigen::Matrix3d initjoint1,initjoint2,initjoint3,initjoint4,initjoint5;

Eigen::Matrix3d homeMat0to1,homeMat1to2,homeMat2to3,homeMat3to4;

Eigen::Vector3d nowEular0to1,nowEular1to2,nowEular2to3,nowEular3to4;
Eigen::Vector3d homeEular0to1,homeEular1to2,homeEular2to3,homeEular3to4;

Eigen::Vector3d store0,store1,store2,store3;

float jointangle[7],handangle;

//Eigen::Matrix<double, 3, 3> J_vect, M_vect,M_cmd;

int16_t msgcount=0;
void PointsPRCallback(const bmirobot_tools::PointsPR::ConstPtr& msg)
{
	int i;
	for(i=0;i<4;i++)
	{
		Px[i]=msg->Px[i]/1.0;
		Py[i]=msg->Py[i]/1.0;
		Pz[i]=msg->Pz[i]/1.0;
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


void move_arm(float jointS[7])
{
	std_msgs::Float64MultiArray msg;
	for(int i = 0;i< 7;i++)
	{
		msg.data.push_back(jointS[i]);
	}

	arm_joint_pub.publish(msg);
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

Eigen::Vector3d normEular(Eigen::Vector3d vect,Eigen::Vector3d *lastvect)
{
	Eigen::Vector3d vecttemp;
	Eigen::Vector3d lasttemp=*lastvect;

	if(((lasttemp[0]-vect[0])<pi/2)&((lasttemp[0]-vect[0])>-pi/2))
	{

	}else
	{
		if(vect[0]<pi/2	)
			lasttemp[1]=0;
		else
			lasttemp[1]=1;	
		//printf("change %f,",lasttemp[1]);
	}
	lasttemp[0]=vect[0];

	if(lasttemp[1]==0)
	{
		vecttemp[0]=-vect[0];
		vecttemp[1]=-vect[1];		
		vecttemp[2]=-vect[2];
		
	}else
	{
		vecttemp[0]=pi-vect[0];
		if(vect[1]>0)
		{
			vecttemp[1]=vect[1]-pi;
		}else
		{
			vecttemp[1]=vect[1]+pi;
		}
		if(vect[2]>0)
		{
			vecttemp[2]=pi-vect[2];
		}else
		{
			vecttemp[2]=-vect[2]-pi;
		}
	}

	*lastvect=lasttemp;
	//printf("change %f,old %f,%f,%f   new %f,%f,%f \n",lasttemp[1],vect(0)*180/pi,vect(1)*180/pi,vect(2)*180/pi,vecttemp(0)*180/pi,vecttemp(1)*180/pi,vecttemp(2)*180/pi);
	return vecttemp;
}


void homeEular()
{
	
	initjoint1=	Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1))
        	  * Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0)) 
        	  * Eigen::AngleAxisd(0, Eigen::Vector3d(1, 0, 0));
	
	initjoint2=Eigen::AngleAxisd(-pi/2, Eigen::Vector3d(0, 0, 1))  
        	   * Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))  
        	   * Eigen::AngleAxisd(pi/2, Eigen::Vector3d(1, 0, 0)); 
	
	initjoint3=Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1))  
        	   * Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))  
        	   * Eigen::AngleAxisd(pi/2, Eigen::Vector3d(1, 0, 0)); 
	
	
	initjoint4=Eigen::AngleAxisd(0/2, Eigen::Vector3d(0, 0, 1))  
        	   * Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))  
        	   * Eigen::AngleAxisd(pi/2, Eigen::Vector3d(1, 0, 0)); 
	
	initjoint5=Eigen::AngleAxisd(0/2, Eigen::Vector3d(0, 0, 1))  
        	   * Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))  
        	   * Eigen::AngleAxisd(-pi/2, Eigen::Vector3d(1, 0, 0)); 
	
	for(int i=0;i<4;i++)
	{
		MRhome[i]= Eigen::AngleAxisd(homePoint[i][5], Eigen::Vector3d(0, 0, 1))  
        		 * Eigen::AngleAxisd(homePoint[i][4], Eigen::Vector3d(0, 1, 0))  
        		 * Eigen::AngleAxisd(homePoint[i][3], Eigen::Vector3d(1, 0, 0)); 			
	}
	
	homeMat0to1 =MRhome[0];
	//homeMat1to2 =MRhome[0].inverse()*MRhome[1];
    
	homeMat1to2=(MRhome[0]*initjoint2).inverse()*MRhome[1]*initjoint3;
	//homeMat1to2=(MRhome[0]).inverse()*MRhome[1];
	homeEular1to2 	= 	(homeMat1to2).eulerAngles(2,1,0);
	homeEular1to2 	=	normEular(homeEular1to2,&store1);
    
	homeMat2to3=(MRhome[1]).inverse()*MRhome[2];
	homeEular2to3 	= 	(homeMat2to3).eulerAngles(2,1,0);
	homeEular2to3 	=	normEular(homeEular2to3,&store2);
    
	homeMat3to4=(MRhome[2]*initjoint4).inverse()*(MRhome[3]*initjoint5);
	//homeMat1to2=(MRhome[2]).inverse()*MRhome[3];
	homeEular3to4 	= (homeMat3to4).eulerAngles(2,1,0);
	homeEular3to4  	=	normEular(homeEular3to4,&store3);
    
	//cout<<endl<< inithome <<endl<< endl; 
	
	//homeEularangle[2] = MRhome[2].eulerAngles(1,2,0);
	
	//std::cout<< MRhome[2] << endl<<homeEularangle[2]<< endl; 
	bestp[0][0]=0;
	bestp[0][1]=0;
	bestp[0][2]=0;


	bestp[1][0]=homeEular1to2[0];
	bestp[1][1]=homeEular1to2[1];
	bestp[1][2]=homeEular1to2[2];

	bestp[2][0]=homeEular2to3[0];
	bestp[2][1]=homeEular2to3[1];
	bestp[2][2]=homeEular2to3[2];

	bestp[3][0]=homeEular3to4[0];
	bestp[3][1]=homeEular3to4[1];
	bestp[3][2]=homeEular3to4[2];
	
  	std::cout<<bestp[3][2]<<endl;

	//exit(0);
}


void testEular()
{

	tf::Matrix3x3 mat1;
	tf::Vector3 vect1;
	Eigen::Matrix3d basehome;
	int ii;

	initjoint1=	Eigen::AngleAxisd(-pi/2, Eigen::Vector3d(0, 0, 1))  
        	  * Eigen::AngleAxisd(-pi/2, Eigen::Vector3d(0, 1, 0))  
        	  * Eigen::AngleAxisd(0, Eigen::Vector3d(1, 0, 0)); 
	
	initjoint2=Eigen::AngleAxisd(-pi/2, Eigen::Vector3d(0, 0, 1))  
        	   * Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))  
        	   * Eigen::AngleAxisd(pi/2, Eigen::Vector3d(1, 0, 0)); 
	
	Eigen::Matrix3d	matrix1,matrix2,matrix3,matrix4;
	Eigen::Matrix3d	abc1,abc2,abc3;
    
	matrix1=Eigen::AngleAxisd(-90.671/180.0*pi, Eigen::Vector3d(0, 0, 1))
          * Eigen::AngleAxisd(-74.718/180*pi, Eigen::Vector3d(0, 1, 0))  
		  * Eigen::AngleAxisd(44.347/180*pi, Eigen::Vector3d(1, 0, 0));  
    
	matrix2=Eigen::AngleAxisd(137.124/180*pi, Eigen::Vector3d(0, 0, 1)) 
          * Eigen::AngleAxisd(3.198/180*pi, Eigen::Vector3d(0, 1, 0))   
		  * Eigen::AngleAxisd(107.746/180*pi, Eigen::Vector3d(1, 0, 0));

	Eigen::Vector3d vect;

	abc1=initjoint2.inverse()*matrix1;
	abc2=initjoint2.inverse()*matrix2;
    
	abc3=abc2-abc1;
	srand((unsigned)time(NULL));
	double det1 = initjoint1.determinant();
	double det2 = abc3.determinant();
	double det3 = abc3.norm();
	std::cout<< abc1 <<endl;
	std::cout<< abc2 <<endl;
	std::cout<< abc3 <<endl;
	std::cout<< rand()%1000<<endl;
	std::cout<< rand()%1000<<endl;


	exit(0);
}

void getmatrix()
{
	for(int i=0;i<4;i++)
	{
		MRnow[i]= Eigen::AngleAxisd(Rz[i], Eigen::Vector3d(0, 0, 1))  
				* Eigen::AngleAxisd(Ry[i], Eigen::Vector3d(0, 1, 0))  
				* Eigen::AngleAxisd(Rx[i], Eigen::Vector3d(1, 0, 0));
		//ROS_INFO("new angle ,%f,%f,%f",Rz[i],Ry[i],Rx[i]);
	}

	Mi2j[0]=	homeMat0to1.inverse()*MRnow[0];
	Mi2j[1]=(MRnow[0]*initjoint2).inverse()*MRnow[1]*initjoint3;
	Mi2j[2]=(MRnow[1]).inverse()*MRnow[2];
	Mi2j[3]=(MRnow[2]*initjoint4).inverse()*(MRnow[3]*initjoint5);

/*
	printf("angle: %10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f \n",
		Rx[0]*180/pi,	Ry[0]*180/pi,	Rz[0]*180/pi,
		Rx[1]*180/pi,	Ry[1]*180/pi,	Rz[1]*180/pi,
		Rx[2]*180/pi,	Ry[2]*180/pi,	Rz[2]*180/pi,
		Rx[3]*180/pi,	Ry[3]*180/pi,	Rz[3]*180/pi);
*/
	Eulari2j[0] = (M0to1.inverse()).eulerAngles(2,1,0);
	//Eulari2j[0] =	normEular(Eulari2j[0],&store0);

	Eulari2j[1] 	= 	(M1to2).eulerAngles(2,1,0);
	//Eulari2j[1]	=	normEular(Eulari2j[1],&store1);

	Eulari2j[2] 	= 	(M2to3).eulerAngles(2,1,0);
	//std::cout<< nowEular2to3 <<endl<<endl;
	//Eulari2j[2] 	= 	normEular(Eulari2j[2],&store2);
	//std::cout<< nowEular2to3 <<endl<<endl;

	Eulari2j[3] 	= 	(M3to4).eulerAngles(2,1,0);
	//Eulari2j[3]	=	normEular(Eulari2j[3],&store3);

}

double atan2(double d1,double d2)
{
	if (d1>0)
	{


	}else
	{

	}	

}

void inverse_angle()
{
	std::cout<< Mi2j[0] <<endl<<endl;


	jointangle[1]=asin(-Mi2j[0](0,0));

	std::cout<< jointangle[1] <<endl<<endl;

	jointangle[2]=atan(-Mi2j[0](1,1));	

	//std::cout<< joint1to2 <<endl<<endl;
	//jointangle[0]=-(nowEular0to1[0]+nowEular0to1[2])




	jointangle[0]=-(nowEular0to1[0]);
	jointangle[1]=-(nowEular0to1[1]);
	jointangle[2]=(nowEular0to1[2]+nowEular1to2[0]-homeEular1to2[0]);
	//jointangle[2]=(nowEular1to2[0]-homeEular1to2[0]);
	jointangle[3]=-(nowEular1to2[1]-homeEular1to2[1]);
	jointangle[4]=-(nowEular1to2[2]-homeEular1to2[2]);
	jointangle[5]=-(nowEular2to3[0]-homeEular2to3[0]+(nowEular2to3[2]-homeEular2to3[2]));
	jointangle[5]=-(nowEular2to3[0]-homeEular2to3[0]);
	jointangle[6]=(nowEular2to3[1]-homeEular2to3[1]);
	handangle=-(nowEular3to4[0]-homeEular3to4[0]);

	//printf("angle: %12f,%12f \n",
	//nowEular1to2[1]*180/pi,	homeEular1to2[1]*180/pi);



/*	printf("angle1: %10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f \n",
		nowEular0to1[0]*180/pi,	nowEular0to1[1]*180/pi,	nowEular0to1[2]*180/pi,
		nowEular1to2[0]*180/pi,	nowEular1to2[1]*180/pi,	nowEular1to2[2]*180/pi,
		nowEular2to3[0]*180/pi,	nowEular2to3[1]*180/pi, nowEular2to3[2]*180/pi,
		nowEular3to4[0]*180/pi);
*/

	printf("angle: %12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f \n",jointangle[0],jointangle[1],jointangle[2],jointangle[3],jointangle[4],jointangle[5],jointangle[6],jointangle[7]);
   
	move_arm(jointangle);
	grasper_control(handangle);
	//testEular();
	//exit(0);
}



void angleupdata()
{


	M0to1=	homeMat0to1.inverse()*MRnow[0];
	nowEular0to1 = (M0to1.inverse()).eulerAngles(2,1,0);
	//nowEular0to1 =	normEular(nowEular0to1,&store0);

	M1to2=(MRnow[0]*initjoint2).inverse()*MRnow[1]*initjoint3;
	nowEular1to2 	= 	(M1to2).eulerAngles(2,1,0);
	//nowEular1to2	=	normEular(nowEular1to2,&store1);

	M2to3=(MRnow[1]).inverse()*MRnow[2];
	nowEular2to3 	= 	(M2to3).eulerAngles(2,1,0);
	//std::cout<< nowEular2to3 <<endl<<endl;
	//nowEular2to3 	= 	normEular(nowEular2to3,&store2);
	//std::cout<< nowEular2to3 <<endl<<endl;

	M3to4=(MRnow[2]*initjoint4).inverse()*(MRnow[3]*initjoint5);
	nowEular3to4 	= 	(M3to4).eulerAngles(2,1,0);
	//nowEular3to4	=	normEular(nowEular3to4,&store3);

	std::cout<< M0to1 <<endl<<endl;
	//std::cout<< joint1to2 <<endl<<endl;
	//jointangle[0]=-(nowEular0to1[0]+nowEular0to1[2]);
	jointangle[0]=-(nowEular0to1[0]);
	jointangle[1]=-(nowEular0to1[1]);
	jointangle[2]=(nowEular0to1[2]+nowEular1to2[0]-homeEular1to2[0]);
	//jointangle[2]=(nowEular1to2[0]-homeEular1to2[0]);
	jointangle[3]=-(nowEular1to2[1]-homeEular1to2[1]);
	jointangle[4]=-(nowEular1to2[2]-homeEular1to2[2]);
	jointangle[5]=-(nowEular2to3[0]-homeEular2to3[0]+(nowEular2to3[2]-homeEular2to3[2]));
	jointangle[5]=-(nowEular2to3[0]-homeEular2to3[0]);
	jointangle[6]=(nowEular2to3[1]-homeEular2to3[1]);
	handangle=-(nowEular3to4[0]-homeEular3to4[0]);

	//printf("angle: %12f,%12f \n",
	//nowEular1to2[1]*180/pi,	homeEular1to2[1]*180/pi);

	printf("angle: %10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f \n",
		Rx[0]*180/pi,	Ry[0]*180/pi,	Rz[0]*180/pi,
		Rx[1]*180/pi,	Ry[1]*180/pi,	Rz[1]*180/pi,
		Rx[2]*180/pi,	Ry[2]*180/pi,	Rz[2]*180/pi,
		Rx[3]*180/pi,	Ry[3]*180/pi,	Rz[3]*180/pi);

	printf("angle1: %10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f \n",
		nowEular0to1[0]*180/pi,	nowEular0to1[1]*180/pi,	nowEular0to1[2]*180/pi,
		nowEular1to2[0]*180/pi,	nowEular1to2[1]*180/pi,	nowEular1to2[2]*180/pi,
		nowEular2to3[0]*180/pi,	nowEular2to3[1]*180/pi, nowEular2to3[2]*180/pi,
		nowEular3to4[0]*180/pi);

	//printf("angle: %12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f \n",jointangle[0]*180/pi,jointangle[1]*180/pi,jointangle[2]*180/pi,jointangle[3]*180/pi,jointangle[4]*180/pi,jointangle[5]*180/pi,jointangle[6]*180/pi,jointangle[7]*180/pi);
   
	move_arm(jointangle);
	grasper_control(handangle);
	//testEular();
	//exit(0);
}



void particle_filter(void)
{ 
	Eigen::Matrix3d tempmat,diffmat;
	double time1=0,time2,timerd[4];

	std::normal_distribution<double> grand(0, 0.001);

    double diff1p[particle_num];
	double bestdiff1;

	int  bestindex[4];
	double besttemp[4];
	int algotime=0,times[4];

//  ROS_INFO("time1");
	for(int jj=0;jj<4;jj++)
	{
		time1= (ros::Time::now()).toSec();
		do
		{
			algotime++;
			for(int i=0;i<particle_num;i++)
			{
				praticle[jj][i][0]=bestp[jj][0] + (grand)(eng);
				praticle[jj][i][1]=bestp[jj][1] + (grand)(eng);
				praticle[jj][i][2]=bestp[jj][2] + (grand)(eng);
			}

			//std::cout<<jj <<" rand  " <<(double)rand() / RAND_MAX*0.01-0.05<<endl;
			for(int i=0;i<particle_num;i++)
			{
				tempmat= Eigen::AngleAxisd(praticle[jj][i][0], Eigen::Vector3d(0, 0, 1))
					   * Eigen::AngleAxisd(praticle[jj][i][1], Eigen::Vector3d(0, 1, 0))
					   * Eigen::AngleAxisd(praticle[jj][i][2], Eigen::Vector3d(1, 0, 0));
				diffmat=Mi2j[jj]-tempmat;
				diff1p[i]=diffmat.norm();
			}

			besttemp[jj]	=diff1p[0];
			bestindex[jj]	=0;
			for(int i=0;i<particle_num;i++)
			{
				if(besttemp[jj]>diff1p[i])
				{
					besttemp[jj]  =diff1p[i];
					bestindex[jj] =i;
				}
			}

			if(besttemp[jj]>0.1)
				std::normal_distribution<double> grand(0, 0.1);
			if(besttemp[jj]>0.01)
				std::normal_distribution<double> grand(0, besttemp[jj]);
			else
				std::normal_distribution<double> grand(0, 0.01);

			bestp[jj][0]=praticle[jj][bestindex[jj]][0];
			bestp[jj][1]=praticle[jj][bestindex[jj]][1];
			bestp[jj][2]=praticle[jj][bestindex[jj]][2];
			//ROS_INFO("min: %f,%f,\n",(ros::Time::now()).toSec(),grand.stddev());

		}while((algotime<50)&&(besttemp[jj]>0.003));

		times[jj]=algotime;
		algotime=0;
		time2= ros::Time::now().toSec();
		timerd[jj]=time2-time1;
		//std::cout<<jj <<" index  " <<bestindex1<<endl;
		//std::cout<<jj <<" min  " << besttemp1<<endl;
		//std::cout<<jj <<" angle  " <<bestp[jj][0]<<"   "<<bestp[jj][1]<<"   "<<bestp[jj][2]<<endl;
	}

	//ROS_INFO("min: %12f,%12f,%12f,%12f,%d,%d,%d,%d,\n",besttemp[0],besttemp[1],besttemp[2],besttemp[3],times[0],times[1],times[2],times[3]);
	//ROS_INFO("min: %12f,%12f,%12f,%12f,\n",timerd[0],timerd[1],timerd[2],timerd[3]);
	
	ROS_INFO("min: %12f,%12f,%12f,%d,\n",bestp[0][0],bestp[0][1],bestp[0][2],times[0]);

	nowEular0to1[0]=bestp[0][0];
	nowEular0to1[1]=bestp[0][1];
	nowEular0to1[2]=bestp[0][2];

	nowEular1to2[0]=bestp[1][0];
	nowEular1to2[1]=bestp[1][1];
	nowEular1to2[2]=bestp[1][2];

	nowEular2to3[0]=bestp[2][0];
	nowEular2to3[1]=bestp[2][1];
	nowEular2to3[2]=bestp[2][2];

	nowEular3to4[0]=bestp[3][0];
	nowEular3to4[1]=bestp[3][1];
	nowEular3to4[2]=bestp[3][2];

	jointangle[0]=-(nowEular0to1[0]);
	jointangle[1]=-(nowEular0to1[1]);
	jointangle[2]=(nowEular0to1[2]+nowEular1to2[0]-homeEular1to2[0]);
	//jointangle[2]=(nowEular1to2[0]-homeEular1to2[0]);
	jointangle[3]=-(nowEular1to2[1]-homeEular1to2[1]);
	jointangle[4]=-(nowEular1to2[2]-homeEular1to2[2]);
	jointangle[5]=-(nowEular2to3[0]-homeEular2to3[0]+(nowEular2to3[2]-homeEular2to3[2]));
	jointangle[5]=-(nowEular2to3[0]-homeEular2to3[0]);
	jointangle[6]=(nowEular2to3[1]-homeEular2to3[1]);
	handangle=-(nowEular3to4[0]-homeEular3to4[0]);

	//printf("angle: %12f,%12f \n",
	//nowEular1to2[1]*180/pi,	homeEular1to2[1]*180/pi);

	printf("angle: %10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f \n",
		Rx[0]*180/pi,	Ry[0]*180/pi,	Rz[0]*180/pi,
		Rx[1]*180/pi,	Ry[1]*180/pi,	Rz[1]*180/pi,
		Rx[2]*180/pi,	Ry[2]*180/pi,	Rz[2]*180/pi,
		Rx[3]*180/pi,	Ry[3]*180/pi,	Rz[3]*180/pi);

	printf("new angle: %10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f \n",
		nowEular0to1[0]*180/pi,	nowEular0to1[1]*180/pi,	nowEular0to1[2]*180/pi,
		nowEular1to2[0]*180/pi,	nowEular1to2[1]*180/pi,	nowEular1to2[2]*180/pi,
		nowEular2to3[0]*180/pi,	nowEular2to3[1]*180/pi, nowEular2to3[2]*180/pi,
		nowEular3to4[0]*180/pi);


	//printf("angle: %12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f \n",jointangle[0]*180/pi,jointangle[1]*180/pi,jointangle[2]*180/pi,jointangle[3]*180/pi,jointangle[4]*180/pi,jointangle[5]*180/pi,jointangle[6]*180/pi,jointangle[7]*180/pi);
    
	move_arm(jointangle);
	grasper_control(handangle);
}


void pulishtf(void)
{
	static tf::TransformBroadcaster br1,br2,br3,br4,br5,br6;

	tf::Matrix3x3 mat1;
	tf::Vector3 vect1;
	Eigen::Matrix3d basehome;
	Eigen::Matrix3d tempmat;

	basehome= Eigen::AngleAxisd(0, 	Eigen::Vector3d(0, 0, 1))  
		    * Eigen::AngleAxisd(pi/2, 		Eigen::Vector3d(0, 1, 0))  
		    * Eigen::AngleAxisd(pi, 	Eigen::Vector3d(1, 0, 0));

	ros::spinOnce();

	tf::matrixEigenToTF(basehome, mat1);
    vect1=tf::Vector3(0.3, 0.0, 0);
	//tf::vectorEigenToTF(nowEular0to1, tra);
    br1.sendTransform(tf::StampedTransform(tf::Transform(mat1, vect1), ros::Time::now(), "base_link", "Polhemuslink"));

    tempmat=(homeMat0to1).inverse()*MRnow[0];
	tf::matrixEigenToTF(tempmat, mat1);
	vect1=tf::Vector3(0, 0, -0.1);
	br2.sendTransform(tf::StampedTransform(tf::Transform(mat1, vect1), ros::Time::now(), "Polhemuslink","testlink1"));

    tempmat=(MRnow[0]).inverse()*MRnow[1];
	tf::matrixEigenToTF(tempmat, mat1);
	vect1=tf::Vector3(0.2, 0.0, 0);
	br3.sendTransform(tf::StampedTransform(tf::Transform(mat1, vect1), ros::Time::now(), "testlink1","testlink2"));

    tempmat=(MRnow[1]).inverse()*MRnow[2];
	tf::matrixEigenToTF(tempmat, mat1);
	vect1=tf::Vector3(0.2, 0.0, 0);
	br4.sendTransform(tf::StampedTransform(tf::Transform(mat1,vect1), ros::Time::now(), "testlink2","testlink3"));

    tempmat=(MRnow[2]).inverse()*MRnow[3];
	tf::matrixEigenToTF(tempmat, mat1);
	vect1=tf::Vector3(0.1, 0.0, 0);
	br5.sendTransform(tf::StampedTransform(tf::Transform(mat1, vect1), ros::Time::now(), "testlink3","testlink4"));

	ros::spinOnce();
}



void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(int i = 0;i < 2;i++)
		hand_joints[i] = msg->position[i];

	for(int i = 0;i < 9;i++)
		arm_joints[i-2] = msg->position[i];
}






int main(int argc, char **argv)
{
	ros::init(argc, argv, "bmirobot_Polhemus_control");

	ros::NodeHandle n("bmirobot");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Subscriber sub = n.subscribe("/bmirobot/PointsPR", 1000, PointsPRCallback);
  	grasp_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/bmirobot/group_hand_controller/command", 0 );
  	arm_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/bmirobot/group_controller/command", 0 );
  	ros::Subscriber sub1 = n.subscribe("/bmirobot/joint_states", 1000, jointStateCallback);

    time_t t = time(0)
;
    char tmp[100];
	struct tm *info;
	info = localtime(&t);
	sprintf(tmp,"/home/bmi/Dropbox/ws_moveit/data/new-PolhemusControl.csv", 1900+info->tm_year, 1+info->tm_mon, info->tm_mday,info->tm_hour,info->tm_min,info->tm_sec);
    std::cout<<tmp<<endl;

	std::ofstream out(tmp);
    out<<"count,time,m1pst,m1spd,m1tq,m1pst,m1spd,m1ctr,m2pst,m2spd,m2tq,m2pst,m2spd,m2ctr"<<endl;

	int step=0;
	int count=0;
	//testEular();


    ros::Time begininit= ros::Time::now();
	double secsinit = begininit.toSec();

	while(ros::ok())
	{
		switch(step)
		{
			case 0:
				printf("first step\n");
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
				pulishtf();
				inverse_angle();
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




