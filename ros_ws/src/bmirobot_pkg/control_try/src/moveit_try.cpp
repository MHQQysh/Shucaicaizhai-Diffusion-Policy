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

#define pi 3.14
// inverse motion and key
using namespace std;
using std::string;

vector<double> data = {0, 0, 0, 0, 0, 0, 0};
vector<double> data_h = {0, 0};

float ljoint_limition[7][2], rjoint_limition[7][2];
double right_joint[7]={0};  // vector
double left_joint[7]={0};
double now_right[7];   // now_right:joint
double lastrightjoints[7];  //
double interrightjoints[7];
double a0[7],a2[7],a3[7];
float Px=0,Py=0,Pz=0,Rx=0,Ry=0,Rz=0; //(Rx,Ry,Rz):angle; (Px,Py,Pz):length
Eigen::Matrix4d right_initMat1,right_initMat2,right_Mat1,right_Mat2;
Eigen::Matrix4d right1_init1,right1_init2,right2_init1,right2_init2;
Eigen::Matrix4d right1_init,right2_init;
Eigen::Matrix4d right_base;
Eigen::Matrix3d MRhome,MRnow;
Eigen::Vector3d MPnow,MPhome;
float rad=0.017;
int num = 3;

float difft=0;

// return: about i (angle space):4 points
void polyline_angle()
{
	int tf = 2;
	for (int i=0;i<7;i++)
	{
		a0[i] = lastrightjoints[i];
		a2[i] = 3/(tf*tf)*(right_joint[i]-lastrightjoints[i]);
		a3[i] = 2/(tf*tf*tf)*(lastrightjoints[i]-right_joint[i]);
		interrightjoints[i] = a0[i]+a2[i]*difft*difft+a3[i]*difft*difft*difft;
	}
}

// init angle
void homeEular()
{
	Eigen::Matrix4d  Mat4d;

	right_base<<	1, 		0, 		0, 		0,
					0, 		0, 		1, 		-0.22,
					0,		-1,		0,		0,
					0,		0,		0,		1.0;

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
	right2_init2<<	1, 		0, 		0, 		0.00,
					0, 		1, 		0, 		0,
					0,		0,		1,		0,
					0,		0,		0,		1.0;
	right2_init=right2_init1*right2_init2;
	
	right_initMat1	=Eigen::Matrix4d::Identity(4,4);
	right_initMat2	=Eigen::Matrix4d::Identity(4,4);

	//std::cout<<"left_initMat1"<<endl<<left_initMat1<<endl;

		MRhome= Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1))
        		 * Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))
        		 * Eigen::AngleAxisd(0, Eigen::Vector3d(1, 0, 0));

		MPhome(0)=0;
		MPhome(1)=0;
		MPhome(2)=0;

		Mat4d=Eigen::Matrix4d::Identity(4,4);
		Mat4d.block<3,3>(0,0)=MRhome;
		Mat4d.block<3,1>(0,3)=MPhome;

	//right_initMat1=Mat4d[1]*right1_init.inverse();
	right_initMat2=Mat4d*right2_init.inverse();			// sensor original to sensor and sensor to robot original, right2_init.inverse(): according to the fixed axies

	std::cout<<"right_initMat2"<<endl<<right_initMat2<<endl;
	//exit(1);

}


// read the limition of URDF
void readlimitjoint()
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

	std::shared_ptr<const urdf::Link> link = model.getLink("left_link1");
	std::cout<< link->child_joints[0]->name << " \n";
	ljoint_limition[0][0] = link->child_joints[0]->limits->lower;
	ljoint_limition[0][1] = link->child_joints[0]->limits->upper;
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
		ljoint_limition[i+1][0] = link->child_joints[index]->limits->lower;
		ljoint_limition[i+1][1] = link->child_joints[index]->limits->upper;
	}
	ljoint_limition[8][0] = -1*ljoint_limition[7][1];
	ljoint_limition[8][1] = -1*ljoint_limition[7][0];
	
	link = model.getLink("right_link1");
	std::cout<< link->child_joints[0]->name << " \n";
	rjoint_limition[0][0] = link->child_joints[0]->limits->lower;
	rjoint_limition[0][1] = link->child_joints[0]->limits->upper;
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
		rjoint_limition[i+1][0] = link->child_joints[index]->limits->lower;
		rjoint_limition[i+1][1] = link->child_joints[index]->limits->upper;
	}
	rjoint_limition[8][0] = -1*rjoint_limition[7][1];
	rjoint_limition[8][1] = -1*rjoint_limition[7][0];
}

// transform, lonely input
void getmatrix()
{
	Eigen::Matrix4d  Matnow4d;
	
		MRnow= Eigen::AngleAxisd(Rz, Eigen::Vector3d(0, 0, 1))
				* Eigen::AngleAxisd(Ry, Eigen::Vector3d(0, 1, 0))
				* Eigen::AngleAxisd(Rx, Eigen::Vector3d(1, 0, 0));

		MPnow(0)=Px;
		MPnow(1)=Py;
		MPnow(2)=Pz;

		Matnow4d	=Eigen::Matrix4d::Identity(4,4);
		Matnow4d.block<3,3>(0,0)=MRnow;
		Matnow4d.block<3,1>(0,3)=MPnow;

	right_Mat2=right_initMat2.inverse()*Matnow4d*right2_init2.inverse();

}


// IK_control
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

	double na1=lastrightjoints[0],na2=lastrightjoints[1],na3=lastrightjoints[2],na4=lastrightjoints[3],na5=lastrightjoints[4],na6=lastrightjoints[5],na7=lastrightjoints[6];
	double nowerr=abs(na1)+abs(na2)+abs(na3)+abs(na4)+abs(na5)+abs(na6)+abs(na7);
	double goals[100][7],goalerr[100];

	std::cout<<"right_Mat2="<<endl<<right_Mat2<<endl;

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
		a1= ta2-sa2;  // ??

		tmat1  <<	0.0, 				cos(a2)*6/25, 			sin(a2)*-6/25,			
					cos(a1)*6/25,	sin(a1)*sin(a2)*6/25,	cos(a2)*sin(a1)*6/25,
					sin(a1)*6/25,	cos(a1)*sin(a2)*-6/25,	cos(a1)*cos(a2)*-6/25;   // coordinate??

		tmat2<<  	(sin(a2)/4)-(121.0/1000)	+x2, 
					cos(a2)*sin(a1)/-4	+y2,
					cos(a1)*cos(a2)/4	+z2;
		
		tmat3=tmat1.inverse()*tmat2;
		//std::cout<<"tmat2="<<endl<<tmat2<<endl;
		//std::cout<<"tmat3="<<endl<<tmat3<<endl;
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
        //printf("err: %12d,%12f,%12f,%12f,%12f,%12f,%12f,%12f\n",i,goalerr[i],a1,a2,a3,a4,a5,a6,a7);
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

	printf("right: %d, %12f, %12f,%12f,%12f,%12f,%12f,%12f,%12f \n",index,goalerr[index],mtgoal[0],mtgoal[1],mtgoal[2],mtgoal[3],mtgoal[4],mtgoal[5],mtgoal[6]);
	//std::cout<<"T04="<<endl<<T04<<endl;
   
}


// feedback_1:sub1 (sensor/motor)
void jointStateCallback(const bmirobot_msg::Robot_jointfd::ConstPtr& msg)
{
	for(int i = 0;i < 7;i++)
		now_right[i] = msg->Joint_fdpst[i];
}

// limit the joint
void armjointupdate()
{
	for(int i=0;i < 7;i++)
    {
		if(right_joint[i]< rjoint_limition[i][0])
			right_joint[i]= rjoint_limition[i][0];
		if(right_joint[i]> ljoint_limition[i][1])
			right_joint[i]= rjoint_limition[i][1];
    }
    for(int i=0;i < 7;i++)
    {
		if(left_joint[i]< ljoint_limition[i][0])
			left_joint[i]= ljoint_limition[i][0];
		if(left_joint[i]> ljoint_limition[i][1])
			left_joint[i]= ljoint_limition[i][1];
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

// vector(data groups) to the data
void vectorTodata(double *mtgoal)
{
	for(int i=0;i<7;i++){
	data[i]=mtgoal[i];
	}
}

//void data_read(const std_msgs::Float64MultiArray & msg)
//{
//    Px=msg.data[0];
//    Py=msg.data[1];
//    Pz=msg.data[2];
//    Rx=0;
//    Ry=0;
//    Rz=0;
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_fk_demo");
    ros::NodeHandle n("bmirobot");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    readlimitjoint();    // read the limition of the joint
    homeEular();
    int k;
    ros::Subscriber sub1 = n.subscribe("/bmirobot/joint_states", 1000, jointStateCallback);  // state of the joint
    ros::Publisher rightarm_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/bmirobot/right_group_controller/command", 0 );
    ros::Publisher chatter_pub3 = n.advertise<std_msgs::Float64MultiArray>("/bmirobot/right_group_hand_controller/command", 1000);
    //ros::Subscriber sub2 = n.subscribe("/angle", 1000, data_read);  // state of the joint
    std_msgs::Float64MultiArray msg;
    std_msgs::Float64MultiArray msg_h;
    while(ros::ok())
    {
    	char c = getch();
    	if (c==119)   // up:w
    	{
    	  Px=Px+0.01;
    	}
    	else if(c==115)  // down:s
    	{
    	  Px=Px-0.01;
    	}
    	else if(c==97)    // left:a
    	{
    	  Py=Py+0.01;
    	}
    	else if(c==100)   // right:d
    	{
    	  Py=Py-0.01;
    	}
    	else if(c==113)   // forward:q
    	{
    	  Pz=Pz+0.01;
    	}
    	else if(c==101)   // backward:e
    	{
    	 Pz=Pz-0.01;
    	}
    	else if(c==106){
    	 Rx=Rx+rad*num;
    	}   // Rx
    	else if(c==108){
    	 Rx=Rx-rad*num;
    	}   // Rx
    	else if(c==105){
    	 Ry=Ry+rad*num;
    	}   // Rx
    	else if(c==107){
    	 Ry=Ry-rad*num;
    	}   // Rx
    	else if(c==117){
    	 Rz=Rz+rad*num;
    	}   // Rx
    	else if(c==111){
    	 Rz=Rz-rad*num;
    	}   // Rx
    	getmatrix();
    	IK_right_control(now_right,right_joint);
    	armjointupdate();
    	for(k=0;k<7;k++){
    	  data[k]=right_joint[k];
    	}
    	msg.data = data;
    	rightarm_joint_pub.publish(msg);
    //IK_right_control(now_right,right_joint);
//    while(ros::ok())
//    {
//    char c = getch();
//    if(c==97){
//    	data_h[0]=data_h[0]+0.5;
//    	msg_h.data = data_h;
//    	chatter_pub3.publish(msg_h);
//    }
//    else if(c==100)
//   {
//   	data_h[0]=data_h[0]-0.5;
//   	msg_h.data = data_h;
 //   	chatter_pub3.publish(msg_h);
 //   }
    
    ros::spinOnce(); //publish once;
    }
 
    ros::shutdown(); 
 
    return 0;
}


