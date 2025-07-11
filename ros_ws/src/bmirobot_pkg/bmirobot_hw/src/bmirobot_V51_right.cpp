#include <bmirobot_hw/bmirobot_V5.h>
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
#define pi          3.1415926
using namespace std;

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;


bmirobot_msg::Robot_ctr rMTctrmsg;
bmirobot_msg::Robot_jointfd rJointfdmsg;
ros::Publisher rJoint_fdpub;
ros::Publisher rMT_ctrpub;

int32_t rfeedbackStart=0;
int32_t initcmd=0;


double rhome_pos[9]={0.00, -0.0, 0, 0.2, 0.1 , 0, 0, 0, 0};

int32_t rMt_mode[9];
int32_t rMt_fdpst[9];
int32_t rMt_fdspd[9];
int32_t rMt_fdcrt[9];
int32_t rMt_fdpwm[9];
int32_t rMt_fdtq[9];
int32_t rMt_abecd[9];
int32_t rMt_rlecd[9];

double rightcmd_updata[9]={0};

float rjoint_limition[9][2];
Eigen::Matrix<double, 9, 9> rM2J;
Eigen::Matrix<double, 9, 1> rJ_vect, rM_vect, rM_cmd;
double rinitpos[9]={0};
double rinitstep[9]={0};

// kalmen filter1
double Q=0.02;
double R=30;
double PP=0.05;
double pst=0;

// kalmen filter2
double Q1=0.02;
double R1=30;
double PP1=0.05;
double pst1=0;


using namespace std;
ofstream rfileout;

double predict()
{
	double pst_pred;
	pst_pred = pst;
	PP = PP+Q;
	pst = pst_pred;
}

double updata(double measure)
{
	double K;
	K = PP/(PP+R);
	pst = pst + K*(measure-pst);
	PP = (1-K)*PP;
}

double predict1()
{
	double pst_pred;
	pst_pred = pst1;
	PP1 = PP1+Q1;
	pst1 = pst_pred;
}

double updata1(double measure)
{
	double K;
	K = PP1/(PP1+R1);
	pst1 = pst1 + K*(measure-pst1);
	PP1 = (1-K)*PP1;
}

double savehandmotor(double goalpst, double fdpst,double speed, int crt)
{
	static double errupdate=0;
	double err=(goalpst-fdpst);
	double differr=errupdate-err;
	 errupdate=err;
	
	double torque=0;
	
	torque= err*6000.0+speed*-500.0;

	if(torque>2000)
		torque= 2000;
	if(torque<-2000)	
		torque=-2000;

	ROS_INFO(" right save:,%12f,%12f,%12f,%12f",goalpst,fdpst,speed,torque);

	return (torque);


}



static double integalErr[8]={0},Err[8]={8},LastErr[8]={0}, dErr[8]={0};

static float P1[8]={0.1,    	0.2,   	0.2,   	0.2,   	0.2,   	0.2,    0,	0};
static float I1[8]={0.0005,  	0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0,	0};
static float D1[8]={0.05,    	0.5,   	0.5,   	0.5,   	0.5,   	0.5,    0,	0};
static float MP1[8]={0.05,    	0.5,   	0.5,   	0.5,   	0.5,   	0.5,    0,	0};

double motor_gravity_PID(int num,double goalpst,double position, double Mforce, double speed, double current)
{
    double pstu=0;
	double rst[7];

    LastErr[num] = Err[num];
    Err[num] = goalpst- position;
    integalErr[num] += Err[num];
    dErr[num] = -speed; // Err[num]-LastErr[num];
    pstu =  MP1[num]*Mforce + P1[num]*Err[num]+ D1[num]*(speed)+ I1[num]*integalErr[num];

    //ROS_INFO("motor pstout:,%12f,%12f,%12f,%12f,%12f,%12f,%12f",pstu,goalpst,Err[num],position,speed,dErr[num],integalErr[num]);
    return (double)(goalpst + pstu);
}



static float pstP[8]={0.1,    	0.2,   	0.2,   	0.2,   	0.2,   	0.2,    0,	0};
static float pstI[8]={0.0005,  	0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0,	0};
static float pstD[8]={0.05,    	0.5,   	0.5,   	0.5,   	0.5,   	0.5,    0,	0};

double motor_pstPID(int num,double goalpst,double position, double speed, double current)
{
    double pstu=0;
    LastErr[num] = Err[num];
    Err[num] = goalpst- position;
    integalErr[num] += Err[num];
    dErr[num] = Err[num]-LastErr[num];
    pstu = pstP[num]*Err[num]+ pstD[num]*(speed)+ pstI[num]*integalErr[num];

    //ROS_INFO("motor pstout:,%12f,%12f,%12f,%12f,%12f,%12f,%12f",pstu,goalpst,Err[num],position,speed,dErr[num],integalErr[num]);
    return (double)(pstu);
}


static float P[8]={2,    2,   2,   2,   2,   2,    0,0};
static float I[8]={0.005,  0.005, 0.005, 0.005, 0.005, 0.005, 0,0};
static float D[8]={0.5,    0.5,   0.5,   0.5,   0.5,   0.5,    0,0};


double motor_spdPID(int num,double goalpst,double position, double speed, double current){
    double spdu=0;
    LastErr[num] = Err[num];
    Err[num] = goalpst- position;
    integalErr[num] += Err[num];
    dErr[num] = Err[num]-LastErr[num];
    spdu = P[num]*Err[num]+ D[num]*(speed)+I[num]*integalErr[num];
    if(spdu>180.0/180*pi)
        spdu=180.0/180*pi;
    if(spdu<-180.0/180*pi)
        spdu=-180.0/180*pi;
    ROS_INFO("motor spdout:,%12f,%12f,%12f,%12f,%12f,%12f,%12f",spdu,goalpst,Err[num],position,speed,dErr[num],integalErr[num]);
    return (double)(spdu);
}


static double tqP[8]={1,  		1,   		1,			1200, 		1200,    	2000,   2000,      	0};
static double tqI[8]={0.001,    0.001,     	0.001,  	1,   		1,  		10,  	10,      0};
static double tqD[8]={4,  		4,   		4,			40, 		40,         10,    10,      	2};


static double tqu[8]={0};
static double ipErr[8]={0}, pErr[8]={0};
static double ivErr[8]={0}, vErr[8]={0}, lvErr[8]={0},dvErr[8]={0};


double motor_tqPID(int num,double goalpst,double position, double speed, double current)
{
    double spdu=0,tquTemp=0;
    double errtqu=0;
    double step=100;
    double tqbound=3000.0;

    pErr[num] = goalpst- position;
    ipErr[num] += pErr[num];
    //dErr[num] = Err[num]-LastErr[num];
    tqu[num] = tqP[num]*pErr[num]+ tqD[num]*(speed)+tqI[num]*ipErr[num];
    ROS_INFO("motor torqe:%2d ,tqu:%12f, gpst:%12f, pst:%12f, spd:%12f, crt:%12f, pE:%10f, ipE:%14f",num,tqu[num],goalpst, position, speed, current, pErr[num], ipErr[num]);

    //ROS_INFO("motor torqe:,%12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f, %14f, %14f",tqu[num], spdu, speed, current, pwmduty, pErr[num], vErr[num], dvErr[num], ivErr[num], ipErr[num]);
    return (double)(tqu[num]);
}


static double PWMpP=3;
static double PWMpI=0.00;
static double PWMpD=0.5;
static double PWMvP=0.1;
static double PWMvI=0.000;
static double PWMvD=0.3;
static double PWMu[8]={0};

double motor_ForwardcontrolPID(int num,double goalpst,double position, double speed, double current)
{
    double spdu=0,PWMTemp=0;
    double errPWMu=0;
    double step=1;
    double PWMbound=100.0;

    pErr[num] = goalpst- position;
    ipErr[num] += pErr[num];
    spdu = PWMpP*pErr[num]+ PWMpD*speed +PWMpI*ipErr[num];
    if(spdu>20*pi)
        spdu=20*pi;
    if(spdu<-20*pi)
        spdu=-20*pi;

    lvErr[num] = vErr[num];
    vErr[num]  = spdu-speed;
    ivErr[num] += vErr[num];
    dvErr[num] = vErr[num]-lvErr[num];
   // tqu1[num]=tqu[num];
    PWMTemp = vErr[num]*PWMvP + dvErr[num]*PWMvD +ivErr[num]*PWMvI;
    if(PWMTemp>PWMbound)
        PWMTemp=PWMbound;
    if(PWMTemp<-PWMbound)
        PWMTemp=-PWMbound;

    errPWMu=PWMTemp-PWMu[num];
    if(errPWMu>step)
    {
        PWMu[num]=PWMu[num]+step;
    }else if(errPWMu<-step)
    {
        PWMu[num]=PWMu[num]-step;
    }else
    {
       PWMu[num]=PWMTemp;
    }
    ROS_INFO("motor PWM:,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f",PWMu[num],spdu,speed,pErr[num],vErr[num],dvErr[num],ivErr[num],ipErr[num],current);
    return (double)(PWMu[num]);
}

void init_right_robot()
{
	rM2J<<   	-1.0/6.0,  	-1.0/3.0,	1.0/6.0,    0,      0,      0,      0,      0,   0,
				-1.0/3.0, 	 1.0/3.0,	1.0/3.0,    0,      0,      0,      0,      0,   0,
				-0.5,  		 0.0,   	-0.5,		0,		0,      0,      0,      0,   0,
				0,      	0,      	0,			0.5,    -0.5,    0,      0,      0,   0,
				0,      	0,      	0,      	-0.5,	-0.5,   0,      0,      0,   0,
				0,      	0,      	0,      	0,      0,      0.5,    -0.5,   0,   0,
				0,      	0,      	0,      	0,      0,      -0.5,   -0.5,   0,   0,
				0,      	0,      	0,      	0,      0,      0,      0,      1,   0,
				0,      	0,      	0,      	0,      0,      0,      0,      0,	 1;

	ros::NodeHandle node;

	char tmp[100];
	//sprintf(tmp,"/home/bmi/ueclab/mydata/ros_noetic/data/right-joint-fd.csv");
	std::cout<<tmp<<endl;
   	rfileout.open(tmp, std::ofstream::out | std::ofstream::binary);

	node.getParam("/home_rjoint1", rhome_pos[0]);
	node.getParam("/home_rjoint2", rhome_pos[1]);
	node.getParam("/home_rjoint3", rhome_pos[2]);
	node.getParam("/home_rjoint4", rhome_pos[3]);
	node.getParam("/home_rjoint5", rhome_pos[4]);
	node.getParam("/home_rjoint6", rhome_pos[5]);
	node.getParam("/home_rjoint7", rhome_pos[6]);
	node.getParam("/home_rjoint8", rhome_pos[7]);

	//std::cout<<rM2J<<endl;
	//exit(-1);
}


void BMIRobot::right_read()
{
	unsigned char motorP[16];
	unsigned char head[2];

	for(int i=0;i < JOINT_NUM+2;i++)
	{
		rM_vect(i)= rJointfdmsg.Joint_fdpst[i];//
	}
	// ROS_INFO("finish read data----------------------");

	//ROS_INFO("right mt pos:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",
	//	rM_vect[0],rM_vect[1],rM_vect[2],rM_vect[3],rM_vect[4],rM_vect[5],rM_vect[6],rM_vect[7]);

	rJ_vect = rM2J*rM_vect;
	// kalman filter
	//for(int i=0;i < JOINT_NUM+2;i++)
	//{
	//	predict();
	//	updata(rJ_vect(i));
	//	rJ_vect(i)=pst;
	//}
	
	//std::ofstream out("/home/jok/图片/rJ_vect.txt",std::ios::app); out<<fixed<<setprecision(2)<<rM_vect[0]<<"\t\t"<<rM_vect[1]<<"\t\t"<<rM_vect[2]<<"\t\t"<<rM_vect[3]<<"\t\t"<<rM_vect[4]<<"\t\t"<<rM_vect[5]<<"\t\t"<<rM_vect[6]<<std::endl;
	  //  out.close();
	
	for(int i=0;i < JOINT_NUM+2;i++)
	{
		//pos[i] =left_cmd[i]+random()%100/1000.0;//; //
		right_pos[i]= rJ_vect(i)-rhome_pos[i];//
		//pos[i]=leftcmd_updata[i];
	}
	//right_pos[JOINT_NUM+1] = -1*right_pos[JOINT_NUM];
	//ROS_INFO("right pos:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",right_pos[0],right_pos[1],right_pos[2],right_pos[3],right_pos[4],right_pos[5],right_pos[6],right_pos[7]);
	//std::ofstream out1("/home/jok/图片/right_pos.txt",std::ios::app); out1<<fixed<<setprecision(2)<<right_pos[0]<<"\t\t"<<right_pos[1]<<"\t\t"<<right_pos[2]<<"\t\t"<<right_pos[3]<<"\t\t"<<right_pos[4]<<"\t\t"<<right_pos[5]<<"\t\t"<<right_pos[6]<<std::endl;
	  //  out1.close();
	if(initcmd==0)
	{
		for(int i=0;i < JOINT_NUM+2;i++)
	    {
			rightcmd_updata[i]=rJ_vect(i)-rhome_pos[i];
		}
		initcmd=1;
	}
}

int32_t testcount=0;
int32_t tstep=0;
ros::Time init_time ;
ros::Time now_time ;
void BMIRobot::right_write()
{
    double pstctr[8],speedctr,torquectr,PWMctr;

	float step = 0.02;

	float maxV = 0;
	int maxI=0;
	float diff[JOINT_NUM+2];
	double diftime;

	//ROS_INFO("right_cmd:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",right_cmd[0],right_cmd[1],right_cmd[2],right_cmd[3],right_cmd[4],right_cmd[5],right_cmd[6],right_cmd[7]);

	for(int i=0;i < JOINT_NUM+2;i++)
    {
		rightcmd_updata[i]=right_cmd[i];
	}
	/*
 	for(int i=0;i < JOINT_NUM+2;i++)
    {
        diff[i] = right_cmd[i] - rightcmd_updata[i];
        if(abs(diff[i]) > maxV)
		{
			maxV = abs(diff[i]);
			maxI = i;
		}
    }
	if(maxV > step)
	{
	    for(int i=0;i < JOINT_NUM+2;i++)
	    {
	      rightcmd_updata[i] = rightcmd_updata[i]+diff[i]/maxV*step;
	    }
    }else
	{	
	    for(int i=0;i < JOINT_NUM+2;i++)
	    {
			rightcmd_updata[i]=rightcmd_updata[i]+diff[i];
		}
	}
	*/

	for(int i=0;i < JOINT_NUM+2;i++)
    {
		if(rightcmd_updata[i]< rjoint_limition[i][0])
			rightcmd_updata[i]= rjoint_limition[i][0];
		if(rightcmd_updata[i]> rjoint_limition[i][1])
			rightcmd_updata[i]= rjoint_limition[i][1];
    }
	//ROS_INFO("rJ_limit:%f,%f,%f,%f,",rjoint_limition[7][0],rjoint_limition[7][1],rjoint_limition[8][0],rjoint_limition[8][1]);

    for(int i=0;i < JOINT_NUM+2;i++)
    {
        rJ_vect(i) = (rightcmd_updata[i]+rhome_pos[i]);
    }
	//ROS_INFO("rJ_vect:%f,%f,%f,%f,%f,%f,%f,%f",rJ_vect(0),rJ_vect(1),rJ_vect(2),rJ_vect(3),rJ_vect(4),rJ_vect(5),rJ_vect(6),rJ_vect(7));
	//std::ofstream out2("/home/jok/图片/rJ_vectp.txt",std::ios::app); out2<<fixed<<setprecision(2)<<rJ_vect(0)<<"\t\t"<<rJ_vect(1)<<"\t\t"<<rJ_vect(2)<<"\t\t"<<rJ_vect(3)<<"\t\t"<<rJ_vect(4)<<"\t\t"<<rJ_vect(5)<<"\t\t"<<rJ_vect(6)<<std::endl;
	  //  out2.close();
    rM_vect= rM2J.inverse()*rJ_vect;
    rM_cmd =rM_vect;
    // kalman filter
    //for(int i=0;i < JOINT_NUM+2;i++)
	//{
	//	predict1();
	//	updata1(rM_cmd(i));
	//	rM_cmd(i)=pst1;
	//}
	//std::cout<<rM2J<<endl;

	double as[7];
	for(int i=0; i<7;i++)
		as[i]=rightcmd_updata[i];
	double Mforce[7];
	//ROS_INFO("dynamic start");
	//dynamic_gravity(as,&Mforce[0]);
	//OS_INFO("right end:%f,%f,%f,%f,%f,%f,%f",Mforce[0],Mforce[1],Mforce[2],Mforce[3],Mforce[4],Mforce[5],Mforce[6]);

    //ROS_INFO("rM_cmd:%f,%f,%f,%f,%f,%f,%f",rM_vect(0),rM_vect(1),rM_vect(2),rM_vect(3),rM_vect(4),rM_vect(5),rM_vect(6),rM_vect(7));
    //std::ofstream out3("/home/jok/图片/rM_vectp.txt",std::ios::app); out3<<fixed<<setprecision(2)<<rM_vect(0)<<"\t\t"<<rM_vect(1)<<"\t\t"<<rM_vect(2)<<"\t\t"<<rM_vect(3)<<"\t\t"<<rM_vect(4)<<"\t\t"<<rM_vect(5)<<"\t\t"<<rM_vect(6)<<std::endl;
	    //out3.close();
    for(int i=0;i < 8;i++)
    {
		switch(rMTctrmsg.mtmode[i]&0x0c)
		{
			case pst_mode:
					//pstctr=	motor_pstPID(i,rM_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],0);
					//pstctr[i]= motor_gravity_PID(i,rM_cmd(i),rJointfdmsg.Joint_fdpst[i], Mforce[i],rJointfdmsg.Joint_fdspd[i],0);
					rMTctrmsg.mtpst[i]=(int32_t)(rM_cmd(i)/pi*18000.0);
					if(i==7)
						rMTctrmsg.mtpst[i]=(int32_t)(rM_cmd(i)*18000.0/pi);
				break;
			case spd_mode:
			       	//speedctr=motor_spdPID(i,rM_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],0);
			        //MTctrmsg.mtspd[i]=(int32_t)(speedctr/pi*18000.0);
				break;
			case tq_mode:
       				torquectr = motor_tqPID(i,rM_vect(i),rJointfdmsg.Joint_fdpst[i],rJointfdmsg.Joint_fdspd[i],rJointfdmsg.Joint_fdctr[i]);
     				rMTctrmsg.mttq[i] =(int32_t) torquectr;
				break;
			case fw_mode:
					//PWMctr=motor_ForwardcontrolPID(i,rM_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],Mt_fdcrt[i]*1.0);
					//MTctrmsg.mtpst[i] =(int32_t) (torquectr);
				break;
		}
		//ROS_INFO("mtmode:%x,   %x",rMTctrmsg.mtmode[i],rMTctrmsg.mtmode[i]&0xc0);
    }
	//rMTctrmsg.mtpst[7]=savehandmotor(rM_cmd(7),rJointfdmsg.Joint_fdpst[7],rJointfdmsg.Joint_fdspd[7],rJointfdmsg.Joint_fdctr[7]);
	//testcount++;
	//MTctrmsg.mttq[0]=testcount;
	//ROS_INFO("test:%d",testcount);
    rMT_ctrpub.publish(rMTctrmsg);
	ros::Time begin = ros::Time::now();
  	if(rfileout.is_open())
  	{
	   	rfileout <<begin<<",";
		for(int i=0;i<8;i++)
		{
			//rfileout <<right_cmd[i]<<","<<right_pos[i]<<","<<rM_cmd(i)<<","<<pstctr[i]<<","<<rJointfdmsg.Joint_fdpst[i]<<","<<Err[i]<<","<<integalErr[i]<<","<<Mforce[i]<<",";
		}
		rfileout<<endl;
  	}
}

void  rFdctrmsgCB(void)
{
	int32_t test=0;
    for(int i=0;i<9;i++)
    {
        rMt_mode[i]	=	fdctrmsgr.mt_mode[i];
        rMt_fdpst[i]= 	fdctrmsgr.mt_Cpst[i];
        rMt_fdspd[i]=(	fdctrmsgr.mt_Cspd[i]);
        rMt_fdcrt[i]=(	fdctrmsgr.mt_incrt[i]);
		test+=fdctrmsgr.mt_incrt[i];
    }
    for(int i=0;i<9;i++)
    {
        rJointfdmsg.Joint_fdpst[i]=rMt_fdpst[i]/18000.0*pi;
        rJointfdmsg.Joint_fdspd[i]=rMt_fdspd[i]/18000.0*pi;
        rJointfdmsg.Joint_fdctr[i]=rMt_fdcrt[i]/1.0;
    }
	//ROS_INFO("right feedback [%d,%d,%d,%d]", fdctrmsgr.mt_Cpst[0],fdctrmsgr.mt_Cpst[1],fdctrmsgr.mt_Cpst[2],fdctrmsgr.mt_Cpst[3]);

    rJoint_fdpub.publish(rJointfdmsg);

	if(test!=0)
		rfeedbackStart=1;
}


int BMIRobot::rightstep1(void)
{
	int rst=0;
	rMTctrmsg.mtID[0]=1;
	rMTctrmsg.mtID[1]=2;
	rMTctrmsg.mtID[2]=3;
	rMTctrmsg.mtID[3]=4;
	rMTctrmsg.mtID[4]=5;
	rMTctrmsg.mtID[5]=6;
	rMTctrmsg.mtID[6]=7;
	rMTctrmsg.mtID[7]=8;
	rMTctrmsg.mtID[8]=9;

	rMTctrmsg.mtmode[0]=rM1_ctr;
	rMTctrmsg.mtmode[1]=rM2_ctr;
	rMTctrmsg.mtmode[2]=rM3_ctr;
	rMTctrmsg.mtmode[3]=rM4_ctr;
	rMTctrmsg.mtmode[4]=rM5_ctr;
	rMTctrmsg.mtmode[5]=rM6_ctr;
	rMTctrmsg.mtmode[6]=rM7_ctr;
	rMTctrmsg.mtmode[7]=rM8_ctr;
	rMTctrmsg.mtmode[8]=rM9_ctr;

	rMT_ctrpub.publish(rMTctrmsg);
	
	if(rfeedbackStart==1)
	if((rMt_mode[0]&0x0f)==rM1_ctr)
	if((rMt_mode[1]&0x0f)==rM2_ctr)
	if((rMt_mode[2]&0x0f)==rM3_ctr)
	if((rMt_mode[3]&0x0f)==rM4_ctr)
	if((rMt_mode[4]&0x0f)==rM5_ctr)
	if((rMt_mode[5]&0x0f)==rM6_ctr)
	if((rMt_mode[6]&0x0f)==rM7_ctr)
	//if((rMt_mode[7]&0x7f)==M8_ctr)
	{
		rst=1;
	}
	for(int i=0;i<8;i++)
	{
		rinitpos[i]=right_pos[i];
		right_cmd[i]=rinitpos[i];
		rinitstep[i]=rinitpos[i]*0.003;
	}
	//ROS_ERROR("change right mode,%d,%d,%x,%x,%x,%x,%x,%x,%x ,%x",rfeedbackStart,rst,
	//	rMt_mode[0]&0x7f,rMt_mode[1]&0x7f,rMt_mode[2]&0x7f,rMt_mode[3]&0x7f,rMt_mode[4]&0x7f,rMt_mode[5]&0x7f,rMt_mode[6]&0x7f,rMt_mode[7]&0x7f);
	ROS_ERROR("step1 right,%f,%f,%f,%f,%f,%f,%f,%f\n",rinitpos[0],rinitpos[1],rinitpos[2],rinitpos[3],rinitpos[4],rinitpos[5],rinitpos[6],rinitpos[7]);
	return rst;
}


int BMIRobot::rightstep2(void)
{
	rMTctrmsg.mtmode[0]=rM1_ctr+0x30;
	rMTctrmsg.mtmode[1]=rM2_ctr+0x30;
	rMTctrmsg.mtmode[2]=rM3_ctr+0x30;
	rMTctrmsg.mtmode[3]=rM4_ctr+0x30;
	rMTctrmsg.mtmode[4]=rM5_ctr+0x30;
	rMTctrmsg.mtmode[5]=rM6_ctr+0x30;
	rMTctrmsg.mtmode[6]=rM7_ctr+0x30;
	rMTctrmsg.mtmode[7]=rM8_ctr+0x30;
	rMTctrmsg.mtmode[8]=rM9_ctr+0x30;
	ROS_ERROR("step2 right,%f,%f,%f,%f,%f,%f,%f,%f",rinitpos[0],rinitpos[1],rinitpos[2],rinitpos[3],rinitpos[4],rinitpos[5],rinitpos[6],rinitpos[7]);

	rMT_ctrpub.publish(rMTctrmsg);
	return 1;
}



int BMIRobot::rightstep3(void)
{
	double total=0;
	for(int i=0;i<8;i++)
	{
		//right_cmd[i]=0;
		right_cmd[i]=rinitpos[i]-0.0001*rinitpos[i];
		total+=fabs(right_cmd[i]);
		rinitpos[i]=right_cmd[i];
	}

	ROS_ERROR("right to home,%f,%f,%f,%f,%f,%f,%f,%f,%f",total,rinitpos[0],rinitpos[1],rinitpos[2],rinitpos[3],rinitpos[4],rinitpos[5]);
	rMT_ctrpub.publish(rMTctrmsg);
	if(total<0.005)
		return 1;
	else
		return 0;	
}

