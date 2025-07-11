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


bmirobot_msg::Robot_ctr lMTctrmsg;
bmirobot_msg::Robot_jointfd lJointfdmsg;
ros::Publisher lJoint_fdpub;
ros::Publisher lMT_ctrpub;

int32_t lfeedbackStart=0;
int32_t linitcmd=0;

double lhome_pos[9]={0.00, -0.00, -0.1, -0.1, 0.1 , -0.1, 0, 0, 0};

int32_t lMt_mode[9];
int32_t lMt_fdpst[9];
int32_t lMt_fdspd[9];
int32_t lMt_fdcrt[9];
int32_t lMt_fdpwm[9];
int32_t lMt_fdtq[9];
int32_t lMt_abecd[9];
int32_t lMt_rlecd[9];

int32_t MpstAng[9];
int32_t MspdAng[9];
int32_t MtqNm[9];

double mt_vel[9]={0};
double leftcmd_updata[9]={0};


float ljoint_limition[9][2];
Eigen::Matrix<double, 9, 9> lM2J;
Eigen::Matrix<double, 9, 1> lJ_vect, lM_vect, lM_cmd;

double linitpos[9]={0};
double linitstep[9]={0};

ofstream lfileout;

double lsavehandmotor(double goalpst, double fdpst,double speed, int crt)
{
	static double errupdate=0;
	double err=(goalpst-fdpst);
	double differr=errupdate-err;
	 errupdate=err;

	double torque=0;
	
	torque= err*6000.0+speed*-500.0;

	if(torque>2000)
		torque=2000;
	if(torque<-2000)	
		torque=-2000;
	ROS_INFO(" left save:,%12f,%12f,%12f,%12f",goalpst,fdpst,speed,torque);
	return (torque);
}

static double integalErr[8]={0},Err[8]={8},LastErr[8]={0}, dErr[8]={0};

static float pstP[8]={0.1,    	0.2,   	0.2,   	0.2,   	0.2,   	0.2,    0,	0};
static float pstI[8]={0.0005,  	0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0,	0};
static float pstD[8]={0.05,    	0.5,   	0.5,   	0.5,   	0.5,   	0.5,    0,	0};

double lmotor_pstPID(int num,double goalpst,double position, double speed, double current){
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


double lmotor_spdPID(int num,double goalpst,double position, double speed, double current){
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



double lmotor_tqPID(int num,double goalpst,double position, double speed, double current)
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

   // ROS_INFO("motor torqe:,%12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f, %14f, %14f",tqu[num], spdu, speed, current, pwmduty, pErr[num], vErr[num], dvErr[num], ivErr[num], ipErr[num]);
    return (double)(tqu[num]);
}



static double PWMpP=3;
static double PWMpI=0.00;
static double PWMpD=0.5;
static double PWMvP=0.1;
static double PWMvI=0.000;
static double PWMvD=0.3;

static double PWMu[8]={0};

double lmotor_ForwardcontrolPID(int num,double goalpst,double position, double speed, double current)
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

void init_left_robot()
{
	ros::NodeHandle node;

	lM2J<<   -1.0/6.0,  	-1.0/3.0,	1.0/6.0,   0,      0,      0,      0,      0, 	0,
				1.0/3.0, 	 -1.0/3.0,	-1.0/3.0,   0,      0,      0,      0,      0,	0,
				-0.5,  		 0.0,   	-0.5,		0,		0,      0,      0,      0,	0,
				0,      	0,      	0,			-0.5,   0.5,   0,      0,      0,	0,
				0,      	0,      	0,      	-0.5,	-0.5,   0,      0,      0,	0,
				0,      	0,      	0,      	0,      0,      0.5,    -0.5,   0,	0,
				0,      	0,      	0,      	0,      0,      -0.5,   -0.5,   0,	0,
				0,      	0,      	0,      	0,      0,      0,      0,      -1,	0,
				0,      	0,      	0,      	0,      0,      0,      0,      0,	1;

	char tmp[100];
	sprintf(tmp,"/home/bmi/ueclab/bmiproject/data/ros_chen/left-joint-fd.csv");
	std::cout<<tmp<<endl;
   	lfileout.open(tmp, std::ofstream::out | std::ofstream::binary);

	node.getParam("/home_ljoint1", lhome_pos[0]);
	node.getParam("/home_ljoint2", lhome_pos[1]);
	node.getParam("/home_ljoint3", lhome_pos[2]);
	node.getParam("/home_ljoint4", lhome_pos[3]);
	node.getParam("/home_ljoint5", lhome_pos[4]);
	node.getParam("/home_ljoint6", lhome_pos[5]);
	node.getParam("/home_ljoint7", lhome_pos[6]);
	node.getParam("/home_ljoint8", lhome_pos[7]);

	//exit(1);
}


void BMIRobot::left_read()
{
	unsigned char motorP[16];
	unsigned char head[2];

	for(int i=0;i < JOINT_NUM+1;i++)
	{
	 	//pos[i] = left_cmd[i];
	 	//vel[i] = 1;
	 	//eff[i] = 1;*/
		lM_cmd(i)= lJointfdmsg.Joint_fdpst[i];
	}
	// ROS_INFO("finish read data----------------------");
	lJ_vect = lM2J*lM_cmd;
	
	for(int i=0;i < JOINT_NUM+2;i++)
	{
		// pos[i] =left_cmd[i]+random()%100/1000.0;//; //
		left_pos[i]= lJ_vect(i)-lhome_pos[i];
		//pos[i]=leftcmd_updata[i];
	}

	//ROS_INFO("left pos:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",left_pos[0],left_pos[1],left_pos[2],left_pos[3],left_pos[4],left_pos[5],left_pos[6],left_pos[7]);

	if(linitcmd==0)
	{
		for(int i=0;i < JOINT_NUM+1;i++)
	    {
			leftcmd_updata[i]=lJ_vect(i)-lhome_pos[i];
			//leftcmd_updata[i]=pos[i];
		}
		linitcmd=1;
	}
}

void BMIRobot::left_write()
{
    double pstctr,speedctr,torquectr,PWMctr;

	float step = 0.02;

	float maxV = 0;
	int maxI=0;
	float diff[JOINT_NUM+2];
	double diftime;

	ROS_INFO("left_cmd:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",left_cmd[0],left_cmd[1],left_cmd[2],left_cmd[3],left_cmd[4],left_cmd[5],left_cmd[6],left_cmd[7]);
	for(int i=0;i < JOINT_NUM+2;i++)
    {
		leftcmd_updata[i]=left_cmd[i];
	}
	/*
 	for(int i=0;i < JOINT_NUM+2;i++)
    {
        diff[i] = left_cmd[i] - leftcmd_updata[i];
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
	      leftcmd_updata[i] = leftcmd_updata[i]+diff[i]/maxV*step;
	    }
    }else
	{	
	    for(int i=0;i < JOINT_NUM+2;i++)
	    {
			leftcmd_updata[i]=leftcmd_updata[i]+diff[i];
		}
	}
	*/

	for(int i=0;i < JOINT_NUM+2;i++)
    {
		if(leftcmd_updata[i]< ljoint_limition[i][0])
			leftcmd_updata[i]= ljoint_limition[i][0];
		if(leftcmd_updata[i]> ljoint_limition[i][1])
			leftcmd_updata[i]= ljoint_limition[i][1];
    }
	ROS_INFO("lJ_limit:%f,%f,%f,%f,",ljoint_limition[7][0],ljoint_limition[7][1],ljoint_limition[8][0],ljoint_limition[8][1]);

    for(int i=0;i < JOINT_NUM+2;i++)
    {
        lJ_vect(i) = (leftcmd_updata[i]+lhome_pos[i]);
    }
	//ROS_INFO("lJ_vect:%f,%f,%f,%f,%f,%f,%f,%f",lJ_vect(0),lJ_vect(1),lJ_vect(2),lJ_vect(3),lJ_vect(4),lJ_vect(5),lJ_vect(6),lJ_vect(7));

    lM_cmd= lM2J.inverse()*lJ_vect;
    lM_cmd =lM_cmd;
    
	//ROS_INFO("lM_cmd:%f,%f,%f,%f,%f,%f,%f,%f",lM_cmd(0),lM_cmd(1),lM_cmd(2),lM_cmd(3),lM_cmd(4),lM_cmd(5),lM_cmd(6),lM_cmd(7));

    for(int i=0;i < 9;i++)
    {
		switch(lMTctrmsg.mtmode[i]&0x0c)
		{
			case pst_mode:
					lMTctrmsg.mtpst[i]=lM_cmd(i)*(18000.0/pi);
					//pstctr=	motor_pstPID(i,lM_cmd(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],0);
					//MTctrmsg.mtpst[i]=(int32_t)(pstctr/pi*18000.0);
				break;
			case spd_mode:
			       	//speedctr=motor_spdPID(i,lM_cmd(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],0);
			        //MTctrmsg.mtspd[i]=(int32_t)(speedctr/pi*18000.0);
				break;
			case tq_mode:
       				torquectr = lmotor_tqPID(i,lM_cmd(i),lJointfdmsg.Joint_fdpst[i],lJointfdmsg.Joint_fdspd[i],lJointfdmsg.Joint_fdctr[i]);
     				lMTctrmsg.mttq[i] =(int32_t) torquectr;
				break;
			case fw_mode:
					//PWMctr=motor_ForwardcontrolPID(i,lM_cmd(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],Mt_fdcrt[i]*1.0);
					//MTctrmsg.mtpst[i] =(int32_t) (torquectr);
				break;
		}
		//ROS_INFO("mtmode:%x,   %x",MTctrmsg.mtmode[i],MTctrmsg.mtmode[i]&0xc0);
    }
	//lMTctrmsg.mtpst[7]=lsavehandmotor(lM_cmd(7),lJointfdmsg.Joint_fdpst[7],lJointfdmsg.Joint_fdspd[7],lJointfdmsg.Joint_fdctr[7]);
	//testcount++;
	//MTctrmsg.mttq[0]=testcount;
	//ROS_INFO("test:%d",testcount);
    lMT_ctrpub.publish(lMTctrmsg);

	ros::Time begin = ros::Time::now();
  	if(lfileout.is_open())
  	{
	   	lfileout <<begin<<",";
		for(int i=0;i<8;i++)
		{
			lfileout <<leftcmd_updata[i]<<","<<left_pos[i]<<",";	
		}
		lfileout<<endl; 
  	}
}

void  lFdctrmsgCB()
{
	int32_t test=0;
    for(int i=0;i<8;i++)
    {
        lMt_mode[i]	=	fdctrmsgl.mt_mode[i];
        lMt_fdpst[i]= 	fdctrmsgl.mt_Cpst[i];
        lMt_fdspd[i]=(	fdctrmsgl.mt_Cspd[i]);
        lMt_fdcrt[i]=(	fdctrmsgl.mt_incrt[i]);
		test+=fdctrmsgl.mt_incrt[i];
    }
    for(int i=0;i<8;i++)
    {
        lJointfdmsg.Joint_fdpst[i]=lMt_fdpst[i]/18000.0*pi;
        lJointfdmsg.Joint_fdspd[i]=lMt_fdspd[i]/18000.0*pi;
        lJointfdmsg.Joint_fdctr[i]=lMt_fdcrt[i]/1.0;
    }
	//ROS_INFO("left feedback [%d,%d,%d,%d]", lMt_fdpst[0],lMt_fdpst[1],lMt_fdpst[2],lMt_fdpst[3]);

    lJoint_fdpub.publish(lJointfdmsg);

	if(test!=0)
		lfeedbackStart=1;
}


int BMIRobot::leftstep1(void)
{
	int rst=0;
	lMTctrmsg.mtID[0]=1;
	lMTctrmsg.mtID[1]=2;
	lMTctrmsg.mtID[2]=3;
	lMTctrmsg.mtID[3]=4;
	lMTctrmsg.mtID[4]=5;
	lMTctrmsg.mtID[5]=6;
	lMTctrmsg.mtID[6]=7;
	lMTctrmsg.mtID[7]=8;
	lMTctrmsg.mtID[8]=9;

	lMTctrmsg.mtmode[0]=lM1_ctr;
	lMTctrmsg.mtmode[1]=lM2_ctr;
	lMTctrmsg.mtmode[2]=lM3_ctr;
	lMTctrmsg.mtmode[3]=lM4_ctr;
	lMTctrmsg.mtmode[4]=lM5_ctr;
	lMTctrmsg.mtmode[5]=lM6_ctr;
	lMTctrmsg.mtmode[6]=lM7_ctr;
	lMTctrmsg.mtmode[7]=lM8_ctr;
	lMTctrmsg.mtmode[8]=lM9_ctr;

	lMT_ctrpub.publish(lMTctrmsg);

	if(lfeedbackStart==1)
	if((lMt_mode[0]&0x7f)==lM1_ctr)
	if((lMt_mode[1]&0x7f)==lM2_ctr)
	if((lMt_mode[2]&0x7f)==lM3_ctr)
	if((lMt_mode[3]&0x7f)==lM4_ctr)
	if((lMt_mode[4]&0x7f)==lM5_ctr)
	if((lMt_mode[5]&0x7f)==lM6_ctr)
	if((lMt_mode[6]&0x7f)==lM7_ctr)
	//if((lMt_mode[7]&0x7f)==M8_ctr)
	{
		rst=1;
	}

	for(int i=0;i<8;i++)
	{
		linitpos[i]=left_pos[i];
		left_cmd[i]=linitpos[i];
		linitstep[i]=linitpos[i]*0.003;
	}

	//ROS_ERROR("change right mode,%d,%d,%x,%x,%x,%x,%x,%x,%x,%x ",lfeedbackStart,rst,
	//	lMt_mode[0]&0x7f,lMt_mode[1]&0x7f,lMt_mode[2]&0x7f,lMt_mode[3]&0x7f,lMt_mode[4]&0x7f,lMt_mode[5]&0x7f,lMt_mode[6]&0x7f,lMt_mode[7]&0x7f);	
	return rst;
}



int BMIRobot::leftstep2(void)
{
	lMTctrmsg.mtmode[0]=lM1_ctr+0x30;
	lMTctrmsg.mtmode[1]=lM2_ctr+0x30;
	lMTctrmsg.mtmode[2]=lM3_ctr+0x30;
	lMTctrmsg.mtmode[3]=lM4_ctr+0x30;
	lMTctrmsg.mtmode[4]=lM5_ctr+0x30;
	lMTctrmsg.mtmode[5]=lM6_ctr+0x30;
	lMTctrmsg.mtmode[6]=lM7_ctr+0x30;
	lMTctrmsg.mtmode[7]=lM8_ctr+0x30;
	lMTctrmsg.mtmode[8]=lM9_ctr+0x30;

	lMT_ctrpub.publish(lMTctrmsg);
	return 1;	
}

int BMIRobot::leftstep3(void)
{
	double total=0;
	for(int i=0;i<8;i++)
	{
		left_cmd[i]=linitpos[i]-0.001*linitpos[i];
		total+=fabs(left_cmd[i]);
		linitpos[i]=left_cmd[i];
	}

	ROS_ERROR("left to home,%f,%f,%f,%f,%f,%f",total,left_cmd[0],(left_cmd[1]),(left_cmd[2]),left_cmd[3],left_cmd[5]);
	lMT_ctrpub.publish(lMTctrmsg);
	if(total<0.005)
		return 1;
	else
		return 0;	
}
