#include <serial/serial.h>
#include <vector>
#include <iomanip> 
#include <ros/console.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <urdf/model.h>
#include "ros/ros.h"
#include <controller_manager/controller_manager.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <bmirobot_msg/Robot_ctr.h>
#include <bmirobot_msg/Robot_fdctr.h>
#include <bmirobot_msg/Robot_fdstatus.h>
#include <bmirobot_msg/Robot_mpu.h>
#include <bmirobot_hw/bmirobot_V5.h>
#include <bmirobot_tools/PointsPR.h>
//#include <bmirobot_hw/bmirobot_com.h>

#define RAD2DEG(x) (x/3.1415926*180)
#define DEG2RAD(x) (x/180.0*3.1415926)

#define ERR_EXIT(m) \
        do \
        { \
                perror(m); \
                exit(EXIT_FAILURE); \
        } while(0)

using namespace std;

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

ofstream mtof1,mtof2,mpuout,sensorout;

static char sendcmd1[500] = {'l','a','r','m',1};
static char sendcmd2[500] = {'r','a','r','m',2};
static char sendcmd3[500] = {'p','a','r','m',3};
int sock1,sock2,sock3;
struct sockaddr_in servaddr1;
struct sockaddr_in servaddr2;
struct sockaddr_in servaddr3;
socklen_t servlen1;

static long ctrtimeout=0;
static int32_t watchdata;


uint16_t msgcount=0,mpucount=0;

int32_t oldpst[18]={0};
int32_t adpst[18]={0};
int16_t pstflag[18]={0};

bmirobot_msg::Robot_fdctr 		fdctrmsgl,fdctrmsgr;
bmirobot_msg::Robot_fdstatus 	fdstatusmsgl,fdstatusmsgr;

bmirobot_msg::Robot_mpu fdmpul,fdmpur;

bmirobot_tools::PointsPR pointmsg;

ros::Publisher  lMt_fdctrpub    ;
ros::Publisher  lMt_fdstatuspub ;
ros::Publisher  lMt_mpupub 	    ;

ros::Publisher  rMt_fdctrpub 	;
ros::Publisher  rMt_fdstatuspub ;
ros::Publisher 	rMt_mpupub 		;
ros::Publisher  Pointspub;

uint8_t feedcount[2];


uint8_t send_time_flag1,send_time_flag2,send_time_flag3;
int16_t send_time_count1,send_time_count2,send_time_count3;

ros::Time point_begin,left_begin,right_begin;

void adjustpst(int16_t pst,int16_t index)
{
	int32_t temp=pst-oldpst[index]	;

	if(temp>18000)
	{
		pstflag[index]=pstflag[index]-1;
	}else
	if(temp<-18000)
	{
		pstflag[index]=pstflag[index]+1;
	}

	if(pstflag[index]==1)
	{
		adpst[index]=18000+(pst+18000);	
	}else if(pstflag[index]==-1)
	{
		adpst[index]=-18000+(pst-18000);
	}else
	{
		adpst[index]=pst;
	}
	oldpst[index]=pst;	
}

void init_com()
{
	ros::NodeHandle node;
	string deviceip;
	node.getParam("/IP", deviceip);

	int remoteport;
	node.getParam("/port", remoteport);

	string saveflag;
	node.getParam("/savedata", saveflag);

	const char* deviceip1=deviceip.c_str();
	std::cout<<deviceip<<endl;
	std::cout<<remoteport<<endl;
	std::cout<<"sdfafasfdsafs"<<endl;

	memset(&servaddr1, 0, sizeof(servaddr1));
	servaddr1.sin_family = AF_INET;
	servaddr1.sin_port = htons(remoteport);
	servaddr1.sin_addr.s_addr =  inet_addr(deviceip1);
	servlen1 =sizeof(servaddr1);

	servaddr2.sin_family = AF_INET;
	servaddr2.sin_port = htons(remoteport+1);
	servaddr2.sin_addr.s_addr =  inet_addr(deviceip1);

	servaddr3.sin_family = AF_INET;
	servaddr3.sin_port = htons(remoteport+2);
	servaddr3.sin_addr.s_addr =  inet_addr(deviceip1);

 	if ((sock1 = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        ERR_EXIT("socket1");

 	if ((sock2 = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        ERR_EXIT("socket2");

 	if ((sock3 = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        ERR_EXIT("socket3");

	time_t t = time(0);
	char tmp[100];
	char* path;
	struct tm *info;
	info = localtime(&t);
	path = getcwd(NULL, 0);
	if(saveflag=="true")
		sprintf(tmp,"/home/bmi/bmiproject/data/%d-%d-%d-%d-%d-%d-mtfd.csv", 1900+info->tm_year, 1+info->tm_mon, info->tm_mday,info->tm_hour,info->tm_min,info->tm_sec);
	else
		sprintf(tmp,"/home/bmi/ueclab/bmiproject/data/ros_chen/new-mtfd.csv");
	//std::cout<<tmp<<"sdf"<<endl;
	mtof1.open(tmp, std::ofstream::out | std::ofstream::binary);

	if(saveflag=="true")
		sprintf(tmp,"/home/bmi/bmiproject/data/%d-%d-%d-%d-%d-%d-mtfd.csv", 1900+info->tm_year, 1+info->tm_mon, info->tm_mday,info->tm_hour,info->tm_min,info->tm_sec);
	else
		sprintf(tmp,"/home/bmi/ueclab/bmiproject/data/ros_chen/new-mtctr.csv");
	//std::cout<<tmp<<"sdf"<<endl;
	mtof2.open(tmp, std::ofstream::out | std::ofstream::binary);


	if(saveflag=="true")
		sprintf(tmp,"/home/bmi/bmiproject/data/%d-%d-%d-%d-%d-%d-mpufd.csv", 1900+info->tm_year, 1+info->tm_mon, info->tm_mday,info->tm_hour,info->tm_min,info->tm_sec);	
	else
		sprintf(tmp,"/home/bmi/ueclab/bmiproject/data/ros_chen/new-mpufd.csv");
	std::cout<<tmp<<endl;
	mpuout.open(tmp, std::ofstream::out | std::ofstream::binary);

	sprintf(tmp,"/home/bmi/ueclab/bmiproject/data/ros_chen/new-polhemus.csv");
	std::cout<<tmp<<endl;
    sensorout.open(tmp, std::ofstream::out | std::ofstream::binary);

	char recvbuf[1024] = {0};
	char sendbuf[5]={0x03,0,0};
	int ret;
	/*
	for(int i=0;i<3;i++)
	{
		ROS_ERROR("test connect");
		sendto(sock1, sendbuf, strlen(sendbuf), 0, (struct sockaddr *)&servaddr1, sizeof(servaddr1));
		sendto(sock2, sendbuf, strlen(sendbuf), 0, (struct sockaddr *)&servaddr2, sizeof(servaddr2));
		sendto(sock3, sendbuf, strlen(sendbuf), 0, (struct sockaddr *)&servaddr3, sizeof(servaddr3));
		//sendto(sock2, sendbuf, strlen(sendbuf), 0, (struct sockaddr *)&servaddr2, sizeof(servaddr2));
		sleep(1);
		ret = recvfrom(sock1, recvbuf, sizeof(recvbuf), 0, NULL, NULL);
		if(ret !=0)
			break;
  	}
	*/
}


void updata_left_send()   
{  
 	for(int i=0;i<9;i++)
    {
       	sendcmd1[5+0+8*i]=(lMTctrmsg.mtID[i]&0xff);
        sendcmd1[5+1+8*i]=(lMTctrmsg.mtmode[i]&0xff);
       	sendcmd1[5+2+8*i]=(lMTctrmsg.mtpst[i]&0xff);
        sendcmd1[5+3+8*i]=(lMTctrmsg.mtpst[i]>>8)&0xff;
       	sendcmd1[5+4+8*i]=(lMTctrmsg.mtspd[i]&0xff);
        sendcmd1[5+5+8*i]=(lMTctrmsg.mtspd[i]>>8)&0xff;
        sendcmd1[5+6+8*i]=(lMTctrmsg.mttq[i]&0xff);
        sendcmd1[5+7+8*i]=((lMTctrmsg.mttq[i])>>8)&0xff;
    }
	if(send_time_count1<=0)
	{
		send_time_count1=255;
		sendto(sock1, sendcmd1, 77, 0, (struct sockaddr *)&servaddr1, sizeof(servaddr1));
	}
}


void updata_right_send()   
{  
	static uint16_t counttest;
    for(int i=0;i<9;i++)
    {
       	sendcmd2[5+0+8*i]=(rMTctrmsg.mtID[i]&0xff);
        sendcmd2[5+1+8*i]=(rMTctrmsg.mtmode[i]&0xff);
       	sendcmd2[5+2+8*i]=(rMTctrmsg.mtpst[i]&0xff);
        sendcmd2[5+3+8*i]=(rMTctrmsg.mtpst[i]>>8)&0xff;
       	sendcmd2[5+4+8*i]=(rMTctrmsg.mtspd[i]&0xff);
        sendcmd2[5+5+8*i]=(rMTctrmsg.mtspd[i]>>8)&0xff;
        sendcmd2[5+6+8*i]=(rMTctrmsg.mttq[i]&0xff);
        sendcmd2[5+7+8*i]=((rMTctrmsg.mttq[i])>>8)&0xff;
    }
	counttest++;
	sendcmd2[2]=(uint8_t)(counttest&0xff);
	sendcmd2[3]=(uint8_t)(counttest>>8);
	if(send_time_count2<=1)
	{
		ROS_INFO("send right flag,%d,%x,%x,%d",counttest,sendcmd2[2],sendcmd2[3],send_time_count2);
		sendto(sock2, sendcmd2, 77, 0, (struct sockaddr *)&servaddr2, sizeof(servaddr2));
		//if(send_time_count2<=1)
		send_time_count2=100;
	}
}


void updata_points_send()   
{  
	sendcmd2[1]=1;
	sendcmd1[1]=1;

	if(send_time_count3<=1)
	{
		sendto(sock3, sendcmd2, 3, 0, (struct sockaddr *)&servaddr3, sizeof(servaddr3));
		ROS_INFO("send points");
		send_time_count3=100;
	}
	//sendcmd2[]
	//ROS_INFO("send points");
}



void updata_left_recevie()
{
		static long statusTX1=0;
		int ret;
		static char recvbuf[1024] = {0};


		for(int j=0;j<1;j++)
		{
			ret = recvfrom(sock1, recvbuf, sizeof(recvbuf), MSG_DONTWAIT, (struct sockaddr *)&servaddr1, &servlen1);
			ros::Time left_begin = ros::Time::now();
			ROS_INFO("data left num:%d,%f,%d",ret,left_begin.toSec(),send_time_count1);
			
			if(ret==-1)
			{
				send_time_count1-=1;
				break;
			}else
			{
				send_time_count1=0;
			}
			if (ret == 419)
			{
				for(int i=0;i<9;i++)
				{
					fdstatusmsgl.mt_mode[i]=(int16_t)(recvbuf[0+5+46*i]&0xff)+(recvbuf[1+5+46*i]&0xff)*0x100;

					fdstatusmsgl.mt_Gpst[i]=(int16_t)((recvbuf[2+5+46*i]&0xff)+(recvbuf[3+5+46*i]&0xff)*0x100);
					fdstatusmsgl.mt_Cpst[i]=(int16_t)((recvbuf[4+5+46*i]&0xff)+(recvbuf[5+5+46*i]&0xff)*0x100);
					fdstatusmsgl.mt_Lpst[i]=(int16_t)((recvbuf[6+5+46*i]&0xff)+(recvbuf[7+5+46*i]&0xff)*0x100);

					fdstatusmsgl.mt_Gspd[i]=(int16_t)((recvbuf[8+5+46*i]&0xff)+(recvbuf[9+5+46*i]&0xff)*0x100);
					fdstatusmsgl.mt_Cspd[i]=(int16_t)((recvbuf[10+5+46*i]&0xff)+(recvbuf[11+5+46*i]&0xff)*0x100);
					fdstatusmsgl.mt_Lspd[i]=(int16_t)((recvbuf[12+5+46*i]&0xff)+(recvbuf[13+5+46*i]&0xff)*0x100);

					fdstatusmsgl.mt_Ctime[i]=(int32_t)(recvbuf[16+5+46*i]&0xff)+(recvbuf[17+5+46*i]&0xff)*0x100;
					fdstatusmsgl.mt_Rtime[i]=(int32_t)(recvbuf[18+5+46*i]&0xff)+(recvbuf[19+5+46*i]&0xff)*0x100;

					fdstatusmsgl.mt_Gtq[i]=(int16_t)(((recvbuf[21+5+46*i]&0xff)*0x100)+(recvbuf[20+5+46*i]&0xff));
					
					fdstatusmsgl.mt_sysclk[i]=(int32_t)(recvbuf[22+5+46*i]&0xff)+(recvbuf[23+5+46*i]&0xff)*0x100+(recvbuf[24+5+46*i]&0xff)*0x10000+(recvbuf[25+5+46*i]&0xff)*0x1000000;
					fdstatusmsgl.mt_smptime[i]=(int32_t)(recvbuf[26+5+46*i]&0xff)+(recvbuf[27+5+46*i]&0xff)*0x100;
					
					fdstatusmsgl.mt_cputmp[i]=(int32_t)(recvbuf[28+5+46*i]&0xff)+(recvbuf[29+5+46*i]&0xff)*0x100;
					fdstatusmsgl.mt_mttmp[i]=(int32_t)(recvbuf[30+5+46*i]&0xff)+(recvbuf[31+5+46*i]&0xff)*0x100;
					
					fdstatusmsgl.mt_incrt[i]=(int16_t)((recvbuf[32+5+46*i]&0xff)+(recvbuf[33+5+46*i]&0xff)*0x100);
					fdstatusmsgl.mt_invlt[i]=(int16_t)((recvbuf[34+5+46*i]&0xff)+(recvbuf[35+5+46*i]&0xff)*0x100);
					
					fdstatusmsgl.mt_PWMduty[i]=(int16_t)((recvbuf[36+5+46*i]&0xff)+(recvbuf[37+5+46*i]&0xff)*0x100);
					fdstatusmsgl.mt_PWMfrq[i]=(int16_t)((recvbuf[38+5+46*i]&0xff)+(recvbuf[39+5+46*i]&0xff)*0x100);
					
					fdstatusmsgl.mt_ecd[i]=((recvbuf[41+5+46*i]*0x100))+(recvbuf[40+5+46*i]&0xff);
					fdstatusmsgl.mt_ecdcnt[i]=(int32_t)(recvbuf[42+5+46*i]&0xff)+(recvbuf[43+5+46*i]&0xff)*0x100+(recvbuf[44+5+46*i]&0xff)*0x10000+(recvbuf[45+5+46*i]&0xff)*0x1000000;
					
					fdctrmsgl.mt_mode[i]=	fdstatusmsgl.mt_mode[i];
					fdctrmsgl.mt_Cpst[i]=	fdstatusmsgl.mt_Cpst[i];
					fdctrmsgl.mt_Cspd[i]=	fdstatusmsgl.mt_Cspd[i];
					fdctrmsgl.mt_incrt[i]=	fdstatusmsgl.mt_incrt[i];
					fdctrmsgl.mt_PWMduty[i]=fdstatusmsgl.mt_PWMduty[i];
				}
			}
			if(ret == 41)
			{
				for(int i=0;i<3;i++)
				{
					fdmpul.mpu_Rx[i]=(int16_t)(recvbuf[0+5+12*i]&0xff)+(recvbuf[1+5+12*i]&0xff)*0x100;
					fdmpul.mpu_Ry[i]=(int16_t)(recvbuf[2+5+12*i]&0xff)+(recvbuf[3+5+12*i]&0xff)*0x100;
					fdmpul.mpu_Rz[i]=(int16_t)(recvbuf[4+5+12*i]&0xff)+(recvbuf[5+5+12*i]&0xff)*0x100;
					fdmpul.mpu_Ax[i]=(int16_t)(recvbuf[6+5+12*i]&0xff)+(recvbuf[7+5+12*i]&0xff)*0x100;
					fdmpul.mpu_Ay[i]=(int16_t)(recvbuf[8+5+12*i]&0xff)+(recvbuf[9+5+12*i]&0xff)*0x100;
					fdmpul.mpu_Az[i]=(int16_t)(recvbuf[10+5+12*i]&0xff)+(recvbuf[11+5+12*i]&0xff)*0x100;
				}
			}
			ros::spinOnce();
		}



		//ROS_INFO("recieve MPU");

		/*
		for(int i=0;i<8;i++)
		{
			adjustpst(fdctrmsgl.mt_Cpst[i],i);
			fdctrmsgl.mt_Cpst[i]=adpst[i];
			//ROS_INFO("read:%d,%d",i,);	
		}
		for(int i=0;i<8;i++)
		{
			adjustpst(fdctrmsgl.mt_Cpst[i],i);
			fdctrmsgl.mt_Cpst[i]=adpst[i];
			//ROS_INFO("read:%d,%d",i,);	
		}*/

		lMt_fdctrpub.publish(fdctrmsgl);
		ros::spinOnce();
		if(statusTX1>50)
		{
			statusTX1=0;
			lMt_fdstatuspub.publish(fdstatusmsgl);
		}else
			statusTX1++;
		lMt_mpupub.publish(fdmpul);
}

void updata_right_recevie()
{
		static long statusTX2=0;

		int ret;
		static char recvbuf[1024] = {0};
		for(int j=0;j<2;j++)
		{
			ret = recvfrom(sock2, recvbuf, sizeof(recvbuf), MSG_DONTWAIT, (struct sockaddr *)&servaddr2, &servlen1);
			ros::Time right_begin = ros::Time::now();
			
			if(ret==-1)
			{
				send_time_count2-=1;
				break;
			}else
			{
				ROS_INFO("receive right flag:%d,%f,%x,%x,%d",ret,right_begin.toSec(),recvbuf[2],recvbuf[3],send_time_count2);
				send_time_count2=0;
			}
			if (ret == 419)
			{
				//ROS_INFO("right arm,%d,%d",fdstatusmsgr.mt_mode[0],fdstatusmsgr.mt_incrt[0]);
				for(int i=0;i<9;i++)
				{
					fdstatusmsgr.mt_mode[i]=(int16_t)(recvbuf[ 0+5+46*i]&0xff)+(recvbuf[ 1+5+46*i]&0xff)*0x100;

					fdstatusmsgr.mt_Gpst[i]=(int16_t)((recvbuf[ 2+5+46*i]&0xff)+(recvbuf[ 3+5+46*i]&0xff)*0x100);
					fdstatusmsgr.mt_Cpst[i]=(int16_t)((recvbuf[ 4+5+46*i]&0xff)+(recvbuf[ 5+5+46*i]&0xff)*0x100);
					fdstatusmsgr.mt_Lpst[i]=(int16_t)((recvbuf[ 6+5+46*i]&0xff)+(recvbuf[ 7+5+46*i]&0xff)*0x100);

					fdstatusmsgr.mt_Gspd[i]=(int16_t)((recvbuf[ 8+5+46*i]&0xff)+(recvbuf[ 9+5+46*i]&0xff)*0x100);
					fdstatusmsgr.mt_Cspd[i]=(int16_t)((recvbuf[ 10+5+46*i]&0xff)+(recvbuf[ 11+5+46*i]&0xff)*0x100);
					fdstatusmsgr.mt_Lspd[i]=(int16_t)((recvbuf[ 12+5+46*i]&0xff)+(recvbuf[ 13+5+46*i]&0xff)*0x100);
					
					fdstatusmsgr.mt_Gtime[i]=(int16_t)((recvbuf[ 14+5+46*i]&0xff)+(recvbuf[ 15+5+46*i]&0xff)*0x100);
					fdstatusmsgr.mt_Ctime[i]=(int32_t)(recvbuf[ 16+5+46*i]&0xff)+(recvbuf[ 17+5+46*i]&0xff)*0x100;
					fdstatusmsgr.mt_Rtime[i]=(int32_t)(recvbuf[ 18+5+46*i]&0xff)+(recvbuf[ 19+5+46*i]&0xff)*0x100;

					fdstatusmsgr.mt_Gtq[i]=(int16_t)(((recvbuf[ 21+5+46*i]&0xff)*0x100)+(recvbuf[ 20+5+46*i]&0xff));
					
					fdstatusmsgr.mt_sysclk[i]=(int32_t)(recvbuf[ 22+5+46*i]&0xff)+(recvbuf[ 23+5+46*i]&0xff)*0x100+(recvbuf[ 24+5+46*i]&0xff)*0x10000+(recvbuf[ 25+5+46*i]&0xff)*0x1000000;
					fdstatusmsgr.mt_smptime[i]=(int32_t)(recvbuf[ 26+5+46*i]&0xff)+(recvbuf[ 27+5+46*i]&0xff)*0x100;
					
					fdstatusmsgr.mt_cputmp[i]=(int32_t)(recvbuf[ 28+5+46*i]&0xff)+(recvbuf[ 29+5+46*i]&0xff)*0x100;
					fdstatusmsgr.mt_mttmp[i]=(int32_t)(recvbuf[ 30+5+46*i]&0xff)+(recvbuf[ 31+5+46*i]&0xff)*0x100;
					
					fdstatusmsgr.mt_incrt[i]=(int16_t)((recvbuf[ 32+5+46*i]&0xff)+(recvbuf[ 33+5+46*i]&0xff)*0x100);
					fdstatusmsgr.mt_invlt[i]=(int16_t)((recvbuf[ 34+5+46*i]&0xff)+(recvbuf[ 35+5+46*i]&0xff)*0x100);
					
					fdstatusmsgr.mt_PWMduty[i]=(int16_t)((recvbuf[ 36+5+46*i]&0xff)+(recvbuf[ 37+5+46*i]&0xff)*0x100);
					fdstatusmsgr.mt_PWMfrq[i]=(int16_t)((recvbuf[ 38+5+46*i]&0xff)+(recvbuf[ 39+5+46*i]&0xff)*0x100);
					
					fdstatusmsgr.mt_ecd[i]=((recvbuf[ 41+5+46*i]*0x100))+(recvbuf[ 40+5+46*i]&0xff);
					fdstatusmsgr.mt_ecdcnt[i]=(int32_t)(recvbuf[ 42+5+46*i]&0xff)+(recvbuf[ 43+5+46*i]&0xff)*0x100+(recvbuf[ 44+5+46*i]&0xff)*0x10000+(recvbuf[ 45+5+46*i]&0xff)*0x1000000;
					
					fdctrmsgr.mt_mode[i]=	fdstatusmsgr.mt_mode[i];
					fdctrmsgr.mt_Cpst[i]=	fdstatusmsgr.mt_Cpst[i];
					fdctrmsgr.mt_Cspd[i]=	fdstatusmsgr.mt_Cspd[i];
					fdctrmsgr.mt_incrt[i]=	fdstatusmsgr.mt_incrt[i];
					fdctrmsgr.mt_PWMduty[i]=fdstatusmsgr.mt_PWMduty[i];
				}
			}
			if(ret == 41)
			{
				for(int i=0;i<3;i++)
				{
					fdmpur.mpu_Rx[i]=(int16_t)(recvbuf[0+5+12*i]&0xff)+(recvbuf[1+5+12*i]&0xff)*0x100;
					fdmpur.mpu_Ry[i]=(int16_t)(recvbuf[2+5+12*i]&0xff)+(recvbuf[3+5+12*i]&0xff)*0x100;
					fdmpur.mpu_Rz[i]=(int16_t)(recvbuf[4+5+12*i]&0xff)+(recvbuf[5+5+12*i]&0xff)*0x100;
					fdmpur.mpu_Ax[i]=(int16_t)(recvbuf[6+5+12*i]&0xff)+(recvbuf[7+5+12*i]&0xff)*0x100;
					fdmpur.mpu_Ay[i]=(int16_t)(recvbuf[8+5+12*i]&0xff)+(recvbuf[9+5+12*i]&0xff)*0x100;
					fdmpur.mpu_Az[i]=(int16_t)(recvbuf[10+5+12*i]&0xff)+(recvbuf[11+5+12*i]&0xff)*0x100;
				}
			}
		}

		rMt_fdctrpub.publish(fdctrmsgr);
		ros::spinOnce();
		statusTX2++;
		if(statusTX2>50)
		{
			statusTX2=0;
			rMt_fdstatuspub.publish(fdstatusmsgr);
		}
		rMt_mpupub.publish(fdmpur);
}

void updata_points_recevie()
{
	int ret;
	static char recvbuf[1024] = {0};

	int index;
	int count;
	
	//std::cout<<"num:"<<count<<"  index:"<<index<<endl;
	//ROS_INFO("new line, num:%d ,index:%d",count,index);
    float tempvalue;
	char* value;

	for(int i=0;i<2;i++)
	{
		ret = recvfrom(sock3, recvbuf, sizeof(recvbuf), MSG_DONTWAIT, (struct sockaddr *)&servaddr3, &servlen1);
		ros::Time point_begin = ros::Time::now();
		
		if(ret==-1)
		{
			send_time_count3-=1;
			break;
		}else
		{
			ROS_INFO("points num:%d,%f,%x,%x",ret,point_begin.toSec(),recvbuf[2],recvbuf[3]);
			send_time_count3=0;
		}
		if (ret == 4*24)
		{
			for(int j=0;j<4;j++)
			{
				for(int k=0;k<6;k++)
				{
					tempvalue=0;
					value=(char *)&tempvalue;
					//ROS_INFO("datanum:%d,char:%x,%x,%x,%x",8+j*4,rxline[8+j*4],rxline[9+j*4],rxline[10+j*4],rxline[11+j*4]);
					value[0]=(recvbuf[j*24+k*4+5]);
					value[1]=(recvbuf[j*24+k*4+1+5]);
					value[2]=(recvbuf[j*24+k*4+2+5]);
					value[3]=(recvbuf[j*24+k*4+3+5]);

					switch(k)
					{
						case 0:		
							pointmsg.Px[j]=tempvalue;
						break;
						case 1:
							pointmsg.Py[j]=tempvalue;
						break;
						case 2:
							pointmsg.Pz[j]=tempvalue;
						break;
						case 3:
							pointmsg.Rz[j]=tempvalue;
						break;
						case 4:
							pointmsg.Ry[j]=tempvalue;
						break;
						case 5:
							pointmsg.Rx[j]=tempvalue;
						break;
					}
				}
			}
		}
	}
	ros::spinOnce();
	Pointspub.publish(pointmsg);
}


void com_save_data()
{
		ros::Time begin = ros::Time::now();
		if(mtof1.is_open())
		{
			mtof1 <<begin<<",";
			for(int i=0;i<8;i++)
			{
				mtof1 <<fdctrmsgl.mt_mode[i]<<","<<fdstatusmsgl.mt_Gpst[i]<<","<<fdctrmsgl.mt_Cpst[i]<<","<<fdstatusmsgl.mt_Gspd[i]<<","<<fdctrmsgl.mt_Cspd[i]<<","<<fdctrmsgl.mt_incrt[i]<<","<<fdctrmsgl.mt_PWMduty[i]<<","<<fdstatusmsgl.mt_ecd[i]<<",";	
			}
			for(int i=0;i<8;i++)
			{
				mtof1 <<fdctrmsgr.mt_mode[i]<<","<<fdstatusmsgr.mt_Gpst[i]<<","<<fdctrmsgr.mt_Cpst[i]<<","<<fdstatusmsgr.mt_Gspd[i]<<","<<fdctrmsgr.mt_Cspd[i]<<","<<fdctrmsgr.mt_incrt[i]<<","<<fdctrmsgr.mt_PWMduty[i]<<","<<fdstatusmsgr.mt_ecd[i]<<",";	
			}
			mtof1<<endl;
			//std::cout<<begin<<endl;
		}
		if(mtof2.is_open())
		{
			mtof2 <<begin<<",";
			for(int i=0;i<8;i++)
			{
				mtof2 <<lMTctrmsg.mtID[i]<<","<<lMTctrmsg.mtmode[i]<<","<<lMTctrmsg.mtpst[i]<<","<<lMTctrmsg.mtspd[i]<<","<<lMTctrmsg.mttq[i]<<",";	
			}
			for(int i=0;i<8;i++)
			{
				mtof2 <<rMTctrmsg.mtID[i]<<","<<rMTctrmsg.mtmode[i]<<","<<rMTctrmsg.mtpst[i]<<","<<rMTctrmsg.mtspd[i]<<","<<rMTctrmsg.mttq[i]<<",";	
			}
			mtof2<<endl; 
			//std::cout<<begin<<endl;
		}
		if(mpuout.is_open())
		{
			mpuout <<begin<<",";
			for(int i=0;i<3;i++)
			{
				mpuout <<fdmpul.mpu_Ax[i]<<","<<fdmpul.mpu_Ay[i]<<","<<fdmpul.mpu_Az[i]<<","<<fdmpul.mpu_Rx[i]<<","<<fdmpul.mpu_Ry[i]<<","<<fdmpul.mpu_Rz[i]<<",";	
			}
			for(int i=0;i<3;i++)
			{
				mpuout <<fdmpur.mpu_Ax[i]<<","<<fdmpur.mpu_Ay[i]<<","<<fdmpur.mpu_Az[i]<<","<<fdmpur.mpu_Rx[i]<<","<<fdmpur.mpu_Ry[i]<<","<<fdmpur.mpu_Rz[i]<<",";	
			}
			mpuout<<endl; 
			//std::cout<<begin<<endl;
		}

  		if(sensorout.is_open())
  		{
		   	sensorout <<begin<<",";
			for(int i=0;i<4;i++)
			{
				sensorout <<pointmsg.Px[i]<<","<<pointmsg.Py[i]<<","<<pointmsg.Pz[i]<<","<<pointmsg.Rx[i]<<","<<pointmsg.Ry[i]<<","<<pointmsg.Rz[i]<<",";	
			}
			sensorout<<endl; 
			//std::cout<<begin<<endl;
	  	}
}


void com_stop()
{
	close(sock1);
	close(sock2);	
	std::cout<<"stop sockets"<<endl;
}

