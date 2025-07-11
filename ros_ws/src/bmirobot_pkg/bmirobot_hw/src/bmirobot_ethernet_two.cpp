#include <bmirobot_hw/bmirobot_V5.h>
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
#include <bmirobot_hw/kalman_filter.h>

#define RAD2DEG(x) (x/3.1415926*180)
#define DEG2RAD(x) (x/180.0*3.1415926)

#define ERR_EXIT(m) \
        do \
        { \
                perror(m); \
                exit(EXIT_FAILURE); \
        } while(0)
#define DEST_PORT1 10000
#define DEST_PORT2 20000
#define DSET_IP_ADDRESS  "192.168.2.181"

using namespace std;

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

ofstream mtfdout,mpuout;

static char sendcmd1[500] = {'f','e','e','d',1};
static char sendcmd2[500] = {'f','e','e','d',2};
int sock1,sock2;
struct sockaddr_in servaddr1;
struct sockaddr_in servaddr2;

static long ctrtimeout=0;
static int32_t watchdata;
static long statusTX1=0,statusTX2=0;

uint16_t msgcount=0,mpucount=0;

int32_t oldpst[8]={0};
int32_t adpst[8]={0};
int16_t pstflag[8]={0};

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

void lCtrmsgCallback(const bmirobot_msg::Robot_ctr::ConstPtr& msg)
{
    for(int i=0;i<8;i++)
    {
       	sendcmd1[5+0+8*i]=(msg->mtID[i]&0xff);
        sendcmd1[5+1+8*i]=(msg->mtmode[i]&0xff);
       	sendcmd1[5+2+8*i]=(msg->mtpst[i]&0xff);
        sendcmd1[5+3+8*i]=(msg->mtpst[i]>>8)&0xff;
       	sendcmd1[5+4+8*i]=(msg->mtspd[i]&0xff);
        sendcmd1[5+5+8*i]=(msg->mtspd[i]>>8)&0xff;
        sendcmd1[5+6+8*i]=(msg->mttq[i]&0xff);
        sendcmd1[5+7+8*i]=((msg->mttq[i])>>8)&0xff;
    }
}


void rCtrmsgCallback(const bmirobot_msg::Robot_ctr::ConstPtr& msg)
{
    for(int i=0;i<8;i++)
    {
       	sendcmd1[69+0+8*i]=(msg->mtID[i]&0xff);
        sendcmd1[69+1+8*i]=(msg->mtmode[i]&0xff);
       	sendcmd1[69+2+8*i]=(msg->mtpst[i]&0xff);
        sendcmd1[69+3+8*i]=(msg->mtpst[i]>>8)&0xff;
       	sendcmd1[69+4+8*i]=(msg->mtspd[i]&0xff);
        sendcmd1[69+5+8*i]=(msg->mtspd[i]>>8)&0xff;
        sendcmd1[69+6+8*i]=(msg->mttq[i]&0xff);
        sendcmd1[69+7+8*i]=((msg->mttq[i])>>8)&0xff;
    }
}



main(int argc, char **argv)
{
	ros::init(argc, argv, "bmirobot_ethernet_driver");

	ros::NodeHandle n("bmirobot");
	ros::Duration period(1.0);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher  lMt_fdctrpub 	= n.advertise<bmirobot_msg::Robot_fdctr>("lMT_fdctr", 100);
	ros::Publisher  lMt_fdstatuspub = n.advertise<bmirobot_msg::Robot_fdstatus>("lMT_fdstatus", 100);
	ros::Publisher  lMt_mpupub 		= n.advertise<bmirobot_msg::Robot_mpu>("lMT_fdmpu", 1000);
	ros::Subscriber lMT_ctrsub 		= n.subscribe("lMT_ctr", 1000, lCtrmsgCallback);

	ros::Publisher  rMt_fdctrpub 	= n.advertise<bmirobot_msg::Robot_fdctr>("rMT_fdctr", 100);
	ros::Publisher  rMt_fdstatuspub = n.advertise<bmirobot_msg::Robot_fdstatus>("rMT_fdstatus", 100);
	ros::Publisher 	rMt_mpupub 		= n.advertise<bmirobot_msg::Robot_mpu>("rMT_fdmpu", 1000);
	ros::Subscriber rMT_ctrsub 		= n.subscribe("rMT_ctr", 1000, rCtrmsgCallback);


  	bmirobot_msg::Robot_fdctr 		fdctrmsgl,fdctrmsgr;
  	bmirobot_msg::Robot_fdstatus 	fdstatusmsgl,fdstatusmsgr;


  	bmirobot_msg::Robot_mpu fdmpu;

  std::cout << argc <<"and"<< argv[1]<<'\n';

  int ret;
  char sendbuf[1024] = {0};
  char recvbuf[1024] = {0};
  char recvbuf2[1024] = {0};

	string deviceip;
	n.getParam("/IP", deviceip);

	int remoteport;
	n.getParam("/port", remoteport);

  string saveflag;
  n.getParam("/savedata", saveflag);

  const char* deviceip1=deviceip.c_str();
//deviceip1=(char *)deviceip;char* aaa = append.c_str()
  std::cout<<deviceip<<endl;
  std::cout<<remoteport<<endl;
  std::cout<<"sdfafasfdsafs"<<endl;

  memset(&servaddr1, 0, sizeof(servaddr1));
  servaddr1.sin_family = AF_INET;
  servaddr1.sin_port = htons(remoteport);
  servaddr1.sin_addr.s_addr =  inet_addr(deviceip1);
  //inet_pton(AF_INET, deviceip, &servaddr1.sin_addr.s_addr);
  //servaddr1.sin_addr.s_addr=deviceip;

  //memset(&servaddr2, 0, sizeof(servaddr2));
  //servaddr2.sin_family = AF_INET;
  //servaddr2.sin_port = htons(DEST_PORT2);
  //servaddr2.sin_addr.s_addr =  inet_addr(DSET_IP_ADDRESS);


  if ((sock1 = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        ERR_EXIT("socket1");
  if ((sock2 = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        ERR_EXIT("socket2");
  
	time_t t = time(0);
	char tmp[100];
	char* path;
	struct tm *info;
	info = localtime(&t);
	path = getcwd(NULL, 0);
	if(saveflag=="true")
		sprintf(tmp,"/home/bmi/bmiproject/ws_pc/data/%d-%d-%d-%d-%d-%d-mtfd.csv", 1900+info->tm_year, 1+info->tm_mon, info->tm_mday,info->tm_hour,info->tm_min,info->tm_sec);
	else
		sprintf(tmp,"/home/bmi/bmiproject/ws_pc/data/new-mtfd.csv");
	std::cout<<tmp<<"sdf"<<endl;
	mtfdout.open(tmp, std::ofstream::out | std::ofstream::binary);

	if(saveflag=="true")
		sprintf(tmp,"/home/bmi/bmiproject/ws_pc/data/%d-%d-%d-%d-%d-%d-mpufd.csv", 1900+info->tm_year, 1+info->tm_mon, info->tm_mday,info->tm_hour,info->tm_min,info->tm_sec);	
	else
		sprintf(tmp,"/home/bmi/bmiproject/ws_pc/data/new-mpufd.csv");
	std::cout<<tmp<<endl;
	mpuout.open(tmp, std::ofstream::out | std::ofstream::binary);



  for(int i=0;i<3;i++)
  {
     sendbuf[0]=0x03;
     ROS_ERROR("test connect");
     sendto(sock1, sendbuf, strlen(sendbuf), 0, (struct sockaddr *)&servaddr1, sizeof(servaddr1));
     //sendto(sock2, sendbuf, strlen(sendbuf), 0, (struct sockaddr *)&servaddr2, sizeof(servaddr2));
     sleep(1);
     ret = recvfrom(sock1, recvbuf, sizeof(recvbuf), 0, NULL, NULL);
     if(ret !=0)
       break;
  }
     
  ROS_ERROR("start run");
  while (ros::ok())
  {
    	ROS_INFO("recieve new start");//133
		sendto(sock1, sendcmd1, 1, 0, (struct sockaddr *)&servaddr1, sizeof(servaddr1));
		//sendto(sock1, sendcmd2, 69, 0, (struct sockaddr *)&servaddr1, sizeof(servaddr1));			
		//for(int i=0;i<1;i++)
		{
			ret = recvfrom(sock1, recvbuf, sizeof(recvbuf), 0, NULL, NULL);
			if (ret == 741)
			{
				recvbuf[ret]='\0';
				if(recvbuf[4]==1)
				{
					ROS_INFO("recieve left arm");
					for(int i=0;i<8;i++)
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
						fdctrmsgl.mt_PWMduty[i]=	fdstatusmsgl.mt_PWMduty[i];
					}

					for(int i=0;i<8;i++)
					{
						fdstatusmsgr.mt_mode[i]=(int16_t)(recvbuf[373+0+5+46*i]&0xff)+(recvbuf[373+1+5+46*i]&0xff)*0x100;

						fdstatusmsgr.mt_Gpst[i]=(int16_t)((recvbuf[373+2+5+46*i]&0xff)+(recvbuf[373+3+5+46*i]&0xff)*0x100);
						fdstatusmsgr.mt_Cpst[i]=(int16_t)((recvbuf[373+4+5+46*i]&0xff)+(recvbuf[373+5+5+46*i]&0xff)*0x100);
						fdstatusmsgr.mt_Lpst[i]=(int16_t)((recvbuf[373+6+5+46*i]&0xff)+(recvbuf[373+7+5+46*i]&0xff)*0x100);

						fdstatusmsgr.mt_Gspd[i]=(int16_t)((recvbuf[373+8+5+46*i]&0xff)+(recvbuf[373+9+5+46*i]&0xff)*0x100);
						fdstatusmsgr.mt_Cspd[i]=(int16_t)((recvbuf[373+10+5+46*i]&0xff)+(recvbuf[373+11+5+46*i]&0xff)*0x100);
						fdstatusmsgr.mt_Lspd[i]=(int16_t)((recvbuf[373+12+5+46*i]&0xff)+(recvbuf[373+13+5+46*i]&0xff)*0x100);

						fdstatusmsgr.mt_Ctime[i]=(int32_t)(recvbuf[373+16+5+46*i]&0xff)+(recvbuf[373+17+5+46*i]&0xff)*0x100;
						fdstatusmsgr.mt_Rtime[i]=(int32_t)(recvbuf[373+18+5+46*i]&0xff)+(recvbuf[373+19+5+46*i]&0xff)*0x100;

						fdstatusmsgr.mt_Gtq[i]=(int16_t)(((recvbuf[373+21+5+46*i]&0xff)*0x100)+(recvbuf[373+20+5+46*i]&0xff));
						
						fdstatusmsgr.mt_sysclk[i]=(int32_t)(recvbuf[373+22+5+46*i]&0xff)+(recvbuf[373+23+5+46*i]&0xff)*0x100+(recvbuf[373+24+5+46*i]&0xff)*0x10000+(recvbuf[373+25+5+46*i]&0xff)*0x1000000;
						fdstatusmsgr.mt_smptime[i]=(int32_t)(recvbuf[373+26+5+46*i]&0xff)+(recvbuf[373+27+5+46*i]&0xff)*0x100;
						
						fdstatusmsgr.mt_cputmp[i]=(int32_t)(recvbuf[373+28+5+46*i]&0xff)+(recvbuf[373+29+5+46*i]&0xff)*0x100;
						fdstatusmsgr.mt_mttmp[i]=(int32_t)(recvbuf[373+30+5+46*i]&0xff)+(recvbuf[373+31+5+46*i]&0xff)*0x100;
						
						fdstatusmsgr.mt_incrt[i]=(int16_t)((recvbuf[373+32+5+46*i]&0xff)+(recvbuf[373+33+5+46*i]&0xff)*0x100);
						fdstatusmsgr.mt_invlt[i]=(int16_t)((recvbuf[373+34+5+46*i]&0xff)+(recvbuf[373+35+5+46*i]&0xff)*0x100);
						
						fdstatusmsgr.mt_PWMduty[i]=(int16_t)((recvbuf[373+36+5+46*i]&0xff)+(recvbuf[373+37+5+46*i]&0xff)*0x100);
						fdstatusmsgr.mt_PWMfrq[i]=(int16_t)((recvbuf[373+38+5+46*i]&0xff)+(recvbuf[373+39+5+46*i]&0xff)*0x100);
						
						fdstatusmsgr.mt_ecd[i]=((recvbuf[373+41+5+46*i]*0x100))+(recvbuf[373+40+5+46*i]&0xff);
						fdstatusmsgr.mt_ecdcnt[i]=(int32_t)(recvbuf[373+42+5+46*i]&0xff)+(recvbuf[373+43+5+46*i]&0xff)*0x100+(recvbuf[373+44+5+46*i]&0xff)*0x10000+(recvbuf[373+45+5+46*i]&0xff)*0x1000000;
						
						fdctrmsgr.mt_mode[i]=	fdstatusmsgr.mt_mode[i];
						fdctrmsgr.mt_Cpst[i]=	fdstatusmsgr.mt_Cpst[i];
						fdctrmsgr.mt_Cspd[i]=	fdstatusmsgr.mt_Cspd[i];
						fdctrmsgr.mt_incrt[i]=	fdstatusmsgr.mt_incrt[i];
						fdctrmsgr.mt_PWMduty[i]=	fdstatusmsgr.mt_PWMduty[i];
					}


					for(int i=0;i<8;i++)
					{
						adjustpst(fdctrmsgl.mt_Cpst[i],i);
						fdctrmsgl.mt_Cpst[i]=adpst[i];
						//ROS_INFO("read:%d,%d",i,);	
					}

					lMt_fdctrpub.publish(fdctrmsgl);
					rMt_fdctrpub.publish(fdctrmsgr);			
					ros::spinOnce();
					statusTX1++;
					if(statusTX1>100)
					{
						statusTX1=0;
						lMt_fdstatuspub.publish(fdstatusmsgl);
					}
				
					ros::spinOnce();
					statusTX2++;
					if(statusTX2>100)
					{
						statusTX2=0;
						rMt_fdstatuspub.publish(fdstatusmsgr);
					}


					ros::Time begin = ros::Time::now();
					if(mtfdout.is_open())
					{
						mtfdout <<begin<<",";
						for(int i=0;i<8;i++)
						{
							mtfdout <<fdctrmsgl.mt_Cpst[i]<<","<<fdctrmsgl.mt_Cspd[i]<<","<<fdctrmsgl.mt_incrt[i]<<","<<fdctrmsgl.mt_PWMduty[i]<<","<<fdstatusmsgl.mt_ecd[i]<<",";
						}
						mtfdout<<endl;
					}
				}
/*				ros::Time begin = ros::Time::now();
				if(mtfdout.is_open())
				{
					mtfdout <<begin<<",";
					for(int i=0;i<8;i++)
					{
						mtfdout <<fdctrmsgl.mt_Cpst[i]<<","<<fdctrmsgl.mt_Cspd[i]<<","<<fdctrmsgl.mt_incrt[i]<<","<<fdctrmsgl.mt_PWMduty[i]<<","<<fdstatusmsgl.mt_ecd[i]<<",";
					}
					mtfdout<<endl;
				}
				*/
				//ROS_INFO("recv1 sucessful:%d,%8x,%8x,%8x,%8x,%8x",ret,fdctrmsgl.mt_ecdcnt[0],recvbuf[42+5+46*0],recvbuf[43+5+46*0],recvbuf[44+5+46*0],recvbuf[45+5+46*0])

			}
			if(ret==100)
			{
				recvbuf[ret]='\0';
				for(int i=0;i<7;i++)
				{
					fdmpu.mpu_Ax[i]=(int16_t)((recvbuf[0+7+12*i]&0xff)+(recvbuf[1+7+12*i]&0xff)*0x100);
					fdmpu.mpu_Ay[i]=(int16_t)((recvbuf[2+7+12*i]&0xff)+(recvbuf[3+7+12*i]&0xff)*0x100);
					fdmpu.mpu_Az[i]=(int16_t)((recvbuf[4+7+12*i]&0xff)+(recvbuf[5+7+12*i]&0xff)*0x100);
					fdmpu.mpu_Rx[i]=(int16_t)((recvbuf[6+7+12*i]&0xff)+(recvbuf[7+7+12*i]&0xff)*0x100);
					fdmpu.mpu_Ry[i]=(int16_t)((recvbuf[8+7+12*i]&0xff)+(recvbuf[9+7+12*i]&0xff)*0x100);
					fdmpu.mpu_Rz[i]=(int16_t)((recvbuf[10+7+12*i]&0xff)+(recvbuf[11+7+12*i]&0xff)*0x100);
				}
				lMt_mpupub.publish(fdmpu);
				ros::spinOnce();
				
				ros::Time mputime = ros::Time::now();
				if(mpuout.is_open())
				{
					mpuout <<mputime<<",";
					for(int i=0;i<7;i++)
					{
						mpuout <<fdmpu.mpu_Ax[i]<<","<<fdmpu.mpu_Ay[i]<<","<<fdmpu.mpu_Az[i]<<","<<fdmpu.mpu_Rx[i]<<","<<fdmpu.mpu_Ry[i]<<","<<fdmpu.mpu_Rz[i]<<",";
					}
					mpuout<<endl;
				}
			}
			//ROS_INFO("recv1 sucessful:%d",ret);
		}
		if (ret == -1)
		{
			ROS_INFO("recv2 failed");
		}else
		{
			// ROS_INFO("recv2 sucessful:%d,",ret);
		}
		ros::spinOnce();
        //usleep(1000);
  }
  close(sock1);
  spinner.stop();
}

