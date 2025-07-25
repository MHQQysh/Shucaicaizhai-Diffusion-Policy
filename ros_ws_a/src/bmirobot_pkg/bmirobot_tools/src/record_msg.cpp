#include "ros/ros.h"
#include <serial/serial.h>
#include <vector>
#include <iomanip> 
#include <ros/console.h> 
#include <Eigen/Core>
#include <Eigen/Geometry>
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
#include <bmirobot_msg/Robot_fdctr.h>
#include <bmirobot_msg/Robot_ctr.h>

using namespace std;

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

#define pi  3.141592653589793

uint16_t msgcount=0;
int32_t fdpst[8],fdspd[8],fdctr[8],fdpwm[8];
int32_t ctrpst[8],ctrspd[8],ctrtq[8];
ofstream out;
ros::Time begininit;

void fdmsgCB(const bmirobot_msg::Robot_fdctr::ConstPtr& msg)
{
	for(int i=0;i<8;i++)
	{
		fdpst[i]=msg->mt_Cpst[i];
		fdspd[i]=msg->mt_Cspd[i];
		fdctr[i]=msg->mt_incrt[i];	
		fdpwm[i]=msg->mt_PWMduty[i];
	}
	msgcount++;
	std::cout<<"jointcb  "<<msgcount<<","<<endl;
}


void CtrmsgCallback(const bmirobot_msg::Robot_ctr::ConstPtr& msg)
{
    for(int i=0;i<8;i++)
    {
       	ctrpst[i]=(msg->mtpst[i]);
       	ctrspd[i]=(msg->mtspd[i]);
        ctrtq[i]=(msg->mttq[i]);
    }
	std::cout<<"ctrcb"<<endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bmirobot_Polhemus_interface");
	
	ros::NodeHandle n("bmirobot");
	ros::AsyncSpinner spinner(1);
	
    ros::Subscriber MT_jointfdsub = n.subscribe("MT_fdctr", 1000,fdmsgCB);
    ros::Subscriber MT_ctrsub = n.subscribe("MT_ctr", 1000, CtrmsgCallback);
    
	spinner.start();
	
    string namestr="robotmsg.csv";

    time_t t = time(0);
    char tmp[100];
    strftime( tmp, sizeof(tmp), "/home/bmi/Dropbox/Programs/record_data/motor/%Y%m%d-%X.csv",localtime(&t) );
    std::cout<<tmp<<endl;

	//out(tmp);
    out.open(tmp, std::ofstream::out | std::ofstream::binary);
    ros::Time begininit= ros::Time::now();
	double secsinit = begininit.toSec();

    //out<<"count,"<<secsinit<<",m1pst,m1spd,m1tq,fm1pst,fm1spd,fm1ctr,m2pst,m2spd,m2tq,fm2pst,fm2spd,fm2ctr"<<endl;



    while(ros::ok())
    {
		ros::Time begin = ros::Time::now();
      	if(out.is_open())
      	{
           	out <<msgcount<<","<<begin<<",";
			for(int i=0;i<8;i++)
			{
				out <<ctrpst[i]<<","<<ctrtq[i]<<",";
				out <<fdpwm[i]<<","<<fdpst[i]<<","<<fdspd[i]<<","<<fdctr[i]<<",";	
			}
			out<<endl; 
      	}
		//std::cout<<msgcount<<","<<secs<<","<<ctrtq[4]<<","<<endl;
		std::cout<<ros::Time::now()<<endl;
		usleep(1000);
    }
  	spinner.stop();
	return 1;

}




