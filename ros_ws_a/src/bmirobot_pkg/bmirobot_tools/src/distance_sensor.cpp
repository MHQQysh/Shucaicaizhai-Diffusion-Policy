#include "ros/ros.h"
#include <serial/serial.h>
#include <vector>
#include <iomanip> 
#include <ros/console.h> 
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <bmirobot_tools/PointsPR.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

using namespace std;

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;


serial::Serial my_serial;
uint8_t txdata[20];
uint8_t rxdata[200];

int rxcnt;
int txcnt;

int Distances[16]={0};

ofstream out;

bmirobot_tools::PointsPR pointmsg;

int enumerate_ports(string portNum)
{
  vector<serial::PortInfo> devices_found = serial::list_ports();

  vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while( iter != devices_found.end() )
  {
    serial::PortInfo device = *iter++;
    if(device.port==portNum)
    {
      return(1);
    }
    //printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
    // device.hardware_id.c_str() );
  }
  printf( "\n!!!!!!!!!!!!!not found the serial port %s\n",portNum.c_str());
  return 0;
}

int initPort()
{
	string portNum="/dev/ttyUSB0";
	std::string rxline;
	int res = enumerate_ports(portNum);
	int flag=0;
	if(res == 0)
		return 0;
	my_serial.setPort(portNum);
	my_serial.setBaudrate(115200);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
	my_serial.setTimeout(timeout);
	my_serial.open();
	cout << "Is the serial port open?";
	if(my_serial.isOpen())
		cout << " Yes." << endl;
	else
		cout << " No." << endl;

	while(flag==0)
	{	
		if(my_serial.isOpen()==0)
			my_serial.open();

		//std::cout<<txdata<<endl;
		//usleep(5000);
		//rxcnt=my_serial.read(rxdata,38);
		rxline=my_serial.readline(65536,"\r\n");
		rxcnt=rxline.length();
		cout << rxcnt << endl;
		if(rxcnt>=1)
		{
			cout << " Polhemus connect success." << endl;
			flag=1;
		}else
		{
			cout << " Polhemus connect fail."<< rxcnt << endl;
			my_serial.close();
			if(my_serial.isOpen())
				cout << " Yes." << endl;
			else
				cout << " No." << endl;
			sleep(1);
			exit(0);

		}	
	}

	while(flag!=5)
	{
		my_serial.read(rxdata,1);
		ROS_INFO("datanum:%6d,%6d",flag,rxdata[0]);
		switch(flag)
		{
			case 1:
				if(rxdata[0]==0x0D)
					flag=2;
				else	flag=1;
			break;
			case 2:
				if(rxdata[0]==0x0A)
					flag=3;
				else	flag=1;
			break;
			case 3:
				if(rxdata[0]==0x0D)
					flag=4;
				else	flag=1;
			break;
			case 4:
				if(rxdata[0]==0x0A)
					flag=5;
				else	flag=1;
			break;
			default:
				flag=1;
			break;
		}
	}



	sleep(1);
	cout << "start continuous." << endl;
	//txdata[0]=0x43;
	//txdata[0]=0x0D;
	//my_serial.write(txdata,2);
	
	return 1;
}



void analysis(uint8_t* rxline)
{
	int index;
	int count;
	
	//std::cout<<"num:"<<count<<"  index:"<<index<<endl;
	//ROS_INFO("new line, num:%d ,index:%d",count,index);
    float tempvalue;
	char* value;
	if((rxline[17*2]==0x0D)&&(rxline[17*2+1]==0x0A))
	{
		for(int j=0;j<16;j++)
		{
			Distances[j]=rxline[2*j+1]+rxline[2*j]*256;
		}			
		ROS_INFO("datanum:%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,",Distances[0],Distances[1],Distances[2],Distances[3],Distances[4],Distances[5],Distances[6],Distances[7],Distances[8],Distances[9],Distances[10],Distances[11],Distances[12],Distances[13],Distances[14],Distances[15]);
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "DistanceSensor_interface");

	ros::NodeHandle n("bmirobot");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	char tmp[100];
	sprintf(tmp,"/home/bmi/Dropbox/ws_moveit/data/new-DistanceSensor.csv");
	std::cout<<tmp<<endl;
    out.open(tmp, std::ofstream::out | std::ofstream::binary);


	ros::Time time1[4],begin;

	initPort();
	std::string rxline;

	while(ros::ok())
	{
		time1[0]=ros::Time::now();
		rxcnt=my_serial.read(rxdata,18*2);
		time1[1]=ros::Time::now();	
		ROS_INFO("datanum:%6d,%6d,%6d,%6d,",rxdata[16*2],rxdata[16*2+1],rxdata[17*2],rxdata[17*2+1]);
		std::cout<<rxcnt<<endl;	
		if((rxdata[17*2]==0x0D)&&(rxdata[17*2+1]==0x0A)&&(rxdata[16*2]==0x0D)&&(rxdata[16*2+1]==0x0A))
		{	
			analysis(rxdata);
			ROS_INFO("new data");
		}else
		{
			ROS_ERROR("not enough data");
			rxline=my_serial.readline(65536,"\r\n");
		}

		std::cout<<"error "<<time1[0]-begin<<"   "<<time1[1]-time1[0];
		begin = ros::Time::now();
		std::cout<<"   "<<begin-time1[1]<<endl;

  		if(out.is_open())
  		{
		   	out <<begin<<",";
			for(int i=0;i<16;i++)
			{
				out <<Distances[i]<<",";	
			}
			out<<endl; 
			
	  	}
		//ROS_INFO("time:%s, new datea",begin);

		//usleep(1000);
	}
	return 1;
}




