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

	{ 
		while(flag==0)
		{	
			if(my_serial.isOpen()==0)
				my_serial.open();
			txdata[0]=0x46;
			txdata[1]=0x31;		
			txdata[2]=0x0d;
			txdata[3]=0x46;
			txdata[4]=0x0d;
			my_serial.write(txdata,5);
			//std::cout<<txdata<<endl;
			//usleep(5000);
			//rxcnt=my_serial.read(rxdata,38);
			rxline=my_serial.readline(65536,"\r\n");
			rxcnt=rxline.length();
			cout << rxcnt << endl;
			if(rxcnt>=9)
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
	}
	sleep(1);
	cout << "start continuous." << endl;
	//txdata[0]=0x43;
	//txdata[0]=0x0D;
	//my_serial.write(txdata,2);
	return 1;
}

void analysisAsic(std::string rxline)
{
	int index;
	index =rxline[1]-'1';	
	//std::cout<<"num:"<<index <<endl;
	ROS_INFO("new line");
	
	for(int j=0;j<6;j++)
	{
		int zhengshu[3]={0,0,0},xiaoshu[3]={0,0,0},sign=1;
		if(rxline[8+j*9]==0x2e)
		{
			if(rxline[7+j*9]=='-')	sign=-1;
			else
			if(rxline[7+j*9]!=' ')
				zhengshu[2]=rxline[7+j*9]-'0';
			else
				zhengshu[2]=0;

			if(rxline[6+j*9]=='-')	sign=-1;
			else
			if(rxline[6+j*9]!=' ')
				zhengshu[1]=rxline[6+j*9]-'0';
			else
				zhengshu[1]=0;

			if(rxline[5+j*9]=='-')	sign=-1;
			else
			if(rxline[5+j*9]!=' ')
				zhengshu[0]=rxline[5+j*9]-'0';
			else
				zhengshu[0]=0;

			if(rxline[4+j*9]=='-')	sign=-1;

			if(rxline[9+j*9]!=' ')
				xiaoshu[0]=rxline[9+j*9]-'0';
			if(rxline[10+j*9]!=' ')
				xiaoshu[1]=rxline[10+j*9]-'0';
			if(rxline[11+j*9]!=' ')
				xiaoshu[2]=rxline[11+j*9]-'0';

			switch(j)
			{
			case 0:
				pointmsg.Px[index]=(zhengshu[0]*100000+zhengshu[1]*10000+zhengshu[2]*1000+xiaoshu[0]*100+xiaoshu[1]*10+xiaoshu[2]*1)*sign;
			break;
			case 1:
				pointmsg.Py[index]=(zhengshu[0]*100000+zhengshu[1]*10000.0+zhengshu[2]*1000+xiaoshu[0]*100+xiaoshu[1]*10+xiaoshu[2]*1)*sign;
			break;
			case 2:
				pointmsg.Pz[index]=(zhengshu[0]*100000+zhengshu[1]*10000.0+zhengshu[2]*1000+xiaoshu[0]*100+xiaoshu[1]*10+xiaoshu[2]*1)*sign;
			break;
			case 3:
				pointmsg.Rz[index]=(zhengshu[0]*100000+zhengshu[1]*10000+zhengshu[2]*1000+xiaoshu[0]*100+xiaoshu[1]*10+xiaoshu[2]*1)*sign;
			break;
			case 4:
				pointmsg.Ry[index]=(zhengshu[0]*100000+zhengshu[1]*10000+zhengshu[2]*1000+xiaoshu[0]*100+xiaoshu[1]*10+xiaoshu[2]*1)*sign;
			break;
			case 5:
				pointmsg.Rx[index]=(zhengshu[0]*100000+zhengshu[1]*10000+zhengshu[2]*1000+xiaoshu[0]*100+xiaoshu[1]*10+xiaoshu[2]*1)*sign;
			break;
			}
		}
	}
}

void analysis(uint8_t* rxline)
{
	int index;
	int count;
	
	//std::cout<<"num:"<<count<<"  index:"<<index<<endl;
	//ROS_INFO("new line, num:%d ,index:%d",count,index);
    float tempvalue;
	char* value;
	if((rxline[0]==0x4c)&&(rxline[1]==0x59))
	{
		index=rxline[2]-1;
		std::cout<<"ok, port:"<<index<<endl;
		for(int j=0;j<6;j++)
		{
			tempvalue=0;
			value=(char *)&tempvalue;
			//ROS_INFO("datanum:%d,char:%x,%x,%x,%x",8+j*4,rxline[8+j*4],rxline[9+j*4],rxline[10+j*4],rxline[11+j*4]);
			value[0]=(rxline[8+j*4]);
			value[1]=(rxline[8+j*4+1]);
			value[2]=(rxline[8+j*4+2]);
			value[3]=(rxline[8+j*4+3]);

			switch(j)
			{
				case 0:		
					pointmsg.Px[index]=tempvalue;
				break;
				case 1:
					pointmsg.Py[index]=tempvalue;
				break;
				case 2:
					pointmsg.Pz[index]=tempvalue;
				break;
				case 3:
					pointmsg.Rz[index]=tempvalue;
				break;
				case 4:
					pointmsg.Ry[index]=tempvalue;
				break;
				case 5:
					pointmsg.Rx[index]=tempvalue;
				break;
			}
		}
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Polhemus_interface");

	ros::NodeHandle n("bmirobot");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher PointsPR = n.advertise<bmirobot_tools::PointsPR>("PointsPR", 1000);

	char tmp[100];
	getwd((char*)tmp);
	sprintf(tmp,"/home/bmi/bmiproject/ws_pc/data/new-polhemus.csv");
	std::cout<<tmp<<endl;
    out.open(tmp, std::ofstream::out | std::ofstream::binary);


	ros::Time time1[4],begin;

	initPort();
	std::string rxline;

	while(ros::ok())
	{
		time1[0]=ros::Time::now();
		my_serial.write("P");
		time1[1]=ros::Time::now();
		rxcnt=my_serial.read(rxdata,34*4);
		time1[2]=ros::Time::now();	

		std::cout<<rxcnt<<endl;
		if(rxcnt==34*4)
		{
			for(int i=0;i<4;i++)
			{
				//rxline=my_serial.readline(65536,"\r\n");
				//rxcnt=my_serial.read(rxdata,38*4);
				analysis(rxdata+34*i);
				//ROS_INFO("new data");
			}
		}else
		{
			ROS_ERROR("not enough data");
			my_serial.read(rxdata,200);
		}
		PointsPR.publish(pointmsg);

		std::cout<<"error "<<time1[0]-begin<<"   "<<time1[1]-time1[0]<<"   "<<time1[2]-time1[1];
		begin = ros::Time::now();
		std::cout<<"   "<<begin-time1[2]<<endl;

  		if(out.is_open())
  		{
		   	out <<begin<<",";
			for(int i=0;i<4;i++)
			{
				out <<pointmsg.Px[i]<<","<<pointmsg.Py[i]<<","<<pointmsg.Pz[i]<<","<<pointmsg.Rx[i]<<","<<pointmsg.Ry[i]<<","<<pointmsg.Rz[i]<<",";	
			}
			out<<endl; 
			//std::cout<<begin<<endl;
	  	}
		//ROS_INFO("time:%s, new datea",begin);

		//usleep(1000);
	}
	return 1;
}




