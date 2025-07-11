#include <bmirobot_hw/bmirobot_V6.h>
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
#include <serial/serial.h>
#include <bmirobot_msg/motorv2_ctr.h>
#include <bmirobot_msg/motorv2_fd.h>

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

ofstream mtfdout,mpuout;

serial::Serial my_serial;

ros::Publisher  motor_ctr_pub;
ros::Publisher  motor_fd_pub;

bmirobot_msg::motorv2_ctr 	mt_ctr;
bmirobot_msg::motorv2_fd 	mt_fd;

#define motor_ids  		7
#define motor_torque  	1

int32_t motor_id[7]={1,2,3,4,5,6,7};
int32_t zero_position[7]={680, 879, 1038, 998, 1044, -405, 404};
//int32_t zero_position[7]={0,0,0,0,0,0,0};
int motor_nums=0;



int32_t positions[5][7]={
	{0,0,0,0,0,0,0},
	{2645, 2745, -2908, -241, -271, 44, -159},
	//{44, -369, -750, 0, -802, -810, -280},
	//{1591, 1659, 286, 321, 1173, 78, -1091},
	{0,0,0,0,0,0,0},
};



unsigned short v2_update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };
    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

void save_data()
{
	ros::Time begin = ros::Time::now();
	if(mtfdout.is_open())
	{
		mtfdout << begin << ",";
		for(int i=0; i<8; i++)
		{
			mtfdout <<mt_fd.mt_position[i]<<","<<mt_fd.mt_speed[i]<<","<<mt_fd.mt_current[i]<<",";
		}
		mtfdout<<endl;
	}
}

void com_init()
{
	string portNum="/dev/ttyUSB0";
	my_serial.setPort(portNum);
	my_serial.setBaudrate(3000000);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
	my_serial.setTimeout(timeout);
	my_serial.open();
	cout << "Is the serial port open?";
	if(my_serial.isOpen())
		cout << " Yes." << endl;
	else
		cout << " No." << endl;

	char tmp[100];
	sprintf(tmp,"/home/bmi/ueclab/data/new-mtfd.csv");
	std::cout<<tmp<<"sdf"<<endl;
	mtfdout.open(tmp, std::ofstream::out | std::ofstream::binary);
}

void v2_com_sync_write(int port, int len, uint8_t* data )
{
	uint8_t send_frame1[500]={0};
	uint16_t crc_rst;
	int loop;

	send_frame1[0]=0xff;
	send_frame1[1]=0xff;
	send_frame1[2]=0xfd;
	send_frame1[3]=0x00;

	send_frame1[4]=0xfe;

	send_frame1[5]=(len+3)		& 0xff;
	send_frame1[6]=((len+3)>>8)	& 0xff;

	send_frame1[7]=0x83;
	for(int i=0; i<len;i++)
	{
		send_frame1[8+i] = data[i];
	}
	crc_rst = v2_update_crc( 0, send_frame1, len+8 );
	send_frame1[ 8+len ] = crc_rst & 0xff;
	send_frame1[ 9+len] = (crc_rst>>8) & 0xff;

	// printf("sync_write:   ");
	// for(loop =0 ; loop < len+10; loop++)
	// 	printf("0x%02X ", send_frame1[loop]);
	// printf("\n");

	//ROS_INFO("crc,%x",crc_rst);
	my_serial.write(send_frame1,len+10);
	//exit(-1);
}

void v2_com_sync_read(int port, int len, uint8_t* data )
{
	uint8_t send_frame1[500]={0};
	uint16_t crc_rst;
	int loop;

	send_frame1[0]=0xff;
	send_frame1[1]=0xff;
	send_frame1[2]=0xfd;
	send_frame1[3]=0x00;

	send_frame1[4]=0xfe;

	send_frame1[5]=(len+3)		& 0xff;
	send_frame1[6]=((len+3)>>8)	& 0xff;

	send_frame1[7]=0x82;
	for(int i=0; i<len;i++)
	{
		send_frame1[8+i] = data[i];
	}
	crc_rst = v2_update_crc(0, send_frame1 , len+8);
	send_frame1[ 8+len ] = crc_rst & 0xff;
	send_frame1[ 9+len ] = (crc_rst>>8) & 0xff;

	// printf("sync_read:   ");
	// for(loop =0 ; loop < len+10; loop++)
	// 		printf("0x%02X ", send_frame1[loop]);
	// printf("\n");

	//ROS_INFO("crc,%x",crc_rst);
	my_serial.write(send_frame1,len+10);
	ROS_INFO("read data");
}

void v2_com_sync_read_all(int port,uint8_t start_cmd,uint8_t count_ids,uint16_t length)
{
	uint8_t read_frame1[500]={0};
	uint16_t back_lens=0;
	uint16_t start_index,id;
	int16_t temp;
	int16_t position1,current1,speed1;
	static int shut_count=0;
	//my_serial.waitByteTimes(length*count_ids);
	back_lens=my_serial.read(read_frame1,length*count_ids);
	ROS_INFO("receive data,%d",back_lens);
	
	if(back_lens==0)
	{
		exit(-1);
	}else
		shut_count=0;

	if((back_lens%length)==0)
	{
		motor_nums=back_lens/length;
		printf("motor_nums %d \n ", motor_nums);
		if((read_frame1[0]==0xff )|(read_frame1[1]==0xff)|(read_frame1[2]==0xfd))
		{
			for(int i=0;i<count_ids;i++)
			{
				start_index = length*i;
				printf("id,%2d;    " , read_frame1[4+start_index]);
				id=read_frame1[4+start_index];
				if((id<8)&(id>0))
				{
					switch(start_cmd)
					{
						case v2_present_current_cmd:
							temp= read_frame1[start_index+9]  +read_frame1[start_index+10]*0x100;
							current1= temp;

							temp= read_frame1[start_index+11] +read_frame1[start_index+12]*0x100 +read_frame1[start_index+13]*0x10000 +read_frame1[start_index+14]*0x1000000;
							speed1=	temp;

							temp= read_frame1[start_index+15] +read_frame1[start_index+16]*0x100 +read_frame1[start_index+17]*0x10000 +read_frame1[start_index+18]*0x1000000;
							position1= temp;

							mt_fd.mt_current[id-1]	=current1;
							mt_fd.mt_speed[id-1]	=speed1;
							mt_fd.mt_position[id-1]	=position1 - zero_position[id-1];
						break;
					}
				}
			}
		}
		motor_fd_pub.publish(mt_fd);
		save_data();
	}

	if((read_frame1[0]!=0xff )|(read_frame1[1]!=0xff)|(read_frame1[2]!=0xfd))
	{
		string line;
		my_serial.flushInput();
		//my_serial.readline();
		ROS_INFO("flush,%d",back_lens);
	}
}



void v2_motor_step1()		// turn on torque
{
	uint8_t data1[30];
	uint16_t length1;
	data1[0] =v2_torque_enable_cmd&0xff;
	data1[1] =(v2_torque_enable_cmd>>8)&0xff;
	data1[2] =2;
	data1[3] =0;	

	for(int i=0;i<7;i++)
	{
		data1[4+i*3] =i+1;		// motor1 id
		data1[5+i*3] =motor_torque;		// motor1 toruqe on
		data1[6+i*3] =1;		// motor1 led on
	}

	length1 = 4+3*7;

	v2_com_sync_write(0, length1, data1);
	ROS_INFO("step2");
}

// send command
void v2_motor_step2()		
{
	uint8_t data1[100];
	uint16_t length1;
	int32_t temp_position;
	data1[0]=v2_goal_position_cmd&0xff;
	data1[1]=(v2_goal_position_cmd>>8)&0xff;
	data1[2] =4;
	data1[3] =0;	

	printf("\n ");
	for(int i=0;i<7;i++)
	{
		data1[4+i*5] =i+1;		// motor id
		temp_position = goal_position[i] + zero_position[i];

		data1[5+i*5]=(uint8_t)(temp_position		)&0xff;		// 
		data1[6+i*5]=(uint8_t)(temp_position>>8		)&0xff;		// 
		data1[7+i*5]=(uint8_t)(temp_position>>16	)&0xff;		// 
		data1[8+i*5]=(uint8_t)(temp_position>>24	)&0xff;		// 

		mt_ctr.mt_position_goal[i] = temp_position;
	}
	
	length1=4+5*7;
	motor_ctr_pub.publish(mt_ctr);
	v2_com_sync_write(0,length1,data1);
}

//read status
void v2_motor_step3()  
{
	uint8_t data1[100];
	uint16_t length1;

	data1[0]=v2_present_current_cmd&0xff;
	data1[1]=(v2_present_current_cmd>>8)&0xff;
	data1[2]=10;

	data1[3]=0;

	data1[4]=motor_id[0];
	data1[5]=motor_id[1];
	data1[6]=motor_id[2];
	data1[7]=motor_id[3];
	data1[8]=motor_id[4];
	data1[9]=motor_id[5];
	data1[10]=motor_id[6];

	length1=11;

	v2_com_sync_read(0,length1,data1);
	v2_com_sync_read_all(0, v2_present_current_cmd, motor_ids, 11+10);
}


//read status
int v2_motor_position()
{
	uint8_t data1[100];
	uint16_t length1;

	for(int i=0; i<motor_nums; i++)
	{
		if(mt_fd.mt_position[i]>2048)
		{
			zero_position[i] += 4096;
		}
		printf("%d,%d,%d,%d,\n ", i,mt_fd.mt_position[i], zero_position[i],motor_nums);
	}
	if(motor_ids != motor_nums)
		return -1;
	return 0;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "bmirobotv61_com");

	ros::NodeHandle n("bmirobot");
	ros::Duration period(1.0);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	motor_ctr_pub 	= n.advertise<bmirobot_msg::motorv2_ctr>("mt_ctr", 100);
	motor_fd_pub = n.advertise<bmirobot_msg::motorv2_fd>("mt_fd", 100);

	char tmp[100];
	int timecount=0;

	ofstream mtof1;
	sprintf(tmp,"/home/bmi/ueclab/bmiproject/data/ros_chen/new-mtfd.csv");
	mtof1.open(tmp, std::ofstream::out | std::ofstream::binary);

	com_init();

	v2_motor_step1();	// change operataion mode

	int16_t steppst=0;
	int16_t count=0;
	bool init_flag=0;

	int index_pos=0;
	int count_traj=0;

	ros::Time begin_time = ros::Time::now();
	ros::Time now_time;

	double diftime;
 	while (ros::ok())
  	{
		v2_motor_step3();
		if(init_flag==0)
		{
			if(v2_motor_position()==0)
				init_flag=1;
			else
				init_flag=0;
		}
		else
		{
			v2_motor_step2();
		}

		printf("steppst: %2d, %2d,   \n", steppst,index_pos);
		switch(steppst)
		{
			case 0:
				now_time = ros::Time::now();
				diftime=(now_time-begin_time).toSec();
				if(diftime>2)
				{
					steppst=1;
					begin_time= ros::Time::now();
				}
			break;

			case 1:
				now_time = ros::Time::now();
				diftime = (now_time-begin_time).toSec();
				for(int i=0;i<7;i++)
				{
					goal_position[i]= (int32_t)(positions[index_pos][i] + (positions[index_pos+1][i]-positions[index_pos][i])/2.0*(1-cos(diftime/10*2*PI)));
					printf("%2d,%5d    ", i,goal_position[i]);
					
				}
				printf("\n");
				if(diftime > 5)
				{
					steppst=2;
					begin_time= ros::Time::now();
				}
			break;

			case 2:
				now_time = ros::Time::now();
				diftime=(now_time-begin_time).toSec();
				if(diftime>2)
				{
					index_pos++;
					if(index_pos>=2)
						steppst=3;
					else
						steppst=1;
					begin_time= ros::Time::now();
				}
			break;
			
			case 3:
				count_traj++;
				index_pos=0;
				if(count_traj>=1)
					steppst=5;
				else
					steppst=1;
				begin_time= ros::Time::now();
			break;
		}

		if(mtof1.is_open())
		{
			mtof1 <<now_time<<",";
			for(int i=0;i<3;i++)
			{
				mtof1 <<mt_fd.mt_pwm[i]<<","<<mt_fd.mt_current[i]<<","<<mt_fd.mt_speed[i]<<","<<mt_fd.mt_position[i];	
			}
			mtof1<<endl;
			//std::cout<<begin<<endl;
		}
  	}
  	spinner.stop();
}






