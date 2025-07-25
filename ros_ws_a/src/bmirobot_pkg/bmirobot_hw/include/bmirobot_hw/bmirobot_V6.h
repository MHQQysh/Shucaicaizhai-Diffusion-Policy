#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <bmirobot_msg/Robot_ctr.h>
#include <bmirobot_msg/Robot_fdctr.h>
#include <bmirobot_msg/Robot_fdstatus.h>
#include <bmirobot_msg/Robot_mpu.h>
#include <bmirobot_msg/Robot_jointfd.h>
#include "ros/time.h"
#include "ros/duration.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <serial/serial.h>

using namespace std;

#define JOINT_NUM 7
#define HAND_JOINT_NUM 2

#define USE_ODOMETER    0
#define Motor_mode      0x00   //motor mode: (0x00 position) (0x04 speed) (0x08 torque) 0x0c forwardcontorl
#define Motor_status    0x02   //motor status (0x00 normal) (0x02 free) (0x03  hold)

#define torque_enable_cmd       64

#define operation_mode_cmd      11

#define goal_position_cmd       116

#define v2_present_pwm_cmd          124
#define v2_present_current_cmd      126
#define v2_present_speed_cmd        128
#define v2_present_position_cmd     132

#define v2_operating_mode_cmd       11
#define v2_torque_enable_cmd        64
#define v2_goal_position_cmd        116

#define current_mode                    0x00
#define speed_mode                      0x01
#define position_mode                   0x03
#define extend_position_mode            0x04
#define current_based_position_mode     0x05
#define pwm_mode                        0x10

#define torque_on                       0x01
#define led_on                          0x01

#define PI          3.1415926



#define Mnormal    0x00
#define Mfree      0x02
#define Mhold      0x03

using std::string;

class BMIRobot : public hardware_interface::RobotHW
{
public:
    BMIRobot();
    ~BMIRobot();
    void read();
    void write();
    void right_read();
    void right_write();
    void left_read();
    void left_write();
    void initrobot();
    int leftstep1();
    int leftstep2();
    int leftstep3();
    int rightstep1();
    int rightstep2();
    int rightstep3();

    //int writePA();
    ros::Time get_time();
    ros::Duration get_period();

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double left_cmd[JOINT_NUM+HAND_JOINT_NUM];
    double left_pos[JOINT_NUM+HAND_JOINT_NUM];
    double left_vel[JOINT_NUM+HAND_JOINT_NUM];
    double left_eff[JOINT_NUM+HAND_JOINT_NUM];
    
    double right_cmd[JOINT_NUM+HAND_JOINT_NUM];
    double right_pos[JOINT_NUM+HAND_JOINT_NUM];
    double right_vel[JOINT_NUM+HAND_JOINT_NUM];
    double right_eff[JOINT_NUM+HAND_JOINT_NUM];

    string portNum;
    ros::Time last_time;
    ros::Duration period;
    serial::Serial my_serial;
};

double motor_position[10];
double motor_speed[10];
double motor_current[10];
double motor_pwm[10];

int32_t goal_position[10];

//extern ros::NodeHandle n;
extern void	init_left_robot();
extern void	init_right_robot();
extern float ljoint_limition[9][2];
extern float rjoint_limition[9][2];
extern void  lFdctrmsgCB();
extern void  rFdctrmsgCB();
extern bmirobot_msg::Robot_ctr lMTctrmsg;
extern bmirobot_msg::Robot_jointfd lJointfdmsg;
extern ros::Publisher lJoint_fdpub;
extern ros::Publisher lMT_ctrpub;

extern bmirobot_msg::Robot_ctr rMTctrmsg;
extern bmirobot_msg::Robot_jointfd rJointfdmsg;
extern ros::Publisher rJoint_fdpub;
extern ros::Publisher rMT_ctrpub;

extern bmirobot_msg::Robot_fdctr 		fdctrmsgl,fdctrmsgr;
extern bmirobot_msg::Robot_fdstatus 	fdstatusmsgl,fdstatusmsgr;

extern bmirobot_msg::Robot_mpu fdmpu;

extern ros::Publisher  lMt_fdctrpub    ;
extern ros::Publisher  lMt_fdstatuspub ;
extern ros::Publisher  lMt_mpupub 	    ;

extern ros::Publisher  rMt_fdctrpub 	;
extern ros::Publisher  rMt_fdstatuspub ;
extern ros::Publisher 	rMt_mpupub 		;

extern ros::Publisher Pointspub;

extern void init_com();
extern void updata_right_send();
extern void updata_right_recevie();
extern void updata_left_send();
extern void updata_left_recevie();
extern void com_save_data();
extern void updata_points_recevie();
extern void updata_points_send();
extern void dynamic_gravity(double as[7],double *rst);

