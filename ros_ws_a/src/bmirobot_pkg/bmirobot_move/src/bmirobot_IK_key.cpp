#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <arpa/inet.h>
#include <random>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <bmirobot_tools/PointsPR.h>
#include <std_msgs/Char.h>


using namespace std;


ros::Publisher key_pub;

class TeleopPR2Keyboard
{
  private:


  public:
    float Point1[6];


    ros::NodeHandle n_;
    ros::Publisher pose_pub_;

    bool dirty;



  void init()
  {
	  dirty = false;
    tf::TransformListener listener;
  	//tf::Transform transform;
  	ros::Duration(1.0).sleep();
    double initP1[7];

    ros::NodeHandle n_private("~");
  }

  ~TeleopPR2Keyboard()   { }
  void keyboardLoop();
  void modifyCmd(char c);
};

int kfd = 0;
struct termios cooked, raw;
TeleopPR2Keyboard* tpk;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bmirobot_keyboard");
  ros::NodeHandle n("bmirobot");

  key_pub = n.advertise<std_msgs::Char>("/bmirobot/key",0);
  
  tpk = new TeleopPR2Keyboard();
  tpk->init();

  signal(SIGINT,quit);
  tpk->keyboardLoop();

  return(0);
}

void TeleopPR2Keyboard::modifyCmd(char c)
{
    float step = 0.1,rstep=1;
   // printf("input:%d\n",c);
    std_msgs::Char newkey;
    newkey.data =c;
    key_pub.publish(newkey);

}
void TeleopPR2Keyboard::keyboardLoop()
{
  char c;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WS' control X-axis");
  puts("Use 'AD' control Y-axis");
  puts("Use 'QE' control Z-axis");
  puts("Use 'IK' control X-rotation ");
  puts("Use 'HJ' control Y-rotation");
  puts("Use 'UO' control Z-rotation");
  puts("Use 'NM' control grasp hand");
  for(;;)
  {
    ros::spinOnce();
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    modifyCmd(c);
  }
}
