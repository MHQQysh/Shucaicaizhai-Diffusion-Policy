// ROS
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>

//gazebo
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetLinkState.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

//key
#include <unistd.h>
#include<sstream>
#include<vector>
#include<stdio.h>
#include<termio.h>

//moveit
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Eigen>


// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

string arm_group_name = "right_arm";
string gripper_group_name = "right_hand";
string gripper = "robot";
string object_name = "box";   // box_grasping
string box_place = "box_2";   // box_placing

robot_state::JointModelGroup* joint_model_group;
Eigen::Isometry3d end_effector_state;   // translation matrix

float translate[3] = {0.0,0.0,0.0};      //  applied for IK solution
Eigen::Matrix3d rotationMatrix;        // transform matrix
std::vector<double> joint_values={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double timeout = 0.1;
int num = 0;

//  set the data (interpolation) 


    // keyboard
char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return buf;
}

    void moveArmToPose(std::vector<double> joint_positions)
    {
        moveit::planning_interface::MoveGroupInterface arm_group(arm_group_name);
        
        
        arm_group.setJointValueTarget(joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            arm_group.execute(plan);
        }
        else
        {
            ROS_ERROR("Unreachable");
        }
        
    }

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "right_hand_joint1";
  posture.joint_names[1] = "right_hand_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "right_hand_joint1";
  posture.joint_names[1] = "right_hand_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

// add object, problem: show in moveit, but not in gazebo
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[0].header.frame_id = "base_link";
  collision_objects[0].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.075;
  collision_objects[0].primitives[0].dimensions[1] = 0.075;
  collision_objects[0].primitives[0].dimensions[2] = 0.075;

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = -0.2;
  collision_objects[0].primitive_poses[0].position.z = -0.15;
  // END_SUB_TUTORIAL
  collision_objects[0].primitive_poses[0].orientation.x = 0;
  collision_objects[0].primitive_poses[0].orientation.y = 0;
  collision_objects[0].primitive_poses[0].orientation.z = 0;
  collision_objects[0].primitive_poses[0].orientation.w = 1;

  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}


    // IK solution moveit
    void moveit_IK(robot_state::RobotStatePtr kinematic_state){
	Eigen::Vector3d translation(translate[0], translate[1], translate[2]);
	for (int i = 0; i < 3; i++)
  		{
    			ROS_INFO("translate: %f", translate[i]);
  		}
	end_effector_state.translation() = translation;
	end_effector_state.linear() = rotationMatrix;
	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);
	if (found_ik)
	{
  		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  		for (int i = 0; i < 7; i++)
  		{
    			ROS_INFO("Joint: %f", joint_values[i]);
  		}
	}
	else
	{
  	ROS_INFO("Did not find IK solution");
	}
   }
    
    // transform
    Eigen::Matrix3d quaternion_to_rotation_matrix(geometry_msgs::Quaternion quaternion) {
    Eigen::Quaterniond eigen_quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    Eigen::Matrix3d rotation_matrix = eigen_quaternion.toRotationMatrix();
    return rotation_matrix;
	}

    
    void setGripperJointValue(const vector<double>& joint_values)
    {
        moveit::planning_interface::MoveGroupInterface gripper_group("right_hand");
        
        gripper_group.setJointValueTarget(joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (gripper_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            ROS_INFO("Plan succeeded!");
            gripper_group.execute(plan);
        }
        else
        {
            ROS_ERROR("Error plan");
        }
        num = 0;
        
    }

    bool grabObject(std::vector<double> joint_positions)
    {
        // 
        char c = getch();
        if(c==106){
        vector<double> open_joint_values = {0.8, -0.8};
        
        setGripperJointValue(open_joint_values);
        
        
        moveArmToPose(joint_positions);

        vector<double> close_joint_values = {0.37, -0.37};
        
        setGripperJointValue(close_joint_values);
        
        

        ros::Duration(1.0).sleep();
       }
       
       // set the box static
       
        return true;
    }

    bool placeObject(std::vector<double> place_pose)
    {
        // 
        moveArmToPose(place_pose);

        // 
        vector<double> open_joint_values = {0.45, -0.45};
        setGripperJointValue(open_joint_values);

        ros::Duration(1.0).sleep();

        return true;
    }
    
    void execute(std::vector<double> joint_values)
    {
        
        geometry_msgs::Pose place_pose;
        place_pose.position.x = 0.3;
        place_pose.position.y = 0.2;
        place_pose.position.z = 0.8;
        place_pose.orientation.w = 1.0;

        if (grabObject(joint_values))
        {
            //if (placeObject(place_pose))
            //{
            //    ROS_INFO("finish");
            //}
            //else
            //{
            //    ROS_ERROR("Error place");
            //}
        }
        else
        {
            ROS_ERROR("Error grasp");
        }
    }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grab_and_place");
    ros::AsyncSpinner spinner(2);
    spinner.start();        // set the spinner
    ros::NodeHandle node_handle_;
    
    
    geometry_msgs::Pose initial_pose, pose_place;    // initial_pose: grasping;  pose_place: placing
    // 
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");       //robot_description
    robot_model::RobotModelPtr kinematic_model;
    kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    joint_model_group = kinematic_model->getJointModelGroup("right_arm");
    kinematic_state->setToDefaultValues();
    
    
    moveit::planning_interface::MoveGroupInterface arm_group(arm_group_name);  //set the motion
    moveit::planning_interface::MoveGroupInterface gripper_group(gripper_group_name);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;   // add the object

    // 
    ros::ServiceClient get_model_state_client = node_handle_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::ServiceClient set_model_state_client = node_handle_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient client = node_handle_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");   // link
    
    gazebo_msgs::SetModelState set_model_state;
    // 
    //arm_group.setGoalPositionTolerance(0.01);
    //arm_group.setGoalOrientationTolerance(0.01);
    //arm_group.setPlanningTime(5.0);
    //gripper_group.setGoalPositionTolerance(0.01);

    //
    gazebo_msgs::GetModelState getModelState;   // get object name
    gazebo_msgs::GetModelState getBoxState;    // get box_2 name
    gazebo_msgs::GetLinkState srv;
    srv.request.link_name = "robot::right_hand1";
    
    
    getModelState.request.model_name = object_name;
    getBoxState.request.model_name = box_place;
    
    gazebo_msgs::ModelState object_state;
    gazebo_msgs::ModelState box_state;
    
    //getModelState.request.model_name = object_name;
    if (get_model_state_client.call(getModelState) && get_model_state_client.call(getBoxState))
    {
    	object_state.pose = getModelState.response.pose;
    	box_state.pose = getBoxState.response.pose;
         //
    }
    
    initial_pose = object_state.pose;
    rotationMatrix = quaternion_to_rotation_matrix(initial_pose.orientation);
    translate[0] = initial_pose.position.x-0.16;
    translate[1] = initial_pose.position.y;
    translate[2] = initial_pose.position.z;
    moveit_IK(kinematic_state);
    // kinematic
    while(ros::ok()){
    execute(joint_values);
    
    // up
    rotationMatrix = quaternion_to_rotation_matrix(initial_pose.orientation);
    translate[0] = initial_pose.position.x-0.16;
    translate[1] = initial_pose.position.y;
    translate[2] = initial_pose.position.z+0.2;
    moveit_IK(kinematic_state);
    char c = getch();
    if(c==107){
    moveArmToPose(joint_values);
    }
    
    // 
    initial_pose = box_state.pose;
    rotationMatrix = quaternion_to_rotation_matrix(initial_pose.orientation);
    translate[0] = initial_pose.position.x-0.17;
    translate[1] = initial_pose.position.y-0.03;
    translate[2] = initial_pose.position.z+0.12;
    moveit_IK(kinematic_state);
    
    if(c==108){
    moveArmToPose(joint_values);
    
    ros::Duration(1.0).sleep();
    vector<double> open_joint_values = {0.8, -0.8};
        
        setGripperJointValue(open_joint_values);
    }
    
    ros::spinOnce();
    }
    return 0;
}

