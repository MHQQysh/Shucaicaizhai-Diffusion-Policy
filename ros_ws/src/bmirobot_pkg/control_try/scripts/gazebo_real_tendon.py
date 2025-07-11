#! /usr/bin/env python3
"""
1. press the key to control the tendon-driven robot
2. recording the video
3. recording the actions and states (the same format as putT)
4. 'O' for start recording , 'N' for stop recoding
"""
import time
from multiprocessing.managers import SharedMemoryManager     # multi-process
import numpy
import click   # the same name with the sam value
import cv2
import  numpy as np
import scipy.spatial.transform as st   # transformation of rigid object
import  rospy
from OpenGL.raw.GL.KHR.debug import glObjectPtrLabel
from std_msgs.msg import Float64MultiArray
from env_gazebo import RealEnv       # real environment
from diffusion.keystroke_counter import KeystrokeCounter
from pynput.keyboard import Key, KeyCode
from diffusion.precise_sleep import precise_wait

# msg
from control_try.msg import robot
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import multiprocessing as mp
from multiprocessing import shared_memory

key_dataset = [0.0,0.0,0.0,0.0,0.0,0.0]
parent_conn, child_conn = mp.Pipe()    # image
parent_conn_1, child_conn_1 = mp.Pipe()    # image
parent_conn_r, child_conn_r = mp.Pipe() # robot
robot_data = {}

@click.command()
@click.option('--output', '-o', required=True, help="Directory to save demonstration dataset.")
@click.option('--vis_camera_idx', default=0, type=int, help="Which RealSense camera to visualize.")
@click.option('--init_joints', '-j', is_flag=True, default=False, help="Whether to initialize robot joint configuration in the beginning.")
@click.option('--frequency', '-f', default=10, type=float, help="Control frequency in Hz.")   # the set of frequency
@click.option('--command_latency', '-cl', default=0.01, type=float, help="Latency between receiving key command to executing on Robot in Sec.")


def main(output, vis_camera_idx, init_joints, frequency, command_latency):
    dt = 1/frequency     # variable time
    global key_dataset
    global robot_dataset
    global parent_conn
    global child_conn
    global parent_conn_r
    global child_conn_r
    #print(shm.name)
    
    with SharedMemoryManager() as shm_manager:       # share memory ring
           with KeystrokeCounter() as keycounter,\
           	RealEnv(
               output_dir=output,
              	# recording resolution
               obs_image_resolution=(640,480),             # image_size !!  1  (1280,720)
               frequency=frequency,
               init_joints=init_joints,
               enable_multi_cam_vis=True,
               record_raw_video=True,                       # record_video !!  2
               # number of threads per camera view for video recording (H.264)
               thread_per_video=3,                       # thread !!   3
               # video recording quality, lower is better (but slower).
               video_crf=21,
               shm_manager=shm_manager,
               child_conn = child_conn,
               child_conn_1 = child_conn_1,
               child_conn_r = child_conn_r) as env:
            cv2.setNumThreads(1)    # set the solving
            
            # realsense exposure   4
            #print("Unready!")
            #env.realsense.set_exposure(exposure=120, gain=0)    
            # realsense white balance  5
            #env.realsense.set_white_balance(white_balance=5900)
            
            print('Ready!')

            #state = env.get_robot_state()       # change use ros  with ros node
            #target_pose = state['TargetTCPPose']
            target_pose = np.array(key_dataset)
            
            
            time.sleep(5.0)
            t_start = time.monotonic()    # start time
            iter_idx = 0
            stop = False         # whether stop or not
            is_recording = False      # c
            
            while not stop:
                # calculate timing
                
                
                #print(frame)
                t_cycle_end = t_start + (iter_idx + 1) * dt   # t_cycle 
                t_sample = t_cycle_end - command_latency      # sample_time
                t_command_target = t_cycle_end + dt           # 

                # pump obs
                obs = env.get_obs()
                
                press_events = keycounter.get_press_events()
                
                for key_stroke in press_events:
                    if key_stroke == KeyCode(char='z'):
                        # Exit program
                        stop = True
                    elif key_stroke == KeyCode(char='v'):
                        # Start recording
                        env.start_episode(t_start + (iter_idx + 2) * dt - time.monotonic() + time.time())
                        keycounter.clear()
                        is_recording = True
                        print('Recording!')
                    elif key_stroke == KeyCode(char='x'):
                        # Stop recording
                        env.end_episode()
                        keycounter.clear()
                        is_recording = False
                        print('Stopped.')
                    elif key_stroke == Key.backspace:
                        # Delete the most recent recorded episode
                        if click.confirm('Are you sure to drop an episode?'):
                            env.drop_episode()
                            KeystrokeCounter.clear()
                            is_recording = False
                        # delete
                stage = keycounter[Key.space]     # stage ?? the number of space

                # visualize
                vis_img = obs[f'camera_{vis_camera_idx}'][-1, :, :, ::-1].copy()     # get the vis_img !!  6
                #print("vis_img",vis_img.shape)
                episode_id = env.replay_buffer.n_episodes                # episode
                text = f'Episode: {episode_id}, Stage: {stage}'
                if is_recording:
                    text += ', Recording!'
                cv2.putText(
                    vis_img,
                    text,
                    (10, 30),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1,
                    thickness=2,
                    color=(255, 255, 255)
                )

                cv2.imshow('default', vis_img)
                cv2.pollKey()

                precise_wait(t_sample)      # wait until ??
                
                # get teleop command
                #key_state = np.array(key_dataset)
                # print(sm_state)
                #dpos = key_state[:3] * (env.max_pos_speed/frequency)
                #print(target_pose.dtype)
                #drot_xyz = key_state[3:]*(env.max_rot_speed/frequency)

                #if not sm.is_button_pressed(0):
                    # translation mode
                    #drot_xyz[:] = 0
                #else:
                    #dpos[:] = 0
                #if not sm.is_button_pressed(1):
                    # 2D translation mode
                    #dpos[2] = 0

		# joint change
                #drot = st.Rotation.from_euler('xyz', drot_xyz)
                
                #target_pose[:3] += dpos
                #target_pose[3:] = (drot * st.Rotation.from_rotvec(target_pose[3:])).as_rotvec()

                # execute teleop command
                env.exec_actions(
                    actions=[target_pose],
                    timestamps=[t_command_target - time.monotonic() + time.time()],    # timestamps ?? (dt+...)
                    stages=[stage])
                precise_wait(t_cycle_end)
                iter_idx += 1
                #print(iter_idx)

def tendon_keycallback(msg):  # get the Pose of key
    for i in range(6):
    	key_dataset[i] = msg.data[i]

def callback1(msg):  # get the color image1, set the dict
	global parent_conn
	image = dict()
	t = time.time()    # set the time of receiving time
	image['time_stamps'] = t
	cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
	frame = cv_image
	image['data'] = frame
	rospy.sleep(0.1)
	parent_conn.send(image)   # send
	#print("callback",frame.shape)

def callback2(msg):  # get the state of robot
    	#print(1)
    	global robot_data
    	global parent_conn_r
    	robot_data['ActualTCPPose'] = np.array([msg.eff_pose[0],msg.eff_pose[1],msg.eff_pose[2],msg.eff_pose[3],msg.eff_pose[4],msg.eff_pose[5]])
    	robot_data['ActualTCPSpeed'] = np.array([msg.eff_spd[0],msg.eff_spd[1],msg.eff_spd[2],msg.eff_spd[3],msg.eff_spd[4],msg.eff_spd[5]])

    	robot_data['ActualQ'] = np.array([msg.joint_pos[0],msg.joint_pos[1],msg.joint_pos[2],msg.joint_pos[3],msg.joint_pos[4],msg.joint_pos[5],msg.joint_pos[6],msg.joint_pos[7],msg.joint_pos[8]])
    	robot_data['ActualQd'] = np.array([msg.joint_spd[0],msg.joint_spd[1],msg.joint_spd[2],msg.joint_spd[3],msg.joint_spd[4],msg.joint_spd[5],msg.joint_spd[6],msg.joint_spd[7],msg.joint_spd[8]])
    	#robot_data["ActualTCPPose"]=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    	#robot_data["ActualTCPSpeed"]=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    	#robot_data["ActualQ"]=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
    	#robot_data["ActualQd"]=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
    	parent_conn_r.send(robot_data)
    	
def callback3(msg):  # get the color image1
	global parent_conn_1
	image = dict()
	t = time.time()    # set the time of receiving time
	image['time_stamps'] = t
	cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
	frame = cv_image
	image['data'] = frame
	rospy.sleep(0.1)
	parent_conn_1.send(image)   # send
	
# %%
if __name__ == '__main__':
    rospy.init_node("main_R",anonymous=True)
    sub1 = rospy.Subscriber("/key",Float64MultiArray,tendon_keycallback)
    sub = rospy.Subscriber("/d435/color/image_raw", Image, callback1)
    sub2 = rospy.Subscriber("/robot_data", robot, callback2)
    sub3 = rospy.Subscriber("/d435_2/color/image_raw", Image, callback3)
    main()
    rospy.spin()
    
