#! /usr/bin/env python3
"""
Usage:
(robodiff)$ python eval_real_robot.py -i <ckpt_path> -o <save_dir> --robot_ip <ip_of_ur5>

================ Human in control ==============
Robot movement:
Move your SpaceMouse to move the robot EEF (locked in xy plane).
Press SpaceMouse right button to unlock z axis.
Press SpaceMouse left button to enable rotation axes.

Recording control:
Click the opencv window (make sure it's in focus).
Press "C" to start evaluation (hand control over to policy).
Press "Q" to exit program.

================ Policy in control ==============
Make sure you can hit the robot hardware emergency-stop button quickly!

Recording control:
Press "S" to stop evaluation and gain control back.
"""

# %%
import time
from multiprocessing.managers import SharedMemoryManager
import click
import cv2
import numpy as np
import torch
import dill
import hydra
import pathlib
import skvideo.io
from omegaconf import OmegaConf  # new resolver
import scipy.spatial.transform as st
from diffusion_policy.real_world.real_env_grasp import RealEnv
#from diffusion_policy.real_world.spacemouse_shared_memory import Spacemouse
from diffusion_policy.common.precise_sleep import precise_wait
from diffusion_policy.real_world.real_inference_util import (
    get_real_obs_resolution,
    get_real_obs_dict)
from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.workspace.base_workspace import BaseWorkspace
from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.cv2_util import get_image_transform

import rospy
from std_msgs.msg import Float64MultiArray
import multiprocessing as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from eval_diffusion.msg import robot

OmegaConf.register_new_resolver("eval", eval, replace=True)

key_dataset = [0.0,0.0,0.0,0.0,0.0,0.0]
parent_conn, child_conn = mp.Pipe()    # image
parent_conn_1, child_conn_1 = mp.Pipe()    # image
parent_conn_r, child_conn_r = mp.Pipe() # robot
robot_data = {}


def tendon_keycallback(msg):  # get the Pose of key
    for i in range(6):
    	key_dataset[i] = msg.data[i]

def callback3(msg):  # get the color image1
	global parent_conn_1
	image = dict()
	t = time.time()    # set the time of receiving time
	image['time_stamps'] = t
	cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
	frame = cv_image
	image['data'] = frame
	#print(frame.shape)
	#cv2.imshow('default', frame)
	#cv2.waitKey(1) 
	rospy.sleep(0.1)
	parent_conn_1.send(image)   # send

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

rospy.init_node("main_r", anonymous=True)
rate = rospy.Rate(10) 
sub1 = rospy.Subscriber("/key", Float64MultiArray, tendon_keycallback)
sub = rospy.Subscriber("/d435/color/image_raw", Image, callback1)
sub3 = rospy.Subscriber("/d435_2/color/image_raw", Image, callback3)
sub2 = rospy.Subscriber("/robot_data", robot, callback2)
pub = rospy.Publisher('/vis_img', Float64MultiArray, queue_size=10)


@click.command()
@click.option('--input', '-i', required=True, help='Path to checkpoint')
@click.option('--output', '-o', required=True, help='Directory to save recording')
#@click.option('--robot_ip', '-ri', required=True, help="UR5's IP address e.g. 192.168.0.204")
@click.option('--match_dataset', '-m', default=None, help='Dataset used to overlay and adjust initial condition')
@click.option('--match_episode', '-me', default=None, type=int, help='Match specific episode from the match dataset')
@click.option('--vis_camera_idx', default=0, type=int, help="Which RealSense camera to visualize.")
@click.option('--init_joints', '-j', is_flag=True, default=False,
              help="Whether to initialize robot joint configuration in the beginning.")
@click.option('--steps_per_inference', '-si', default=6, type=int, help="Action horizon for inference.")
@click.option('--max_duration', '-md', default=60, help='Max duration for each epoch in seconds.')
@click.option('--frequency', '-f', default=10, type=float, help="Control frequency in Hz.")
@click.option('--command_latency', '-cl', default=0.01, type=float,
              help="Latency between receiving SapceMouse command to executing on Robot in Sec.")
def main(input, output, match_dataset, match_episode,
         vis_camera_idx, init_joints,
         steps_per_inference, max_duration,
         frequency, command_latency):   # robot_ip
    # load match_dataset
    match_camera_idx = 0
    episode_first_frame_map = dict()   # match_dataset = None
    if match_dataset is not None:
        match_dir = pathlib.Path(match_dataset)
        match_video_dir = match_dir.joinpath('videos')
        for vid_dir in match_video_dir.glob("*/"):
            episode_idx = int(vid_dir.stem)
            match_video_path = vid_dir.joinpath(f'{match_camera_idx}.mp4')
            if match_video_path.exists():
                frames = skvideo.io.vread(
                    str(match_video_path), num_frames=1)
                episode_first_frame_map[episode_idx] = frames[0]
    print(f"Loaded initial frame for {len(episode_first_frame_map)} episodes")

    # load checkpoint
    ckpt_path = input
    payload = torch.load(open(ckpt_path, 'rb'), pickle_module=dill)
    cfg = payload['cfg']
    cls = hydra.utils.get_class(cfg._target_)
    workspace = cls(cfg)
    workspace: BaseWorkspace
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)

    # hacks for method-specific setup.
    action_offset = 0
    delta_action = False
    #print('11111')
    if 'diffusion' in cfg.name:
        # diffusion model
        policy: BaseImagePolicy
        policy = workspace.model
        if cfg.training.use_ema:
            policy = workspace.ema_model

        device = torch.device('cuda')
        policy.eval().to(device)

        # set inference params
        policy.num_inference_steps = 16  # DDIM inference iterations
        policy.n_action_steps = policy.horizon - policy.n_obs_steps + 1

    elif 'robomimic' in cfg.name:
        # BCRNN model
        policy: BaseImagePolicy
        policy = workspace.model

        device = torch.device('cuda')
        policy.eval().to(device)

        # BCRNN always has action horizon of 1
        steps_per_inference = 1
        action_offset = cfg.n_latency_steps
        delta_action = cfg.task.dataset.get('delta_action', False)

    elif 'ibc' in cfg.name:
        policy: BaseImagePolicy
        policy = workspace.model
        policy.pred_n_iter = 5
        policy.pred_n_samples = 4096

        device = torch.device('cuda')
        policy.eval().to(device)
        steps_per_inference = 1
        action_offset = 1
        delta_action = cfg.task.dataset.get('delta_action', False)
    else:
        raise RuntimeError("Unsupported policy type: ", cfg.name)

    # setup experiment
    dt = 1 / frequency

    obs_res = get_real_obs_resolution(cfg.task.shape_meta)
    n_obs_steps = cfg.n_obs_steps
    print("n_obs_steps: ", n_obs_steps)
    print("steps_per_inference:", steps_per_inference)
    print("action_offset:", action_offset)

    with SharedMemoryManager() as shm_manager:
        with RealEnv(                                # change/fix the realenv  !!!!
                output_dir=output,
                #robot_ip=robot_ip,
                frequency=frequency,
                n_obs_steps=n_obs_steps,
                obs_image_resolution=obs_res,
                obs_float32=True,
                init_joints=init_joints,
                enable_multi_cam_vis=True,
                record_raw_video=True,
                # number of threads per camera view for video recording (H.264)
                thread_per_video=2,
                # video recording quality, lower is better (but slower).
                video_crf=21,
                shm_manager=shm_manager,
                child_conn = child_conn,
                child_conn_1 = child_conn_1,
                child_conn_r = child_conn_r) as env:             #  Spacemouse(shm_manager=shm_manager) as sm
            #print(1111111111)
            cv2.setNumThreads(2)

            # Should be the same as demo
            # realsense exposure
            #env.realsense.set_exposure(exposure=120, gain=0)           # change the realsense !!!!
            # realsense white balance
            #env.realsense.set_white_balance(white_balance=5900)

            print("Waiting for realsense")
            time.sleep(1.0)

            print("Warming up policy inference")
            obs = env.get_obs()
            with torch.no_grad():
                policy.reset()
                obs_dict_np = get_real_obs_dict(
                    env_obs=obs, shape_meta=cfg.task.shape_meta)
                obs_dict = dict_apply(obs_dict_np,
                                      lambda x: torch.from_numpy(x).unsqueeze(0).to(device))
                result = policy.predict_action(obs_dict)
                action = result['action'][0].detach().to('cpu').numpy()
                assert action.shape[-1] == 2
                del result

            print('Ready!')
            while True:
                # ========= human control loop ==========
                print("Human in control!")
                #print('1111111')
                state = env.get_robot_state()
                print(state)
                #target_pose = state['TargetTCPPose']
                target_pose = np.array(key_dataset)          # need to be fix
                #print(target_pose)
                t_start = time.monotonic()
                iter_idx = 0
                while True and match_dataset is not None:
                    # calculate timing
                    t_cycle_end = t_start + (iter_idx + 1) * dt
                    t_sample = t_cycle_end - command_latency
                    t_command_target = t_cycle_end + dt

                    # pump obs
                    
                    obs = env.get_obs()
                    
                    # visualize
                    episode_id = env.replay_buffer.n_episodes
                    vis_img = obs[f'camera_{vis_camera_idx}'][-1]
                    match_episode_id = episode_id
                    print('match_episode_id',match_episode_id)
                    print('episode_first_frame_map',episode_first_frame_map)
                    if match_episode is not None:
                        match_episode_id = match_episode
                    if match_episode_id in episode_first_frame_map:
                    	 match_img = episode_first_frame_map[match_episode_id]
                    	 ih, iw, _ = match_img.shape
                    	 oh, ow, _ = vis_img.shape
                    	 tf = get_image_transform(
                    	 	input_res=(iw, ih),
                    	 	output_res=(ow, oh),
                    	 	bgr_to_rgb=False)
                    	 match_img = tf(match_img).astype(np.float32) / 255
                    	 vis_img = np.minimum(vis_img, match_img)

                    text = f'Episode: {episode_id}'
                    cv2.putText(
                        vis_img,
                        text,
                        (10, 20),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.5,
                        thickness=1,
                        color=(255, 255, 255)
                    )
                    cv2.imshow('default', vis_img[..., ::-1])
                    key_stroke = cv2.pollKey()
                    if key_stroke == ord('q'):
                        # Exit program
                        env.end_episode()
                        exit(0)
                    elif key_stroke == ord('c'):
                        # Exit human control loop
                        # hand control over to the policy
                        break

                    precise_wait(t_sample)
                    # get teleop command
                    #sm_state = sm.get_motion_state_transformed()
                    # print(sm_state)
                    #dpos = sm_state[:3] * (env.max_pos_speed / frequency)
                    #drot_xyz = sm_state[3:] * (env.max_rot_speed / frequency)

                    #if not sm.is_button_pressed(0):
                        # translation mode
                        #drot_xyz[:] = 0
                    #else:
                        #dpos[:] = 0
                    #if not sm.is_button_pressed(1):
                        # 2D translation mode
                        #dpos[2] = 0

                    #drot = st.Rotation.from_euler('xyz', drot_xyz)
                    #target_pose[:3] += dpos
                    #target_pose[3:] = (drot * st.Rotation.from_rotvec(
                        #target_pose[3:])).as_rotvec()
                    # clip target pose
                    #target_pose[:2] = np.clip(target_pose[:2], [0.25, -0.45], [0.77, 0.40])

                    target_pose = np.array(key_dataset)

                    # execute teleop command
                    env.exec_actions(
                        actions=[target_pose],
                        timestamps=[t_command_target - time.monotonic() + time.time()])
                    precise_wait(t_cycle_end)
                    iter_idx += 1

                # ========== policy control loop ==============
                try:
                    # start episode
                    global pub
                    policy.reset()
                    start_delay = 1.0
                    eval_t_start = time.time() + start_delay
                    t_start = time.monotonic() + start_delay
                    env.start_episode(eval_t_start)
                    # wait for 1/30 sec to get the closest frame actually
                    # reduces overall latency
                    frame_latency = 1 / 30
                    precise_wait(eval_t_start - frame_latency, time_func=time.time)
                    print("Started!")
                    iter_idx = 0
                    term_area_start_timestamp = float('inf')
                    perv_target_pose = None
                    while True:
                        # calculate timing
                        t_cycle_end = t_start + (iter_idx + steps_per_inference) * dt

                        # get obs
                        #print('get_obs')
                        obs = env.get_obs()
                        obs_timestamps = obs['timestamp']
                        print(f'Obs latency {time.time() - obs_timestamps[-1]}')

                        # run inference
                        with torch.no_grad():
                            s = time.time()
                            obs_dict_np = get_real_obs_dict(
                                env_obs=obs, shape_meta=cfg.task.shape_meta)
                            obs_dict = dict_apply(obs_dict_np,
                                                  lambda x: torch.from_numpy(x).unsqueeze(0).to(device))
                            result = policy.predict_action(obs_dict)
                            # this action starts from the first obs step
                            action = result['action'][0].detach().to('cpu').numpy()
                            print('Inference latency:', time.time() - s)

                        # convert policy action to env actions
                        #print("delta_action",delta_action)
                        if delta_action:
                            assert len(action) == 1
                            if perv_target_pose is None:
                                perv_target_pose = obs['robot_eef_pose'][-1]
                            this_target_pose = perv_target_pose.copy()
                            this_target_pose[[0, 1]] += action[-1]
                            perv_target_pose = this_target_pose
                            this_target_poses = np.expand_dims(this_target_pose, axis=0)
                        else:
                            this_target_poses = np.zeros((len(action), len(target_pose)), dtype=np.float64)
                            this_target_poses[:] = target_pose
                            this_target_poses[:, [0, 1]] = action

                        # deal with timing
                        # the same step actions are always the target for
                        action_timestamps = (np.arange(len(action), dtype=np.float64) + action_offset
                                             ) * dt + obs_timestamps[-1]
                        action_exec_latency = 0.01
                        curr_time = time.time()
                        is_new = action_timestamps > (curr_time + action_exec_latency)
                        if np.sum(is_new) == 0:
                            # exceeded time budget, still do something
                            this_target_poses = this_target_poses[[-1]]
                            # schedule on next available step
                            next_step_idx = int(np.ceil((curr_time - eval_t_start) / dt))
                            action_timestamp = eval_t_start + (next_step_idx) * dt
                            print('Over budget', action_timestamp - curr_time)
                            action_timestamps = np.array([action_timestamp])
                        else:
                            this_target_poses = this_target_poses[is_new]
                            action_timestamps = action_timestamps[is_new]

                        # clip actions
                        this_target_poses[:, :2] = np.clip(
                            this_target_poses[:, :2], [0.25, -0.45], [0.77, 0.40])

                        # visualize
                        
                        episode_id = env.replay_buffer.n_episodes
                        print('episode_id',episode_id)
                        #vis_img = obs[f'camera_{vis_camera_idx}'][-1]
                        vis_img = obs[f'camera_{vis_camera_idx}'][-1,:,:,::-1].copy()
                        #print('vis_camera_idx',vis_camera_idx)
                        text = 'Episode: {}, Time: {:.1f}'.format(
                            episode_id, time.monotonic() - t_start
                        )
                        #pub.publish(vis_img)
                        #print('text',text)
                        cv2.putText(
                            vis_img,
                            text,
                            (10, 20),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.5,
                            thickness=1,
                            color=(255, 255, 255)
                        )
                        print('vis_img_shape',vis_img.shape)
                        #print('vis_img',vis_img[..., ::-1])
                        #cv2.namedWindow('default', cv2.WINDOW_NORMAL)
                        #cv2.imshow('default', vis_img[..., ::-1])
                        #key_stroke = cv2.pollKey()
                        #cv2.imshow('default', vis_img)
                        #cv2.pollKey()
                        #if key_stroke == ord('s'):
                            # Stop episode
                            # Hand control back to human
                            #env.end_episode()
                            #print('Stopped.')
                            #break
                        # execute actions
                        pub.publish(Float64MultiArray(data=this_target_poses))
                        env.exec_actions(
                            actions=this_target_poses,
                            timestamps=action_timestamps
                        )
                        #print("this_target_poses",this_target_poses)
                        print(f"Submitted {len(this_target_poses)} steps of actions.")
                        # auto termination
                        terminate = False
                        if time.monotonic() - t_start > max_duration:
                            terminate = True
                            print('Terminated by the timeout!')

                        term_pose = np.array(
                            [3.40948500e-01, 2.17721816e-01, 4.59076878e-02, 2.22014183e+00, -2.22184883e+00,
                             -4.07186655e-04])
                        curr_pose = obs['robot_eef_pose'][-1]
                        dist = np.linalg.norm((curr_pose - term_pose)[:2], axis=-1)
                        if dist < 0.03:         # dist for the control !!!!
                            # in termination area
                            curr_timestamp = obs['timestamp'][-1]
                            if term_area_start_timestamp > curr_timestamp:
                                term_area_start_timestamp = curr_timestamp
                            else:
                                term_area_time = curr_timestamp - term_area_start_timestamp
                                if term_area_time > 0.5:
                                    terminate = True
                                    print('Terminated by the policy!')
                        else:
                            # out of the area
                            term_area_start_timestamp = float('inf')

                        if terminate:
                            env.end_episode()
                            break

                        # wait for execution
                        precise_wait(t_cycle_end - frame_latency)
                        iter_idx += steps_per_inference
                        print('finish')

                except KeyboardInterrupt:
                    print("Interrupted!")
                    # stop robot.
                    env.end_episode()

                print("Stopped.")


# %%
if __name__ == '__main__':
    
    main()
    rospy.spin()
    
