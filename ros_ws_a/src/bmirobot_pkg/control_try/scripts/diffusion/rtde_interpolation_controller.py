import os
import time
import enum
import rospy
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
import scipy.interpolate as si
import scipy.spatial.transform as st
import numpy as np
from control_try.msg import robot

from diffusion.shared_memory_queue import (
    SharedMemoryQueue, Empty)
from diffusion.shared_memory_ring_buffer import SharedMemoryRingBuffer


class Command(enum.Enum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2


class RTDEInterpolationController(mp.Process):
    """
    To ensure sending command to the robot with predictable latency
    this controller need its separate process (due to python GIL)
    """


    def __init__(self,
            shm_manager: SharedMemoryManager, 
            frequency=125, 
            lookahead_time=0.1, 
            gain=300,
            max_pos_speed=0.25, # 5% of max speed
            max_rot_speed=0.16, # 5% of max speed
            launch_timeout=3,
            tcp_offset_pose=None,
            payload_mass=None,
            payload_cog=None,
            joints_init=None,
            joints_init_speed=1.05,
            soft_real_time=False,
            verbose=False,
            receive_keys=None,
            get_max_k=128,
            child_conn_r = None
            ):
        """
        frequency: CB2=125, UR3e=500
        lookahead_time: [0.03, 0.2]s smoothens the trajectory with this lookahead time
        gain: [100, 2000] proportional gain for following target position
        max_pos_speed: m/s
        max_rot_speed: rad/s
        tcp_offset_pose: 6d pose
        payload_mass: float
        payload_cog: 3d position, center of gravity
        soft_real_time: enables round-robin scheduling and real-time priority
            requires running scripts/rtprio_setup.sh before hand.

        """
        # verify
        assert 0 < frequency <= 500
        assert 0.03 <= lookahead_time <= 0.2
        assert 100 <= gain <= 2000
        assert 0 < max_pos_speed
        assert 0 < max_rot_speed
        if tcp_offset_pose is not None:
            tcp_offset_pose = np.array(tcp_offset_pose)
            assert tcp_offset_pose.shape == (6,)
        if payload_mass is not None:
            assert 0 <= payload_mass <= 5
        if payload_cog is not None:
            payload_cog = np.array(payload_cog)
            assert payload_cog.shape == (3,)
            assert payload_mass is not None
        if joints_init is not None:
            joints_init = np.array(joints_init)
            assert joints_init.shape == (6,)

        super().__init__(name="RTDEPositionalController")
        #rospy.init_node('subscribe_node',anonymous=True)      # ros
        self.robot_data = {}
        #self.robot = robot
        self.child_conn_r = child_conn_r
        
        self.frequency = frequency
        self.lookahead_time = lookahead_time
        self.gain = gain
        self.max_pos_speed = max_pos_speed
        self.max_rot_speed = max_rot_speed
        self.launch_timeout = launch_timeout
        self.tcp_offset_pose = tcp_offset_pose
        self.payload_mass = payload_mass
        self.payload_cog = payload_cog
        self.joints_init = joints_init
        self.joints_init_speed = joints_init_speed
        self.soft_real_time = soft_real_time
        self.verbose = verbose
        
        # build ring buffer
        if receive_keys is None:
            receive_keys = [
                "ActualTCPPose",
                "ActualTCPSpeed",
                "ActualQ",
                "ActualQd"
           ]
        self.robot_data["ActualTCPPose"]=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.robot_data["ActualTCPSpeed"]=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.robot_data["ActualQ"]=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self.robot_data["ActualQd"]=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        
        rtde_r = self.robot_data   # ros
        #print(rtde_r)
        example = dict()
        #print(rtde_r["ActualTCPPose"])
        for key in receive_keys:
            	example[key] = rtde_r[key]
            	example['robot_receive_timestamp'] = time.time()
            	ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            	shm_manager=shm_manager,
            	examples=example,
            	get_max_k=get_max_k,
            	get_time_budget=0.2,
            	put_desired_frequency=frequency
            	)
        self.ready_event = mp.Event()
        self.ring_buffer = ring_buffer
        self.receive_keys = receive_keys
    
    # ========= launch method ===========
    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[RTDEPositionalController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        if wait:
            self.stop_wait()

    def start_wait(self):
        self.ready_event.wait(self.launch_timeout)
        assert self.is_alive()
    
    def stop_wait(self):
        self.join()
    
    @property
    def is_ready(self):
        return self.ready_event.is_set()

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= receive APIs =============
    
    
    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k,out=out)
    
    def get_all_state(self):
        return self.ring_buffer.get_all()
    
    # ========= main loop in process ============
    def run(self):
        # enable soft real-time
        
        if self.soft_real_time:
            os.sched_setscheduler(
                0, os.SCHED_RR, os.sched_param(20))
                
        
        try:
            if self.verbose:
                print(f"[RTDEPositionalController] Connect to robot: {robot_ip}")

            # main loop
            dt = 1. / self.frequency
            
            iter_idx = 0

            keep_running = True
            while keep_running:

                # send command to robot
                t_now = time.monotonic()
                # diff = t_now - pose_interp.times[-1]
                # if diff > 0:
                #     print('extrapolate', diff)
                
                # update robot state
                rtde_r = self.child_conn_r.recv()      # self.robot_data: >0
                #print(rtde_r['ActualQ'])
                state = dict()
                for key in self.receive_keys:
                    state[key] = np.array(rtde_r[key])   # rtde_r->ros-related
                state['robot_receive_timestamp'] = time.time()
                self.ring_buffer.put(state)

                # first loop successful, ready to receive command
                if iter_idx == 0:
                    self.ready_event.set()
                iter_idx += 1

                if self.verbose:
                    print(f"[RTDEPositionalController] Actual frequency {1/(time.perf_counter() - t_start)}")

        finally:
            # manditory cleanup
            self.ready_event.set()

            if self.verbose:
                print(f"[RTDEPositionalController] Disconnected from robot: {robot_ip}")
