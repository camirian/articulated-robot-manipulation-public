#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import time
import copy
import math

class PerformPick(Node):
    def __init__(self):
        super().__init__('perform_pick')
        
        # Subscribe to Object Pose (from Perception Node)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/object_pose',
            self.pose_callback,
            1)
            
        # Subscribe to MoveIt Completion Signal
        self.move_complete_sub = self.create_subscription(
            Bool,
            '/robot_status/move_complete',
            self.move_complete_callback,
            10)
        self.motion_finished = False
            
        # Publisher to MoveIt Interface
        self.target_publisher = self.create_publisher(PoseStamped, '/target_pose', 1)
        
        # Gripper Service
        self.gripper_client = self.create_client(SetBool, '/control_gripper')
        # Check service availability (non-blocking in init)
        
        self.buffer = []
        self.samples_needed = 5
        self.target_locked = False
        
        self.get_logger().info('Coordinator Ready (Robust Sequencing).')
        
        # Start Main Sequence in a detached timer/thread (simplification for script)
        # We will use a Timer to drive the logic "tick" style or just a simple run method if called directly.
        # But rclpy.spin blocks.
        # Ideally, we used a state machine class. For this script, we'll use a One-Shot Timer.
        # self.timer = self.create_timer(1.0, self.main_logic_step) # Removed Timer
        self.state = "OBSERVE" 
        self.retry_count = 0

    def move_complete_callback(self, msg):
        if msg.data:
            self.get_logger().info('Received Robot "DONE" Signal.')
            self.motion_finished = True

    def pose_callback(self, msg):
        if self.state == "WAIT_FOR_DETECTION":
            self.buffer.append(msg)
            if len(self.buffer) >= self.samples_needed:
                self.target_locked = True
                
    # Removed main_logic_step (Timer) - Logic runs in Main Thread now

    def publish_pose(self, x, y, z, qx, qy, qz, qw):
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = "world" # Hardcoded World Frame for observation
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        self.target_publisher.publish(p)

    def execute_observation_phase(self):
        self.get_logger().info(f'--- PHASE 1: OBSERVATION (Retry {self.retry_count}) ---')
        
        # 1. Move to "Look Down" Pose
        # High above table, looking straight down.
        # Panda Home: x=0.3, z=0.5.
        # Rotation: Pointing Down (-Z). 
        # Quaternion: (1, 0, 0, 0) is 180 deg rotation around X axis.
        
        # Hardcoding the Observation Pose
        # UPGRADE: Center on the Target (X=0.45) for optimal FOV
        obs_x = 0.45
        obs_y = 0.0
        obs_z = 0.6 # High up
        
        self.get_logger().info('Moving to STABLE Observation Pose...')
        self.publish_pose(obs_x, obs_y, obs_z, 1.0, 0.0, 0.0, 0.0)
        
        # If we are retrying, rotate wrist?
        # TODO: Implement wrist rotation offset if needed
        
        # ROBUST SYNC: Wait for "Done" Signal instead of sleep
        self.motion_finished = False
        self.get_logger().info('Waiting for confirmation from MoveIt...')
        
        start_wait = time.time()
        timeout = 60.0
        while not self.motion_finished:
            # Check timeout
            if time.time() - start_wait > timeout:
                self.get_logger().error('Motion Timed Out! Aborting.')
                raise SystemExit
            # Background thread handles callbacks, we just wait
            time.sleep(0.1)
            
        self.get_logger().info('Robot confirmed arrival. Starting Perception...')
        
        # STABILIZE CAMERA
        self.get_logger().info('Stabilizing Camera...')
        time.sleep(2.0) 
        
        # TRIGGER DETECTION
        self.found_target = None
        self.get_logger().info('Scanning for target...')
        self.state = "WAIT_FOR_DETECTION"
        self.buffer = []
        
        # Wait for callback
        scan_duration = 5.0 # Increased from 3.0
        start_wait = time.time()
        while time.time() - start_wait < scan_duration:
            # Background thread handles callbacks
            time.sleep(0.1)
            if self.target_locked:
                break
        
        if self.target_locked:
            self.get_logger().info('TARGET ACQUIRED!')
            self.execute_pick_sequence()
        else:
            self.get_logger().info('NO TARGET DETECTED.')
            if self.retry_count < 2:
                self.retry_count += 1
                self.get_logger().info('Retrying with Wrist Rotation...')
                # Todo: Add wrist rotation logic
                self.execute_observation_phase()
            else:
                self.get_logger().error('Max Retries Reached. Aborting.')
                raise SystemExit

    def execute_pick_sequence(self):
        self.get_logger().info('--- PHASE 2: EXECUTE PICK ---')
         # Best target
        target = self.buffer[-1]
        
        # Stop listening
        self.state = "EXECUTING"
        
        # Publisher helper to convert Pose (Camera Frame) to PoseStamped
        # Wait, the PerceptionNode outputs Pose in CAMERA frame?
        # If perception node outputs Pose in CAMERA frame, we need to transform it to WORLD?
        # OR does the move_to_pose handle frame_id?
        # Ideally, move_to_pose (MoveIt) handles any frame_id. So we just pass it through.
        
        # 1. PRE-GRASP
        pre_grasp = copy.deepcopy(target)
        # World Frame: Z is Up. Object is at Z~0.05. We want to be ABOVE it.
        # So we ADD height.
        pre_grasp.pose.position.z += 0.20
        self.target_publisher.publish(pre_grasp)
        self.get_logger().info(f'Moving to Pre-Grasp (Z={pre_grasp.pose.position.z:.2f})...')
        
        # Wait for move (Reuse robust wait? Or sleep? For now sleep is safer/simpler for sequence)
        # But ideally we use the signal. Let's use the signal!
        self.wait_for_move_complete()
        
        # 2. APPROACH
        approach = copy.deepcopy(target)
        approach.pose.position.z += 0.13 # Just above object
        self.target_publisher.publish(approach)
        self.get_logger().info(f'Approaching (Z={approach.pose.position.z:.2f})...')
        self.wait_for_move_complete()
        
        # 3. GRASP
        self.get_logger().info('Grasping...')
        req = SetBool.Request()
        req.data = True
        self.gripper_client.call_async(req)
        time.sleep(2.0)
        
        # 4. LIFT
        self.target_publisher.publish(pre_grasp) # Back up
        self.get_logger().info('Lifting...')
        self.wait_for_move_complete()
        
        self.get_logger().info('DONE.')
        raise SystemExit

    def wait_for_move_complete(self):
        self.motion_finished = False
        start_wait = time.time()
        timeout = 60.0
        while not self.motion_finished:
            if time.time() - start_wait > timeout:
                self.get_logger().error('Motion Timed Out!')
                raise SystemExit
            time.sleep(0.1)

def main(args=None):
    import threading
    rclpy.init(args=args)
    node = PerformPick()
    
    # Spin in a background thread to handle callbacks (deadlock prevention)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        # Run Logic in Main Thread
        node.execute_observation_phase()
        
        # Keep main thread alive if needed, but execute_observation_phase has exit logic
        while rclpy.ok():
            time.sleep(1.0)
            
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
