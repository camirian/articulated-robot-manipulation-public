#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from simple_manipulation import detection

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Subscribers (Use BEST_EFFORT QoS for Isaac Sim compatibility)
        self.subscription_rgb = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        self.subscription_depth = self.create_subscription(
            Image,
            '/camera/depth',
            self.depth_callback,
            qos_profile_sensor_data)
        self.subscription_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            qos_profile_sensor_data)
            
        # Publisher
        self.publisher_pose = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.publisher_mask = self.create_publisher(Image, '/perception/mask', 10)
        
        # Tools
        self.br = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # State
        self.latest_depth_image = None
        self.camera_intrinsics = None
        self.camera_frame_id = "panda_hand_camera"
        self.last_log_time = self.get_clock().now()
        self.last_image_time = self.get_clock().now()
        
        # Watchdog Timer (Check for camera data every 3 seconds)
        self.watchdog_timer = self.create_timer(3.0, self.watchdog_callback)
        
        # Parameters
        self.declare_parameter('debug_mode', True)
        self.debug_mode = self.get_parameter('debug_mode').value
        
        self.get_logger().info('Perception Node Started (Robust Red Detection + TF).')

    def info_callback(self, msg):
        if self.camera_intrinsics is None:
            self.camera_intrinsics = np.array(msg.k).reshape((3, 3))
            self.camera_frame_id = msg.header.frame_id
            self.destroy_subscription(self.subscription_info)
    
    def watchdog_callback(self):
        """Check if camera data is being received"""
        time_since_last_image = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
        if time_since_last_image > 3.0:
            self.get_logger().warn(
                f'No Camera Data received for {time_since_last_image:.1f}s! '
                'Check Topic Names and QoS (Isaac Sim requires BEST_EFFORT).'
            )

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth Callback Error: {e}')

    def image_callback(self, msg):
        # Update watchdog timestamp
        self.last_image_time = self.get_clock().now()
        
        if self.latest_depth_image is None or self.camera_intrinsics is None:
            # Throttle waiting logs
            if (self.get_clock().now() - self.last_log_time).nanoseconds > 1e9: # 1s
                self.get_logger().info('Waiting for Depth/Intrinsics...')
                self.last_log_time = self.get_clock().now()
            return

        try:
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")

            # --- Detect red cube (pure helper; mask returned for debug) ---
            cx, cy, max_area, mask = detection.detect_red_cube(cv_image)

            # Publish Debug Mask
            debug_msg = self.br.cv2_to_imgmsg(mask, encoding="mono8")
            debug_msg.header = msg.header
            self.publisher_mask.publish(debug_msg)

            if cx is not None:
                d = detection.depth_at(self.latest_depth_image, cx, cy)

                if d is not None:
                    # Back-project to camera frame (pure helper)
                    X, Y, Z = detection.backproject(
                        cx, cy, d, self.camera_intrinsics)

                    # Create Pose in Camera Frame
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = self.camera_frame_id
                    pose_msg.pose.position.x = float(X)
                    pose_msg.pose.position.y = float(Y)
                    pose_msg.pose.position.z = float(Z)
                    pose_msg.pose.orientation.w = 1.0  # Identity in Camera Frame

                    # TRANSFORM TO WORLD FRAME (panda_link0)
                    try:
                        # Wait for transform (0.1s timeout)
                        transform = self.tf_buffer.lookup_transform(
                            "panda_link0",
                            self.camera_frame_id,
                            rclpy.time.Time())

                        pose_world = do_transform_pose(pose_msg.pose, transform)

                        # Overwrite Orientation with Fixed Grasp Orientation (Down)
                        # Top-Down Grasp: q(1,0,0,0) (180 about X)
                        grasp_x, grasp_y, grasp_z, grasp_w = detection.GRASP_ORIENTATION_DOWN
                        pose_world.orientation.x = grasp_x
                        pose_world.orientation.y = grasp_y
                        pose_world.orientation.z = grasp_z
                        pose_world.orientation.w = grasp_w

                        final_msg = PoseStamped()
                        final_msg.header.stamp = pose_msg.header.stamp
                        final_msg.header.frame_id = "panda_link0"
                        final_msg.pose = pose_world

                        # SANITY CHECK: Widen Z-Range (0.0 to 0.50)
                        z_world = pose_world.position.z
                        if 0.0 <= z_world < 0.50:
                            self.publisher_pose.publish(final_msg)
                            self.get_logger().info(f'Target in World Frame: Z={z_world:.2f} [VALID]')
                        else:
                            self.get_logger().warn(f'Ignored Ghost Object at Z={z_world:.2f}')

                    except Exception as tf_ex:
                        self.get_logger().warn(f'TF Error: {tf_ex}')

            if self.debug_mode:
                cv2.imshow("Debug: Mask", mask)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Image Callback Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
