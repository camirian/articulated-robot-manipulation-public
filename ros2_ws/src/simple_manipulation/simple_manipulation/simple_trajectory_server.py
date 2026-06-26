import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import time
import math

class SimpleTrajectoryServer(Node):
    def __init__(self):
        super().__init__('simple_trajectory_server')
        
        # Action Servers
        self._arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/panda_arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        self._hand_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/panda_hand_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        # Publisher to Isaac Sim (Global topic for all joints)
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        
        self.get_logger().info('Simple Trajectory Server (Arm + Hand) has been started.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        goal = goal_handle.request
        trajectory = goal.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points
        
        if not points:
            self.get_logger().info('Received empty trajectory')
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        start_time = self.get_clock().now()
        
        # Simple execution: Iterate through points and publish them
        # In a real controller, we would interpolate. 
        # Here we rely on MoveIt sending points with reasonable time spacing
        # and we wait for the duration of each point.
        
        last_time_from_start = 0.0
        
        for point in points:
            # Calculate time to wait
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            duration = time_from_start - last_time_from_start
            
            if duration > 0:
                time.sleep(duration)
            
            # Publish JointState
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = joint_names
            msg.position = point.positions
            msg.velocity = point.velocities
            # msg.effort = point.effort
            
            self.publisher_.publish(msg)
            
            last_time_from_start = time_from_start

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        self.get_logger().info('Goal succeeded')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectoryServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
