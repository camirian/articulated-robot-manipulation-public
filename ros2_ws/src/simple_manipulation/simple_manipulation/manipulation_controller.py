import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ManipulationController(Node):
    """
    A ROS 2 node that controls a Franka robot in Isaac Sim to perform a pick and place task.
    This node acts as a simple state machine, publishing joint commands to move the robot
    through a predefined sequence of states.
    """
    def __init__(self):
        super().__init__('manipulation_controller')
        
        # Publisher for the robot's joint commands
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        
        # Timer to drive the state machine
        self.timer = self.create_timer(2.0, self.state_machine_callback)
        
        self.joint_names = [
            "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", 
            "panda_joint5", "panda_joint6", "panda_joint7", 
            "panda_finger_joint1", "panda_finger_joint2"
        ]
        
        # --- DEFINE THE STATES AND POSES ---
        self.states = [
            "HOME", 
            "PRE_GRASP", 
            "GRASP", 
            "CLOSE_GRIPPER", 
            "LIFT", 
            "MOVE_TO_PLACE", 
            "PLACE",
            "OPEN_GRIPPER",
            "RETURN_HOME"
        ]
        self.current_state_index = 0
        
        self.poses = {
            "HOME": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04],
            "PRE_GRASP": [0.0, 0.1, 0.0, -1.8, 0.0, 1.9, 0.785, 0.04, 0.04],
            "GRASP": [0.0, 0.45, 0.0, -1.35, 0.0, 1.75, 0.785, 0.04, 0.04],
            "CLOSE_GRIPPER": [0.0, 0.45, 0.0, -1.35, 0.0, 1.75, 0.785, 0.0, 0.0],
            "LIFT": [0.0, 0.45, 0.0, -1.35, -0.2, 1.9, 0.785, 0.0, 0.0],
            "MOVE_TO_PLACE": [1.2, 0.45, 0.0, -1.35, -0.2, 1.9, 0.785, 0.0, 0.0],
            "PLACE": [1.2, 0.6, 0.0, -1.2, -0.2, 1.8, 0.785, 0.0, 0.0],
            "OPEN_GRIPPER": [1.2, 0.6, 0.0, -1.2, -0.2, 1.8, 0.785, 0.04, 0.04],
            "RETURN_HOME": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04]
        }

    def state_machine_callback(self):
        """
        The main callback that executes on a timer. It publishes the command
        for the current state and advances to the next state.
        """
        if self.current_state_index >= len(self.states):
            self.get_logger().info('Pick and place sequence complete.')
            self.timer.cancel()
            return

        current_state = self.states[self.current_state_index]
        self.get_logger().info(f'--- Executing State: {current_state} ---')
        
        # Get the joint positions for the current state
        target_positions = self.poses[current_state]
        
        # Create and publish the trajectory message
        self.publish_joint_command(target_positions)
        
        # Advance to the next state for the next timer callback
        self.current_state_index += 1

    def publish_joint_command(self, positions):
        """
        Publishes a JointState message to the robot's controller.
        """
        msg = JointState()
        msg.name = self.joint_names
        msg.position = [float(p) for p in positions]
        # msg.velocity = [] # Optional
        # msg.effort = []   # Optional
        
        self.publisher_.publish(msg)
        self.get_logger().info('Published joint command.')


def main(args=None):
    rclpy.init(args=args)
    controller_node = ManipulationController()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()