#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

class MoveItInterfaceNode : public rclcpp::Node
{
public:
  MoveItInterfaceNode() : Node("moveit_interface_node")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10, std::bind(&MoveItInterfaceNode::topic_callback, this, std::placeholders::_1));
    
    // Initialize Completion Publisher
    completion_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/robot_status/move_complete", 10);
    
    // We spin up a separate thread for MoveIt because it needs to block
    // while the node continues to process callbacks (like TF).
    move_group_thread_ = std::thread([this]() {
      // Initialize MoveGroupInterface
      // Note: We need to wait for the node to be fully spun up? 
      // Actually, MoveGroupInterface takes the node as a shared pointer.
      // But we can't pass 'this' shared_from_this() in constructor.
      // So we will initialize it lazily or use a separate node for it?
      // Standard pattern: create the interface in the thread or after construction.
    });
  }

  void init_moveit()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "panda_arm");
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized for panda_arm");
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!move_group_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroup not initialized!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Received target pose");
    
    move_group_->setPoseTarget(*msg);

    // FIX: Explicitly select OMPL pipeline FIRST, then force RRTConnect planner
    move_group_->setPlanningPipelineId("ompl");
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setStartStateToCurrentState();
    move_group_->setGoalTolerance(0.05);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
      move_group_->execute(my_plan);

      // Signal Completion
      auto status_msg = std_msgs::msg::Bool();
      status_msg.data = true;
      completion_publisher_->publish(status_msg);
      RCLCPP_INFO(this->get_logger(), "Motion Complete. Signal sent.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      auto status_msg = std_msgs::msg::Bool();
      status_msg.data = false;
      completion_publisher_->publish(status_msg);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completion_publisher_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::thread move_group_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItInterfaceNode>();
  
  // Initialize MoveIt after node creation so shared_from_this() works
  node->init_moveit();

  // MultiThreadedExecutor is often safer for MoveIt
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
