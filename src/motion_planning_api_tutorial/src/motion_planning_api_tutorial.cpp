
#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_api_tutorial");

int main(int argc, char **argv)
{
  printf("hello world motion_planning_api_tutorial package\n");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> motion_planning_api_tutorial_node =
      rclcpp::Node::make_shared("motion_planning_api_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(motion_planning_api_tutorial_node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  const std::string PLANNING_GROUP = "panda_arm";

  robot_model_loader::RobotModelLoader robot_model_loader(motion_planning_api_tutorial_node, "robot_description");
  const moveit::core::RobotModelPtr &robot_model = robot_model_loader.getModel();
  RCLCPP_INFO(LOGGER, "Model frame: %s", robot_model->getModelFrame().c_str());
  const moveit::core::JointModelGroup *joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP);
  RCLCPP_INFO(LOGGER, "Joint model group: %s", joint_model_group->getName().c_str());

  return 0;
}
