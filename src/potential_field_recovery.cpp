#include "potential_field_recovery.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <memory>
#include <thread>
#include <utility>

#include "tf2/utils.h"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_recoveries/recovery.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include "visualization_msgs/msg/Marker.h"

namespace potential_field_recovery {

PotentialFieldRecovery::PotentialFieldRecovery()
    : Recovery<Action>(), costmap_sub_(nullptr) {}
PotentialFieldRecovery::~PotentialFieldRecovery() {}

void PotentialFieldRecovery::onConfigure() {

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to retrieve node"};
  }
  std::shared_ptr<tf2_ros::Buffer> tf = this->tf_;
  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
      node, "local_costmap/costmap_raw");

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  vel_pub_->publish(cmd_vel);

  // name_ = name;
  // costmap_ = costmap_ros;

  nav2_util::declare_parameter_if_not_declared(node, "sim_granularity",
                                               rclcpp::ParameterValue(0.017));
  node->get_parameter("sim_granularity", sim_granularity_);

  nav2_util::declare_parameter_if_not_declared(node, "frequency",
                                               rclcpp::ParameterValue(20.0));
  node->get_parameter("frequency", frequency_);

  nav2_util::declare_parameter_if_not_declared(node, "min_dist",
                                               rclcpp::ParameterValue(0.4));
  node->get_parameter("min_dist", min_dist_);

  nav2_util::declare_parameter_if_not_declared(node, "max_rotational_vel",
                                               rclcpp::ParameterValue(0.2));
  node->get_parameter("max_rotational_vel", max_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(node, "max_trans_vel",
                                               rclcpp::ParameterValue(0.3));
  node->get_parameter("max_trans_vel", max_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(node, "tolerance",
                                               rclcpp::ParameterValue(0.05));
  node->get_parameter("tolerance", tolerance_);

  RCLCPP_INFO(node->get_logger(), "The recovery plugin has been initialized.");
}

Status PotentialFieldRecovery::onRun(
    const std::shared_ptr<const Action::Goal> command) {

  // auto node = node_.lock();

  geometry_msgs::msg::PoseStamped current_pose;

  // using the utils of the navigation2 framework to grab the current robot pose
  if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_,
                                 robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return Status::FAILED;
  }

  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double theta = tf2::getYaw(current_pose.pose.orientation);

  double start_offset = 0 - angles::normalize_angle(theta);


    if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_,
                                   robot_base_frame_, transform_tolerance_)) {
      RCLCPP_INFO(logger_, "Current robot pose is not available.");
    }

    double robot_x = current_pose.pose.position.x;
    double robot_y = current_pose.pose.position.y;
    double robot_orientation = tf2::getYaw(current_pose.pose.orientation);

    unsigned int width = costmap_sub_->getCostmap()->getSizeInCellsX();
    unsigned int height = costmap_sub_->getCostmap()->getSizeInCellsY();

    double cur_world_x = 0.0;
    double cur_world_y = 0.0;

    double cur_robot_x = current_pose.pose.position.x;
    double cur_robot_y = current_pose.pose.position.y;

    float target_x = 0.0, target_y = 0.0;
    float target_r = 0.0, target_phi = 0.0;

    float cur_min_dist = 100000.0;

    /*
        TO-DO: Insert the visualization messages !
    */

    for (int posX = 0; posX < width; ++posX) {
      for (int posY = 0; posY < height; ++posY) {
        float CellCost = costmap_sub_->getCostmap()->getCost(posX, posY);
        costmap_sub_->getCostmap()->mapToWorld(posX, posY, cur_world_x,
                                               cur_world_y);
        if (costmap_sub_->getCostmap()->getCost(posX, posY) !=
            nav2_costmap_2d::FREE_SPACE) {
          double dx = cur_world_x - cur_robot_x;
          double dy = cur_world_y - cur_robot_y;

          geometry_msgs::msg::Point p;

          p.x = cur_world_x;
          p.y = cur_world_y;
          p.z = 0.0;
          // TO-DO: add push back for visualization

          if (fabs(dx) >= 0.01 && fabs(dy) >= 0.01) {
            float factor = 1.f / ((dx * dx + dy * dy) * (dx * dx + dy * dy));

            float d = sqrt(dx * dx + dy * dy);
            if (d < cur_min_dist) {
              cur_min_dist = d;
            }

            target_x -= factor * dx;
            target_y -= factor * dy;
          }
        }
      }
    }

    // TO-DO publish visualization and clear the vis-points


    target_phi = angles::normalize_angle(atan2(target_y, target_x) - robot_orientation);

    float drive_part_x = 0.0;
    float drive_part_y = 0.0;

    geometry_msgs::msg::Twist cmd_vel;

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;

    drive_part_x = std::cos(target_phi);
    drive_part_y = std::sin(target_phi);

    cmd_vel.linear.x = drive_part_x * max_trans_vel_;
    cmd_vel.linear.y = drive_part_y * max_trans_vel_;

    vel_pub_->publish(cmd_vel);
  

  return Status::SUCCEEDED;
}
}; // namespace potential_field_recovery
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(potential_field_recovery::PotentialFieldRecovery,
                       nav2_core::Recovery)
