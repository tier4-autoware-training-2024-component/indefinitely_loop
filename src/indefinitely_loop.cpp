#include "route_handler/route_handler.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tier4_planning_msgs/srv/set_lanelet_route.hpp"

autoware_planning_msgs::msg::LaneletSegment constLaneletToLaneletSegment(
  const lanelet::ConstLanelet & lanelet)
{
  autoware_planning_msgs::msg::LaneletSegment lanelet_segment;
  lanelet_segment.preferred_primitive.id = lanelet.id();
  lanelet_segment.preferred_primitive.primitive_type = lanelet.attribute("type").value();
  lanelet_segment.primitives.emplace_back(lanelet_segment.preferred_primitive);
  return lanelet_segment;
}

class IndefinitelyLoopNode : public rclcpp::Node
{
public:
  IndefinitelyLoopNode() : Node("indefinitely_loop"), had_map_msg_(nullptr)
  {
    route_handler_ptr_ = std::make_shared<route_handler::RouteHandler>();

    had_map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
      "/map/vector_map", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&IndefinitelyLoopNode::hadMapCallback, this, std::placeholders::_1));

    route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
      "/planning/mission_planning/route", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&IndefinitelyLoopNode::routeCallback, this, std::placeholders::_1));

    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", 1,
      std::bind(&IndefinitelyLoopNode::odometryCallback, this, std::placeholders::_1));

    set_lanelet_route_client_ = create_client<tier4_planning_msgs::srv::SetLaneletRoute>(
      "/planning/mission_planning/mission_planner/set_lanelet_route");

    // wait for service
    while (!set_lanelet_route_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }

    RCLCPP_INFO(get_logger(), "Indefinitely loop node has been started.");
  }

private:
  // subscriber of HAD map
  autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr had_map_msg_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr had_map_sub_;

  // subscriber of route
  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route_msg_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;

  // subscribe odometry
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_msg_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  // client for set_lanelet_route service
  rclcpp::Client<tier4_planning_msgs::srv::SetLaneletRoute>::SharedPtr set_lanelet_route_client_;

  std::shared_ptr<route_handler::RouteHandler> route_handler_ptr_;

  bool is_next_goal_pose_set_ = false;
  geometry_msgs::msg::Pose next_goal_pose_;

  void hadMapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
  {
    std::cerr << "HAD map received" << std::endl;
    had_map_msg_ = msg;
    route_handler_ptr_->setMap(*msg);
  }

  void routeCallback(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
  {
    std::cerr << "Route received" << std::endl;
    route_msg_ = msg;
    route_handler_ptr_->setRoute(*msg);
    // std::cerr << "Start Pose: " << route_handler_ptr_->getStartPose().position.x << ", "
    //           << route_handler_ptr_->getStartPose().position.y << std::endl;
    // std::cerr << "Goal Pose: " << route_handler_ptr_->getGoalPose().position.x << ", "
    //           << route_handler_ptr_->getGoalPose().position.y << std::endl;
  }

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odometry_msg_ = msg;
    if (route_msg_ && had_map_msg_ && odometry_msg_) {
      lanelet::ConstLanelet current_lanelet_sequence;
      route_handler_ptr_->getClosestLaneletWithinRoute(
        odometry_msg_->pose.pose, &current_lanelet_sequence);
      auto next_lanelet_sequence = route_handler_ptr_->getNextLanelets(current_lanelet_sequence);

      // std::cerr << "Current lanelet sequence: " << current_lanelet_sequence.id() << std::endl;
      // // std::cerr << "Next lanelet size: " << next_lanelet_sequence.size() << std::endl;
      // for (const auto & lanelet_sequence : next_lanelet_sequence) {
      //   std::cerr << "Next lanelet sequence: " << lanelet_sequence.id() << std::endl;
      // }

      if (route_handler_ptr_->isInGoalRouteSection(current_lanelet_sequence)) {
        std::cerr << "In goal route section" << std::endl;
        tier4_planning_msgs::srv::SetLaneletRoute::Request::SharedPtr request =
          std::make_shared<tier4_planning_msgs::srv::SetLaneletRoute::Request>();
        request->header = route_msg_->header;
        if (!is_next_goal_pose_set_) {
          next_goal_pose_ = route_handler_ptr_->getStartPose();
          is_next_goal_pose_set_ = true;
        }
        request->goal_pose = next_goal_pose_;
        next_goal_pose_ = route_handler_ptr_->getGoalPose();

        // autoware_planning_msgs::msg::LaneletSegment segment;
        // segment.preferred_primitive.
        request->segments.emplace_back(constLaneletToLaneletSegment(current_lanelet_sequence));
        request->segments.emplace_back(constLaneletToLaneletSegment(next_lanelet_sequence.front()));

        // request->uuid = route_msg_->uuid;

        auto result_future = set_lanelet_route_client_->async_send_request(request);

        // std::cerr << "Waiting for service response..." << std::endl;

        result_future.wait_for(std::chrono::seconds(
          1));  // You can use a timeout or another mechanism to avoid indefinite waiting

        // if (result_future.valid()) {
        //   auto response = result_future.get();
        //   if (response) {
        //     RCLCPP_INFO(
        //       this->get_logger(), "Service call succeeded. Response: %s",
        //       response->status.message.c_str());
        //   } else {
        //     RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        //   }
        // } else {
        //   RCLCPP_ERROR(this->get_logger(), "Future not valid after waiting.");
        // }
      }
    }
  }
};

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndefinitelyLoopNode>());
  rclcpp::shutdown();

  return 0;
}
