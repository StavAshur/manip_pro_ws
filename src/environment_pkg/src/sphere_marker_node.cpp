#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class SphereMarkerNode : public rclcpp::Node
{
public:
  SphereMarkerNode() : Node("sphere_marker_node")
  {
    // Declare parameters with default values
    this->declare_parameter("center_x", 0.0);
    this->declare_parameter("center_y", 0.0);
    this->declare_parameter("center_z", 0.0);
    this->declare_parameter("radius", 0.05);
    this->declare_parameter("color_r", 0.0);
    this->declare_parameter("color_g", 1.0);
    this->declare_parameter("color_b", 0.0);
    this->declare_parameter("color_a", 0.8);
    this->declare_parameter("frame_id", "world");

    // Read them once at startup
    this->get_parameter("center_x", cx_);
    this->get_parameter("center_y", cy_);
    this->get_parameter("center_z", cz_);
    this->get_parameter("radius", radius_);
    this->get_parameter("color_r", cr_);
    this->get_parameter("color_g", cg_);
    this->get_parameter("color_b", cb_);
    this->get_parameter("color_a", ca_);
    this->get_parameter("frame_id", frame_id_);

    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "visualization_marker", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&SphereMarkerNode::publish_marker, this));

    RCLCPP_INFO(this->get_logger(), "Sphere marker node started with parameters:");
    RCLCPP_INFO(this->get_logger(), " center = (%.2f, %.2f, %.2f)", cx_, cy_, cz_);
    RCLCPP_INFO(this->get_logger(), " radius = %.3f", radius_);
  }

private:
  void publish_marker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "sphere_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = cx_;
    marker.pose.position.y = cy_;
    marker.pose.position.z = cz_;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = radius_ * 2.0;
    marker.scale.y = radius_ * 2.0;
    marker.scale.z = radius_ * 2.0;

    marker.color.r = cr_;
    marker.color.g = cg_;
    marker.color.b = cb_;
    marker.color.a = ca_;

    marker.lifetime = rclcpp::Duration(0, 0);

    publisher_->publish(marker);
  }

  // Parameters
  double cx_, cy_, cz_, radius_;
  double cr_, cg_, cb_, ca_;
  std::string frame_id_;

  // ROS objects
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SphereMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
