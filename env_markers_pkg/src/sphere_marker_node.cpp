#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sphere_marker_node");
  ros::NodeHandle nh;


  std::string yaml_path;
  if (!nh.getParam("config_file", yaml_path))
  {
    yaml_path = ros::package::getPath("env_markers_pkg") + "/config/sphere_params.yaml";
    ROS_WARN_STREAM("Parameter 'config_file' not specified, using default: " << yaml_path);
  }
  else
  {
    ROS_INFO_STREAM("Using YAML file: " << yaml_path);
  }
  // Load YAML
  YAML::Node config = YAML::LoadFile(yaml_path);
  auto pos = config["sphere"]["position"];
  auto color = config["sphere"]["color"];
  double radius = config["sphere"]["radius"].as<double>();

  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "sphere_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pos["x"].as<double>();
  marker.pose.position.y = pos["y"].as<double>();
  marker.pose.position.z = pos["z"].as<double>();
  marker.pose.orientation.w = 1.0;
  marker.scale.x = radius * 2;
  marker.scale.y = radius * 2;
  marker.scale.z = radius * 2;
  marker.color.r = color["r"].as<double>();
  marker.color.g = color["g"].as<double>();
  marker.color.b = color["b"].as<double>();
  marker.color.a = color["a"].as<double>();

  ros::Rate rate(10);
  while (ros::ok())
  {
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);
    rate.sleep();
  }

  return 0;
}
