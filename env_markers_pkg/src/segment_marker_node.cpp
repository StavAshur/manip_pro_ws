#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segment_marker_node");
    ros::NodeHandle nh;


    // Load parameters
    std::string frame_id;
    double thickness;
    double color_r, color_g, color_b, color_a;
    std::vector<double> point_a, point_b;

    nh.param<std::string>("segment/frame_id", frame_id, "world");
    nh.param<double>("segment/thickness", thickness, 0.02);
    nh.param<double>("segment/color/r", color_r, 0.0);
    nh.param<double>("segment/color/g", color_g, 0.0);
    nh.param<double>("segment/color/b", color_b, 1.0);
    nh.param<double>("segment/color/a", color_a, 1.0);

    nh.getParam("segment/point_a", point_a);
    nh.getParam("segment/point_b", point_b);

    if (point_a.size() != 3 || point_b.size() != 3) {
        ROS_ERROR("Invalid segment points in YAML (must be 3-element arrays)");
        return 1;
    }

    // Create marker
    visualization_msgs::Marker line;
    line.header.frame_id = frame_id;
    line.ns = "env_segments";
    line.id = 1;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.scale.x = thickness;

    line.color.r = color_r;
    line.color.g = color_g;
    line.color.b = color_b;
    line.color.a = color_a;

    geometry_msgs::Point a, b;
    a.x = point_a[0]; a.y = point_a[1]; a.z = point_a[2];
    b.x = point_b[0]; b.y = point_b[1]; b.z = point_b[2];

    line.points.push_back(a);
    line.points.push_back(b);

    ros::Publisher marker_pub =
        nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::Rate rate(5);
    while (ros::ok()) {
        line.header.stamp = ros::Time::now();
        marker_pub.publish(line);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
