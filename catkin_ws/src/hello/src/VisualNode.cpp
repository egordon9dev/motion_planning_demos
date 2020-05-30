#include "VisualNode.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

VisualNode::VisualNode(ros::NodeHandle& nh) {
    path_pub = nh.advertise<visualization_msgs::MarkerArray>("path", 10);
    start_point.x = start_point.y = goal_point.x = goal_point.y = 0;
}

void VisualNode::setPath(std::vector<geometry_msgs::Point>&& points) {
    path_points = std::move(points);
}

void VisualNode::run() {
    if (path_points.size() < 2) {
        return;
    }

    geometry_msgs::Point start_point = path_points.front();
    geometry_msgs::Point goal_point = path_points.back();

    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "local";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = .1;
    path_marker.color.r = 0;
    path_marker.color.g = 1;
    path_marker.color.b = 0;
    path_marker.color.a = 1;
    path_marker.pose.orientation.x = 0;
    path_marker.pose.orientation.y = 0;
    path_marker.pose.orientation.z = 0;
    path_marker.pose.orientation.w = 1;
    path_marker.lifetime = ros::Duration(1000);
    path_marker.points = path_points;

    visualization_msgs::Marker start_marker;
    start_marker.header.frame_id = "local";
    start_marker.header.stamp = ros::Time::now();
    start_marker.ns = "test";
    start_marker.id = 1;
    start_marker.type = visualization_msgs::Marker::SPHERE;
    start_marker.action = visualization_msgs::Marker::ADD;
    start_marker.pose.position = start_point;
    start_marker.pose.orientation.x = 0;
    start_marker.pose.orientation.y = 0;
    start_marker.pose.orientation.z = 0;
    start_marker.pose.orientation.w = 1;
    start_marker.color.r = 1;
    start_marker.color.g = 0;
    start_marker.color.b = 0;
    start_marker.color.a = 1;
    start_marker.scale.x = 1;
    start_marker.scale.y = 1;
    start_marker.scale.z = 1;
    start_marker.lifetime = ros::Duration(10);

    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "local";
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "test";
    goal_marker.id = 2;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.position = goal_point;
    goal_marker.pose.orientation.x = 0;
    goal_marker.pose.orientation.y = 0;
    goal_marker.pose.orientation.z = 0;
    goal_marker.pose.orientation.w = 1;
    goal_marker.color.r = 0;
    goal_marker.color.g = 0;
    goal_marker.color.b = 1;
    goal_marker.color.a = 1;
    goal_marker.scale.x = 1;
    goal_marker.scale.y = 1;
    goal_marker.scale.z = 1;
    goal_marker.lifetime = ros::Duration(10);

    visualization_msgs::MarkerArray marker_arr;
    marker_arr.markers.push_back(start_marker);
    marker_arr.markers.push_back(goal_marker);
    marker_arr.markers.push_back(path_marker);
    path_pub.publish(marker_arr);
}