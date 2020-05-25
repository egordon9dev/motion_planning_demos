#ifndef VISUAL_NODE_HPP
#define VISUAL_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

class VisualNode {
public:
    VisualNode(ros::NodeHandle& nh);

    void setPath(std::vector<geometry_msgs::Point>&& points);
    
    void run();

private:
    ros::Publisher path_pub;

    std::vector<geometry_msgs::Point> path_points;
};

#endif