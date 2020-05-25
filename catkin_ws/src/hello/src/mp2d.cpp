#include <iostream>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


bool isStateValid(const ob::State *state) {
    return true;
}

void plan()
{
    auto space(std::make_shared<ob::SE2StateSpace>());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    auto space_info(std::make_shared<ob::SpaceInformation>(space));
    space_info->setStateValidityChecker(isStateValid);
    auto problem(std::make_shared<ob::ProblemDefinition>(space_info));

    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(start_point.x);
    start->setY(start_point.y);
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(goal_point.x);
    goal->setY(goal_point.y);
    
    problem->setStartAndGoalStates(start, goal);

    space_info->setup();
    auto planner(std::make_shared<og::RRTConnect>(space_info));
    planner->setProblemDefinition(problem);
    planner->setup();
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);
    if (solved)
    {
        og::PathGeometric* path = problem->getSolutionPath()->as<og::PathGeometric>();
        const std::vector<ob::State*>& states = path->getStates();
        path_points.clear();
        for(const auto* generic_state : states) {
            const ob::SE2StateSpace::StateType* state = generic_state->as<ob::SE2StateSpace::StateType>();
            geometry_msgs::Point point;
            point.x = state->getX();
            point.y = state->getY();
            point.z = 0;
            path_points.push_back(point);
        }
    }
}

void display_path() {
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "mp2d");
    ros::NodeHandle nh;

    path_pub = nh.advertise<visualization_msgs::MarkerArray>("path", 10);
    obstacles_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate rate(1);

    while (ros::ok()) {
        start_point.x = start_point.y = -5;
        goal_point.x = goal_point.y = 5;
        start_point.z = goal_point.z = 0;

        plan();
        display_path();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}