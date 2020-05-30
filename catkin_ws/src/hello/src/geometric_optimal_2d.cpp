/** 
 * Assymptotically-Optimal Geometric Motion Planning Demo
 * plans a path from start to goal using an assymptotically optimal
 * planner. this takes around 20-50 ms to run a full plan (with 1000 samples)
 * several planners can be used e.g. og::FMT, og::BFMT, og::InformedRRTstar
 * 
 * how to run:
 *  $ ./hello geometric_optimal_2d
 *  
 * */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ros/ros.h>
#include "VisualNode.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state) {
    return true;
}

std::vector<geometry_msgs::Point> plan()
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
    start->setX(-5);
    start->setY(-5);
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(5);
    goal->setY(5);
    
    problem->setStartAndGoalStates(start, goal);
    ob::OptimizationObjectivePtr objective = std::make_shared<ob::PathLengthOptimizationObjective>(space_info);
    problem->setOptimizationObjective(objective);

    space_info->setup();
    auto planner(std::make_shared<og::FMT>(space_info));
    planner->setNumSamples(150);
    planner->setProblemDefinition(problem);
    planner->setup();
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);
    if (solved)
    {
        og::PathGeometric* path = problem->getSolutionPath()->as<og::PathGeometric>();
        const std::vector<ob::State*>& states = path->getStates();
        std::vector<geometry_msgs::Point> path_points;
        for(const auto* generic_state : states) {
            const ob::SE2StateSpace::StateType* state = generic_state->as<ob::SE2StateSpace::StateType>();
            geometry_msgs::Point point;
            point.x = state->getX();
            point.y = state->getY();
            point.z = 0;
            path_points.push_back(point);
        }
        return std::move(path_points);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dmp2d");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    VisualNode visual_node(nh);

    while (ros::ok()) {
        ros::Time t0 = ros::Time::now();
        visual_node.setPath(plan());
        double dt = ros::Duration(ros::Time::now() - t0).toSec();
        ROS_INFO("planing time: %.6f", dt);
        visual_node.run();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}