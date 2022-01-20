#include <ros/ros.h>

#include "elevator_localizer.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "elevator_localizer_node");

    ros::NodeHandle n;

    elevator_localizer::ElevatorLocalizer ElevatorLocalizer_;

    ros::spin();

    return (0);
}
