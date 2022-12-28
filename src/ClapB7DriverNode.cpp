//
// Created by arslan on 22.05.2022.
//
#include "ClapB7Driver.hpp"

std::shared_ptr<ClapB7Driver> driver;

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "ClapB7DriverNode");
    ros::NodeHandle nh;
    driver = std::shared_ptr<ClapB7Driver>(new ClapB7Driver(nh));

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}