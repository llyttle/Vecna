#include <ros/ros.h>

int main(int argc, char **argv)
{
    /**
     * TODO: I think cin is getting in the way of clean exits of the code. ^C in the terminal is slow
     */
    ros::init(argc, argv, "test_subscriber");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    std::string name;
    while (ros::ok())
    {
        if (nh.getParam("/pallet_library/AA_dynamic_type_param", name))
        {
            std::cout << "current name: " << name << std:: endl;
        }
        rate.sleep();
    }

    return 0;
}