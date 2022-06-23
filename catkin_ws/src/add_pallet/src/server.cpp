#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <add_pallet/dynamic_paramsConfig.h>


void callback(add_pallet::dynamic_paramsConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d %s",
             config.int_param,
             config.bool_param?"True":"False");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "server");

    dynamic_reconfigure::Server<add_pallet::dynamic_paramsConfig> server;
    dynamic_reconfigure::Server<add_pallet::dynamic_paramsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Spinning node");

    ros::spin();

    return 0;
}