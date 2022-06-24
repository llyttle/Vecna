#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <memory>

// Param Configuration file
#include <add_pallet/dynamic_paramsConfig.h>

// FeatureDetectionInfo class
#include <add_pallet/pallet_features.h>


void FeatureDetectionInfo::FeatureDetectionInfo::initializeParams(const float width, const float width_tolerance, const float depth, const float depth_tolerance) {
        s_width = width;
        s_width_tolerance = width_tolerance;
        s_depth = depth;
        s_depth_tolerance = depth_tolerance;
}

void callback(add_pallet::dynamic_paramsConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: [%s] - [%d x %d]",
             config.pallet_type_param.c_str(),
             config.width_param,
             config.depth_param);

    std::shared_ptr<FeatureDetectionInfo> palletpointer = std::make_shared<FeatureDetectionInfo>(config.pallet_type_param);

    palletpointer->initializeParams(config.width_param, 0.1, config.depth_param, 0.1);

    palletpointer->info();


}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pallet_library_node");

    dynamic_reconfigure::Server<add_pallet::dynamic_paramsConfig> server;
    dynamic_reconfigure::Server<add_pallet::dynamic_paramsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
