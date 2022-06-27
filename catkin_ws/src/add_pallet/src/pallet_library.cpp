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

void dynamic_callback(add_pallet::dynamic_paramsConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: [%s]: [%f-%f] x [%f-%f]",
             config.AA_type_param.c_str(),
             config.AA_max_width_param,
             config.AA_min_width_param,
             config.AA_max_depth_param,
             config.AA_min_depth_param);

    std::map<std::string,float> map_i;
    map_i["max_depth"] = config.AA_max_depth_param;
    map_i["max_width"] = config.AA_max_width_param;
    map_i["min_depth"] = config.AA_min_depth_param;
    map_i["min_width"] = config.AA_min_width_param;


    ros::param::set(ros::this_node::getName()+"/"+config.AA_type_param, map_i);
    // TODO: change this path to something not requiring launch from /cfg
    // Dump the entire /pallet_library parameter server to dynamic_params.yaml
    ROS_INFO("%i", system("echo 'ROS Params dumped at : ' $(pwd); rosparam dump dynamic_params.yaml /pallet_library/"));

    // std::shared_ptr<FeatureDetectionInfo> palletpointer = std::make_shared<FeatureDetectionInfo>(config.pallet_type_param);

    // palletpointer->initializeParams(config.width_param, 0.1, config.depth_param, 0.1);

    // palletpointer->info();


}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pallet_library");

    dynamic_reconfigure::Server<add_pallet::dynamic_paramsConfig> server;
    dynamic_reconfigure::Server<add_pallet::dynamic_paramsConfig>::CallbackType f;

    f = boost::bind(&dynamic_callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    return 0;
}
