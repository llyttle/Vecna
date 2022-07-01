#include <ros/ros.h>
#include <memory>
#include <sstream>
#include <vector>

// Dynamic Reconfigure includes
#include <dynamic_reconfigure/server.h>

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
    ROS_INFO("Reconfigure Request: [%s]", config.AA_dynamic_type_param.c_str());

    std::map<std::string,std::string> write_map;

    write_map["doubles_string"] = config.AA_dynamic_doubles_string;
    write_map["bools_string"]   = config.AA_dynamic_bools_string;
    write_map["ints_string"]   = config.AA_dynamic_ints_string;

    ROS_INFO("Doubles: [%s]", write_map["doubles_string"].c_str());
    ROS_INFO("Bools: [%s]", write_map["bools_string"].c_str());
    ROS_INFO("Integers: [%s]", write_map["ints_string"].c_str());

    // std::stringstream ss(write_map["double_vals"]);

    // std::map<std::string std::string> M;
    // while (ss.good())
    // {
    //     std::string substr1, substr2;
    //     std::getline(ss,substr1,",");
    //     std::getline(substr1,substr2,":");
    //     // M[substr1] = substr1.substr(substr1.size()+1);
    //     std::cout << substr1 << std::endl << substr2 << std::endl;
    // }


    if (!config.AA_dynamic_save_change_param)
    {
        ROS_INFO("Pallet not saved to YAML");
    } else
    {
        ROS_INFO("Saving pallet configuration to YAML");
        /**
         * Write incomming reconfig to the dynamic_params.yaml library as a struct to preserve ordering
         * Otherwise, identical values under different names are not kept
         */
        ros::param::set(ros::this_node::getName()+"/"+config.AA_dynamic_type_param, write_map);

        /**
         * Dump the entire /pallet_library service to dynamic_params.yaml
         * TODO: change this path to something not requiring launch from a specific location
         * TODO: fix rosparam dump to dump all parameters in server simultaniously
         */
        int resp;
        resp = system("rosparam dump -v /home/loren.lyttle/tests/catkin_ws/src/add_pallet/cfg/dynamic_params.yaml /pallet_library/");
    }

    // std::shared_ptr<FeatureDetectionInfo> palletpointer = std::make_shared<FeatureDetectionInfo>(config.AA_type_param);
    // palletpointer->initializeParams(config.AA_max_width_param, 0.1, config.AA_max_depth_param, 0.1);
    // palletpointer->info();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pallet_library");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<add_pallet::dynamic_paramsConfig> server;
    dynamic_reconfigure::Server<add_pallet::dynamic_paramsConfig>::CallbackType f;

    server.setCallback(dynamic_callback);

    ros::spin();
    return 0;
}
