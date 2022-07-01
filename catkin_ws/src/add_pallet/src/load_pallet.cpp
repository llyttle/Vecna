#include <iostream>
#include <ros/ros.h>

// Dynamic reconfigure parameter includes
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

// YAML library for reading
#include <yaml-cpp/yaml.h>

using namespace dynamic_reconfigure;
using namespace std;



void load_pallet(std::string name)
{
    std::string path;
    if (!ros::param::get("/pallet_library/AA_yaml_file_path", path))
    {
        ROS_INFO("Required Parameter 'AA_yaml_file_path' was not specified");
    } else
    {
        YAML::Node param_library;
        param_library = YAML::LoadFile(path.c_str());
        YAML::Node m = param_library[name];

        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::StrParameter string_param;
        // dynamic_reconfigure::DoubleParameter w_param, d_param;
        dynamic_reconfigure::Config conf;

        if (!m)
        {
            ROS_INFO("Pallet specified: '%s' is not found", name.c_str());
        }

        /**
         * NTS: Using a for loop to read through pallet parameters keeps the code flexible
         */
        std::map<std::string,float> read_map;

        for(YAML::const_iterator it=m.begin();it != m.end();++it)
        {
            std::string key = it->first.as<std::string>();
            key = "A_"+key+"_param";
            double value = it->second.as<double>();
            read_map[key] = value;
        }





        string_param.name = "AA_type_param";
        string_param.value = "MARIO";
        conf.strs.push_back(string_param);


        srv_req.config = conf;

        // Push parameter updates to the pallet_library parameter server
        ros::service::call("/pallet_library/set_parameters", srv_req, srv_resp);
    }
}

int main(int argc, char **argv)
{
    /**
     * Init load_pallet node and search for the pallet_name param.
     * pallet_name is passed into the command line with _pallet_name:= <SOME_NAME>
     */
    ros::init(argc, argv, "load_pallet");
    ros::NodeHandle nh("~");

    std::string type;
    nh.getParam("type", type);

    ROS_INFO("Pallet to Load: [%s]", type.c_str());

    load_pallet(type);

    return 0;
}