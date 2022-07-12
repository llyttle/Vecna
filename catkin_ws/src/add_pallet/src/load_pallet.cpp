#include <ros/ros.h>
#include <iostream>

// YAML library for reading
#include <yaml-cpp/yaml.h>

#include <add_pallet/pallet_features.h>

using namespace std;

template <typename T>
std::map<std::string,T> yaml_to_type_map(YAML::Node n)
{
    std::map<std::string,T> map;

    for(YAML::const_iterator it=n.begin();it!=n.end();++it)
    {
        std::string key = it->first.as<std::string>();
        key = "A_"+key+"_param";
        T value = it->second.as<T>();
        map[key] = value;
    }

    return map;
}

void load_pallet(std::string name)
/**
 * Load parameters from the dynamic_params yaml file into the parameter server.
 * Create a pallet class that uses those parameters
 */
{
    std::string path;
    if (!ros::param::get("/pallet_library/AA_dynamic_yaml_file_path_param", path))
    {
        ROS_INFO("Required Parameter 'AA_dynamic_yaml_file_path_param' was not specified");
    } else
    {
        /**
         * First create a YAML node for the file as a whole. One must also be made for the specific pallet that is being read
         */
        YAML::Node Pallet_Library;
        Pallet_Library = YAML::LoadFile(path.c_str());
        YAML::Node Pallet_Params = Pallet_Library[name];

        if (!Pallet_Params)
        {
            ROS_INFO("Pallet specified: '%s' is not found", name.c_str());
        } else
        {
            /**
             * NTS: Using a for loop to read through pallet parameters keeps the code flexible
             */
            update_server(
                name,
                yaml_to_type_map<bool>(Pallet_Params["bools"]),
                yaml_to_type_map<double>(Pallet_Params["doubles"]),
                yaml_to_type_map<int>(Pallet_Params["ints"])
            );
        }
    }
}

int main(int argc, char **argv)
{
    /**
     * Init load_pallet node and search for the /active group for pallets to enable.
     * pallets can be passed to the command line with:       <pallet_name>:=true
     */
    ros::init(argc, argv, "load_pallet");

    std::map<std::string, bool> enable;
    ros::param::get("/pallet_library/active", enable);

    for (const auto &p : enable)
    {
        if (p.second)
        {
            ROS_INFO("Load Pallet Request: [%s]", p.first.c_str());
            load_pallet(p.first);
        }
    }

    return 0;
}