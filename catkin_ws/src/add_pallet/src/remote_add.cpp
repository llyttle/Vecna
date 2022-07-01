#include <iostream>
#include <ros/ros.h>
#include <numeric>

// Dynamic reconfigure parameter includes
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

using namespace dynamic_reconfigure;
using namespace std;

template <class T>
string type_map_to_string(map<string,T>  &m)
{
    /**
     * Convert a map of type <std::string, TYPE> to a string
     * Delimeter = ','
     * Key/Value seperator = ';'
     */
    string output = "";

	for (auto it = m.cbegin(); it != m.cend(); it++)
    {
		output += (it->first) + ":" + (std::to_string(it->second)) + ",";
	}

	return output.substr(0, output.size() - 1 );
}

void SetParams(std::string pallet_name)
{
    ReconfigureRequest srv_req;
    ReconfigureResponse srv_resp;
    BoolParameter save_change_param;
    StrParameter type_param, doubles_string, bools_string, ints_string;
    Config conf;

    /**
     * Pallet Type Parameter (name used for identification)
     */
    type_param.name = "AA_dynamic_type_param";
    type_param.value = pallet_name;
    conf.strs.push_back(type_param);

    /**
     * Save Change Parameter (switches whether new configurations are saved to library)
     *
     */
    save_change_param.name = "AA_dynamic_save_change_param";
    save_change_param.value = true;
    conf.bools.push_back(save_change_param);

    /**
     * Bool Parameters
     */
    std::map<std::string,bool> B;
    B["test_switch_true"] = true;
    B["test_switch_false"] = false;
    bools_string.name = "AA_dynamic_bools_string";
    bools_string.value = type_map_to_string<bool>(B);
    conf.strs.push_back(bools_string);


    /**
     * Double Parameters
     */
    std::map<std::string,double> D;
    D["max_depth"] = 2.0;
    D["max_width"] = 2.0;
    D["min_depth"] = 1.0;
    D["min_width"] = 1.0;
    doubles_string.name = "AA_dynamic_doubles_string";
    doubles_string.value = type_map_to_string<double>(D);
    conf.strs.push_back(doubles_string);

    /**
     * Integer Parameters
     */
    std::map<std::string,int> I;
    I["test_int1"] = 67;
    I["test_int2"] = 14;
    I["test_int3"] = 8;
    ints_string.name = "AA_dynamic_ints_string";
    ints_string.value = type_map_to_string<int>(I);
    conf.strs.push_back(ints_string);

    /**
     * add parameters as server request and push updates to the pallet_library server
     */
    srv_req.config = conf;
    ros::service::call("/pallet_library/set_parameters", srv_req, srv_resp);
}

int main(int argc, char **argv)
{
    /**
     * TODO: I think cin is getting in the way of clean exits of the code. ^C in the terminal is slow
     */
    ros::init(argc, argv, "remote_add");
    // ros::NodeHandle nh;
    // ros::Rate rate(10);

    // while (ros::ok())
    // {

        SetParams("mario");

        // rate.sleep();
    // }

    return 0;
}