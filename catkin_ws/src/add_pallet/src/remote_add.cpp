#include <iostream>
#include <ros/ros.h>

// Dynamic reconfigure parameter includes
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

using namespace dynamic_reconfigure;
using namespace std;

int main(int argc, char **argv) {
    /* TODO: I think cin is getting in the way of clean exits of the code. ^C in the terminal is slow */

    ros::init(argc, argv, "remote_add_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    while (ros::ok()) {
        // Dynamic reconfigure variables
        ReconfigureRequest srv_req;
        ReconfigureResponse srv_resp;
        StrParameter string_param;
        DoubleParameter w_param, d_param;
        Config conf;

        string pallet_name;
        float width, depth;


        // int number_to_get;
        // nh.getParam("/pallet_library/depth_param", number_to_get);
        // cout << "Read Integer: " << number_to_get << endl;

        // ros::param::dump("cfg/dynamic_params.yaml");




        rate.sleep(); // Give some time for other node to initialize and print its statements

        cout << "Name pallet: ";
        cin >> pallet_name;
        cout << "Enter pallet width and depth:" << endl;
        cin >> width >> depth;

        // push pallet type (name used for identification)
        string_param.name = "AA_type_param";
        string_param.value = pallet_name;
        conf.strs.push_back(string_param);

        // push pallet width (cm)
        w_param.name = "AA_max_width_param";
        w_param.value = width;
        conf.doubles.push_back(w_param);

        // push palled depth (cm)
        d_param.name = "AA_max_depth_param";
        d_param.value = depth;
        conf.doubles.push_back(d_param);

        srv_req.config = conf;

        ros::service::call("/pallet_library/set_parameters", srv_req, srv_resp);
        rate.sleep();
    }

    return 0;
}