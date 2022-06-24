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
        IntParameter w_param, d_param;
        Config conf;

        string type;
        int width, depth;

        rate.sleep(); // Wait for other node to initialize and print its statements

        cout << "Name pallet: ";
        cin >> type;
        cout << "Enter pallet width and depth:" << endl;
        cin >> width >> depth;

        // push pallet type (name used for identification)
        string_param.name = "pallet_type_param";
        string_param.value = type;
        conf.strs.push_back(string_param);

        // push pallet width (cm)
        w_param.name = "width_param";
        w_param.value = width;
        conf.ints.push_back(w_param);

        // push palled depth (cm)
        d_param.name = "depth_param";
        d_param.value = depth;
        conf.ints.push_back(d_param);

        srv_req.config = conf;

        ros::service::call("/pallet_library_node/set_parameters", srv_req, srv_resp);
        rate.sleep();
    }

    return 0;
}