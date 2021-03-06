#include <iostream>
#include <ros/ros.h>
#include <numeric>

// Dynamic reconfigure parameter includes
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

template <class T>
std::string type_map_to_string(std::map<std::string,T>  &m)
{
    /**
     * Convert a map of type <std::string, TYPE> to a string
     * Delimeter = ','
     * Key/Value seperator = ';'
     */
    std::string output = "";

	for (auto it = m.cbegin(); it != m.cend(); it++)
    {
		output += (it->first) + ":" + (std::to_string(it->second)) + ",";
	}

	return output.substr(0, output.size() - 1 );
}

void SetParams(std::string pallet_name)
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::BoolParameter save_change_param;
    dynamic_reconfigure::StrParameter type_param, doubles_string, bools_string, ints_string;
    dynamic_reconfigure::Config conf;

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

    B["m_istDetectorEnabled"] = true;
    B["m_disableOverlappingThreeLegGroups"] = true;
    B["extractEndCaps"] = true;

    bools_string.name = "AA_dynamic_bools_string";
    bools_string.value = type_map_to_string<bool>(B);
    conf.strs.push_back(bools_string);


    /**
     * Double Parameters
     */
    std::map<std::string,double> D;

    D["m_legFrontFacePtsMaxClusterDimension"] = 5.0;
    D["m_innerLegSpacingNominal"] = 5.0;
    D["m_innerLegSpacingTolerance"] = 5.0;
    D["m_outerLegSpacingNominal"] = 5.0;
    D["m_outerLegSpacingTolerance"] = 5.0;
    D["m_maxInterLegYawDiff"] = 5.0;
    D["m_palletDepth"] = 5.0;
    D["frontFaceClusteringTolerance"] = 5.0;
    D["legFrontFaceToPalletFaceXOffset"] = 5.0;

    doubles_string.name = "AA_dynamic_doubles_string";
    doubles_string.value = type_map_to_string<double>(D);
    conf.strs.push_back(doubles_string);

    /**
     * Integer Parameters
     */
    std::map<std::string,int> I;

    I["m_legFrontFacePtsMinClusterSize"] = 5;
    I["m_legFrontFacePtsMaxClusterSize"] = 5;

    ints_string.name = "AA_dynamic_ints_string";
    ints_string.value = type_map_to_string<int>(I);
    conf.strs.push_back(ints_string);

    /**
     * add parameters as server request and push updates to the PalletLibraryNode server
     */
    srv_req.config = conf;
    ros::service::call("PalletLibraryNodeMain/set_parameters", srv_req, srv_resp);      //safety_carriage_lidar_feature_detector  <-- new root for dynamic reconfigure server
}

int main(int argc, char **argv)
{
    /**
     * TODO: I think cin is getting in the way of clean exits of the code. ^C in the terminal is slow
     */
    ros::init(argc, argv, "remote_add");
    // ros::NodeHandle nh("~");

    // std::string type;
    // nh.getParam("type", type);

    SetParams("stringer40_front_face");

    return 0;
}