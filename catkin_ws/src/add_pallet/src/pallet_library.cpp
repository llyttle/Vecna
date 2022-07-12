// Standard includes
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <algorithm>

// include boost
#include <boost/any.hpp>

// Include PalletLibraryNode
#include <add_pallet/PalletLibraryNode.h>


/**
 * ==================================================================================================================================================================================
 * PalletLibraryNode Class
 * ==================================================================================================================================================================================
 */

PalletLibraryNode::PalletLibraryNode(ros::NodeHandle& nh, std::map<std::string,bool> enable)
        : m_node(nh),
          m_palletpocketdetector(std::make_shared<PalletPocketDetection>(nh)){
    dynamic_reconfigure::Server<add_pallet::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&PalletLibraryNode::dynamic_callback, this, _1, _2);
    m_server.setCallback(f);

    std::string path;
    if (!m_node.getParam("AA_dynamic_yaml_file_path_param", path))
        ROS_WARN("Required Parameter 'AA_dynamic_yaml_file_path_param' was not specified");
    else
        m_Pallet_Library = YAML::LoadFile(path.c_str());


    for (const auto &p : enable) {
        if (p.second) {
            ROS_INFO("Load Pallet Request: [%s]", p.first.c_str());
            PalletLibraryNode::load_pallet(p.first);
        }
    }
}



void PalletLibraryNode::load_pallet(std::string type)
{
/**
 * Load parameters from the dynamic_params yaml file into the parameter server.
 */
    YAML::Node Pallet_Params = m_Pallet_Library[type];

    if (!Pallet_Params) {
        ROS_WARN("Pallet [%s] is not found in YAML", type.c_str());
    } else {
        update_server(
            type,
            PalletLibraryNode::yaml2map<bool>(Pallet_Params["bools"]),
            PalletLibraryNode::yaml2map<double>(Pallet_Params["doubles"]),
            PalletLibraryNode::yaml2map<int>(Pallet_Params["ints"])
        );
    }
}



void PalletLibraryNode::dynamic_callback(add_pallet::dynamic_paramsConfig &config, uint32_t level)
{
    ROS_INFO("Dynamic Reconfigure Request: [%s]", config.AA_dynamic_type_param.c_str());

    /**
    * Write incomming reconfig to the dynamic_params.yaml library as a struct to preserve ordering
    * Otherwise, identical values under different names are not kept
    */
    update_server(
        config.AA_dynamic_type_param,
        PalletLibraryNode::string2map<bool>(config.AA_dynamic_bools_string),
        PalletLibraryNode::string2map<double>(config.AA_dynamic_doubles_string),
        PalletLibraryNode::string2map<int>(config.AA_dynamic_ints_string)
    );

    if (config.AA_dynamic_type_param==PalletLibraryNode::m_palletpocketdetector->m_stringer40ThreeLegFrontFaceDetector->m_palletType) {
        PalletLibraryNode::m_palletpocketdetector->m_stringer40ThreeLegFrontFaceDetector->updateParams(m_node);
    }
    else if (config.AA_dynamic_type_param==PalletLibraryNode::m_palletpocketdetector->m_stringer50ThreeLegFrontFaceDetector->m_palletType) {
        PalletLibraryNode::m_palletpocketdetector->m_stringer50ThreeLegFrontFaceDetector->updateParams(m_node);
    }
    else {
        ROS_WARN("Pallet type [%s] Does not have an active detector", config.AA_dynamic_type_param.c_str());
    }
}



void PalletLibraryNode::update_server(std::string const& type, std::map<std::string,bool> const& bool_map, std::map<std::string,double> const& double_map, std::map<std::string,int> const& int_map)
{
    const std::string pallet_namespace = "persistent/" + type;

    m_node.setParam(pallet_namespace + "/bools", bool_map);
    m_node.setParam(pallet_namespace + "/doubles", double_map);
    m_node.setParam(pallet_namespace + "/ints", int_map);
}



template<typename T>
std::map<std::string,T> PalletLibraryNode::string2map(std::string const& s)
{
    std::map<std::string,T> m;

    std::string::size_type key_pos = 0;
    std::string::size_type key_end;
    std::string::size_type val_pos;
    std::string::size_type val_end;

    std::string key;
    T val;

    while((key_end = s.find(':', key_pos)) != std::string::npos)
    {
        if((val_pos = s.find_first_not_of(": ", key_end)) == std::string::npos)         //"key:val,key2:val2"
            break;

        val_end = s.find(',', val_pos);

        key = s.substr(key_pos, key_end - key_pos);
        std::istringstream iss(s.substr(val_pos, val_end - val_pos));
        iss >> val;

        m.emplace(key, val);

        key_pos = val_end;
        if(key_pos != std::string::npos)
            ++key_pos;
    }

    return m;
}



template <typename T>
std::map<std::string,T> PalletLibraryNode::yaml2map(YAML::Node n)
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


/**
 * ==================================================================================================================================================================================
 * PalletPocketDetection Class
 * ==================================================================================================================================================================================
 */

PalletPocketDetection::PalletPocketDetection(ros::NodeHandle& n)
        :
        m_node(n) {

    bool m_debugPalletDetection = false;

    m_stringer40ThreeLegFrontFaceDetector->initializeParams(m_node /* nodeHandle */,
                                true /* m_isDetectorEnabled */,
                                3 /* m_legFrontFacePtsMinClusterSize */,
                                15 /* m_legFrontFacePtsMaxClusterSize */,
                                0.04 /* m_legFrontFacePtsMaxClusterDimension */,
                                0.46 /* m_innerLegSpacingNominal */, 0.05 /* m_innerLegSpacingTolerance */,
                                0.944 /* m_outerLegSpacingNominal */, 0.05 /* m_outerLegSpacingTolerance */,
                                false /* m_disableOverlappingThreeLegGroups */,
                                DEG2RAD(5.0) /* m_maxInterLegYawDiff */,
                                1.22 /* m_palletDepth */,
                                m_debugPalletDetection /* publishClustersCloud */,
                                true /* extractEndCaps */,
                                0.02 /* frontFaceClusteringTolerance */,
                                0.0 /* legFrontFaceToPalletFaceXOffset */);
    m_stringer50ThreeLegFrontFaceDetector->initializeParams(m_node /* nodeHandle */,
                                true /* m_isDetectorEnabled */,
                                3 /* m_legFrontFacePtsMinClusterSize */,
                                15 /* m_legFrontFacePtsMaxClusterSize */,
                                0.04 /* m_legFrontFacePtsMaxClusterDimension */,
                                0.46 /* m_innerLegSpacingNominal */, 0.05 /* m_innerLegSpacingTolerance */,
                                0.944 /* m_outerLegSpacingNominal */, 0.05 /* m_outerLegSpacingTolerance */,
                                false /* m_disableOverlappingThreeLegGroups */,
                                DEG2RAD(5.0) /* m_maxInterLegYawDiff */,
                                1.22 /* m_palletDepth */,
                                m_debugPalletDetection /* publishClustersCloud */,
                                true /* extractEndCaps */,
                                0.02 /* frontFaceClusteringTolerance */,
                                0.0 /* legFrontFaceToPalletFaceXOffset */);
}



/**
 * ==================================================================================================================================================================================
 * PalletDetectionInfo Classes
 * ==================================================================================================================================================================================
 */

// Constructors
// ==================================================================================================================================================================================
PalletPocketDetection::PalletThreeLegFrontFaceDetectionInfo::PalletThreeLegFrontFaceDetectionInfo(const std::string palletType)
        :
        m_palletType(palletType),
        m_parameterPrefix("persistent/"+palletType) {
    ROS_INFO("Pallet Detector for [%s] Initialized", m_palletType.c_str());
}

PalletPocketDetection::PalletThreeLegSideFaceDetectionInfo::PalletThreeLegSideFaceDetectionInfo(const std::string palletType)
        :
        m_palletType(palletType),
        m_parameterPrefix("persistent/"+palletType) {
    ROS_INFO("Pallet Detector for [%s] Initialized", m_palletType.c_str());
}





// Destructors
// ==================================================================================================================================================================================
PalletPocketDetection::PalletThreeLegFrontFaceDetectionInfo::~PalletThreeLegFrontFaceDetectionInfo()
{
    ROS_INFO("Pallet Detector for [%s] Destroyed", m_palletType.c_str());
}

PalletPocketDetection::PalletThreeLegSideFaceDetectionInfo::~PalletThreeLegSideFaceDetectionInfo()
{
    ROS_INFO("Pallet Detector for [%s] Destroyed", m_palletType.c_str());
}





// Parameter Initialize Functions
// ==================================================================================================================================================================================
void PalletPocketDetection::PalletThreeLegFrontFaceDetectionInfo::initializeParams(ros::NodeHandle& node,
        const bool isDetectorEnabled,
        const int legFrontFacePtsMinClusterSize,
        const int legFrontFacePtsMaxClusterSize,
        const double legFrontFacePtsMaxClusterDimension,
        const double innerLegSpacingNominal,
        const double innerLegSpacingTolerance,
        const double outerLegSpacingNominal,
        const double outerLegSpacingTolerance,
        const bool disableOverlappingThreeLegGroups,
        const double maxInterLegYawDiff,
        const double palletDepth,
        const bool publishClustersCloud,
        const bool extractEndCaps,
        const double frontFaceClusteringTolerance,
        const double legFrontFaceToPalletFaceXOffset) {
    m_isDetectorEnabled = isDetectorEnabled;
    m_legFrontFacePtsMinClusterSize = legFrontFacePtsMinClusterSize;
    m_legFrontFacePtsMaxClusterSize = legFrontFacePtsMaxClusterSize;
    m_legFrontFacePtsMaxClusterDimension = legFrontFacePtsMaxClusterDimension;
    m_innerLegSpacingNominal = innerLegSpacingNominal;
    m_innerLegSpacingTolerance = innerLegSpacingTolerance;
    m_outerLegSpacingNominal = outerLegSpacingNominal;
    m_outerLegSpacingTolerance = outerLegSpacingTolerance;
    m_disableOverlappingThreeLegGroups = disableOverlappingThreeLegGroups;
    m_maxInterLegYawDiff = maxInterLegYawDiff;
    m_palletDepth = palletDepth;
    m_publishClustersCloud = publishClustersCloud;
    // if (m_publishClustersCloud) {
    //     m_clustersCloudPubTopic = "pallet_front_face_detection_" + m_palletType + "_clusters";
    //     m_clustersCloudPub = node.advertise<sensor_msgs::PointCloud2>(m_clustersCloudPubTopic, 1, true);
    // }
    m_extractEndCapClusters = extractEndCaps;
    m_frontFaceClusteringTol = frontFaceClusteringTolerance;
    m_legFrontFaceToPalletFaceXOffset = legFrontFaceToPalletFaceXOffset;
}

void PalletPocketDetection::PalletThreeLegSideFaceDetectionInfo::initializeParams(const bool isDetectorEnabled,
                                                                                  const double innerSpacingNominal,
                                                                                  const double outerSpacingNominal,
                                                                                  const double innerSpacingTolerance,
                                                                                  const double outerSpacingTolerance,
                                                                                  const double yawTolerance,
                                                                                  const double yDiffTolerance,
                                                                                  const double legFrontFaceToPalletFaceXOffset) {
  m_isDetectorEnabled = isDetectorEnabled;
  m_innerSideSpacingNominal = innerSpacingNominal;
  m_outerSideSpacingNominal = outerSpacingNominal;
  m_innerSideSpacingTol = innerSpacingTolerance;
  m_outerSideSpacingTol = outerSpacingTolerance;
  m_interSideYawTol = yawTolerance;
  m_sidesFrontPtYDiffTolerance = yDiffTolerance;
  m_legFrontFaceToPalletFaceXOffset = legFrontFaceToPalletFaceXOffset;
}




// Parameter Update Functions
// ==================================================================================================================================================================================
// template <typename T>
// void PalletPocketDetection::PalletThreeLegFrontFaceDetectionInfo::update_specific(ros::NodeHandle& m_node, T* pointer, const std::string name)
// {
//     if (std::is_same<T, bool>::value){
//         m_node.param(m_parameterPrefix + "/bools/" + name.substr(2),
//                     *pointer, *pointer);

//         std::cout << m_parameterPrefix + "/bools/" + name.substr(2) << " : " << *pointer << std::endl;
//     }
//     else if (std::is_same<T, double>::value) {
//         m_node.param(m_parameterPrefix + "/doubles/" + name.substr(2),
//                     *pointer, *pointer);
//     }
//     else if (std::is_same<T, int>::value) {
//         m_node.param(m_parameterPrefix +"/ints/" + name.substr(2),
//                     *pointer, *pointer);
//     }
//     else {
//         ROS_WARN("UNKNOWN TYPE SENT TO UPDATE_SPECIFIC");
//     }
// }

void PalletPocketDetection::PalletThreeLegFrontFaceDetectionInfo::updateParams(ros::NodeHandle& m_node)
{

    m_node.param(m_parameterPrefix + "bools/isDetectorEnabled",
                m_isDetectorEnabled,
                m_isDetectorEnabled);
    m_node.param(m_parameterPrefix + "ints/legFrontFacePtsMinClusterSize",
                m_legFrontFacePtsMinClusterSize,
                m_legFrontFacePtsMinClusterSize);
    m_node.param(m_parameterPrefix + "ints/legFrontFacePtsMaxClusterSize",
                m_legFrontFacePtsMaxClusterSize,
                m_legFrontFacePtsMaxClusterSize);
    m_node.param(m_parameterPrefix + "doubles/legFrontFacePtsMaxClusterDimension",
                m_legFrontFacePtsMaxClusterDimension,
                m_legFrontFacePtsMaxClusterDimension);
    m_node.param(m_parameterPrefix + "doubles/innerLegSpacingNominal",
                m_innerLegSpacingNominal,
                m_innerLegSpacingNominal);
    m_node.param(m_parameterPrefix + "doubles/innerLegSpacingTolerance",
                m_innerLegSpacingTolerance,
                m_innerLegSpacingTolerance);
    m_node.param(m_parameterPrefix + "doubles/outerLegSpacingNominal",
                m_outerLegSpacingNominal,
                m_outerLegSpacingNominal);
    m_node.param(m_parameterPrefix + "doubles/outerLegSpacingTolerance",
                m_outerLegSpacingTolerance,
                m_outerLegSpacingTolerance);
    m_node.param(m_parameterPrefix + "bools/disableOverlappingThreeLegGroups",
                m_disableOverlappingThreeLegGroups,
                m_disableOverlappingThreeLegGroups);
    m_node.param(m_parameterPrefix + "doubles/maxInterLegYawDiff",
                m_maxInterLegYawDiff,
                m_maxInterLegYawDiff);
    m_node.param(m_parameterPrefix + "doubles/palletDepth",
                m_palletDepth,
                m_palletDepth);
    m_node.param(m_parameterPrefix + "bools/publishClustersCloud",
                m_publishClustersCloud,
                m_publishClustersCloud);
    m_node.param(m_parameterPrefix + "doubles/extractEndCapClusters",
                m_extractEndCapClusters,
                m_extractEndCapClusters);
    m_node.param(m_parameterPrefix + "doubles/frontFaceClusteringTol",
                m_frontFaceClusteringTol,
                m_frontFaceClusteringTol);
    m_node.param(m_parameterPrefix + "doubles/legFrontFaceToPalletFaceXOffset",
                m_legFrontFaceToPalletFaceXOffset,
                m_legFrontFaceToPalletFaceXOffset);

    // update_specific<bool>(m_node, SPLIT(m_isDetectorEnabled));

}

void PalletPocketDetection::PalletThreeLegSideFaceDetectionInfo::updateParams(ros::NodeHandle& m_node)
{
    m_node.param(m_parameterPrefix + "bools/isDetectorEnabled",
                m_isDetectorEnabled,
                m_isDetectorEnabled);
    m_node.param(m_parameterPrefix + "doubles/innerSideSpacingNominal",
                m_innerSideSpacingNominal,
                m_innerSideSpacingNominal);
    m_node.param(m_parameterPrefix + "doubles/outerSideSpacingNominal",
                m_outerSideSpacingNominal,
                m_outerSideSpacingNominal);
    m_node.param(m_parameterPrefix + "doubles/innerSideSpacingTol",
                m_innerSideSpacingTol,
                m_innerSideSpacingTol);
    m_node.param(m_parameterPrefix + "doubles/outerSideSpacingTol",
                m_outerSideSpacingTol,
                m_outerSideSpacingTol);
    m_node.param(m_parameterPrefix + "doubles/interSideYawTol",
                m_interSideYawTol,
                m_interSideYawTol);
    m_node.param(m_parameterPrefix + "doubles/sidesFrontPtYDiffTolerance",
                m_sidesFrontPtYDiffTolerance,
                m_sidesFrontPtYDiffTolerance);
}





int main()
{
    return 0;
}