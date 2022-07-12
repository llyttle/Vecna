#pragma once

// Include ROS
#include <ros/ros.h>

// Dynamic Reconfigure include server
#include <dynamic_reconfigure/server.h>

// Include dynamic parameter configure file
#include <add_pallet/dynamic_paramsConfig.h>

// YAML library for reading
#include <yaml-cpp/yaml.h>

// Include for constexpr
#include <type_traits>

#define DEG2RAD(x) ((x)*0.017453293)

#define SPLIT(p) &p, #p

/**
* Stores relevant information for detectiPalletThreeLegFrontFaceDetectionInfong the front face of a three-leg pallet
*/
class PalletPocketDetection
{
public:

    /**
     * Constructor
     */
    PalletPocketDetection(ros::NodeHandle& n);

    class PalletThreeLegFrontFaceDetectionInfo
    {
    public:
        /**
         * Constructor
         */
        PalletThreeLegFrontFaceDetectionInfo(std::string palletType);

        /**
         * Destructor
         */
        ~PalletThreeLegFrontFaceDetectionInfo();

        /**
         * Initialize the parameters
         */
        void initializeParams(ros::NodeHandle& n, const bool isDetectorEnabled, const int legFrontFacePtsMinClusterSize,
                            const int legFrontFacePtsMaxClusterSize, const double legFrontFacePtsMaxClusterDimension,
                            const double innerLegSpacingNominal, const double innerLegSpacingTolerance,
                            const double outerLegSpacingNominal, const double outerLegSpacingTolerance,
                            const bool disableOverlappingThreeLegGroups, const double maxInterLegYawDiff, const double palletDepth,
                            const bool publishClustersCloud, const bool extractEndCaps, const double frontFaceClusteringTolerance,
                            const double legFrontFaceToPalletFaceXOffset);

        /**
         * Update parameters if callback is used
         */
        void updateParams(ros::NodeHandle& m_node);

        template <typename T>
        void update_specific(ros::NodeHandle& m_node, T* pointer, std::string name);

        /**
         * Pallet-type identifier
         */
        const std::string m_palletType;

        /**
         * Flag to enable/disable this detector
         */
        bool m_isDetectorEnabled = true;

        /**
         * Minimum cluster size for pallet leg front-face detection
         */
        int m_legFrontFacePtsMinClusterSize = 3;

        /**
         * Maximum cluster size for pallet leg front-face detection
         */
        int m_legFrontFacePtsMaxClusterSize = 15;

        /**
         * Max width/height of the pallet leg front-face cluster, in meters
         */
        double m_legFrontFacePtsMaxClusterDimension = 0.02;

        /**
         * Nominal inner spacing (pocket size) between two legs, in meters
         */
        double m_innerLegSpacingNominal = 0.46; // pallet inside-width between any two inner legs

        /**
         * Tolerance on spacing (pocket size) between two legs, in meters
         */
        double m_innerLegSpacingTolerance = 0.05;

        /**
         * Nominal outer spacing (pocket size) between two legs, in meters
         */
        double m_outerLegSpacingNominal = 0.944; // pallet inside-width between two outer legs

        /**
         * Tolerance on outer spacing (pocket size) between two legs, in meters
         */
        double m_outerLegSpacingTolerance = 0.05;

        /**
         * Allow three-leg groups to overlap (overlapping detections are removed during post)
         */
        bool m_disableOverlappingThreeLegGroups = false;

        /**
         * Max difference in yaw between each inner leg pair and the outer leg pair [rad]
         */
        double m_maxInterLegYawDiff = DEG2RAD(5.0);

        /**
         * Nominal depth of a pallet, in meters
         */
        double m_palletDepth = 1.22; // nominal pallet depth is 48in

        /**
         * RANSAC distance threshold for fitting a line to the front face points [m]
         */
        double m_frontFaceRansacDistanceThreshold = 0.50; // in meters

        /**
         * X offset between the front-face of the pallet legs and pallet/payload overhang (e.g., top/bottom boards, payload)
         * Pallet detection Tf is offset relative to the detected pallet legs with this X offset.
         */
        double m_legFrontFaceToPalletFaceXOffset = 0.0; // in meters

        /**
         * Flag to enable publishing face clusters
         */
        bool m_publishClustersCloud = false;

        /**
         * Topic to publish face clusters
         */
        std::string m_clustersCloudPubTopic = "";

        /**
         * Publisher for face clusters
         */
        ros::Publisher m_clustersCloudPub;

        /**
         * Whether to extract end-cap clusters from lines (usually only true on stringer)
         */
        bool m_extractEndCapClusters = false;

        /**
         * PCL clustering tolerance, in meters. Increase to make bigger clusters.
         */
        double m_frontFaceClusteringTol = 0.02;

        /**
         * Prefix path for dynamic parameters
         */
        const std::string m_parameterPrefix;
    };

    /**
     * Stores relevant information for detecting the front face of a three-leg pallet
     */
    class PalletThreeLegSideFaceDetectionInfo
    {
    public:
        /**
         * Constructor
         */
        PalletThreeLegSideFaceDetectionInfo(const std::string palletType);


        /**
         * Destructor
         */
        ~PalletThreeLegSideFaceDetectionInfo();


        /**
         * Initialize the parameters
         */
        void initializeParams(const bool isDetectorEnabled, const double innerSpacingNominal, const double outerSpacingNominal,
                            const double innerSpacingTolerance, const double outerSpacingTolerance, const double yawTolerance,
                            const double yDiffTolerance, const double legFrontFaceToPalletFaceXOffset);

        /**
         * Update parameters if callback is used
         */
        void updateParams(ros::NodeHandle& m_node);

        /**
         * Pallet-type identifier
         */
        const std::string m_palletType;

        /**
         * Flag to enable/disable this detector
         */
        bool m_isDetectorEnabled = true;

        /**
         * Nominal spacing between two inner sides [m]
         */
        double m_innerSideSpacingNominal = 0.46;

        /**
         * Nominal spacing between two outer sides [m]
         */
        double m_outerSideSpacingNominal = 0.944;

        /**
         * Tolerance of inner side spacing [m]
         */
        double m_innerSideSpacingTol = 0.05;

        /**
         * Tolerance of outer side spacing [m]
         */
        double m_outerSideSpacingTol = 0.05;

        /**
         * Yaw tolerance between two sides [deg]
         */
        double m_interSideYawTol = 6.0;

        /**
         * Y diff tolerance [m]
         */
        double m_sidesFrontPtYDiffTolerance = 0.6;

        /**
         * X offset between the front-face of the pallet legs and pallet/payload overhang (e.g., top/bottom boards, payload)
         * Pallet detection Tf is offset relative to the detected pallet legs with this X offset.
         */
        double m_legFrontFaceToPalletFaceXOffset = 0.0; // in meters

        /**
         * Path prefix for dynamic parameters
         */
        const std::string m_parameterPrefix;
    };

    ros::NodeHandle m_node;

    std::shared_ptr<PalletThreeLegFrontFaceDetectionInfo> m_stringer40ThreeLegFrontFaceDetector =       //Detector
            std::make_shared<PalletThreeLegFrontFaceDetectionInfo>("stringer40ThreeLegFrontFace");      // Pallet
    std::shared_ptr<PalletThreeLegFrontFaceDetectionInfo> m_stringer50ThreeLegFrontFaceDetector =
            std::make_shared<PalletThreeLegFrontFaceDetectionInfo>("stringer50ThreeLegFrontFace");
    std::shared_ptr<PalletThreeLegSideFaceDetectionInfo> m_stringer40ThreeLegSideFaceDetector =
        std::make_shared<PalletThreeLegSideFaceDetectionInfo>("stringer40ThreeLegSideFace");
};





class PalletLibraryNode
{
public:
    /**
     * Constructor
     */
    PalletLibraryNode(ros::NodeHandle& nh, std::map<std::string,bool> enable);

    /**
     * Load pallet from YAML library
     */
    void load_pallet(std::string type);

    /**
     * Callback for dynamic reconfigure.
     * Pushes configuration to parameter server and updates relevant PalletDetectionInfo classes
     */
    void dynamic_callback(add_pallet::dynamic_paramsConfig &config, uint32_t level);

    /**
     * Add parameters of type bool, double, or int to the parameter server with corresponding names. If parameter already exists, it is updated.
     */
    void update_server(std::string const& type, std::map<std::string,bool> const& bool_map, std::map<std::string,double> const& double_map, std::map<std::string,int> const& int_map);

    /**
     * Convert string dictionary (',' delimited) to a map of type T
     */
    template<typename T>
    std::map<std::string,T> string2map(std::string const& s);

    /**
     *
     */
    template <typename T>
    std::map<std::string,T> yaml2map(YAML::Node n);

    /**
    * Instance of class to detect pallet pockets in 2D laser scan
    */
    std::shared_ptr<PalletPocketDetection> m_palletpocketdetector;

    /**
     * Dynamic Reconfigure Server
     */
    dynamic_reconfigure::Server<add_pallet::dynamic_paramsConfig> m_server;

    /**
     * YAML node that holds the dynamic_params.yaml pallet library
     */
    YAML::Node m_Pallet_Library;

    /**
     * ROS node handle
     */
    ros::NodeHandle m_node;
};