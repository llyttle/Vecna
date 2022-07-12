#include <add_pallet/PalletLibraryNode.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "PalletLibraryNodeMain");
    ros::NodeHandle nh("~");

    std::map<std::string, bool> enable;
    nh.getParam("active", enable);

    PalletLibraryNode PLN(nh, enable);

    ros::spin();
    return 0;
}