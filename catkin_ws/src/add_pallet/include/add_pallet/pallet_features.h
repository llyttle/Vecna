#include <iostream>
using namespace std;

class FeatureDetectionInfo {
    private:
        string name;
    public:
        FeatureDetectionInfo(const string PalletType) {
            name = PalletType;
            cout << "=======================================" << endl;
            cout << "Entity Created" << "                        |" << endl;
        }

        ~FeatureDetectionInfo() {
            cout << "Entity Destroyed" << "                      |" << endl;
            cout << "=======================================" << endl;
        }

        void info() {
            cout << "Pallet type '" << name << "' added" << endl;
            cout << name << " dimensions: " << s_width << "x" << s_depth << endl;
        }

        void initializeParams(const float width, const float width_tolerance, const float depth, const float depth_tolerance);

        float s_width;
        float s_width_tolerance;
        float s_depth;
        float s_depth_tolerance;
};