#include <iostream>
#include <pointers.h>
#include <string.h>
#include <memory>
using namespace std;

int main() {
    string name;
    int w, d;

    cout << "Name pallet: ";
    cin >> name;
    cout << "Enter pallet width and depth:" << endl;
    cin >> w;
    cin >> d;

    std::shared_ptr<FeatureDetectionInfo> palletpointer = std::make_shared<FeatureDetectionInfo>(name);

    palletpointer->initializeParams(w, 0.1, d, 0.1);

    palletpointer->info();

    return 0;
}

void FeatureDetectionInfo::FeatureDetectionInfo::initializeParams(const float width, const float width_tolerance, const float depth, const float depth_tolerance) {
        s_width = width;
        s_width_tolerance = width_tolerance;
        s_depth = depth;
        s_depth_tolerance = depth_tolerance;
}