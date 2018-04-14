#include <iostream>
#include <string>

#include "Localizer.h"

using namespace ORB_SLAM2;
int main(int argc, char** argv) {
    std::string map = "/home/pang/software/ORB_SLAM2/MH01_map.bin";
    std::string configFile = "/home/pang/software/ORB_SLAM2/Examples/Monocular/EuRoC.yaml";
    std::string vocFile = "/home/pang/software/ORB_SLAM2/Vocabulary/ORBvoc.bin";

    Localizer localizer(map, configFile, vocFile);
    return 0;
}