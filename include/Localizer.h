#ifndef  LOCALIZER_H
#define  LOCALIZER_H

#include <memory>
#include <string>

#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "Frame.h"
#include "ORBVocabulary.h"


namespace  ORB_SLAM2 {
    class Map;
    class KeyFrameDatabase;


    class Localizer {
    public:
        Localizer(const std::string& mapFile,
                  const std::string& configFile,
                  const std::string& vocFile);

        Eigen::Isometry3d LocateFrame(const cv::Mat & frame);

        // Save / Load the current map for Mono Execution
        void SaveMap(const std::string &filename);
        void LoadMap(const std::string &filename);

    private:
        Map* mpMap;
        KeyFrameDatabase* mpKeyFrameDatabase;
        ORBVocabulary* mpORBVocabulary;

        Frame mCurFrame;
    };
}

#endif