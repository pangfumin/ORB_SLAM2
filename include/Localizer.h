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
    class ORBextractor;


    class Localizer {
    public:
        Localizer(const std::string& mapFile,
                  const std::string& configFile,
                  const std::string& vocFile);

        bool LocateFrame(const cv::Mat & frame, const double ts, Eigen::Isometry3d& T_wc );

        // Save / Load the current map for Mono Execution
        void SaveMap(const std::string &filename);
        void LoadMap(const std::string &filename);

    private:
        void RecoverMap();
        bool Relocalization();

        //Calibration matrix
        cv::Mat mK;
        cv::Mat mDistCoef;

        //ORB
        ORBextractor* mpORBextractor;

        Map* mpMap;
        KeyFrameDatabase* mpKeyFrameDatabase;
        ORBVocabulary* mpORBVocabulary;

        Frame mCurrentFrame;

        std::map<long unsigned int,long unsigned int> mKeyframe_id_index;
        std::map<long unsigned int,long unsigned int> mMappoint_id_index;
    };
}

#endif