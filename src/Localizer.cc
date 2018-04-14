#include "Localizer.h"

#include <iostream>
#include <fstream>

#include "Map.h"
#include "KeyFrameDatabase.h"
#include "ORBmatcher.h"

namespace  ORB_SLAM2 {

    Localizer::Localizer(const std::string& mapFile,
                         const std::string& configFile,
                         const std::string& vocFile) {
        mpMap = new Map();
        mpORBVocabulary = new ORBVocabulary();
        mpORBVocabulary->loadFromBinaryFile(vocFile);

        mpKeyFrameDatabase = new KeyFrameDatabase(*mpORBVocabulary);


        LoadMap(mapFile);
    }

    Eigen::Isometry3d Localizer::LocateFrame(const cv::Mat & frame) {

        Eigen::Isometry3d T_wc;
        return T_wc;
    }


    void Localizer::LoadMap(const std::string &filename)
    {
        {
            std::ifstream is(filename);


            boost::archive::binary_iarchive ia(is, boost::archive::no_header);
            //ia >> mpKeyFrameDatabase;
            ia >> mpMap;

        }

        std::cout << std::endl << filename <<" : Map Loaded!" << std::endl;


    }

    void Localizer::SaveMap(const std::string &filename)
    {
        std::ofstream os(filename);
        {
            ::boost::archive::binary_oarchive oa(os, ::boost::archive::no_header);
            //oa << mpKeyFrameDatabase;
            oa << mpMap;
        }
        std::cout << std::endl << "Map saved to " << filename << std::endl;

    }
}