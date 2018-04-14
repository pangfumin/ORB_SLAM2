#include <iostream>
#include <string>

#include "Localizer.h"


using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<int64_t> &vTimeStamps);

using namespace ORB_SLAM2;


int main(int argc, char** argv) {
    std::string map = "/home/pang/software/ORB_SLAM2/MH01_map.bin";
    std::string configFile = "/home/pang/software/ORB_SLAM2/Examples/Monocular/EuRoC.yaml";
    std::string vocFile = "/home/pang/software/ORB_SLAM2/Vocabulary/ORBvoc.bin";
    std::string imageFolder = "/media/pang/Plus/dataset/MH_02_easy/mav0/cam0/data/";
    std::string imageTsFile = "/home/pang/software/ORB_SLAM2/Examples/Monocular/EuRoC_TimeStamps/MH02.txt";

    Localizer localizer(map, configFile, vocFile);

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<int64_t> vTimestamps;
    LoadImages(imageFolder, imageTsFile, vstrImageFilenames, vTimestamps);

    int n = vstrImageFilenames.size();
    std::cout<<"Load image: "<< n <<std::endl;

    for (int i = 0; i < n; i++ ) {
        cv::Mat image = cv::imread(vstrImageFilenames[i],CV_LOAD_IMAGE_UNCHANGED);


        Eigen::Isometry3d T_cw;
        bool success = localizer.LocateFrame(image, 0, T_cw);
        LocateResult lr;
        std::vector<cv::KeyPoint> inlier_point2d;
        std::vector<cv::KeyPoint> inlier_reprojected;
        if (success) {
           lr  = localizer.getCurrentLocateResult();
            inlier_point2d = lr.inlier_point2Ds;
            inlier_reprojected = localizer.ProjectLandmarkToFrame();
        }

        std::cout<<"Locate success : "<< success << " " <<lr.num_inlier<<std::endl;

        cv::Mat colorImage;
        cv::cvtColor(image,colorImage,CV_GRAY2RGB);
        cv::Scalar red(0,0,255);
        cv::Scalar blue(255,0,0);
        cv::drawKeypoints(colorImage,inlier_point2d,colorImage,red);
        cv::drawKeypoints(colorImage,inlier_reprojected,colorImage,blue);
        cv::imshow("image",colorImage);
        cv::waitKey(1);
    }



    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<int64_t> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            int64_t t;
            ss >> t;
            vTimeStamps.push_back(t);

        }
    }
}
