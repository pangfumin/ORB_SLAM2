/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <Converter.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);


void LoadExternPoseMeasurement(const string &strExtPosePath,
                vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> &vIsoPoses,
                               vector<double> &vTimeStamps);



int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings "
                "path_to_image_folder path_to_times_file"
                " extern_pose_measurement" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

    vector<double> vExternPoseTs;
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> vExternPose;
    LoadExternPoseMeasurement(string(argv[5]), vExternPose, vExternPoseTs);
    int externPoseCnt = 0;

    int nImages = vstrImageFilenames.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    cout<<"Extern Pose: " <<  vExternPose.size()<<std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    Eigen::Isometry3d T_IC = Eigen::Isometry3d::Identity();
    T_IC.matrix() << 0.0148655429818, -0.999880929698,   0.00414029679422, -0.0216401454975,
                     0.999557249008,   0.0149672133247,  0.025715529948,   -0.064676986768,
                     -0.0257744366974, 0.00375618835797, 0.999660727178,   0.00981073058949,
                      0.0, 0.0, 0.0, 1.0;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        while(externPoseCnt < vExternPoseTs.size() && vExternPoseTs.at(externPoseCnt) < tframe )
            externPoseCnt ++;

        Eigen::Isometry3d T_WC = vExternPose.at(externPoseCnt) * T_IC;

        // Pass the image to the SLAM system
        cv::Mat pose = ORB_SLAM2::Converter::toCvMat(T_WC.matrix());
//        std::cout<<T_WC.matrix()<<std::endl;
        SLAM.TrackMonocular(im, pose, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
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
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadExternPoseMeasurement(const string &strExtPosePath,
                               vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> &vIsoPoses,
                               vector<double> &vTimeStamps) {
    ifstream fTimes;
    fTimes.open(strExtPosePath.c_str());
    string oneLine;
    while(!fTimes.eof())
    {
        std::getline(fTimes, oneLine);
        std::stringstream stream(oneLine);
        std::string s;
        std::getline(stream, s, ',');
//        std::cout<<"t: "<<s << " ";
        if (s.empty()) break;
        double t1 = atof(s.c_str());
        t1 = t1 * 1e-9;  // [ns] to [s]
        vTimeStamps.push_back(t1);

        Eigen::Vector3d t_WS;
        Eigen::Vector4d q_WS;

        for (int j = 0; j < 3; ++j) {
            std::getline(stream, s, ',');
//            std::cout<<s <<" ";
            t_WS[j] = atof(s.c_str());
        }

        for (int j = 0; j < 4; ++j) {
            std::getline(stream, s, ',');
//            std::cout<<s <<" ";
            q_WS[j] = atof(s.c_str());
        }
//        std::cout<<std::endl;

        Eigen::Isometry3d Pose = Eigen::Isometry3d::Identity();
        Pose.matrix().topLeftCorner(3,3) = Eigen::Quaterniond(q_WS.data()).toRotationMatrix();
        Pose.matrix().topRightCorner(3,1) = t_WS;
        vIsoPoses.push_back(Pose);
    }
}
