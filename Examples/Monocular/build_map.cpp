
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
//#include <vins_canvas.h>

using namespace std;

const int PLOT_BUFFER_SIZE = 1000;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<int64_t> &vTimeStamps);


std::mutex pose_mutex;
bool availible_pose = false;
Eigen::Isometry3d tracked_T_wc;

void poseCallback(const Eigen::Isometry3d& pose) {
    pose_mutex.lock();
    availible_pose = true;
    tracked_T_wc = pose;
    std::cout<<"pose: \n"<<pose.matrix() <<std::endl;
    pose_mutex.unlock();
}

bool getTrackedPose(Eigen::Isometry3d& pose ) {
    pose_mutex.lock();
    bool b = availible_pose;
    pose = tracked_T_wc;
    pose_mutex.unlock();
    return b;
}


int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./build_localization_map path_to_vocabulary path_to_dataset" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<int64_t> vTimestamps;
    std::string dataset_path = std::string(argv[2]);
    std::string strPathTimes  = dataset_path + "/fisheye.txt";

    std::cout<<"Fisheye : "<< strPathTimes<<std::endl;
    std::cout<<"voc:"<< std::string(argv[1])<<std::endl;
    LoadImages(dataset_path, strPathTimes, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }


    std::string camera_config = dataset_path + "/camera_fov.yaml";
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],camera_config,ORB_SLAM2::System::MONOCULAR,true);


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        int64_t tframe = vTimestamps[ni];

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

        std::cout<<"Processing image: "<<tframe<< " " << ni <<"/"<< nImages<<std::endl;

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);



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
            T = (vTimestamps[ni+1]-tframe)/1e9;
        else if(ni>0)
            T = (tframe-vTimestamps[ni-1])/1e9;

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
    std::string trajectory_save = dataset_path + "/map/KeyFrameTrajectory.txt";
    SLAM.SaveKeyFrameTrajectoryTUM(trajectory_save);

    std::string binary_map_save ="map.bin";
    SLAM.SaveMap(binary_map_save);

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
            std::string file = strImagePath + "/cam0/" + ss.str() + ".jpg";
            vstrImages.push_back(file);
            int64_t t;
            ss >> t;
            vTimeStamps.push_back(t);

        }
    }
}