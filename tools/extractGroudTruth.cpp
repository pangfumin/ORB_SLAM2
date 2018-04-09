#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <okvis/Time.hpp>
using namespace std;
void LoadQueryTs(const std::string& queryFile, vector<okvis::Time> &vTimeStamps ) {
    ifstream fTimes;
    fTimes.open(queryFile.c_str());
    string oneLine;
    while(!fTimes.eof()) {
        std::getline(fTimes, oneLine);
        std::stringstream stream(oneLine);

        std::string s;
        std::getline(stream, s, ' ');
        if (s.empty()) continue;

        std::string nanoseconds = s.substr(
                s.size() - 9, 9);
        std::string seconds = s.substr(
                0, s.size() - 9);
        okvis::Time t = okvis::Time(std::stoi(seconds), std::stoi(nanoseconds));
        //std::cout<<"t: "<<setprecision(16)<<t1 <<std::endl;
        vTimeStamps.push_back(t);
        //std::cout<<"t: "<< t <<std::endl;
    }
    fTimes.close();

}

void LoadExternPoseMeasurement(const string &strExtPosePath,
                               vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> &vIsoPoses,
                               vector<okvis::Time> &vTimeStamps) {
    ifstream fTimes;
    fTimes.open(strExtPosePath.c_str());
    string oneLine;
    while(!fTimes.eof())
    {
        std::getline(fTimes, oneLine);
        std::stringstream stream(oneLine);
        std::string s;
        std::getline(stream, s, ',');
        if (s.empty()) continue;
        //std::cout<< s <<std::endl;
        std::string nanoseconds = s.substr(
                s.size() - 9, 9);
        std::string seconds = s.substr(
                0, s.size() - 9);
        okvis::Time t = okvis::Time(std::stoi(seconds), std::stoi(nanoseconds));
        //std::cout<<"t: "<<t <<std::endl;

        vTimeStamps.push_back(t);

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
    fTimes.close();
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout <<"example: \n extract keyframe_query.txt groundtruth.txt"<<std::endl;
        return 1;
    }

    std::string keyframe_query = std::string(argv[1]);
    std::string groundtruth = std::string(argv[2]);
    std::cout <<"keyframe_query: "<<  keyframe_query<<std::endl;
    std::cout <<"groundtruth: "<<  groundtruth<<std::endl;

    std::string output_result = "../keyframe_groundtruth.txt";

    std::ofstream ofs;
    ofs.open(output_result);

    std::vector<okvis::Time> gt_ts, query_ts;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> pose;

    LoadExternPoseMeasurement(groundtruth,  pose, gt_ts);
    LoadQueryTs(keyframe_query, query_ts);

    int gt_cnt = 0;

    for(int i = 0; i < query_ts.size() ; i++ ) {
        double diff = 0;
        while(gt_cnt < gt_ts.size()) {
            diff = query_ts.at(i).toSec() -  gt_ts.at(gt_cnt).toSec();
            if (diff < 0.005) {
                Eigen::Isometry3d T = pose.at(gt_cnt);
                Eigen::Matrix3d R = T.matrix().topLeftCorner(3,3);
                Eigen::Quaterniond q(R);
                Eigen::Vector3d t = T.matrix().topRightCorner(3,1);
                ofs << gt_ts.at(gt_cnt).toNSec()
                    << " "<< t.x() << " "<< t.y() << " "<<t.z() << " "<< q.w() << " "<< q.x() << " "<< q.y()<<" "<<q.z() <<std::endl;
                break;
            }

            gt_cnt++;
        }
        //std::cout<< query_ts.at(i) << " "<< gt_ts.at(gt_cnt) << " " << diff <<std::endl;
    }



    ofs.close();

    return 0;
}