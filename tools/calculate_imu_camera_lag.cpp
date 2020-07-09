#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "csv.h"
#include "temporal-buffer.h"
#include "../include/Time.hpp"

struct StampedQuaternion {
    uint64_t  ts_;
    Eigen::Quaterniond quat_;
};

struct StampedGyro {
    uint64_t  ts_;
    Eigen::Vector3d gyro_;
};

const double SMALL_EPS = 1e-10;

Eigen::Vector3d
logAndTheta(const Eigen::Matrix3d & m, double * theta)
{
    Eigen::Quaterniond quat(m);

    double n = Eigen::Vector3d(quat.x(), quat.y(), quat.z()).norm();
    double w = quat.w();
    double squared_w = w*w;

    double two_atan_nbyw_by_n;
    // Atan-based log thanks to
    //
    // C. Hertzberg et al.:
    // "Integrating Generic Sensor Fusion Algorithms with Sound State
    // Representation through Encapsulation of Manifolds"
    // Information Fusion, 2011

    if (n < SMALL_EPS)
    {
        // If quaternion is normalized and n=1, then w should be 1;
        // w=0 should never happen here!
        assert(fabs(w)>SMALL_EPS);

        two_atan_nbyw_by_n = 2./w - 2.*(n*n)/(w*squared_w);
    }
    else
    {
        if (fabs(w)<SMALL_EPS)
        {
            if (w>0)
            {
                two_atan_nbyw_by_n = M_PI/n;
            }
            else
            {
                two_atan_nbyw_by_n = -M_PI/n;
            }
        }
        two_atan_nbyw_by_n = 2*atan(n/w)/n;
    }

    *theta = two_atan_nbyw_by_n*n;
    return two_atan_nbyw_by_n * Eigen::Vector3d(quat.x(), quat.y(), quat.z());
}



void loadCameraQuat(const std::string list_file, ze::Buffer<double, 4>& camera_quat_buffer) {
    std::ifstream ifs(list_file);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open camera list file: " << list_file
                  << std::endl;
        return ;
    }

    std::string one_line;
    while (!ifs.eof()) {
        std::getline(ifs, one_line);
        std::stringstream stream(one_line);
        std::string timestamp_ns_str;
        double qw, qx, qy,qz;
        stream >> timestamp_ns_str >> qw >> qx >> qy >> qz;
//        std::cout << timestamp_ns_str << " " << qw << " " << qx << " " << qy << " " << qz << std::endl;
        if (timestamp_ns_str.empty())
            break;

        Eigen::Vector4d v(qx,qy,qz,qw);
        camera_quat_buffer.insert(std::stol(timestamp_ns_str), v);
    }
}

void loadImu(const std::string& list_file, std::vector<StampedGyro>& vecIMU) {

        /// Note: imu file is a comma separated value (CSV) files
    io::CSVReader<7> in(list_file);
    //        io::CSVReader<8> in(list_file);
    double ts, ax, ay, az, gx, gy, gz;
    //        while(in.read_row(timestamp_us_str, tmp, ax, ay,az, gx,gy,gz)){
    while(in.read_row(ts,  gx,gy,gz, ax, ay,az)){
//        std::cout << Time(ts) << " " << gx << " " << gy <<  " " << gz << " " << ax << " " << ay << " " << ax << std::endl;
        StampedGyro gyro;
        gyro.ts_ = Time(ts).toNSec();
        gyro.gyro_ = Eigen::Vector3d(gx,gy,gz);
        vecIMU.push_back(gyro);
    }


}

bool getQuaternionAt(ze::Buffer<double, 4>& camera_quat_buffer, uint64_t& ts, Eigen::Quaterniond* quat) {
    Eigen::Vector3d a;
    auto old_new = camera_quat_buffer.getOldestAndNewestStamp();
    if(!std::get<2>(old_new)) return false;
    if(ts < std::get<0>(old_new) ) return false;
    if (ts > std::get<1>(old_new)) return false;

    auto before_itr = camera_quat_buffer.iterator_equal_or_before(ts);
    Eigen::Quaterniond before(before_itr->second.w(), before_itr->second.x(), before_itr->second.y(),before_itr->second.z());
    uint64_t  ts_before = before_itr->first;
    if (before_itr->first == ts) {

        *quat = before;
        return true;
    } else {
        auto after_itr = camera_quat_buffer.iterator_equal_or_after(ts);
        Eigen::Quaterniond after(after_itr->second.w(), after_itr->second.x(), after_itr->second.y(),after_itr->second.z());

        uint64_t ts_after = after_itr->first;
        double alpha = (double)(ts - ts_before) / (double)(ts_after - ts_before);
        *quat = before.slerp(alpha, after);
        return true;
    }

    return false;
}

Eigen::Vector3d calculateVelocity(Eigen::Quaterniond& Q_ws0, Eigen::Quaterniond& Q_ws1, const double dt) {
    Eigen::Matrix3d m0 = Q_ws0.toRotationMatrix();
    Eigen::Matrix3d m1 = Q_ws1.toRotationMatrix();
    Eigen::Matrix3d delta = m0.inverse() * m1;
    double theta;
    Eigen::Vector3d velocity = logAndTheta(delta,&theta) /dt;
    return velocity;
}

int main (int argc, char** argv) {
    /*
     *  Postprocessing using plot_gyro.m matlab scripts
     */

    std::string camera_rotation_file = "/home/pang/orb_debug.txt";
    std::string imu_file = "/home/pang/dataset/zjucvg/A7/imu/data.txt";

    std::ofstream gyros_ofs("/home/pang/gyros.txt");
    Eigen::Matrix3d Ric;
//    Ric << 0, -1, 0, -1, 0, 0, 0, 0, -1;
    Eigen::Quaterniond Qic( 0, 0.707107, -0.707107, 0);
//    Eigen::Quaterniond Qic();
    Ric = Qic.toRotationMatrix();
    Eigen::Quaterniond Qci= Qic.inverse();
//
    ze::Buffer<double, 4> camera_quat_buffer;
    loadCameraQuat(camera_rotation_file, camera_quat_buffer);
    std::cout << "load camera quaternion: " << camera_quat_buffer.size() << std::endl;

    std::vector<StampedGyro> vecImu;
    loadImu(imu_file, vecImu);
    std::cout << "load imu : " << vecImu.size() << std::endl;

    std::vector<StampedQuaternion> corresponding_imu_quat_from_camera;
    std::vector<StampedGyro> corresponding_imu_gyro;
    for (auto gyro : vecImu) {
        Eigen::Quaterniond quat;
        if (getQuaternionAt(camera_quat_buffer, gyro.ts_, &quat)) {
//            std::cout << " find corresponding quat" << std::endl;
            corresponding_imu_quat_from_camera.push_back({gyro.ts_, quat*Qci});
            corresponding_imu_gyro.push_back(gyro);
        } else {
            std::cout << "can not find corresponding quat" << std::endl;
        }
    }

    std::vector<StampedGyro> corresponding_calculated_imu_gyro;
    for (int i = 0; i < corresponding_imu_quat_from_camera.size() -1; i++) {
        auto av = calculateVelocity(corresponding_imu_quat_from_camera[i].quat_ ,
                corresponding_imu_quat_from_camera[i+1].quat_,
                (double)(corresponding_imu_quat_from_camera[i+1].ts_ - corresponding_imu_quat_from_camera[i].ts_) / 1e9);
        corresponding_calculated_imu_gyro.push_back({corresponding_imu_quat_from_camera[i].ts_, av});


    }
    corresponding_calculated_imu_gyro.push_back(corresponding_calculated_imu_gyro.back());
    std::cout << "get  camera imu: " << corresponding_imu_quat_from_camera.size() << " " << corresponding_imu_gyro.size() << " " << corresponding_calculated_imu_gyro.size() << std::endl;

    for (int i = 0; i < corresponding_calculated_imu_gyro.size(); i ++ ) {
        auto a = corresponding_imu_gyro.at(i);
        auto b = corresponding_calculated_imu_gyro.at(i);
        gyros_ofs << a.gyro_[0] << " " << a.gyro_[1] << " " << a.gyro_[2] << " " << b.gyro_[0] << " " << b.gyro_[1] << " " << b.gyro_[2] << std::endl;
        std::cout << a.gyro_[0] << " " << a.gyro_[1] << " " << a.gyro_[2] << " " << b.gyro_[0] << " " << b.gyro_[1] << " " << b.gyro_[2] << std::endl;

    }

    gyros_ofs.close();

    return 0;
}