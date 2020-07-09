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



void loadImuCameraQuat(const std::string list_file, std::vector<Eigen::Quaterniond>& vecImuQuat,  std::vector<Eigen::Quaterniond>& vecCameraQuat) {
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
        double imu_qw, imu_qx, imu_qy, imu_qz;
        double cam_qw, cam_qx, cam_qy, cam_qz;
        stream >> timestamp_ns_str >> imu_qw >> imu_qx >> imu_qy >> imu_qz >> cam_qw >> cam_qx >> cam_qy >> cam_qz  ;
//        std::cout << timestamp_ns_str << " " << qw << " " << qx << " " << qy << " " << qz << std::endl;
        if (timestamp_ns_str.empty())
            break;

        vecImuQuat.push_back(Eigen::Quaterniond(imu_qw, imu_qx, imu_qy, imu_qz));
        vecCameraQuat.push_back(Eigen::Quaterniond(cam_qw, cam_qx, cam_qy, cam_qz));
    }
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

    std::string camera_imu_file = "/home/pang/quat.txt";

    std::ofstream gyros_ofs("/home/pang/gyros.txt");
    Eigen::Matrix3d Ric;
    Ric << 0, -1, 0, -1, 0, 0, 0, 0, -1;
    Eigen::Quaterniond Qic(Ric);
    Eigen::Quaterniond Qci = Qic.inverse();

    std::vector<Eigen::Quaterniond> imu_vec;
    std::vector<Eigen::Quaterniond> cam_vec;
    loadImuCameraQuat(camera_imu_file, imu_vec, cam_vec);
    std::cout << "imu camera: " << imu_vec.size() << " " << cam_vec.size() << std::endl;

    std::vector<Eigen::Vector3d> gyro_from_cam;
    std::vector<Eigen::Vector3d> gyro_from_imu;
    for (int i=0; i < imu_vec.size() - 1; i++) {
        auto av = calculateVelocity(imu_vec[i], imu_vec[i+1],1);
        gyro_from_imu.push_back(av);
    }

    for (int i=0; i < imu_vec.size() - 1; i++) {
        auto q0 =cam_vec[i]*Qci;
        auto q1 =cam_vec[i+1]*Qci;
        auto av = calculateVelocity(q0, q1,1);
        gyro_from_cam.push_back(av);
    }

    for (int i = 0; i < gyro_from_cam.size(); i ++ ) {
        auto a = gyro_from_cam.at(i);
        auto b = gyro_from_imu.at(i);
        gyros_ofs << a[0] << " " << a[1] << " " << a[2] << " " << b[0] << " " << b[1] << " " << b[2] << std::endl;
        std::cout << a[0] << " " << a[1] << " " << a[2] << " " << b[0] << " " << b[1] << " " << b[2] << std::endl;

    }

    gyros_ofs.close();

    return 0;
}