//
// Created by msun on 1/15/22.
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <random>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int main(int argc, char** argv) {
    // Read in parameters
    std::string run_path;
    float angle_lb, angle_ub;
    float trans_lb, trans_ub;
    float scale_lb, scale_ub;
    if(argc < 8) {
        std::cerr << "Missing parameter!" << std::endl;
        exit(-1);
    } else {
        run_path = argv[1];
        angle_lb = std::atof(argv[2]);
        angle_ub = std::atof(argv[3]);
        trans_lb = std::atof(argv[4]);
        trans_ub = std::atof(argv[5]);
        scale_lb = std::atof(argv[6]);
        scale_ub = std::atof(argv[7]);
    }

    // Initialize random number generator
    std::default_random_engine engine(std::random_device{}());
    std::uniform_real_distribution sgn_dist(-1.0, 1.0);

    // Generate rotation
    std::uniform_real_distribution angle_dist(angle_lb, angle_ub);
    float ang = angle_dist(engine) * sgn(sgn_dist(engine));

    std::uniform_real_distribution<float> axis_dist(0, 1);
    float axis_x = axis_dist(engine);
    float axis_y = axis_dist(engine);
    // if(axis_y < 0.0) { axis_y *= -1.0; }
    float axis_z = axis_dist(engine);
    Eigen::Vector3f ang_axis(axis_x, axis_y, axis_z);
    std::cout << "ang_axis: " << ang_axis.transpose() << std::endl;
    ang_axis /= ang_axis.norm();
    Sophus::SO3f rot_sophus = Sophus::SO3f::exp(ang_axis * ang);
    Eigen::Matrix3f rot = rot_sophus.matrix();

    // Generate translation
    std::uniform_real_distribution trans_dist(trans_lb, trans_ub);
    float x = trans_dist(engine) * sgn(sgn_dist(engine));
    float y = trans_dist(engine) * sgn(sgn_dist(engine));
    float z = trans_dist(engine) * sgn(sgn_dist(engine));
    Eigen::Vector3f trans(x, y, z);

    // Combine them to a transformation matrix
    Sophus::SE3f sophus_T(rot, trans);
    Eigen::Matrix4f T = sophus_T.matrix();
    std::cout << "Generate transformation matrix:\n"
              << T << std::endl;

    // Generate scale
    std::uniform_real_distribution scale_dist(scale_lb, scale_ub);
    float scale = scale_dist(engine);
    std::cout << "Generate scale: " << scale << std::endl;

    // Save the transformation matrix and scale to file
    std::stringstream out_ss;
    out_ss << run_path << "gt_T.txt";
    std::ofstream out_fs(out_ss.str());
    out_fs << T(0,0) << " " << T(0,1) << " " << T(0,2) << " " << T(0,3) << "\n"
           << T(1,0) << " " << T(1,1) << " " << T(1,2) << " " << T(1,3) << "\n"
           << T(2,0) << " " << T(2,1) << " " << T(2,2) << " " << T(2,3) << "\n"
           << T(3,0) << " " << T(3,1) << " " << T(3,2) << " " << T(3,3) << "\n"
           << scale << "\n";
    out_fs.close();
    std::cout << "Save transformation matrix to file: " << out_ss.str() << std::endl;
    std::cout << std::endl;

    std::stringstream out_scale_ss;
    out_scale_ss << run_path << "gt_scale.txt";
    std::ofstream out_scale_fs(out_scale_ss.str());
    out_scale_fs << scale << std::endl;
    out_scale_fs.close();

    return 0;
}
