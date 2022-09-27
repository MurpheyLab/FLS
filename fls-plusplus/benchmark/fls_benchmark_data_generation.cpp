//
// Created by msun on 1/14/22.
//

#include "fls/fls.h"

#include <string>
#include <fstream>

void addNormalNoise(fls::PointCloud::Ptr cloud, double stddev) {
    std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<float> normal_dist(0, stddev);
    for(int i=0; i<cloud->size(); i++) {
        cloud->points[i].x += normal_dist(generator);
        cloud->points[i].y += normal_dist(generator);
        cloud->points[i].z += normal_dist(generator);
    }
}

void addUniformNoise(fls::PointCloud::Ptr cloud, double bound) {
    std::default_random_engine generator(std::random_device{}());
    std::uniform_real_distribution<float> uniform_dist(-bound, bound);
    for(int i=0; i<cloud->size(); i++) {
        cloud->points[i].x += uniform_dist(generator);
        cloud->points[i].y += uniform_dist(generator);
        cloud->points[i].z += uniform_dist(generator);
    }
}

int main(int argc, char** argv) {
    // Read in parameters from command line
    std::string run_path;
    std::string cloud_file;
    std::string noise_type; // "normal" or "uniform"
    float noise_level; // stddev or bnd
    int num_pts;

    if(argc < 3) {
        std::cerr << "Missing parameters!" << std::endl;
        exit(-1);
    } else {
        run_path = argv[1];
        cloud_file = argv[2];
    }

    // Read original point cloud data for benchmarking (PLY file only)
    fls::PointCloud::Ptr raw_cloud(new fls::PointCloud);
    if(pcl::io::loadPLYFile<fls::Point>(cloud_file, *raw_cloud) == -1) {
        std::cerr << "Failed to read in original point cloud file!" << std::endl;
        exit(-1);
    }
    std::cout << "Read in raw point cloud from ["
              << cloud_file << "] with " << raw_cloud->size() << " points." << std::endl;
    
    // Check before downsample
    if(num_pts > raw_cloud->size()) {
        num_pts = raw_cloud->size();
    }

    // Move source cloud to origin
    double src_cx = 0;
    double src_cy = 0;
    double src_cz = 0;
    for(size_t i=0; i<raw_cloud->size(); i++) {
        src_cx += raw_cloud->points[i].x / raw_cloud->size();
        src_cy += raw_cloud->points[i].y / raw_cloud->size();
        src_cz += raw_cloud->points[i].z / raw_cloud->size();
    }
    for(size_t i=0; i<raw_cloud->size(); i++) {
        raw_cloud->points[i].x -= src_cx;
        raw_cloud->points[i].y -= src_cy;
        raw_cloud->points[i].z -= src_cz;
    }

    // // Downsample the original point cloud
    // pcl::RandomSample<fls::Point> filter;
    // filter.setSample(num_pts);
    // fls::PointCloud::Ptr src_cloud(new fls::PointCloud);
    // filter.setInputCloud(raw_cloud);
    // filter.filter(*src_cloud);
    // assert(src_cloud->size() == num_pts);
    // std::cout << "Downsample original point cloud to [" << src_cloud->size() << "] points as the source cloud." << std::endl;

    // Read in ground truth transformation
    std::stringstream gt_ss;
    gt_ss << run_path << "gt_T.txt";
    std::fstream gt_fs(gt_ss.str(), std::ios_base::in);
    float T_11, T_12, T_13, T_14,
           T_21, T_22, T_23, T_24,
           T_31, T_32, T_33, T_34,
           T_41, T_42, T_43, T_44;
    float scale;
    gt_fs >> T_11 >> T_12 >> T_13 >> T_14
          >> T_21 >> T_22 >> T_23 >> T_24
          >> T_31 >> T_32 >> T_33 >> T_34
          >> T_41 >> T_42 >> T_43 >> T_44
          >> scale;
    gt_fs.close();

    Eigen::Matrix4f gt_T;
    gt_T << T_11, T_12, T_13, T_14,
            T_21, T_22, T_23, T_24,
            T_31, T_32, T_33, T_34,
            T_41, T_42, T_43, T_44;
    Eigen::Matrix3f gt_R = gt_T.topLeftCorner(3, 3);
    Eigen::Vector3f gt_t = gt_T.topRightCorner(3, 1);
    std::cout << "Read in ground-truth transformation:\n"
              << gt_T << std::endl;
    std::cout << "Read in ground-truth scale: " << scale << std::endl;

    // Transform the source cloud to destination
    fls::PointCloud::Ptr dst_cloud(new fls::PointCloud);
    for(size_t i=0; i<raw_cloud->size(); i++) {
        Eigen::Vector3f src_p, dst_p;

        src_p[0] = raw_cloud->points[i].x;
        src_p[1] = raw_cloud->points[i].y;
        src_p[2] = raw_cloud->points[i].z;

        dst_p = gt_R * src_p + gt_t;
        fls::Point p(dst_p[0], dst_p[1], dst_p[2]);
        dst_cloud->points.push_back(p);
    }
    std::cout << "Transform the source cloud to destination cloud." << std::endl;

    
    // for(size_t i=0; i<dst_cloud->size(); i++) {
    //     dst_cloud->points[i].x -= src_cx;
    //     raw_cloud->points[i].y -= src_cy;
    //     dst_cloud->points[i].z -= src_cz;
    // } 

    // Add noise to the destination cloud
    // if(noise_type == "normal") {
    //     addNormalNoise(dst_cloud, noise_level);
    // } else if(noise_type == "uniform") {
    //     addUniformNoise(dst_cloud, noise_level);
    // } else {
    //     std::cerr << "Unsupported noise type: [" << noise_type << "]\n";
    //     exit(-1);
    // }
    // std::cout << "Add [" << noise_type << "] with level [" << noise_level << "] to destination cloud." << std::endl;

    // Scale the source cloud ("backwards")
    for(size_t i=0; i<raw_cloud->size(); i++) {
        raw_cloud->points[i].x /= scale;
        raw_cloud->points[i].y /= scale;
        raw_cloud->points[i].z /= scale;
    }
    std::cout << "Scale back the source cloud with scale: " << scale << std::endl;

    // Save both clouds to PLY files
    std::stringstream src_file_ss;
    src_file_ss << run_path << "src.ply";
    pcl::io::savePLYFile(src_file_ss.str(), *raw_cloud);
    std::cout << "Save source cloud to file: " << src_file_ss.str() << std::endl;

    std::stringstream dst_file_ss;
    dst_file_ss << run_path << "dst.ply";
    pcl::io::savePLYFile(dst_file_ss.str(), *dst_cloud);
    std::cout << "Save destination cloud to file: " << dst_file_ss.str() << std::endl;
    std::cout << std::endl;

    return 0;
}
