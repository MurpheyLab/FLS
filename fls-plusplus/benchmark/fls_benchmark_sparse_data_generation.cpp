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
    int num_pts;
    float density_ratio;

    if(argc < 5) {
        std::cerr << "Missing parameters!" << std::endl;
        exit(-1);
    } else {
        run_path = argv[1];
        cloud_file = argv[2];
        num_pts = std::atoi(argv[3]);
        density_ratio = std::atof(argv[4]);
    }

    std::string noise_type = "uniform"; // "normal" or "uniform"
    float noise_level = 0.0; // stddev or bnd

    // Read original point cloud data for benchmarking (PLY file only)
    fls::PointCloud::Ptr src_cloud(new fls::PointCloud);
    if(pcl::io::loadPLYFile<fls::Point>(cloud_file, *src_cloud) == -1) {
        std::cerr << "Failed to read in original point cloud file!" << std::endl;
        exit(-1);
    }
    std::cout << "Read in raw point cloud from ["
              << cloud_file << "] with " << src_cloud->size() << " points." << std::endl;

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
    for(size_t i=0; i<src_cloud->size(); i++) {
        Eigen::Vector3f src_p, dst_p;

        src_p[0] = src_cloud->points[i].x;
        src_p[1] = src_cloud->points[i].y;
        src_p[2] = src_cloud->points[i].z;

        dst_p = gt_R * src_p + gt_t;
        fls::Point p(dst_p[0], dst_p[1], dst_p[2]);
        dst_cloud->points.push_back(p);
    }
    std::cout << "Transform the source cloud to destination cloud." << std::endl;

    // Downsample the source point cloud
    pcl::RandomSample<fls::Point> filter;
    filter.setSample(int(num_pts * density_ratio));
    filter.setInputCloud(src_cloud);
    filter.filter(*src_cloud);
    assert(src_cloud->size() == int(num_pts * density_ratio));
    std::cout << "Downsample source point cloud to [" << src_cloud->size() << std::endl;

    // Downsample the source point cloud
    filter.setSample(num_pts);
    filter.setInputCloud(dst_cloud);
    filter.filter(*dst_cloud);
    assert(dst_cloud->size() == num_pts);
    std::cout << "Downsample destination point cloud to [" << src_cloud->size() << std::endl;

    // Save both clouds to PLY files
    std::stringstream src_file_ss;
    src_file_ss << run_path << "src.ply";
    pcl::io::savePLYFile(src_file_ss.str(), *src_cloud);
    std::cout << "Save source cloud to file: " << src_file_ss.str() << std::endl;

    std::stringstream dst_file_ss;
    dst_file_ss << run_path << "dst.ply";
    pcl::io::savePLYFile(dst_file_ss.str(), *dst_cloud);
    std::cout << "Save destination cloud to file: " << dst_file_ss.str() << std::endl;
    std::cout << std::endl;

    return 0;
}
