//
// Created by msun on 1/15/22.
//

#include "fls/fls.h"

int main(int argc, char** argv) {
    // Read in parameters
    std::string src_file;
    std::string dst_file;
    std::string gt_file;
    float ps1, ps2, ps3, scale;

    if(argc < 8) {
        std::cerr << "Missing parameters!" << std::endl;
        exit(-1);
    } else {
        src_file = argv[1];
        dst_file = argv[2];
        gt_file = argv[3];
        ps1 = std::atof(argv[4]);
        ps2 = std::atof(argv[5]);
        ps3 = std::atof(argv[6]);
        scale = std::atof(argv[7]);
    }

    // Read in files
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPLYFile(src_file, *src_cloud) == -1) {
        std::cerr << "Failed to read in source cloud from: " << src_file << std::endl;
        exit(-1);
    }
    std::cout << "Read in source cloud from [" << src_file << "] with " << src_cloud->size() << " points." << std::endl;
    // for(fls::Point p : src_cloud->points) {
    //     std::cout << p.x << " " << p.y << " " << p.z << std::endl;
    // }

    // Shuffle the src clouds first
    std::default_random_engine engine(std::random_device{}());
    std::shuffle(src_cloud->points.begin(), src_cloud->points.end(), engine);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPLYFile(dst_file, *dst_cloud) == -1) {
        std::cerr << "Failed to read in destination cloud from: " << dst_file << std::endl;
        exit(-1);
    }
    std::cout << "Read in destination cloud from [" << dst_file << "] with " << dst_cloud->size() << " points." << std::endl;

    std::fstream gt_fs(gt_file, std::ios_base::in);
    float T_11, T_12, T_13, T_14,
            T_21, T_22, T_23, T_24,
            T_31, T_32, T_33, T_34,
            T_41, T_42, T_43, T_44;
    gt_fs >> T_11 >> T_12 >> T_13 >> T_14
          >> T_21 >> T_22 >> T_23 >> T_24
          >> T_31 >> T_32 >> T_33 >> T_34
          >> T_41 >> T_42 >> T_43 >> T_44;
    gt_fs.close();

    Eigen::Matrix4f gt_T;
    gt_T << T_11, T_12, T_13, T_14,
            T_21, T_22, T_23, T_24,
            T_31, T_32, T_33, T_34,
            T_41, T_42, T_43, T_44;
    std::cout << "Read in ground-truth transformation as:\n"
              << gt_T << std::endl;
    Sophus::SE3f T(gt_T);

    // Transform the source cloud to get the ground-truth destination cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr gt_dst_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(pcl::PointXYZRGB p : src_cloud->points) {
        Eigen::Vector3f src_p(p.x, p.y, p.z);
        Eigen::Vector3f dst_p = T * src_p;
        pcl::PointXYZRGB gt_p(dst_p.x(), dst_p.y(), dst_p.z());
        gt_dst_cloud->points.push_back(gt_p);
    }

    // Visualize points
    // fls::visualizeThreePointClouds(src_cloud, dst_cloud, gt_dst_cloud,
    //                                ps1, ps2, ps3, scale);

    return 0;
}