//
// Created by msun on 1/15/22.
//

#include "fls/fls.h"

int main(int argc, char** argv) {
    // Read in parameters
    std::string run_path, algo_name;
    float ps1, ps2, ps3, scale;

    if(argc < 7) {
        std::cerr << "Missing parameters!" << std::endl;
        exit(-1);
    } else {
        run_path = argv[1];
        algo_name = argv[2];
        ps1 = std::atof(argv[3]);
        ps2 = std::atof(argv[4]);
        ps3 = std::atof(argv[5]);
        scale = std::atof(argv[6]);
    }

    // Preprocess run_path information
    std::stringstream src_file_ss;
    src_file_ss << run_path << "src.ply";
    std::string src_file = src_file_ss.str();

    std::stringstream dst_file_ss;
    dst_file_ss << run_path << "dst.ply";
    std::string dst_file = dst_file_ss.str();

    std::stringstream gt_file_ss;
    gt_file_ss << run_path << algo_name << "_T.txt";
    std::string gt_file = gt_file_ss.str();

    std::stringstream scale_file_ss;
    scale_file_ss << run_path << algo_name << "_scale.txt";
    std::string scale_file = scale_file_ss.str();

    // Read in files
    fls::PointCloud::Ptr src_cloud(new fls::PointCloud);
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

    fls::PointCloud::Ptr dst_cloud(new fls::PointCloud);
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

    std::fstream scale_fs(scale_file, std::ios_base::in);
    float gt_s;
    scale_fs >> gt_s;
    scale_fs.close();
    std::cout << "Read in scale transformation: " << gt_s << std::endl;

    // Transform the source cloud to get the ground-truth destination cloud
    fls::PointCloud::Ptr gt_dst_cloud(new fls::PointCloud);
    for(fls::Point p : src_cloud->points) {
        Eigen::Vector3f src_p(p.x * gt_s, p.y * gt_s, p.z * gt_s); 
        Eigen::Vector3f dst_p = T * src_p;
        fls::Point gt_p(dst_p.x(), dst_p.y(), dst_p.z());
        gt_dst_cloud->points.push_back(gt_p);
    }

    // Visualize points
    fls::visualizeThreePointClouds(src_cloud, dst_cloud, gt_dst_cloud,
                                   ps1, ps2, ps3, scale);

    return 0;
}