//
// Created by msun on 1/15/22.
//

#include "fls/fls.h"

#include <string>
#include <fstream>

int main(int argc, char** argv) {
    // Read in parameters
    std::string input_path;
    std::string file_name_no_ply;
    float leaf_size;
    if(argc < 4) {
        std::cerr << "Missing parameter!" << std::endl;
        exit(-1);
    } else {
        input_path = argv[1];
        file_name_no_ply = argv[2];
        leaf_size = std::atof(argv[3]);
    }

    // Read in point cloud
    std::stringstream in_ss;
    in_ss << input_path << file_name_no_ply << ".ply";
    fls::PointCloud::Ptr in_cloud(new fls::PointCloud);
    if(pcl::io::loadPLYFile(in_ss.str(), *in_cloud) == -1) {
        std:cerr << "Failed to read point cloud from: " << in_ss.str() << std::endl;
        exit(-1);
    }
    std::cout << "Read in point cloud file [" << in_ss.str() << "] with " << in_cloud->size() << " points." << std::endl;

    // Downsample the cloud: using voxel grid
    pcl::VoxelGrid<fls::Point> voxel_filter;
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    fls::PointCloud::Ptr out_cloud_1(new fls::PointCloud);
    voxel_filter.setInputCloud(in_cloud);
    voxel_filter.filter(*out_cloud_1);
    std::cout << "Downsample original point cloud to [" << out_cloud_1->size() << "] points for the first cloud." << std::endl;

    // Downsample the cloud: using random sampling
    pcl::RandomSample<fls::Point> random_filter;
    random_filter.setSample(out_cloud_1->size());
    fls::PointCloud::Ptr out_cloud_2(new fls::PointCloud);
    voxel_filter.setInputCloud(in_cloud);
    voxel_filter.filter(*out_cloud_2);
    std::cout << "Downsample original point cloud to [" << out_cloud_2->size() << "] points for the second cloud." << std::endl;

    // Downsample the cloud


    // Save the down-sampled point clouds
    std::stringstream out_ss_1, out_ss_2;
    out_ss_1 << input_path << file_name_no_ply << "_ds1.ply";
    out_ss_2 << input_path << file_name_no_ply << "_ds2.ply";
    pcl::io::savePLYFile(out_ss_1.str(), *out_cloud_1);
    pcl::io::savePLYFile(out_ss_2.str(), *out_cloud_2);
    std::cout << "Save downsampled point cloud 1 to: " << out_ss_1.str() << std::endl;
    std::cout << "Save downsampled point cloud 2 to: " << out_ss_2.str() << std::endl;
    std::cout << std::endl;

    return 0;
}
