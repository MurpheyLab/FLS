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
    int num_pts;
    if(argc < 4) {
        std::cerr << "Missing parameter!" << std::endl;
        exit(-1);
    } else {
        input_path = argv[1];
        file_name_no_ply = argv[2];
        num_pts = std::atoi(argv[3]);
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

    // Check
    if(num_pts > in_cloud->size()) {
        num_pts = in_cloud->size();
    }

    // Downsample the cloud
    pcl::RandomSample<fls::Point> filter;
    filter.setSample(num_pts);
    fls::PointCloud::Ptr out_cloud(new fls::PointCloud);
    filter.setInputCloud(in_cloud);
    filter.filter(*out_cloud);
    assert(out_cloud->size() == num_pts);
    std::cout << "Downsample original point cloud to [" << out_cloud->size() << "] points as the source cloud." << std::endl;

    // Save the down-sampled point cloud
    std::stringstream out_ss;
    out_ss << input_path << file_name_no_ply << "_res.ply";
    pcl::io::savePLYFile(out_ss.str(), *out_cloud);
    std::cout << "Save scaled point cloud to: " << out_ss.str() << std::endl;
    std::cout << std::endl;

    return 0;
}
