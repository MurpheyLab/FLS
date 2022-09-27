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
    if(argc < 3) {
        std::cerr << "Missing parameter!" << std::endl;
        exit(-1);
    } else {
        input_path = argv[1];
        file_name_no_ply = argv[2];
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

    // Find the boundaries
    double bnds[6];
    fls::getPointCloudBoundary(in_cloud, bnds);
    double len_x = bnds[1] - bnds[0];
    double len_y = bnds[3] - bnds[2];
    double len_z = bnds[5] - bnds[4];
    double max_len = std::max(len_x, std::max(len_y, len_z));
    double scale = 1 / max_len;
    std::cout << "Found scale: " << scale << std::endl;

    // Scale the cloud
    fls::PointCloud::Ptr out_cloud(new fls::PointCloud);
    for(fls::Point p : in_cloud->points) {
        fls::Point scaled_p;
        scaled_p.x = p.x * scale;
        scaled_p.y = p.y * scale;
        scaled_p.z = p.z * scale;
        out_cloud->points.push_back(scaled_p);
    }

    // Save the scaled point cloud
    std::stringstream out_ss;
    out_ss << input_path << file_name_no_ply << "_unit.ply";
    pcl::io::savePLYFile(out_ss.str(), *out_cloud);
    std::cout << "Save scaled point cloud to: " << out_ss.str() << std::endl;
    std::cout << std::endl;

    // std::stringstream out_scale_ss;
    // out_scale_ss << input_path << "gt_scale.txt";
    // std::ofstream out_scale_fs(out_scale_ss.str());
    // out_scale_fs << scale << std::endl;
    // out_scale_fs.close();

    return 0;
}
