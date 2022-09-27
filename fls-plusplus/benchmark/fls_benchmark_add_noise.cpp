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

    if(argc < 5) {
        std::cerr << "Missing parameters!" << std::endl;
        exit(-1);
    } else {
        run_path = argv[1];
        cloud_file = argv[2];
        noise_type = argv[3];
        noise_level = std::atof(argv[4]);
    }

    // Read original point cloud data for benchmarking (PLY file only)
    fls::PointCloud::Ptr raw_cloud(new fls::PointCloud);
    std::stringstream cloud_file_ss;
    cloud_file_ss << run_path << cloud_file << ".ply";
    if(pcl::io::loadPLYFile<fls::Point>(cloud_file_ss.str(), *raw_cloud) == -1) {
        std::cerr << "Failed to read in original point cloud file!" << std::endl;
        exit(-1);
    }
    std::cout << "Read in raw point cloud from ["
              << cloud_file << ".ply] with " << raw_cloud->size() << " points." << std::endl;

    // Add noise to the destination cloud
    if(noise_type == "normal") {
        addNormalNoise(raw_cloud, noise_level);
    } else if(noise_type == "uniform") {
        addUniformNoise(raw_cloud, noise_level);
    } else {
        std::cerr << "Unsupported noise type: [" << noise_type << "]\n";
        exit(-1);
    }
    std::cout << "Add [" << noise_type << "] with level [" << noise_level << "] to destination cloud." << std::endl;

    std::stringstream dst_file_ss;
    dst_file_ss << run_path << cloud_file << "_ns.ply";
    pcl::io::savePLYFile(dst_file_ss.str(), *raw_cloud);
    std::cout << "Save noisy cloud to file: " << dst_file_ss.str() << std::endl;
    std::cout << std::endl;

    return 0;
}
