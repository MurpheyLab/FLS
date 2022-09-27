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

    // Read in ground-truth scale
    std::stringstream scale_ss;
    scale_ss << input_path << "gt_scale.txt";
    std::fstream scale_fs(scale_ss.str(), std::ios_base::in);
    float scale;
    scale_fs >> scale;
    scale_fs.close();
    std::cout << "pcd_scaler scale: " << scale << std::endl;

    for(size_t i=0; i<in_cloud->size(); i++) {
        in_cloud->points[i].x /= scale;
        in_cloud->points[i].y /= scale;
        in_cloud->points[i].z /= scale;
    }

    // save new point cloud and center data
    std::stringstream out_ss;
    out_ss << input_path << file_name_no_ply << "_scaled.ply";
    pcl::io::savePLYFile(out_ss.str(), *in_cloud);
    std::cout << "Save scaled point cloud to: " << out_ss.str() << std::endl;
    std::cout << std::endl;

    return 0;
}
