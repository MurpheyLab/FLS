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

    // center point cloud
    double src_cx = 0;
    double src_cy = 0;
    double src_cz = 0;
    for(fls::Point p : in_cloud->points) {
        src_cx += p.x / in_cloud->size();
        src_cy += p.y / in_cloud->size();
        src_cz += p.z / in_cloud->size();
    }

    for(size_t i=0; i<in_cloud->size(); i++) {
        in_cloud->points[i].x -= src_cx;
        in_cloud->points[i].y -= src_cy;
        in_cloud->points[i].z -= src_cz;
    }

    // save new point cloud and center data
    std::stringstream out_ss;
    out_ss << input_path << file_name_no_ply << "_centered.ply";
    pcl::io::savePLYFile(out_ss.str(), *in_cloud);
    std::cout << "Save scaled point cloud to: " << out_ss.str() << std::endl;
    std::cout << std::endl;

    std::stringstream out_xyz_ss;
    out_xyz_ss << input_path << file_name_no_ply << "_cxyz.txt";
    std::ofstream out_xyz_fs(out_xyz_ss.str());
    out_xyz_fs << src_cx << " " << src_cy << " " << src_cz << std::endl;
    out_xyz_fs.close();

    return 0;
}
