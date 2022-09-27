//
// Created by msun on 12/30/21.
//

#ifndef FLS_FLS_COMMON_H
#define FLS_FLS_COMMON_H

#include <iostream>
#include <fstream>
#include <random>
#include <cmath>
#include <chrono>
#include <cassert>
#include <algorithm>
#include <thread>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/pfh.h>
#include <pcl/features/vfh.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/correspondence_estimation.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/se3.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <omp.h>

#endif //FLS_FLS_COMMON_H
