//
// Created by msun on 12/29/21.
//

#ifndef FLS_PLUSPLUS_FLS_UTILS_H
#define FLS_PLUSPLUS_FLS_UTILS_H

#include "fls/fls_common.h"
#include "fls/fls_types.h"

namespace fls
{
    /*
     * Basic point cloud processing
     */
    void visualizeTwoPointClouds(const PointCloud::Ptr src_cloud,
                                 const PointCloud::Ptr dst_cloud,
                                 double p_size=3, double scale=1);

    void visualizeThreePointClouds(const PointCloud::Ptr src_cloud,
                                   const PointCloud::Ptr dst_cloud,
                                   const PointCloud::Ptr est_dst_cloud,
                                   double p1_size=3, double p2_size=3, double p3_size=3,
                                   double scale=1);

    void writePointCloudToTxt(const PointCloud::Ptr cloud,
                              const std::string &filename);

    void getPointCloudBoundary(const PointCloud::Ptr cloud,
                               double *bounds);

    void getPointCloudBoundaryMax(const PointCloud::Ptr cloud,
                                  double *bounds);

    /*
     * Advanced point cloud processing
     */
    float getAngularError(Eigen::Matrix3f R_exp, Eigen::Matrix3f R_est);

    /*
     * FLS utilities
     */
    double hk_unit(double k, double x, double x_l); // TODO: DOUBLE CHECK!!!
    double hk3(double k1, double k2, double k3,
               double x1, double x2, double x3,
               double x1_l, double x2_l, double x3_l);
    double lamk_unit(double k1, double k2, double k3);
}

#endif //FLS_PLUSPLUS_FLS_UTILS_H
