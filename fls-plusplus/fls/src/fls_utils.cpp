//
// Created by msun on 12/29/21.
//

#include "fls/fls_utils.h"

namespace fls
{
    void visualizeTwoPointClouds(const PointCloud::Ptr src_cloud,
                                 const PointCloud::Ptr dst_cloud,
                                 double p_size, double scale) {
        // Visualize both point clouds
        boost::shared_ptr<pcl::visualization::PCLVisualizer>
                viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0); // black

        pcl::visualization::PointCloudColorHandlerCustom<Point>
                src_color(src_cloud, 255, 0, 0); // red
        viewer->addPointCloud<Point>(src_cloud, src_color, "source cloud");
        viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                p_size, "source cloud");

        pcl::visualization::PointCloudColorHandlerCustom<Point>
                filtered_src_color(dst_cloud, 0, 255, 0); // green
        viewer->addPointCloud<Point>(dst_cloud, filtered_src_color,
                                     "filtered source cloud");
        viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                p_size, "filtered source cloud");

        viewer->addCoordinateSystem(scale, "global"); // I guess the first is scale
        viewer->initCameraParameters();
        viewer->setCameraPosition(0, 3, 0,    0, 0, 0,   0, 0, 1);
        viewer->setCameraFieldOfView(0.523599);
        viewer->setCameraClipDistances(0.00522511, 50);

        // Wait until visualizer window is closed
        while(!viewer->wasStopped()) {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    void visualizeThreePointClouds(const PointCloud::Ptr src_cloud,
                                   const PointCloud::Ptr dst_cloud,
                                   const PointCloud::Ptr est_dst_cloud,
                                   double p1_size, double p2_size, double p3_size,
                                   double scale) {
        // Initialize PCL visualizer
        boost::shared_ptr<pcl::visualization::PCLVisualizer>
                viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(255, 255, 255); // white

        // Set up first point cloud
        pcl::visualization::PointCloudColorHandlerCustom<Point>
                src_color(src_cloud, 1, 115, 178); // red
        viewer->addPointCloud<Point>(src_cloud, src_color, "source cloud");
        viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                p1_size, "source cloud");

        // Set up second point cloud
        pcl::visualization::PointCloudColorHandlerCustom<Point>
                dst_color(dst_cloud, 2, 158, 115); // green
        viewer->addPointCloud<Point>(dst_cloud, dst_color,
                                     "destination cloud");
        viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                p2_size, "destination cloud");

        // Set up third point cloud
        pcl::visualization::PointCloudColorHandlerCustom<Point>
                est_dst_color(est_dst_cloud, 213, 94, 0); // blue
        viewer->addPointCloud<Point>(est_dst_cloud, est_dst_color,
                                     "estimated destination cloud");
        viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                p3_size, "estimated destination cloud");

        viewer->addCoordinateSystem(scale, "global"); // I guess the first is scale
        viewer->initCameraParameters();
        viewer->setCameraPosition(0, 3, 0,    0, 0, 0,   0, 0, 1);
        viewer->setCameraFieldOfView(0.523599);
        viewer->setCameraClipDistances(0.00522511, 50);

        // Wait until visualizer window is closed
        while(!viewer->wasStopped()) {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    void writePointCloudToTxt(const PointCloud::Ptr cloud,
                              const std::string &filename) {
        std::ofstream cloud_file(filename);

        for(Point p : cloud->points) {
            cloud_file << p.x << "," << p.y << "," << p.z << "\n";
        }

        cloud_file.close();
    }

    void getPointCloudBoundary(const PointCloud::Ptr cloud,
                               double *bounds) {
        double x_min = cloud->points[0].x;
        double x_max = cloud->points[0].x;

        double y_min = cloud->points[0].y;
        double y_max = cloud->points[0].y;

        double z_min = cloud->points[0].z;
        double z_max = cloud->points[0].z;

        for(Point p : cloud->points) {
            if(p.x > x_max) { x_max = p.x; }
            if(p.x < x_min) { x_min = p.x; }

            if(p.y > y_max) { y_max = p.y; }
            if(p.y < y_min) { y_min = p.y; }

            if(p.z > z_max) { z_max = p.z; }
            if(p.z < z_min) { z_min = p.z; }
        }

        bounds[0] = x_min;
        bounds[1] = x_max;
        bounds[2] = y_min;
        bounds[3] = y_max;
        bounds[4] = z_min;
        bounds[5] = z_max;
    }

    void getPointCloudBoundaryMax(const PointCloud::Ptr cloud,
                                  double *bounds) {
        double x_min =  9999.0;
        double x_max = -9999.0;

        double y_min =  9999.0;
        double y_max = -9999.0;

        double z_min =  9999.0;
        double z_max = -9999.0;

        for(Point p : cloud->points) {
            double norm = sqrt(
                    p.x*p.x + p.y*p.y + p.z*p.z
                    );
            if(norm > x_max) {
                x_max = norm;
                y_max = norm;
                z_max = norm;
            }
            if(-norm < x_min) {
                x_min = -norm;
                y_min = -norm;
                z_min = -norm;
            }
        }

        bounds[0] = x_min;
        bounds[1] = x_max;
        bounds[2] = y_min;
        bounds[3] = y_max;
        bounds[4] = z_min;
        bounds[5] = z_max;
    }

    float getAngularError(Eigen::Matrix3f R_exp, Eigen::Matrix3f R_est) {
        return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0)));
    }

    /*
     * FLS utilities
     */
    // double hk_unit(double k, double x, double x_l) { // TODO: DOUBLE CHECK!!!
    //     if(k==0 || x==0) {
    //         return 1.0;
    //     } else {
    //         return (sin(2.0 * k * x_l - 2.0 * k * x) - 2.0 * k * x + 2.0 * k * x_l) / (4.0 * x);
    //     }
    // }

    double hk3(double k1, double k2, double k3,
               double x1_l, double x2_l, double x3_l,
               double x1_h, double x2_h, double x3_h) {
        // double hk1_l = hk_unit(k1, x1_l, x1_l);
        // double hk1_h = hk_unit(k1, x1_h, x1_l);
        //
        // double hk2_l = hk_unit(k2, x2_l, x2_l);
        // double hk2_h = hk_unit(k2, x2_h, x2_l);
        //
        // double hk3_l = hk_unit(k3, x3_l, x3_l);
        // double hk3_h = hk_unit(k3, x3_h, x3_l);
        //
        // // return std::sqrt((hk1_h * hk2_h * hk3_h) - (hk1_l * hk2_l * hk3_l));
        // return (hk1_h * hk2_h * hk3_h);// - (hk1_l * hk2_l * hk3_l);

        return std::sqrt(0.5 * (x1_h-x1_l) * 0.5 * (x2_h-x2_l) * 0.5 * (x3_h-x3_l));
    }

    double
    lamk_unit(double k1, double k2, double k3) {
        return 1 / pow(1.0 + k1*k1 + k2*k2 + k3*k3, 2);
        // return 1 / pow(exp(k1*k1 + k2*k2 + k3*k3), 2);
    }
}