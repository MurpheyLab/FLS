//
// Created by msun on 12/29/21.
//

#ifndef HELLO_CERES_FLS_H
#define HELLO_CERES_FLS_H

#include "fls/fls_common.h"
#include "fls/fls_types.h"
#include "fls/fls_utils.h"
#include "local_parameterization_se3.h"

// #define _USE_MATH_DEFINES

namespace fls
{
    /*
     * FLS Registration
     */
    class fls {
    public:
        fls() { flip_src_dst_flag = false; };

        void setSourceCloud(const PointCloud::Ptr in_cloud) {
            src_cloud = in_cloud;
        };
        void setDestinationCloud(const PointCloud::Ptr in_cloud) {
            dst_cloud = in_cloud;
        };
        void setNumCoefficients(const int numk1_,
                                     const int numk2_,
                                     const int numk3_) {
            numk1 = numk1_;
            numk2 = numk2_;
            numk3 = numk3_;
        }
        void setNumThreads(int num_threads_) {
            num_threads = num_threads_;
        }

        void preprocessForScaleEstimation();

        void preprocessPointClouds();

        void applyScaleEstimate();

        void registration(bool symmetry_check, bool report);

        void estimateScale(const int numk=5,
                           const bool report=true);

        Eigen::Matrix4f getTransformation() {
            return T.cast<float>();
        };
        double getScale() {
            return scale;
        }

        void flip_src_dst(bool flag);

    private:
        PointCloud::Ptr src_cloud;
        PointCloud::Ptr dst_cloud;
        int numk1, numk2, numk3;
        int num_threads;
        float src_cx, src_cy, src_cz;
        float dst_cx, dst_cy, dst_cz;
        Eigen::Matrix4f T;
        Eigen::Matrix4f mat_AAp, mat_BpB;
        double bnds[6];

        std::vector<double> src_pt_dists, dst_pt_dists;
        double max_d;
        double scale;
        bool flip_src_dst_flag;
    };

    /*
     * The point set class
     */
    class PointSet {
    public:
        PointSet();
        PointSet(const std::vector<std::vector<double>> input_set);
        PointSet(const PointCloud::Ptr input_cloud);

        void config_ks(int numK1, int numK2, int numK3);
        std::vector<std::vector<double>> get_ks();
        std::vector<double> get_hks();
        std::vector<double> get_lamks();
        void config_bnds(const double *bnds);

        void compute_coefficients();
        std::vector<double> get_coefficients();

        std::vector<std::vector<double>> get_set();
        std::vector<Eigen::Vector3d> get_set_eigen();

        void get_set_boundaries_max(double *bounds);

    private:
        // points, indices and coefficients
        std::vector<std::vector<double>> set;
        std::vector<std::vector<double>> ks;
        std::vector<double> cks;
        std::vector<double> hks;
        std::vector<double> lamks;

        // search space boundaries
        double x1_l=0, x2_l=0, x3_l=0;
        double x1_h=0, x2_h=0, x3_h=0;
    };

    /*
     * FLS residual function class for Ceres (scale estimation)
     */
    struct FLSResidualScale{
        FLSResidualScale(double x1_l, double k1,
                         double hk, double lamk, double ck_dst,
                         std::vector<double> points, double num_pts)
                : x1_l(x1_l), k1(k1), hk(hk), lamk(lamk),
                  ck_dst(ck_dst), points(points), num_pts(num_pts) {}

        template<typename T>
        bool operator()(const T* const scale,
                        T *residual) const {
            T ck = static_cast<T>(0.0);
            T t_p;
            T s = scale[0];
            for(double point : points) {
                // convert point to template T
                t_p = static_cast<T>(point);

                // scale point
                t_p *= s;

                // compute coefficient
                T val_1 = static_cast<T>( cos((t_p-x1_l) * k1) );
                ck += val_1 / static_cast<T>(hk);
            }
            ck = ck / static_cast<T>(num_pts);

            residual[0] = (ck_dst - ck) * sqrt(lamk);
            return true;
        }

        // Factory to hide the construction of the CostFunction object from the client node
        static ceres::CostFunction* Create(double x1_l, double k1, double hk, double lamk,
                                           double ck_dst, std::vector<double> points,
                                           double num_pts) {
            return (new ceres::AutoDiffCostFunction<FLSResidualScale, 1, 1>(
                    new FLSResidualScale(x1_l, k1, hk, lamk, ck_dst,
                                         points, num_pts)
            ));
        }

    private:
        // lower bounds
        double x1_l;
        // basis function indices
        double k1;
        // normalization term hk
        double hk;
        // norm penalty weight term
        double lamk;
        // destination coefficient
        double ck_dst;
        // source point cloud
        std::vector<double> points;
        int num_pts;
    };

    /*
     * FLS residual function class for Ceres (Sophus SE3)
     */
    struct FLSResidualSE3{
        FLSResidualSE3(double x1_l, double x2_l, double x3_l,
                       double k1, double k2, double k3,
                       double hk, double lamk, double ck_dst,
                       std::vector<Eigen::Vector3d> points,
                       double num_pts)
                : x1_l(x1_l), x2_l(x2_l), x3_l(x3_l),
                  k1(k1), k2(k2), k3(k3), hk(hk), lamk(lamk),
                  ck_dst(ck_dst), points(points), num_pts(num_pts) {}

        double fk3(const double x1, const double x2, const double x3) {
            double val_1 = cos((x1-x1_l) * k1);
            double val_2 = cos((x2-x2_l) * k2);
            double val_3 = cos((x3-x3_l) * k3);
            return val_1 * val_2 * val_3 / hk;
        }

        template<typename T>
        bool operator()(const T* const sPose,
                        T *sResiduals) const {
            using Vector3T = Eigen::Matrix<T, 3, 1>;
            Eigen::Map<Sophus::SE3<T> const> const pose(sPose);
            Eigen::Map<Vector3T> residuals(sResiduals);

            T ck = static_cast<T>(0);
            for(Eigen::Vector3d src_point : points) {
                Vector3T point_src_prime = pose * src_point;
                point_src_prime = pose * src_point.cast<T>();

                // compute coefficient
                T val_1 = static_cast<T>( cos((point_src_prime[0]-x1_l) * k1) );
                T val_2 = static_cast<T>( cos((point_src_prime[1]-x2_l) * k2) );
                T val_3 = static_cast<T>( cos((point_src_prime[2]-x3_l) * k3) );
                ck += val_1 * val_2 * val_3 / static_cast<double>(hk);
            }
            ck = ck / static_cast<T>(num_pts);

            residuals[0] = (ck_dst - ck) * sqrt(lamk);
            return true;
        }

        // Factory to hide the construction of the CostFunction object from the client node
        static ceres::CostFunction* Create(double x1_l, double x2_l, double x3_l,
                                           double k1, double k2, double k3,
                                           double hk, double lamk, double ck_dst,
                                           std::vector<Eigen::Vector3d> points,
                                           double num_pts) {
            return (new ceres::AutoDiffCostFunction<FLSResidualSE3, 1, Sophus::SE3d::num_parameters>(
                    new FLSResidualSE3(x1_l, x2_l, x3_l,
                                              k1, k2, k3, hk, lamk, ck_dst,
                                              points, num_pts)
            ));
        }

    private:
        // lower bounds
        double x1_l, x2_l, x3_l;
        // basis function indices
        double k1, k2, k3;
        // normalization term hk
        double hk;
        // norm penalty weight term
        double lamk;
        // destination coefficient
        double ck_dst;
        // source point cloud
        std::vector<Eigen::Vector3d> points;
        int num_pts;
    };

    /*
     * FLS residual function class for Ceres
     */
    struct FLSResidual{
        FLSResidual(const double x1_l, const double x1_h,
                    const double x2_l, const double x2_h,
                    const double x3_l, const double x3_h,
                    const double k1, const double k2, const double k3,
                    const double ck_dst, const int num_pts,
                    std::vector<std::vector<double>> points)
                    : x1_l(x1_l), x2_l(x2_l), x3_l(x3_l),
                      x1_h(x1_h), x2_h(x2_h), x3_h(x3_h),
                      k1(k1), k2(k2), k3(k3),
                      ck_dst(ck_dst), num_pts(num_pts),
                      points(points)  {
            hk = hk3(k1, k2, k3, x1_l, x2_l, x3_l, x1_h, x2_h, x3_h);
            if(hk==0) {hk=1;}
            lamk = lamk_unit(k1, k2, k3);
        }

        double fk3(const double x1, const double x2, const double x3) {
            double val_1 = cos((x1-x1_l) * k1);
            double val_2 = cos((x2-x2_l) * k2);
            double val_3 = cos((x3-x3_l) * k3);
            return val_1 * val_2 * val_3 / hk;
        }

        template<typename T>
        bool operator()(const T* const pose,
                        T *residual) const {
            T ck = static_cast<T>(0.0);
            T t_p[3];
            T p[3];
            for(std::vector<double> point : points) {
                // convert point to template T
                p[0] = static_cast<T>(point[0]);
                p[1] = static_cast<T>(point[1]);
                p[2] = static_cast<T>(point[2]);

                // rotate point
                ceres::AngleAxisRotatePoint(pose, p, t_p);

                // translate point
                t_p[0] += pose[3];
                t_p[1] += pose[4];
                t_p[2] += pose[5];

                // compute coefficient
                T val_1 = static_cast<T>( cos((t_p[0]-x1_l) * k1) );
                T val_2 = static_cast<T>( cos((t_p[1]-x2_l) * k2) );
                T val_3 = static_cast<T>( cos((t_p[2]-x3_l) * k3) );
                ck += val_1 * val_2 * val_3 / static_cast<double>(hk);
            }
            ck = ck / static_cast<T>(num_pts);

            residual[0] = (ck_dst - ck) * sqrt(lamk);
            return true;
        }

        // Factory to hide the construction of the CostFunction object from the client node
        static ceres::CostFunction* Create(const double x1_l, const double x1_h,
                                           const double x2_l, const double x2_h,
                                           const double x3_l, const double x3_h,
                                           const double k1, const double k2, const double k3,
                                           const double ck_dst, const int num_pts,
                                           std::vector<std::vector<double>> points) {
            return (new ceres::AutoDiffCostFunction<FLSResidual, 1, 6>(
                    new FLSResidual(x1_l, x1_h, x2_l, x2_h, x3_l, x3_h,
                                    k1, k2, k3, ck_dst, num_pts, points)
                    ));
        }

    private:
        // search space boundaries
        double x1_l, x1_h;
        double x2_l, x2_h;
        double x3_l, x3_h;
        // basis function indices
        double k1, k2, k3;
        // normalization term hk
        double hk;
        // norm penalty weight term
        double lamk;
        // destination coefficient
        double ck_dst;
        // source point cloud
        std::vector<std::vector<double>> points;
        int num_pts;
    };

    /*
     * FLS residual function class for Ceres (translation only)
     */
    struct FLSTranslationResidual{
        FLSTranslationResidual(const double x1_l, const double x1_h,
                    const double x2_l, const double x2_h,
                    const double x3_l, const double x3_h,
                    const double k1, const double k2, const double k3,
                    const double ck_dst, const int num_pts,
                    std::vector<std::vector<double>> points)
                : x1_l(x1_l), x2_l(x2_l), x3_l(x3_l),
                  x1_h(x1_h), x2_h(x2_h), x3_h(x3_h),
                  k1(k1), k2(k2), k3(k3),
                  ck_dst(ck_dst), num_pts(num_pts),
                  points(points)  {
            hk = hk3(k1, k2, k3, x1_l, x2_l, x3_l, x1_h, x2_h, x3_h);
            if(hk==0) {hk=1;} // TODO: BETTER CONVERSION SCHEME
            lamk = lamk_unit(k1, k2, k3);
        }

        double fk3(const double x1, const double x2, const double x3) {
            double val_1 = cos((x1-x1_l) * k1);
            double val_2 = cos((x2-x2_l) * k2);
            double val_3 = cos((x3-x3_l) * k3);
            return val_1 * val_2 * val_3 / hk;
        }

        template<typename T>
        bool operator()(const T* const pose,
                        T *residual) const {
            T ck = static_cast<T>(0.0);
            T t_p[3];
            for(std::vector<double> point : points) {
                // convert point to template T
                t_p[0] = static_cast<T>(point[0]);
                t_p[1] = static_cast<T>(point[1]);
                t_p[2] = static_cast<T>(point[2]);

                // translate point
                t_p[0] += pose[0];
                t_p[1] += pose[1];
                t_p[2] += pose[2];

                // compute coefficient
                T val_1 = static_cast<T>( cos((t_p[0]-x1_l) * k1) );
                T val_2 = static_cast<T>( cos((t_p[1]-x2_l) * k2) );
                T val_3 = static_cast<T>( cos((t_p[2]-x3_l) * k3) );
                ck += val_1 * val_2 * val_3 / static_cast<double>(hk);
            }
            ck = ck / static_cast<T>(num_pts);

            residual[0] = (ck_dst - ck) * sqrt(lamk);
            return true;
        }

        // Factory to hide the construction of the CostFunction object from the client node
        static ceres::CostFunction* Create(const double x1_l, const double x1_h,
                                           const double x2_l, const double x2_h,
                                           const double x3_l, const double x3_h,
                                           const double k1, const double k2, const double k3,
                                           const double ck_dst, const int num_pts,
                                           std::vector<std::vector<double>> points) {
            return (new ceres::AutoDiffCostFunction<FLSTranslationResidual, 1, 3>(
                    new FLSTranslationResidual(x1_l, x1_h, x2_l, x2_h, x3_l, x3_h,
                                    k1, k2, k3, ck_dst, num_pts, points)
            ));
        }

    private:
        // search space boundaries
        double x1_l, x1_h;
        double x2_l, x2_h;
        double x3_l, x3_h;
        // basis function indices
        double k1, k2, k3;
        // normalization term hk
        double hk;
        // norm penalty weight term
        double lamk;
        // destination coefficient
        double ck_dst;
        // source point cloud
        std::vector<std::vector<double>> points;
        int num_pts;
    };

    /*
     * FLS residual function class for Ceres (rotation only)
     */
    struct FLSRotationResidual{
        FLSRotationResidual(const double x1_l, const double x1_h,
                               const double x2_l, const double x2_h,
                               const double x3_l, const double x3_h,
                               const double k1, const double k2, const double k3,
                               const double ck_dst, const int num_pts,
                               std::vector<std::vector<double>> points)
                : x1_l(x1_l), x2_l(x2_l), x3_l(x3_l),
                  x1_h(x1_h), x2_h(x2_h), x3_h(x3_h),
                  k1(k1), k2(k2), k3(k3),
                  ck_dst(ck_dst), num_pts(num_pts),
                  points(points)  {
            hk = hk3(k1, k2, k3, x1_l, x2_l, x3_l, x1_h, x2_h, x3_h);
            if(hk==0) {hk=1;} // TODO: BETTER CONVERSION SCHEME
            lamk = lamk_unit(k1, k2, k3);
        }

        double fk3(const double x1, const double x2, const double x3) {
            double val_1 = cos((x1-x1_l) * k1);
            double val_2 = cos((x2-x2_l) * k2);
            double val_3 = cos((x3-x3_l) * k3);
            return val_1 * val_2 * val_3 / hk;
        }

        template<typename T>
        bool operator()(const T* const pose,
                        T *residual) const {
            T ck = static_cast<T>(0.0);
            T t_p[3];
            T p[3];
            for(std::vector<double> point : points) {
                // convert point to template T
                p[0] = static_cast<T>(point[0]);
                p[1] = static_cast<T>(point[1]);
                p[2] = static_cast<T>(point[2]);

                // rotate point
                ceres::AngleAxisRotatePoint(pose, p, t_p);

                // compute coefficient
                T val_1 = static_cast<T>( cos((t_p[0]-x1_l) * k1) );
                T val_2 = static_cast<T>( cos((t_p[1]-x2_l) * k2) );
                T val_3 = static_cast<T>( cos((t_p[2]-x3_l) * k3) );
                ck += val_1 * val_2 * val_3 / static_cast<double>(hk);
            }
            ck = ck / static_cast<T>(num_pts);

            residual[0] = (ck_dst - ck) * sqrt(lamk);
            return true;
        }

        // Factory to hide the construction of the CostFunction object from the client node
        static ceres::CostFunction* Create(const double x1_l, const double x1_h,
                                           const double x2_l, const double x2_h,
                                           const double x3_l, const double x3_h,
                                           const double k1, const double k2, const double k3,
                                           const double ck_dst, const int num_pts,
                                           std::vector<std::vector<double>> points) {
            return (new ceres::AutoDiffCostFunction<FLSRotationResidual, 1, 3>(
                    new FLSRotationResidual(x1_l, x1_h, x2_l, x2_h, x3_l, x3_h,
                                               k1, k2, k3, ck_dst, num_pts, points)
            ));
        }

    private:
        // search space boundaries
        double x1_l, x1_h;
        double x2_l, x2_h;
        double x3_l, x3_h;
        // basis function indices
        double k1, k2, k3;
        // normalization term hk
        double hk;
        // norm penalty weight term
        double lamk;
        // destination coefficient
        double ck_dst;
        // source point cloud
        std::vector<std::vector<double>> points;
        int num_pts;
    };
}

#endif //HELLO_CERES_FLS_H
