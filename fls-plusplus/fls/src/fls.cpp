//
// Created by msun on 12/29/21.
//

#include "fls/fls.h"

namespace fls
{
    void fls::preprocessPointClouds() {
        // Pre-process the point clouds (align the geometrical center)
        src_cx = 0;
        src_cy = 0;
        src_cz = 0;
        double gt_s = 1.0;
        for (int i = 0; i < src_cloud->size(); i++) {
            src_cloud->points[i].x *= gt_s;
            src_cloud->points[i].y *= gt_s;
            src_cloud->points[i].z *= gt_s;

            src_cx += src_cloud->points[i].x / src_cloud->size();
            src_cy += src_cloud->points[i].y / src_cloud->size();
            src_cz += src_cloud->points[i].z / src_cloud->size();
        }

        dst_cx = 0;
        dst_cy = 0;
        dst_cz = 0;
        for (int i = 0; i < dst_cloud->size(); i++) {
            dst_cx += dst_cloud->points[i].x / dst_cloud->size();
            dst_cy += dst_cloud->points[i].y / dst_cloud->size();
            dst_cz += dst_cloud->points[i].z / dst_cloud->size();
        }

        std::cout << src_cx << " " << src_cy << " " << src_cz << " "
                  << dst_cx << " " << dst_cy << " " << dst_cz << std::endl;

        for (int i = 0; i < src_cloud->size(); i++) {
            src_cloud->points[i].x -= src_cx;
            src_cloud->points[i].y -= src_cy;
            src_cloud->points[i].z -= src_cz;
        }
        for (int i = 0; i < dst_cloud->size(); i++) {
            dst_cloud->points[i].x -= dst_cx;
            dst_cloud->points[i].y -= dst_cy;
            dst_cloud->points[i].z -= dst_cz;
        }

        // Compute the FLS boundaries
        PointSet src_set(src_cloud);
        PointSet dst_set(dst_cloud);
        double src_bnds[6], dst_bnds[6];
        // getPointCloudBoundaryMax(src_cloud, src_bnds);
        // getPointCloudBoundaryMax(dst_cloud, dst_bnds);
        getPointCloudBoundary(src_cloud, src_bnds);
        getPointCloudBoundary(dst_cloud, dst_bnds);
        for(int i=0; i<6; i+=2) { bnds[i] = std::min(src_bnds[i], dst_bnds[i]); }
        for(int i=1; i<6; i+=2) { bnds[i] = std::max(src_bnds[i], dst_bnds[i]); }
        // double bnd_len = bnds[1] - bnds[0];
        // for(int i=0; i<6; i+=2) { bnds[i] -= 2 * bnd_len; }
        // for(int i=1; i<6; i+=2) { bnds[i] += 2 * bnd_len; }
        std::vector<double> bnd_lens;
        for(int i=0; i<6; i+=2) {
            double bnd_len = bnds[i+1] - bnds[i];
            bnd_lens.push_back(bnd_len);
        }
        for(int i=0; i<3; i++) {
            bnds[i*2+0] -= 0.2 * bnd_lens[i];
            bnds[i*2+1] += 0.2 * bnd_lens[i];
        }

        // Naive boundary construction
        if(dst_cx <= 0) {
            bnds[0] = -1.0;
            bnds[1] = 1.0;
        } 
        if(dst_cx > 0) {
            bnds[0] = -1.0;
            bnds[1] = 1.0;
        }
        if(dst_cy <= 0) {
            bnds[2] = -1.0;
            bnds[3] = 1.0;
        } 
        if(dst_cy > 0) {
            bnds[2] = -1.0;
            bnds[3] = 1.0;
        }
        if(dst_cz <= 0) {
            bnds[4] = -1.0;
            bnds[5] = 1.0;
        } 
        if(dst_cz > 0) {
            bnds[4] = -1.0;
            bnds[5] = 1.0;
        }
    }

    void fls::registration(bool symmetry_check, bool report) {
        // Output FLS boundaries
        if(report) {
            std::cout << "Search spcae bounds: \n"
                      << "x: [" << bnds[0] << " , " << bnds[1] << "]" << std::endl
                      << "y: [" << bnds[2] << " , " << bnds[3] << "]" << std::endl
                      << "z: [" << bnds[4] << " , " << bnds[5] << "]" << std::endl;
        }

        // Compute destination set FLS coefficients
        PointSet src_set(src_cloud);
        PointSet dst_set(dst_cloud);

        dst_set.config_bnds(bnds);
        dst_set.config_ks(numk1, numk2, numk3);
        dst_set.compute_coefficients();
        std::vector<double> dst_cks = dst_set.get_coefficients();
        std::vector<std::vector<double>> ks = dst_set.get_ks();
        std::vector<double> hks = dst_set.get_hks();
        std::vector<double> lamks = dst_set.get_lamks();

        // for(double ck : dst_cks) {
        //     std::cout << ck << ", ";
        // }

        // Start Ceres optimization
        Sophus::SE3d opt_pose;

        std::cout << "symmetry check: " << symmetry_check << ", report: " << report << std::endl;
        if(symmetry_check) {// Four attempts
        } else {
            Sophus::SE3d pose;
            ceres::Problem problem;
            problem.AddParameterBlock(pose.data(), Sophus::SE3d::num_parameters,
                                      new Sophus::test::LocalParameterizationSE3);

            for (int i = 0; i < ks.size(); i++) {
                std::vector<double> k = ks[i];
                double hk = hks[i];
                double dst_ck = dst_cks[i];
                double lamk = lamks[i];

                ceres::CostFunction *cost_function = FLSResidualSE3::Create(
                        bnds[0], bnds[2], bnds[4], k[0], k[1], k[2],
                        hk, lamk, dst_ck,
                        src_set.get_set_eigen(), src_set.get_set_eigen().size()
                );
                problem.AddResidualBlock(cost_function, nullptr, pose.data());
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            options.minimizer_progress_to_stdout = true;
            options.num_threads = num_threads;
            options.gradient_tolerance = 1e-10;
            options.max_num_iterations = 25;
            options.function_tolerance = 1e-12;
            if (!report) {
                options.logging_type = ceres::SILENT;
            }

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            if (report) {
                std::cout << summary.BriefReport() << std::endl;
            }
            
            if(flip_src_dst_flag == true) {
                opt_pose = pose.inverse();
            } else {
                opt_pose = pose;
            }
        }

        Eigen::Matrix4f mat_AAp;
        Eigen::Matrix4f mat_BpB;

        if(flip_src_dst_flag == true) {
            mat_AAp << 1.0, 0.0, 0.0, -dst_cx,
                0.0, 1.0, 0.0, -dst_cy,
                0.0, 0.0, 1.0, -dst_cz,
                0.0, 0.0, 0.0, 1.0;
            mat_BpB << 1.0, 0.0, 0.0, src_cx,
                    0.0, 1.0, 0.0, src_cy,
                    0.0, 0.0, 1.0, src_cz,
                    0.0, 0.0, 0.0, 1.0;
        } else {
            mat_AAp << 1.0, 0.0, 0.0, -src_cx,
                0.0, 1.0, 0.0, -src_cy,
                0.0, 0.0, 1.0, -src_cz,
                0.0, 0.0, 0.0, 1.0;
            mat_BpB << 1.0, 0.0, 0.0, dst_cx,
                    0.0, 1.0, 0.0, dst_cy,
                    0.0, 0.0, 1.0, dst_cz,
                    0.0, 0.0, 0.0, 1.0;
        }

        T = opt_pose.cast<float>().matrix();
        T = mat_BpB * T * mat_AAp;

        // T = opt_pose.cast<float>().matrix();
    }

    void fls::preprocessForScaleEstimation() {
        std::cout << "Start preprocessing!" << std::endl;
        src_pt_dists.clear();
        dst_pt_dists.clear();
        max_d = 0.0;

        // Compute point-wise distances from the source cloud
        for(int i=0; i<src_cloud->size(); i++) {
            Eigen::Vector3d p1(src_cloud->points[i].x,
                               src_cloud->points[i].y,
                               src_cloud->points[i].z);
            for(int j=i+1; j<src_cloud->size(); j++) {
                Eigen::Vector3d p2(src_cloud->points[j].x,
                                   src_cloud->points[j].y,
                                   src_cloud->points[j].z);
                double dist = (p1 - p2).norm();
                src_pt_dists.push_back(dist);

                if(dist > max_d) { max_d = dist; }
            }
        }

        // Compute point-wise distances from the source cloud
        for(int i=0; i<dst_cloud->size(); i++) {
            Eigen::Vector3d p1(dst_cloud->points[i].x,
                               dst_cloud->points[i].y,
                               dst_cloud->points[i].z);
            for(int j=i+1; j<dst_cloud->size(); j++) {
                Eigen::Vector3d p2(dst_cloud->points[j].x,
                                   dst_cloud->points[j].y,
                                   dst_cloud->points[j].z);
                double dist = (p1 - p2).norm();
                dst_pt_dists.push_back(dist);

                if(dist > max_d) { max_d = dist; }
            }
        }
    }

    void fls::estimateScale(const int numk,
                            const bool report) {
        std::cout << max_d << " " << src_pt_dists.size() << " " << dst_pt_dists.size() << std::endl;

        assert(max_d > 0);
        assert(!src_pt_dists.empty() && !dst_pt_dists.empty());

        std::vector<double> ks;
        std::vector<double> hks;
        std::vector<double> lamks;
        for(int i=0; i<numk; i++) {
            double k = static_cast<double>(i) * M_PI / (4 * max_d);
            ks.push_back(k);

            // double hk_l = hk_unit(k, 0, 0);
            // double hk_h = hk_unit(k, 4 * max_d, 0);
            // double hk = hk_h - hk_l;
            // if(hk == 0) {hk = 1.0;}
            double hk = 2 * max_d;
            hks.push_back(hk);

            double lamk = 1 / pow(1.0 + k*k, 2);
            lamks.push_back(lamk);
        };

        std::vector<double> dst_cks;
        for(int i=0; i<ks.size(); i++) {
            double ck = 0.0;
            double k = ks[i];
            double hk = hks[i];
            for(double dst_dist : dst_pt_dists) {
                ck += cos(dst_dist * k) / hk;
            }
            ck /= static_cast<double>(dst_pt_dists.size());
            dst_cks.push_back(ck);
        }

        double est_s[1] = {1.0};
        ceres::Problem problem;
        for(int i=0; i<ks.size(); i++) {
            double k = ks[i];
            double hk = hks[i];
            double dst_ck = dst_cks[i];
            double lamk = lamks[i];

            ceres::CostFunction *cost_function = FLSResidualScale::Create(0.0, k, hk, lamk,
                                                                          dst_ck, src_pt_dists, src_pt_dists.size());
            problem.AddResidualBlock(cost_function, nullptr, est_s);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.num_threads = num_threads;
        if(report) {
            options.minimizer_progress_to_stdout = true;
        } else {
            options.minimizer_progress_to_stdout = false;
            options.logging_type = ceres::SILENT;
        }

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if(report) {
            std::cout << summary.FullReport() << std::endl;
        }

        scale = est_s[0];
    }

    void fls::applyScaleEstimate() {
        for(int i=0; i<src_cloud->size(); i++) {
            src_cloud->points[i].x *= scale;
            src_cloud->points[i].y *= scale;
            src_cloud->points[i].z *= scale;
        }
    }

    void fls::flip_src_dst(bool flag) {
        flip_src_dst_flag = flag;
    }

    PointSet::PointSet() {
        set.clear();
    }

    PointSet::PointSet(const std::vector<std::vector<double>> input_set) {
        set.clear();
        for(std::vector<double> elmt : input_set) {
            set.push_back(elmt);
        }
    }

    PointSet::PointSet(const PointCloud::Ptr input_cloud) {
        set.clear();
        for(Point p : input_cloud->points) {
            std::vector<double> elmt;
            elmt.push_back(p.x);
            elmt.push_back(p.y);
            elmt.push_back(p.z);
            set.push_back(elmt);
        }
    }

    void PointSet::config_ks(int numK1, int numK2, int numK3) {
        // Make sure the boundaries are specified
        assert(x1_l!=x1_h && x2_l!=x2_h && x3_l!=x3_h);

        ks.clear();
        hks.clear();
        lamks.clear();

        // ks.resize(numK1 * numK2 * numK3);
        // hks.resize(numK1 * numK2 * numK3);
        // lamks.resize(numK1 * numK2 * numK3);

        for(int K1=0; K1<numK1; K1++) {
            for(int K2=0; K2<numK2; K2++) {
                for(int K3=0; K3<numK3; K3++) {
                    // int idx = K1*(numK2 * numK3) + K2*(numK3) + numK3;
                    
                    double k1 = static_cast<double>(K1) * M_PI / (x1_h - x1_l);
                    double k2 = static_cast<double>(K2) * M_PI / (x2_h - x2_l);
                    double k3 = static_cast<double>(K3) * M_PI / (x3_h - x3_l);

                    std::vector<double> k{k1, k2, k3};
                    ks.push_back(k);
                    // ks[idx] = k;

                    double hk = hk3(k1, k2, k3, x1_l, x2_l, x3_l, x1_h, x2_h, x3_h);
                    if(hk == 0) {hk = 1.0;}
                    hks.push_back(hk);
                    // hks[idx] = hk;

                    double lamk = lamk_unit(k1, k2, k3);
                    lamks.push_back(lamk);
                    // lamks[idx] = lamk;
                }
            }
        }
    }

    std::vector<std::vector<double>> PointSet::get_ks() {
        return ks;
    }

    std::vector<double> PointSet::get_hks() {
        return hks;
    }

    void PointSet::config_bnds(const double *bnds) {
        x1_l = bnds[0];
        x1_h = bnds[1];
        x2_l = bnds[2];
        x2_h = bnds[3];
        x3_l = bnds[4];
        x3_h = bnds[5];
    }

    void PointSet::compute_coefficients() {
        // Make sure the point set is not empty
        assert(!set.empty());
        // Make sure the boundaries are specified
        assert(x1_l!=x1_h && x2_l!=x2_h && x3_l!=x3_h);
        // Make sure the coefficients and hk are computed
        assert(!ks.empty() && !hks.empty() && !lamks.empty());

        // Clear ck set
        cks.clear();
        cks.resize(ks.size());

        // compute coefficients
        // #pragma omp parallel for
        for(int i=0; i<ks.size(); i++) {
            std::vector<double> k = ks[i];
            double hk = hks[i];
            double ck = 0.0;

            // #pragma omp parallel for reduction(+:ck)
            for(std::vector<double> p : set) {
                double fk1_p = cos((p[0]-x1_l) * k[0]);
                double fk2_p = cos((p[1]-x2_l) * k[1]);
                double fk3_p = cos((p[2]-x3_l) * k[2]);

                ck += fk1_p * fk2_p * fk3_p / hk;
            }

            ck /= static_cast<double>(set.size());
            // cks.push_back(ck);
            cks[i] = ck;
        }
    }

    std::vector<double> PointSet::get_coefficients() { return cks; }

    std::vector<std::vector<double>> PointSet::get_set() { return set; }

    void PointSet::get_set_boundaries_max(double *bounds) {
        double x_min =  9999.0;
        double x_max = -9999.0;

        double y_min =  9999.0;
        double y_max = -9999.0;

        double z_min =  9999.0;
        double z_max = -9999.0;

        for(std::vector<double> p : set) {
            double norm = sqrt(
                    p[0]*p[0] + p[1]*p[1] + p[2]*p[2]
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

    std::vector<Eigen::Vector3d> PointSet::get_set_eigen() {
        std::vector<Eigen::Vector3d> _ret;
        for(std::vector<double> point : set) {
            Eigen::Vector3d eigen_point(point[0], point[1], point[2]);
            _ret.push_back(eigen_point);
        }

        return _ret;
    }

    std::vector<double> PointSet::get_lamks() {
        return lamks;
    }
}
