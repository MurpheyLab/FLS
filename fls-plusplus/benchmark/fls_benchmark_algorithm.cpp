//
// Created by msun on 1/14/22.
//

#include "fls/fls.h"

#include <string>

float getAngularError(Eigen::Matrix3f R_exp, Eigen::Matrix3f R_est) {
    return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0)));
}

////////////////////////////////////////////////////////
/*
 * Here we define a templated object to evaluate the residual.
 * Again, there will be a residual for each observation/point.
 */
struct RegistrationResidual{
    RegistrationResidual(Eigen::Vector3d src_point,
                         Eigen::Vector3d dst_point)
            : src_point(src_point), dst_point(dst_point) {}

    template<typename T>
    bool operator()(const T* const sPose,
                    T* sResiduals) const {
        using Vector3T = Eigen::Matrix<T, 3, 1>;
        Eigen::Map<Sophus::SE3<T> const> const pose(sPose);
        Eigen::Map<Vector3T> residuals(sResiduals);

        Vector3T point_src_prime = pose * src_point;
        point_src_prime = pose * src_point.cast<T>();

        residuals = point_src_prime - dst_point;
        return true;
    }

    // factory to hide the construction of the CostFunction object
    // from the client node
    static ceres::CostFunction* Create(Eigen::Vector3d src_point,
                                       Eigen::Vector3d dst_point) {
        return (new ceres::AutoDiffCostFunction<RegistrationResidual, 3, Sophus::SE3d::num_parameters>(
                new RegistrationResidual(src_point, dst_point)));
    }

private:
    Eigen::Vector3d src_point;
    Eigen::Vector3d dst_point;
};

Eigen::Matrix4f registerKnownCorrespondences(const fls::PointCloud::Ptr src_cloud,
                                             const fls::PointCloud::Ptr dst_cloud,
                                             int num_threads) {
    Sophus::SE3d pose;
    ceres::Problem problem;
    problem.AddParameterBlock(pose.data(), Sophus::SE3d::num_parameters,
                              new Sophus::test::LocalParameterizationSE3);

    for(int i=0; i<src_cloud->size(); i++) {
        Eigen::Vector3d src_point(src_cloud->points[i].x, src_cloud->points[i].y, src_cloud->points[i].z);
        Eigen::Vector3d dst_point(dst_cloud->points[i].x, dst_cloud->points[i].y, dst_cloud->points[i].z);

        ceres::CostFunction* cost_function = RegistrationResidual::Create(src_point, dst_point);
        problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), pose.data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = num_threads;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    return pose.cast<float>().matrix();
}
////////////////////////////////////////////////////////

int main(int argc, char** argv) {
    // Check angular error


    // Read in parameters from command line
    std::string run_path;
    std::string algo_name;
    float algo_param;
    int est_scale;

    if(argc < 5) {
        std::cerr << "Missing parameters!" << std::endl;
        exit(-1);
    } else {
        run_path = argv[1];
        algo_name = argv[2];
        algo_param = std::atof(argv[3]);
        est_scale = std::atoi(argv[4]);
    }

    // Read in source and destination clouds
    std::stringstream src_file_ss;
    src_file_ss << run_path << "src.ply";
    fls::PointCloud::Ptr src_cloud(new fls::PointCloud);
    pcl::io::loadPLYFile(src_file_ss.str(), *src_cloud);
    std::cout << "Read in source cloud from [" << src_file_ss.str() << "] with [" << src_cloud->size() << "] points." << std::endl;

    std::stringstream dst_file_ss;
    dst_file_ss << run_path << "dst.ply";
    fls::PointCloud::Ptr dst_cloud(new fls::PointCloud);
    pcl::io::loadPLYFile(dst_file_ss.str(), *dst_cloud);
    std::cout << "Read in destination cloud from [" << dst_file_ss.str() << "] with [" << dst_cloud->size() << "] points." << std::endl;

    // Read in the ground truth scale
    std::stringstream gt_ss;
    gt_ss << run_path << "gt_T.txt";
    std::cout << "Start reading gt_file: " << gt_ss.str() << std::endl;
    std::fstream gt_fs(gt_ss.str(), std::ios_base::in);
    float T_11, T_12, T_13, T_14,
            T_21, T_22, T_23, T_24,
            T_31, T_32, T_33, T_34,
            T_41, T_42, T_43, T_44;
    float gt_s;
    gt_fs >> T_11 >> T_12 >> T_13 >> T_14
          >> T_21 >> T_22 >> T_23 >> T_24
          >> T_31 >> T_32 >> T_33 >> T_34
          >> T_41 >> T_42 >> T_43 >> T_44
          >> gt_s;
    gt_fs.close();

    // Eigen::Matrix4f gt_T;
    // gt_T << T_11 , T_12 , T_13 , T_14 ,
    //         T_21 , T_22 , T_23 , T_24 ,
    //         T_31 , T_32 , T_33 , T_34 ,
    //         T_41 , T_42 , T_43 , T_44;
    // std::cout << "Test gt_T:\n" << gt_T << std::endl;
    std::cout << "Read in ground-truth scale for LS: " << gt_s << std::endl;
    
    float src_cx = 0;
    float src_cy = 0;
    float src_cz = 0;

    float dst_cx = 0;
    float dst_cy = 0;
    float dst_cz = 0;

    if (algo_name != "fls" && algo_name != "fls_scale") {
        // Preprocess point clouds (move both to the coordinate frame)
        src_cx = 0;
        src_cy = 0;
        src_cz = 0;
        for (int i = 0; i < src_cloud->size(); i++) {
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
    }

    // Choose an algorithm to test
    Eigen::Matrix4f opt_T;
    float opt_s;
    std::cout << "Start test algorithm [" << algo_name << "] with parameter [" << algo_param << "]." << std::endl;

    std::chrono::steady_clock::time_point begin, end;
    double elapsed_time;
    int num_correspondences;
    if(algo_name == "fls") {
        // Shuffle the src clouds first
        std::default_random_engine engine(std::random_device{}());
        std::shuffle(src_cloud->points.begin(), src_cloud->points.end(), engine);

        fls::fls fls_reg;
        if(src_cloud->size() <= dst_cloud->size() || est_scale == 1) {
            fls_reg.setSourceCloud(src_cloud);
            fls_reg.setDestinationCloud(dst_cloud);
            fls_reg.flip_src_dst(false);
            std::cout << "Keep original src and dst order." << std::endl;
        } else {
            fls_reg.setSourceCloud(dst_cloud);
            fls_reg.setDestinationCloud(src_cloud);
            fls_reg.flip_src_dst(true);
            std::cout << "Flip src and dst cloud." << std::endl;
        }
        fls_reg.setNumCoefficients(int(algo_param), int(algo_param), int(algo_param));
        fls_reg.setNumThreads(fls::max_num_threads);

        begin = std::chrono::steady_clock::now();

        if(est_scale == 1) {
            fls_reg.preprocessForScaleEstimation();
            fls_reg.estimateScale(int(algo_param), false); 
            fls_reg.applyScaleEstimate();

            opt_s = fls_reg.getScale();

            // if(src_cloud->size() > dst_cloud->size()) {
            //     opt_s = 1.0 / opt_s;
            // }
        } else {
            opt_s = 1.0;
        }

        fls_reg.preprocessPointClouds();
        fls_reg.registration(false, true);

        end = std::chrono::steady_clock::now();
        elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000000.0;

        opt_T = fls_reg.getTransformation().matrix();
    } else if(algo_name == "fls_scale") {
        // Shuffle the src clouds first
        std::default_random_engine engine(std::random_device{}());
        std::shuffle(src_cloud->points.begin(), src_cloud->points.end(), engine);

        fls::fls fls_reg;

        fls_reg.setSourceCloud(src_cloud);
        fls_reg.setDestinationCloud(dst_cloud);
        fls_reg.flip_src_dst(false);
        std::cout << "Keep original src and dst order." << std::endl;

        fls_reg.setNumCoefficients(int(algo_param), int(algo_param), int(algo_param));
        fls_reg.setNumThreads(fls::max_num_threads);

        begin = std::chrono::steady_clock::now();

        fls_reg.preprocessForScaleEstimation();
        fls_reg.estimateScale(int(algo_param), false); 
        // fls_reg.applyScaleEstimate();

        opt_s = fls_reg.getScale();
        std::cout << "FLS scale estimation: " << opt_s << std::endl;

        for(size_t i=0; i<src_cloud->size(); i++) {
            src_cloud->points[i].x *= opt_s;
            src_cloud->points[i].y *= opt_s;
            src_cloud->points[i].z *= opt_s;
        }
        fls_reg.setSourceCloud(dst_cloud);
        fls_reg.setDestinationCloud(src_cloud);
        fls_reg.flip_src_dst(true);
        std::cout << "After scale estimation, flip original src and dst order." << std::endl;

        fls_reg.preprocessPointClouds();
        fls_reg.registration(false, true);

        end = std::chrono::steady_clock::now();
        elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000000.0;

        opt_T = fls_reg.getTransformation().matrix();

    }  else if(algo_name == "ls") {
        // first scale the source cloud with ground truth
        for(int i=0; i<src_cloud->size(); i++) {
            src_cloud->points[i].x *= gt_s;
            src_cloud->points[i].y *= gt_s;
            src_cloud->points[i].z *= gt_s;
        }
        std::cout << "Scale back the source cloud with ground-truth scale: " << gt_s << std::endl;

        begin = std::chrono::steady_clock::now();
        opt_T = registerKnownCorrespondences(src_cloud, dst_cloud, fls::max_num_threads);
        end = std::chrono::steady_clock::now();
        elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000000.0;
        opt_s = gt_s;
    } else {
        std::cerr << "Unsupported algorithm!" << std::endl;
        exit(-1);
    }

    std::cout << "Test algorithm [" << algo_name << "] finished in [" << elapsed_time << "] seconds, with transformation:\n"
              << opt_T << "\n"
              << "scale: " << opt_s << std::endl;

    // Post-process optimal transformation
    Eigen::Matrix4f mat_AAp;
    mat_AAp << 1.0, 0.0, 0.0, -src_cx,
               0.0, 1.0, 0.0, -src_cy,
               0.0, 0.0, 1.0, -src_cz,
               0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4f mat_BpB;
    mat_BpB << 1.0, 0.0, 0.0, dst_cx,
               0.0, 1.0, 0.0, dst_cy,
               0.0, 0.0, 1.0, dst_cz,
               0.0, 0.0, 0.0, 1.0;

    opt_T = mat_BpB * opt_T * mat_AAp;

    // Save optimization results to file
    std::stringstream output_ss_T;
    output_ss_T << run_path << algo_name << "_T.txt";
    std::ofstream output_fs_T(output_ss_T.str());
    output_fs_T << opt_T(0, 0) << " " << opt_T(0, 1) << " " << opt_T(0, 2) << " " << opt_T(0, 3) << "\n"
                << opt_T(1, 0) << " " << opt_T(1, 1) << " " << opt_T(1, 2) << " " << opt_T(1, 3) << "\n"
                << opt_T(2, 0) << " " << opt_T(2, 1) << " " << opt_T(2, 2) << " " << opt_T(2, 3) << "\n"
                << opt_T(3, 0) << " " << opt_T(3, 1) << " " << opt_T(3, 2) << " " << opt_T(3, 3) << "\n";
    std::cout << "Save transformation result to: " << output_ss_T.str() << std::endl;
    output_fs_T.close();

    std::stringstream output_ss_scale;
    output_ss_scale << run_path << algo_name << "_scale.txt";
    std::ofstream output_fs_scale(output_ss_scale.str());
    output_fs_scale << opt_s << "\n";
    std::cout << "Save scale result to: " << output_ss_scale.str() << std::endl;
    output_fs_scale.close();

    std::stringstream output_ss_time;
    output_ss_time << run_path << algo_name << "_time.txt";
    std::ofstream output_fs_time(output_ss_time.str());
    output_fs_time << elapsed_time << "\n";
    std::cout << "Save timing result to: " << output_ss_time.str() << std::endl;
    output_fs_time.close();

    return 0;
}
