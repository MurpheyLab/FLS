import os
import subprocess


model_names = ["airplane", "bed", "chair", "desk", "guitar", "mantel", "monitor", "piano", "sofa", "stairs"]


def model_datagen(model):
    original_dir = "../ModelNet40_subset/"
    test_dir = "../modelnet40_noise_test/"

    os.makedirs(test_dir, exist_ok=True)

    # Generate test data first
    file_list = os.listdir(original_dir + model + "/test/")
    for model_file in file_list:
        for noise_level in [0.0, 0.01, 0.02, 0.03, 0.04, 0.05]:
            object = model_file.split('.')[0]
            filename = original_dir + model + "/test/" + object + ".off"

            print('filename: ', filename)

            # make dir for the current object
            run_dir = test_dir + object + "_" + str(noise_level) + "/"
            os.makedirs(run_dir, exist_ok=True)

            # copy original modelnet point cloud to the test directory
            subprocess.run(["cp", filename, run_dir + object + ".off"])

            # convert OFF file to PLY file
            subprocess.run(["ctmconv", run_dir + object + ".off", run_dir + object + ".ply"])

            # downsample and scale point cloud
            subprocess.run(["fls_obj2ply", run_dir + object + ".ply", run_dir + object + "_cad.pcd", "-1", "-no_vis_result"])
            subprocess.run(["pcl_pcd2ply", run_dir + object + "_cad.pcd", run_dir + object + "_cad.ply"])
            subprocess.run(["fls_unit_scaler", run_dir, object + "_cad"])

            # generate ground-truth pose and scale
            subprocess.run(["fls_benchmark_pose_generation", run_dir, "0.0", "1.57", "1.0", "2.0", "1.0", "1.0"]) # modify the last two parameters [scale_lb, scale_ub] to generate data with unknown scale
            subprocess.run(["fls_benchmark_data_generation", run_dir, run_dir + object + "_cad_unit.ply"])
            
            # downsample source and destination clouds to the same size, and add noise
            subprocess.run(["fls_downsampler", run_dir, "src", "1024"])
            subprocess.run(["fls_benchmark_add_noise", run_dir, "src_res", "normal", str()])
            subprocess.run(["mv", run_dir + "src_res_ns.ply", run_dir + "src.ply"])

            subprocess.run(["fls_downsampler", run_dir, "dst", "1024"])
            subprocess.run(["fls_benchmark_add_noise", run_dir, "dst_res", "normal", str(noise_level)])
            subprocess.run(["mv", run_dir + "dst_res_ns.ply", run_dir + "dst.ply"])

        #     break
        # break

import sys

# model = sys.argv[1]
for model in model_names:
    model_datagen(model)