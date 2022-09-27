import os
import subprocess


model_names = ["airplane", "bed", "chair", "desk", "guitar", "mantel", "monitor", "piano", "sofa", "stairs"]


def model_datagen(model):
    original_dir = "../ModelNet40_subset/"
    test_dir = "../modelnet40_partial_overlap_test/"

    os.makedirs(test_dir, exist_ok=True)

    # Generate test data first
    file_list = os.listdir(original_dir + model + "/test/")
    for model_file in file_list:
        noise_level = 0.02
        object = model_file.split('.')[0]
        filename = original_dir + model + "/test/" + object + ".off"

        print('filename: ', filename)

        # make dir for the current object
        run_dir = test_dir + object + "/"
        os.makedirs(run_dir, exist_ok=True)

        # copy original modelnet point cloud to the test directory
        subprocess.run(["cp", filename, run_dir + object + ".off"])

        # convert OFF file to PLY file
        subprocess.run(["ctmconv", run_dir + object + ".off", run_dir + object + ".ply"])

        # downsample and scale point cloud
        subprocess.run(["fls_obj2ply", run_dir + object + ".ply", run_dir + object + "_cad.pcd", "3", "-no_vis_result"])  # only samples 3 views to creata partial overlap
        subprocess.run(["pcl_pcd2ply", run_dir + object + "_cad.pcd", run_dir + object + "_cad.ply"])
        subprocess.run(["fls_unit_scaler", run_dir, object + "_cad"])

        # generate ground-truth pose and scale
        subprocess.run(["fls_benchmark_pose_generation", run_dir, "0.0", "1.57", "1.0", "2.0", "1.0", "1.0"])  # modify the last two parameters [scale_lb, scale_ub] to generate data with unknown scale
        subprocess.run(["fls_benchmark_data_generation", run_dir, run_dir + object + "_cad_unit.ply"])
        
        # downsample source and destination clouds to the same size, and add noise
        subprocess.run(["fls_downsampler", run_dir, "src", "4096"])
        subprocess.run(["fls_benchmark_add_noise", run_dir, "src_res", "normal", str()])
        subprocess.run(["mv", run_dir + "src_res_ns.ply", run_dir + "src.ply"])

        subprocess.run(["fls_downsampler", run_dir, "dst", "512"])  # downsample with different densities
        subprocess.run(["fls_benchmark_add_noise", run_dir, "dst_res", "normal", str(noise_level)])
        subprocess.run(["mv", run_dir + "dst_res_ns.ply", run_dir + "dst.ply"])

        #     break
        # break

import sys

# model = sys.argv[1]
for model in model_names:
    model_datagen(model)