import os
import subprocess


model_names = ["airplane", "bed", "chair", "desk", "guitar", "mantel", "monitor", "piano", "sofa", "stairs"]

original_dir = "../ModelNet40_subset/"
test_dir = "../modelnet40_partial_overlap_test/"

# Run test later
for model in model_names:
    file_list = os.listdir(original_dir + model + "/test/")

    for model_file in file_list:
        object = model_file.split('.')[0]

        run_dir = test_dir + object + "/"

        # Run FLS with known scale
        subprocess.run(["fls_benchmark_algorithm", run_dir, "fls", "5", "0"])  # please set the last parameter as 1 to enable scale estimation for FLS
        subprocess.run(["eval_err", run_dir+"fls_T.txt", run_dir+"gt_T.txt"])
        