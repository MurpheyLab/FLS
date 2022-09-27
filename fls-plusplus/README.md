# FLS-plusplus

FLS-plusplus is a C++ library for point cloud registration.

## Dependencies

FLS-plusplus depends on the following libraries:
 - Glog: Install through `sudo apt install libgoogle-glog-dev`.

 - [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page#Download): Install through `libeigen3-dev`.

 - [Ceres](http://ceres-solver.org/): Install through `sudo apt install libceres-dev`.

 - Sophus: Download source code from [https://github.com/strasdat/Sophus](https://github.com/strasdat/Sophus), follow standard CMake project build and install procedure (may need to install FMT through `sudo apt install libfmt-dev`).

 - [PCL](https://pointclouds.org/): Install through `sudo apt install libpcl-dev pcl-tools`.

 - Boost: Install through `sudo apt install libboost-all-dev`.

 - [OpenCV](https://opencv.org/): Install through `libopencv-dev`.

 - OpenMP: (Required only for multi-threading support) Install through `sudo apt install libopenmpi-dev`.

 - [OpenCTM](http://openctm.sourceforge.net/): (Required only for benchmark tools) Install through `sudo apt install openctm-tools`.

## Installation

~~~bash
cd fls-plusplus
mkdir build && cd build
cmake .. -DBUILD_BENCHMARK=ON # set the flag as ON to build benchmark tools
sudo make install
~~~

## Usage

FLS provides benchmark tools for generate test data (the test data is provided under `ModelNet40_subset`). To further simplify the usage, we provide Python scripts under the `python` directory. 

### Test on noisy point clouds
To generate test data with point clouds cover by Gaussian noise:
~~~bash
cd python
python3 noise_test_datagen.py
~~~

To evaluate FLS on the generated noisy point cloud:
~~~bash
cd python
python3 noise_test_benchmark.py
~~~

To analyze data and generate the plot (Fig 2. in paper), run the Jupyter notebook `noise_test_data_analysis.ipynb` under the directory `data_process`.

### Test on partially overlapped point clouds with varying densities
To generate test data with point clouds cover by Gaussian noise:
~~~bash
cd python
python3 partial_overlap_test_datagen.py
~~~

To evaluate FLS on the generated noisy point cloud:
~~~bash
cd python
python3 partial_overlap_test_benchmark.py
~~~

To analyze data and generate the plot (Fig 2. in paper), run the Jupyter notebook `partial_overlap_test_data_analysis.ipynb` under the directory `data_process`.

*Note that, the default scripts test FLS with known scale. To test FLS with unknown scale, please read the scripts and modify the corresponding test parameters.*

