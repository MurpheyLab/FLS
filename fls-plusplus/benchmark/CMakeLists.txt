add_executable(fls_benchmark_data_generation fls_benchmark_data_generation.cpp)
target_link_libraries(fls_benchmark_data_generation
    fls)

add_executable(fls_benchmark_data_generation_old fls_benchmark_data_generation_old.cpp)
target_link_libraries(fls_benchmark_data_generation_old
    fls)

add_executable(fls_benchmark_partial_data_generation fls_benchmark_partial_data_generation.cpp)
target_link_libraries(fls_benchmark_partial_data_generation
    fls)

add_executable(fls_benchmark_sparse_data_generation fls_benchmark_sparse_data_generation.cpp)
target_link_libraries(fls_benchmark_sparse_data_generation
    fls)

add_executable(fls_benchmark_algorithm fls_benchmark_algorithm.cpp)
target_link_libraries(fls_benchmark_algorithm
    fls)

add_executable(fls_unit_scaler fls_unit_scaler.cpp)
target_link_libraries(fls_unit_scaler
    fls)

add_executable(fls_downsampler fls_downsampler.cpp)
target_link_libraries(fls_downsampler
    fls)

add_executable(fls_downsampler_two fls_downsampler_two.cpp)
target_link_libraries(fls_downsampler_two
    fls)

find_package(Eigen REQUIRED)
find_package(Sophus REQUIRED)
if(Sophus_FOUND)
    message("-- Found Sophus")
    get_target_property(Sophus_INCLUDE_DIR Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
    include_directories(AFTER ${Sophus_INCLUDE_DIR})
endif()
add_executable(fls_benchmark_pose_generation fls_benchmark_pose_generation.cpp)
target_link_libraries(fls_benchmark_pose_generation
    Eigen3::Eigen
    Sophus::Sophus)

add_executable(fls_benchmark_visualization fls_benchmark_visualization.cpp)
target_link_libraries(fls_benchmark_visualization fls)

add_executable(fls_benchmark_visualization_rgb fls_benchmark_visualization_rgb.cpp)
target_link_libraries(fls_benchmark_visualization_rgb fls)

add_executable(fls_benchmark_visualization_new fls_benchmark_visualization_new.cpp)
target_link_libraries(fls_benchmark_visualization_new fls)

add_executable(fls_benchmark_add_noise fls_benchmark_add_noise.cpp)
target_link_libraries(fls_benchmark_add_noise fls)

add_executable(eval_err eval_err.cpp)
target_link_libraries(eval_err fls)

add_executable(fls_pcd_center fls_pcd_center.cpp)
target_link_libraries(fls_pcd_center fls)

add_executable(fls_pcd_scaler fls_pcd_scaler.cpp)
target_link_libraries(fls_pcd_scaler fls)

add_executable(fls_rescale_gt fls_rescale_gt.cpp)
target_link_libraries(fls_rescale_gt fls)

# A special treat for PCL
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
find_package(PCL 1.10 REQUIRED COMPONENTS
    common
    io
    visualization
    filters
    features
    console
    memory
    keypoints
    registration)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(fls_obj2ply fls_obj2ply.cpp)
target_link_libraries(fls_obj2ply 
    fls ${PCL_LIBRARIES})

install(TARGETS
        fls_benchmark_data_generation
        fls_benchmark_data_generation_old
        fls_benchmark_partial_data_generation
        fls_benchmark_sparse_data_generation
        fls_benchmark_algorithm
        fls_unit_scaler
        fls_downsampler
        fls_downsampler_two
        fls_benchmark_pose_generation
        fls_benchmark_visualization
        fls_benchmark_visualization_rgb
        fls_benchmark_visualization_new
        fls_benchmark_add_noise
        eval_err
        fls_obj2ply
        fls_pcd_center
        fls_rescale_gt
        fls_pcd_scaler
        DESTINATION /usr/local/bin)
