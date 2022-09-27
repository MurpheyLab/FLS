//
// Created by msun on 12/29/21.
//

#ifndef FLS_FLS_TYPES_H
#define FLS_FLS_TYPES_H

#include "fls/fls_common.h"

namespace fls
{
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    const int max_num_threads = 12;
    // const int max_num_threads = int((std::thread::hardware_concurrency() / 2) / 10) * 10;
}

#endif //FLS_FLS_TYPES_H
