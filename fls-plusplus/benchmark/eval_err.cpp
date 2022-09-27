#include "fls/fls.h"

#include <string>
#include <fstream>

int main(int argc, char** argv) {
    std::string filename_1, filename_2;

    if(argc < 3) {
        std::cerr << "Missing parameters!" << std::endl;
        exit(-1);
    }
    filename_1 = argv[1];
    filename_2 = argv[2];

    float T_11, T_12, T_13, T_14,
          T_21, T_22, T_23, T_24,
          T_31, T_32, T_33, T_34,
          T_41, T_42, T_43, T_44;

    std::fstream file_1(filename_1, std::ios_base::in);
    file_1 >> T_11 >> T_12 >> T_13 >> T_14
           >> T_21 >> T_22 >> T_23 >> T_24
           >> T_31 >> T_32 >> T_33 >> T_34
           >> T_41 >> T_42 >> T_43 >> T_44;
    file_1.close();

    Eigen::Matrix4f T1;
    T1 << T_11, T_12, T_13, T_14,
          T_21, T_22, T_23, T_24,
          T_31, T_32, T_33, T_34,
          T_41, T_42, T_43, T_44;
    Eigen::Matrix3f R1 = T1.topLeftCorner(3, 3);
    Eigen::Vector3f t1 = T1.topRightCorner(3, 1);

    std::fstream file_2(filename_2, std::ios_base::in);
    file_2 >> T_11 >> T_12 >> T_13 >> T_14
           >> T_21 >> T_22 >> T_23 >> T_24
           >> T_31 >> T_32 >> T_33 >> T_34
           >> T_41 >> T_42 >> T_43 >> T_44;
    file_2.close();

    Eigen::Matrix4f T2;
    T2 << T_11, T_12, T_13, T_14,
          T_21, T_22, T_23, T_24,
          T_31, T_32, T_33, T_34,
          T_41, T_42, T_43, T_44;
    Eigen::Matrix3f R2 = T2.topLeftCorner(3, 3);
    Eigen::Vector3f t2 = T2.topRightCorner(3, 1);

    float rot_err;
    rot_err = std::abs(std::acos(std::min(std::max(0.5 * ((R1.transpose() * R2).trace() - 1.0), -1.0), 1.0)));
    float trans_err = (t1 - t2).norm();

    std::cout << "Rotation error: " << rot_err << "\nTranslation error: " << trans_err << std::endl;

    return 0;
}
