#include "icp/icp.hpp"
#include <unordered_map>
#include <cmath>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>


Eigen::Vector2d errorij(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Matrix3d &T){
    Eigen::Vector3d p_homogeneous(p.x(), p.y(), 1);
    Eigen::Vector3d transformed_p_homo = (T* p_homogeneous);
    double w = transformed_p_homo.z();
    Eigen::Vector2d transformed_p;
    if (w != 0.0){
        transformed_p.x() = transformed_p_homo.x()/w;
        transformed_p.y() = transformed_p_homo.y()/w;
    }else{
        transformed_p =Eigen::Vector2d::Zero();
    }
    return transformed_p - q;
}


Eigen::Matrix<double, 2, 3> Jacobian(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Matrix3d &T){
    Eigen::Vector3d p_homogeneous(p.x(), p.y(), 1);
    Eigen::Vector3d transformed_p_homo = (T* p_homogeneous);
    double w = transformed_p_homo.z();
    Eigen::Vector2d transformed_p;
    if (w != 0.0){
        transformed_p.x() = transformed_p_homo.x()/w;
        transformed_p.y() = transformed_p_homo.y()/w;
    }else{
        transformed_p =Eigen::Vector2d::Zero();
    }

    double x = transformed_p.x();
    double y = transformed_p.y();

    Eigen::Matrix<double, 2, 3> J(2,3);
    J << 1, 0, -y ,
            0,1, x;
    return J;
}

void linear_system(const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &correspondences,
                    const Eigen::Matrix3d &T,
                    Eigen::Matrix<double, 3, 3> &H,
                    Eigen::Vector3d &b){
    H.setZero();
    b.setZero();

for(const auto &corr :correspondences){
    const  Eigen::Vector2d &p =corr.first;
    const Eigen::Vector2d &q =corr.second;
    Eigen::Vector2d e = errorij(p,q, T);
    Eigen::MatrixXd J = Jacobian(p,q,T);
    H += J.transpose()*J;
    b += J.transpose()*e;
}
 }


Eigen::Vector3d delta_x(const Eigen::Matrix<double,3, 3> &H, const Eigen::Vector3d &b){
    Eigen::LDLT<Eigen::Matrix<double,3,3>> ldlt(H);
    if(ldlt.info()== Eigen::NumericalIssue){
      std::cerr << "H is not positive definite or is singular." << std::endl;
        return Eigen::Vector3d::Zero();   
    }
    return -ldlt.solve(b);
}


Eigen::Matrix3d apply_transformation(const Eigen::Vector3d &delta_x, const Eigen::Matrix3d &T){
    double cos_theta = std::cos(delta_x.z());
    double sin_theta = std::sin(delta_x.z());
    Eigen::Matrix3d v2t;
    v2t << cos_theta, -sin_theta, delta_x.x(),
            sin_theta, cos_theta, delta_x.y(),
            0, 0, 1;
    return v2t*T;

}








