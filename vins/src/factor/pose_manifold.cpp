/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "pose_manifold.h"

int PoseManifold::AmbientSize() const {
    return 7;
}

int PoseManifold::TangentSize() const {
    return 6;
}

bool PoseManifold::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}
bool PoseManifold::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

bool PoseManifold::PlusJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jaco(jacobian);

    jaco.topRows<6>().setIdentity();
    jaco.bottomRows<1>().setZero();

    return true;
}

bool PoseManifold::Minus(const double *y, const double *x, double *y_minus_x) const {
    Eigen::Map<const Eigen::Vector3d> p_y(y);
    Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);

    Eigen::Map<const Eigen::Vector3d> p_x(x);
    Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);

    Eigen::Map<Eigen::Vector3d> p_y_minus_x(y_minus_x);
    Eigen::Map<Eigen::Vector3d> q_y_minus_x(y_minus_x + 3);

    p_y_minus_x = p_y - p_x;

    const Eigen::AngleAxisd axisd((q_x.inverse() * q_y).normalized());
    q_y_minus_x = axisd.angle() * axisd.axis();

    return true;
}

bool PoseManifold::MinusJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jaco(jacobian);

    jaco.rightCols<6>().setIdentity();
    jaco.leftCols<1>().setZero();

    return true;
}
