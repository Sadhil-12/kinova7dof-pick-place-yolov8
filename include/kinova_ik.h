#ifndef KINOVA_IK_H
#define KINOVA_IK_H

#include <array>

using Matrix4 = std::array<std::array<double,4>,4>;
using Matrix3 = std::array<std::array<double,3>,3>;
using Matrix6x7 = std::array<std::array<double,7>,6>;
using Vector3 = std::array<double,3>;
using Vector7 = std::array<double,7>;
using JointVector = std::array<double, 7>;

JointVector inverse_kinematics(
    double x,
    double y,
    double z,
    const JointVector& q_init
);

JointVector inverse_kinematics_fixed_orientation(
    const Vector3& p_des,
    const Matrix3& R_des,
    const JointVector& q_init
);

#endif
