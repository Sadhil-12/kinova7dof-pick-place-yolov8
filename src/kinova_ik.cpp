#include <iostream>
#include <array>
#include <cmath>
#include <vector>

constexpr double PI = 3.141592653589793;

// DH parameters
std::array<double, 7> a = {0, 0, 0, 0, 0, 0, 0};

std::array<double, 7> alpha = {
    PI / 2, PI / 2, PI / 2, PI / 2, PI / 2, PI / 2, PI
};

std::array<double, 7> d = {
    -0.2755, 0.0, -0.4100, -0.0098, -0.3111, 0.0, -0.2636 //0.2755 actual d1
};

// Aliases for matrix/vector types
using Matrix4 = std::array<std::array<double,4>,4>;
using Matrix3 = std::array<std::array<double,3>,3>;
using Matrix6x7 = std::array<std::array<double,7>,6>;
using Matrix3x7 = std::array<std::array<double,7>,3>;
using Vector3 = std::array<double,3>;
using Vector7 = std::array<double,7>;

// Declaration of helping functions
Vector3 orientation_error(const Matrix3& R1, const Matrix3& R0);
double norm3(const Vector3& v);
Matrix4 forward_kinematics_T(const Vector7& q);
Matrix6x7 compute_jacobian_pose(const Vector7& q);

// Identity matrix
Matrix4 identity4()
{
    Matrix4 I{};
    for (int i = 0; i < 4; i++)
        I[i][i] = 1.0;
    return I;
}

// Matrix multiplication
Matrix4 matmul(const Matrix4 &A, const Matrix4 &B)
{
    Matrix4 C{};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

// DH transform
Matrix4 dh_transform(double a, double alpha, double d, double theta)
{
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    Matrix4 T{{
        {ct, -st * ca,  st * sa, a * ct},
        {st,  ct * ca, -ct * sa, a * st},
        {0.0,      sa,       ca,      d},
        {0.0,     0.0,      0.0,    1.0}
    }};
    return T;
}

// Forward kinematics: returns (x, y, z)
Vector3 forward_kinematics(const Vector7 &q)
{
    Matrix4 T = identity4();

    for (int i = 0; i < 7; i++)
    {
        Matrix4 A = dh_transform(a[i], alpha[i], d[i], q[i]);
        T = matmul(T, A);
    }

    return {T[0][3], T[1][3], T[2][3]};
}


// Forward kinematics: returns full transformation matrix (orientation + position)
Matrix4 forward_kinematics_T(const Vector7& q)
{
    Matrix4 T = identity4();

    for (int i = 0; i < 7; ++i)
    {
        Matrix4 Ti = dh_transform(a[i], alpha[i], d[i], q[i]);
        T = matmul(T, Ti);
    }

    return T;
}


// Jacobian for position only (3x7)
Matrix3x7 compute_jacobian(const Vector7 &q)
{
    constexpr double eps = 1e-6;
    Matrix3x7 J{};

    Vector3 p0 = forward_kinematics(q);

    for (int i = 0; i < 7; i++)
    {
        Vector7 q_perturbed = q;
        q_perturbed[i] += eps;

        Vector3 p1 = forward_kinematics(q_perturbed);

        for (int j = 0; j < 3; j++)
            J[j][i] = (p1[j] - p0[j]) / eps;
    }

    return J;
}

// Jacobian for pose + orientation (6x7)
Matrix6x7 compute_jacobian_pose(const Vector7& q)
{
    constexpr double eps = 1e-6;
    Matrix6x7 J{};

    Matrix4 T0 = forward_kinematics_T(q);

    Vector3 p0 = { T0[0][3], T0[1][3], T0[2][3] };
    Matrix3 R0 = {{
        { T0[0][0], T0[0][1], T0[0][2] },
        { T0[1][0], T0[1][1], T0[1][2] },
        { T0[2][0], T0[2][1], T0[2][2] }
    }};

    for (int i = 0; i < 7; ++i)
    {
        Vector7 q_disturbed = q;
        q_disturbed[i] += eps;

        Matrix4 T1 = forward_kinematics_T(q_disturbed);

        Vector3 p1 = { T1[0][3], T1[1][3], T1[2][3] };
        Matrix3 R1 = {{
            { T1[0][0], T1[0][1], T1[0][2] },
            { T1[1][0], T1[1][1], T1[1][2] },
            { T1[2][0], T1[2][1], T1[2][2] }
        }};

        // Position part
        for (int k = 0; k < 3; ++k)
            J[k][i] = (p1[k] - p0[k]) / eps;

        // Orientation part
        Vector3 eR = orientation_error(R1, R0);
        for (int k = 0; k < 3; ++k)
            J[k + 3][i] = eR[k] / eps;
    }

    return J;
}


// Inverse kinematics (Jacobian transpose)
Vector7 inverse_kinematics(
    double x, double y, double z,
    const Vector7& q_init
)
{
    Vector7 q = q_init;
    const Vector3 target = {x, y, z};

    constexpr int max_iter = 1500;
    constexpr double tol  = 1e-4;
    constexpr double gain = 0.4;
    // int count=0;
    double norm=0.0;
    for (int iter = 0; iter < max_iter; ++iter)
    {   
        Vector3 p = forward_kinematics(q);

        Vector3 err;
        err[0] = target[0] - p[0];
        err[1] = target[1] - p[1];
        err[2] = target[2] - p[2];

        norm = norm3(err);

        if (norm < tol)
            break;

        auto J = compute_jacobian(q);

        for (int i = 0; i < 7; ++i)
        {
            double dq = 0.0;
            for (int j = 0; j < 3; ++j)
                dq += J[j][i] * err[j];

            q[i] += gain * dq;
        }
        // count++;
    }
    // std::cout<<"\nIterations:"<<count<<", Error:"<<norm<<std::endl;
    return q;
}


// Inverse kinematics with fixed orientation
Vector7 inverse_kinematics_fixed_orientation(
    const Vector3& p_des,
    const Matrix3& R_des,
    const Vector7& q_init
)
{
    Vector7 q = q_init;

    constexpr int max_iter = 1500;
    constexpr double tol = 1e-4;
    constexpr double gain_p = 0.4;
    constexpr double gain_R = 0.3;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        Matrix4 T = forward_kinematics_T(q);

        Vector3 p = { T[0][3], T[1][3], T[2][3] };
        Matrix3 R = {{
            {T[0][0], T[0][1], T[0][2]},
            {T[1][0], T[1][1], T[1][2]},
            {T[2][0], T[2][1], T[2][2]}
        }};

        Vector3 e_p = {
            p_des[0] - p[0],
            p_des[1] - p[1],
            p_des[2] - p[2]
        };

        Vector3 e_R = orientation_error(R, R_des);

        if (norm3(e_p) < tol && norm3(e_R) < tol)
            break;

        Matrix6x7 J = compute_jacobian_pose(q);

        Vector7 dq{};
        for (int i = 0; i < 7; ++i)
        {
            // Position contribution
            for (int j = 0; j < 3; ++j)
                dq[i] += gain_p * J[j][i] * e_p[j];

            // Orientation contribution
            for (int j = 0; j < 3; ++j)
                dq[i] -= gain_R * J[j+3][i] * e_R[j];
        }

        for (int i = 0; i < 7; ++i)
            q[i] += dq[i];
    }

    return q;
}


// Vector norm
double norm3(const Vector3& v)
{
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}


// Orientation error (angle-axis)
Vector3 orientation_error(const Matrix3& R1, const Matrix3& R0)
{
    Matrix3 R_err{};

    // R_err = R1 * R0^T
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                R_err[i][j] += R1[i][k] * R0[j][k];

    double trace = R_err[0][0] + R_err[1][1] + R_err[2][2];
    double cos_theta = (trace - 1.0) / 2.0;

    // Clamp for numerical safety
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));

    double theta = std::acos(cos_theta);

    if (theta < 1e-6)
        return {0.0, 0.0, 0.0};

    Matrix3 skew{};
    double denom = 2.0 * std::sin(theta);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            skew[i][j] = (R_err[i][j] - R_err[j][i]) / denom;

    return {
        theta * skew[2][1],
        theta * skew[0][2],
        theta * skew[1][0]
    };
}

// Test
// int main()
// {
//     EstimateWorkVolumeFixedOrientation_WithProgress();
//     return 0;
// }