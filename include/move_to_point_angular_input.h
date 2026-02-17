#ifndef MOVE_TO_POINT_ANGULAR_INPUT_H
#define MOVE_TO_POINT_ANGULAR_INPUT_H

#include <array>
#include <windows.h>    // Remove if not using Windows
#include <algorithm>

// Kinova SDK
#include "CommandLayer.h"

// Used aliases
using JointVector = std::array<double, 7>;
using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<std::array<double, 3>, 3>;

struct Vec3
{
    double x;
    double y;
    double z;
};

// =========================
// Kinova controller class
// =========================
class KinovaControl
{
private:
    // DLL handle
    HINSTANCE commandLayer_handle;

    // Kinova SDK function pointers
    int (*MyInitAPI)();
    int (*MyCloseAPI)();
    int (*MySendBasicTrajectory)(TrajectoryPoint command);
    int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int& result);
    int (*MySetActiveDevice)(KinovaDevice device);
    int (*MyGetAngularCommand)(AngularPosition&);

public:
    KinovaControl();
    ~KinovaControl();

    bool Initialize();
    void Close();

    JointVector GetCurrentJointAnglesInRad();
    
    void MoveAllActuatorsToAngles(
        double theta1,
        double theta2,
        double theta3,
        double theta4,
        double theta5,
        double theta6,
        double theta7
    );

    void MoveFingersNormalized(
        double u1,  // Finger 1 ∈ [0,1]
        double u2,  // Finger 2 ∈ [0,1]
        double u3   // Finger 3 ∈ [0,1]
    );

    void WaitUntilReached(const JointVector& q_target, double tol_deg = 3.0);

    bool MoveToTargetIK(
        double x_world,
        double y_world,
        double z_world
    );

    bool MoveToTargetIKOrientFix(
        double x_w,
        double y_w,
        double z_w,
        const Matrix3& R_des
    );

    void PickPlaceVertical(
        double x_target,
        double y_target,
        double z_target
    );

    void PickPlaceHorizontal(
        double x_target,
        double y_target,
        double z_target
    );

    
};

#endif // MOVE_TO_POINT_ANGULAR_INPUT_H
