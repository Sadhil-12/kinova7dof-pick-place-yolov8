#include "CommandLayer.h"
#include "CommunicationLayer.h"
#include <iostream>
#include <array>
#include <cmath>
#include "move_to_point_angular_input.h"
#include "kinova_ik.h"
#include <algorithm>

using namespace std;

using Matrix3 = std::array<std::array<double,3>,3>;
using JointVector = std::array<double, 7>;
using Vector3 =std::array<double,3>;

KinovaControl::KinovaControl() : commandLayer_handle(nullptr),
    MyInitAPI(nullptr), MyCloseAPI(nullptr),
    MySendBasicTrajectory(nullptr), MyGetDevices(nullptr),
    MySetActiveDevice(nullptr), MyGetAngularCommand(nullptr) {}

KinovaControl::~KinovaControl() {
    Close();
}

// Initialize connection and load DLL
bool KinovaControl::Initialize() {
    std::cout << "Stub Initialize called" << std::endl;                 // REMOVE THIS IF NOT IN TESTING
    return true; // Always succeed for testing

    // SetDllDirectoryA("C:\\Program Files (x86)\\JACO-SDK\\API\\x64\\");
    // commandLayer_handle = LoadLibraryA("C:\\Program Files (x86)\\JACO-SDK\\API\\x64\\CommandLayerWindows.dll");
    // if (!commandLayer_handle) {
    //     cout << "Failed to load DLL!" << endl;
    //     return false;
    // }

    // MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
    // MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
    // MyGetDevices = (int(*)(KinovaDevice[MAX_KINOVA_DEVICE], int&)) GetProcAddress(commandLayer_handle, "GetDevices");
    // MySetActiveDevice = (int(*)(KinovaDevice)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
    // MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
    // MyGetAngularCommand = (int(*)(AngularPosition&)) GetProcAddress(commandLayer_handle, "GetAngularCommand");

    // if (!MyInitAPI || !MyCloseAPI || !MySendBasicTrajectory || !MyGetDevices || !MySetActiveDevice) {
    //     cout << "Failed to load all functions!" << endl;
    //     return false;
    // }

    // if (MyInitAPI() != 1) {
    //     cout << "InitAPI failed!" << endl;
    //     return false;
    // }

    // KinovaDevice list[MAX_KINOVA_DEVICE];
    // int result;
    // int devicesCount = MyGetDevices(list, result);
    // if (devicesCount <= 0) {
    //     cout << "No devices detected!" << endl;
    //     return false;
    // }

    // MySetActiveDevice(list[0]);
    // cout << "\n Device initialized successfully. \n" << endl;
    // return true;
    
}

// Close and cleanup
void KinovaControl::Close() {
    if (MyCloseAPI) {
        MyCloseAPI();
    }
    if (commandLayer_handle) {
        FreeLibrary(commandLayer_handle);
        commandLayer_handle = nullptr;
    }
}

void KinovaControl::MoveAllActuatorsToAngles(
    double theta1,
    double theta2,
    double theta3,
    double theta4,
    double theta5,
    double theta6,
    double theta7
)
{
    constexpr double PI = 3.141592653589793;
    constexpr double RAD2DEG = 180.0 / PI;

    // 1. Read current state (actuators + fingers)
    AngularPosition current;
    MyGetAngularCommand(current);

    TrajectoryPoint point;
    point.InitStruct();
    point.Position.Type = ANGULAR_POSITION;

    // 2. Set arm joints (absolute, degrees)
    point.Position.Actuators.Actuator1 = theta1 * RAD2DEG;
    point.Position.Actuators.Actuator2 = theta2 * RAD2DEG;
    point.Position.Actuators.Actuator3 = theta3 * RAD2DEG;
    point.Position.Actuators.Actuator4 = theta4 * RAD2DEG;
    point.Position.Actuators.Actuator5 = theta5 * RAD2DEG;
    point.Position.Actuators.Actuator6 = theta6 * RAD2DEG;
    point.Position.Actuators.Actuator7 = theta7 * RAD2DEG; //315105;

    // // 3. Preserve finger positions
    point.Position.Fingers.Finger1 = current.Fingers.Finger1;
    point.Position.Fingers.Finger2 = current.Fingers.Finger2;
    point.Position.Fingers.Finger3 = current.Fingers.Finger3;

    int status = MySendBasicTrajectory(point);

    cout << "Absolute joint command sent. Status: " << status << endl;
    cout << "Current Arm joints: "
         << point.Position.Actuators.Actuator1 << ", "
         << point.Position.Actuators.Actuator2 << ", "
         << point.Position.Actuators.Actuator3 << ", "
         << point.Position.Actuators.Actuator4 << ", "
         << point.Position.Actuators.Actuator5 << ", "
         << point.Position.Actuators.Actuator6 << ", "
         << point.Position.Actuators.Actuator7 << endl;
}

// Map raw encoder values to normalized [0,1] range for fingers
double mapFingerInput(double u, double open_val, double closed_val)
{
    if (u < 0.0) u = 0.0;
    if (u > 1.0) u = 1.0;

    return open_val + u * (closed_val - open_val);
}

// Move Fingers using normalized values where [0.0] is fully open and [1.0] is fully closed
void KinovaControl::MoveFingersNormalized(
    double u1,
    double u2,
    double u3 
)
{   
    // Raw encoder values
    constexpr double F1_OPEN   = 55.0; 
    constexpr double F1_CLOSED = 7300.0;

    constexpr double F2_OPEN   = 0.0;
    constexpr double F2_CLOSED = 3400.0;

    constexpr double F3_OPEN   = 30.0;
    constexpr double F3_CLOSED = 7300.0;

    // Normalized values
    double f1 = mapFingerInput(u1, F1_OPEN, F1_CLOSED);
    double f2 = mapFingerInput(u2, F2_OPEN, F2_CLOSED);
    double f3 = mapFingerInput(u3, F3_OPEN, F3_CLOSED);
    AngularPosition stable;
    for (int i = 0; i < 50; ++i) 
    {
        MyGetAngularCommand(stable);
        Sleep(20);
    }

    TrajectoryPoint point;
    point.InitStruct();
    point.Position.Type = ANGULAR_POSITION;

    // Keep arm joints unchanged
    AngularPosition current;
    MyGetAngularCommand(current);

    point.Position.Actuators.Actuator1 = current.Actuators.Actuator1;
    point.Position.Actuators.Actuator2 = current.Actuators.Actuator2;
    point.Position.Actuators.Actuator3 = current.Actuators.Actuator3;
    point.Position.Actuators.Actuator4 = current.Actuators.Actuator4;
    point.Position.Actuators.Actuator5 = current.Actuators.Actuator5;
    point.Position.Actuators.Actuator6 = current.Actuators.Actuator6;
    point.Position.Actuators.Actuator7 = current.Actuators.Actuator7;

    // Finger commands (raw encoder units)
    point.Position.Fingers.Finger1 = static_cast<float>(f1);
    point.Position.Fingers.Finger2 = static_cast<float>(f2);
    point.Position.Fingers.Finger3 = static_cast<float>(f3);

    MySendBasicTrajectory(point);
    cout << "Finger command sent: "<< f1 << ", " << f2 << ", " << f3 << endl;
}


// Joint limits for Kinova Gen2 7DOF (in radians)
struct JointLimit
{
    double min_rad;
    double max_rad;
};
static const JointLimit joint_limits[7] =
{
    { -10.471976,  10.471976 },  // J1  (-600°, 600°)  
    {  0.802851,   5.445427  },  // J2  (46°, 312°)    
    { -10.471976,  10.471976 },  // J3  (-600°, 600°)  
    {  0.523599,   5.759587  },  // J4  (30°, 330°)    
    { -10.471976,  10.471976 },  // J5  (-600°, 600°) 
    {  1.134464,   5.131268  },  // J6  (65°, 294°)    
    { -10.471976,  10.471976 }   // J7  (-600°, 600°)  
};


// Converting world coordinates to IK frame
void WorldToIK(double xw, double yw, double zw,
               double& xi, double& yi, double& zi)
{
    xi = -xw;
    yi = -yw;
    zi = -zw;
}

// Check if a point is within the robot's workspace for a given grasp type
bool WithinWorkspace(double x, double y, double z, double maxR = 0.95)
{
    constexpr double z0 = 0.37;   // base / shoulder height

    double dz = z - z0;

    double r = std::sqrt(x*x + y*y + dz*dz);
    return r <= maxR;
}

bool WithinWorkspaceVertical(double x,double y,double z,double maxR = 0.75)
{
    constexpr double base_height = 0.375;   // shoulder height from ground

    double dx = x;
    double dy = y;
    double dz = z - base_height;

    double r = std::sqrt(dx*dx + dy*dy + dz*dz);
    return r <= maxR;
}

bool WithinWorkspaceHorizontalPureY(double x, double y, double z)
{
    constexpr double z0 = 0.37;
    constexpr double R = 0.95;

    double dz = z - z0;

    double r = std::sqrt(x*x + y*y + dz*dz);
    return r <= R;
}

bool WithinWorkspaceHorizontalPureX(double x, double y, double z)
{
    constexpr double z0 = 0.37;
    constexpr double R = 0.75;
    constexpr double x_offset = 0.23;

    double dx = std::abs(x) - x_offset;
    double dz = z - z0;

    double r = std::sqrt(dx*dx + y*y + dz*dz);
    return r <= R;
}


constexpr double PI = 3.141592653589793;

// Normalize angles to be within limits and close to a reference configuration
bool normalize_with_limits(
    JointVector& q,
    const JointVector& q_ref)
{
    for (int i = 0; i < 7; ++i)
    {
        double best = q[i];
        double best_err = std::numeric_limits<double>::infinity();
        bool found = false;

        for (int k = -3; k <= 3; ++k)
        {
            double candidate = q[i] + 2.0 * PI * k;

            if (candidate < joint_limits[i].min_rad ||
                candidate > joint_limits[i].max_rad)
                {continue;}

            else
            {
                double err = std::abs(candidate - q_ref[i]);

                if (err < best_err)
                {
                    best = candidate;
                    best_err = err;
                    found = true;
                }
            }    
        }

        if (!found)
            return false;   // no valid equivalent angle exists

        q[i] = best;
    }

    return true;
}

//Home position in radians (converted from degrees)
JointVector HomePosition()
{
    return {
        4.710000,  // 269.91°
        2.840000,  // 162.73°
        0.000000,  //   0.00°
        0.750000,  //  42.97°
        4.620000,  // 264.72°
        4.480000,  // 256.68°
        4.880000   // 279.64°
    };
}


// Grasp Poses
Matrix3 R_GRASP_VERTICAL = {{
    { 1.0,  0.0,  0.0 },
    { 0.0,  1.0,  0.0 },
    { 0.0,  0.0,  1.0 }
}};

Matrix3 R_GRASP_horizontal_pure_Y = {{
    { 0.0, 1.0,  0.0 },
    { 0.0, 0.0,  -1.0 },
    { -1.0, 0.0,  0.0 }
}};

Matrix3 R_GRASP_horizontal_pure_pos_X = {{
    {  0.0,  0.0, -1.0 },
    {  0.0, 1.0,  0.0 },
    { 1.0,  0.0,  0.0 }
}};

Matrix3 R_GRASP_horizontal_pure_neg_X = {{
    {  0.0,  0.0, 1.0 },
    {  0.0, 1.0,  0.0 },
    { -1.0,  0.0,  0.0 }
}};

// Identify Grasp Type based on desired orientation
enum class GraspWorkspaceType
{
    PURE_X,
    PURE_Y,
    VERTICAL
};
GraspWorkspaceType IdentifyGraspType(const Matrix3& R)
{
    if (R == R_GRASP_horizontal_pure_pos_X ||
        R == R_GRASP_horizontal_pure_neg_X)
        return GraspWorkspaceType::PURE_X;

    if (R == R_GRASP_horizontal_pure_Y)
        return GraspWorkspaceType::PURE_Y;

    if (R == R_GRASP_VERTICAL)
        return GraspWorkspaceType::VERTICAL;
    
    return GraspWorkspaceType::VERTICAL; // default to vertical if unknown
}

// Choose workspace by grasp
bool WithinWorkspaceByGrasp(double x,
                            double y,
                            double z,
                            GraspWorkspaceType type)
{
    switch (type)
    {
        case GraspWorkspaceType::PURE_X:
            return WithinWorkspaceHorizontalPureX(x, y, z);

        case GraspWorkspaceType::PURE_Y:
            return WithinWorkspaceHorizontalPureY(x, y, z);

        case GraspWorkspaceType::VERTICAL:
            return WithinWorkspace(x, y, z);

        default:
            return false;
    }
}

// Inverse kinematics with free orientation
bool KinovaControl::MoveToTargetIK(
    double x_w,
    double y_w,
    double z_w
)
{
    std::cout << "\nIntermediate WORLD target: ("
              << x_w << ", " << y_w << ", " << z_w << ")\n";

    double x_ik, y_ik, z_ik;
    WorldToIK(x_w, y_w, z_w, x_ik, y_ik, z_ik);

    if (!WithinWorkspace(x_w, y_w, z_w))
    {
        std::cout << "Target outside workspace\n";
        return false;
    }

    JointVector q_seed = HomePosition();   // or GetCurrentJointAngles()

    JointVector q_final =
        inverse_kinematics(x_ik, y_ik, z_ik, q_seed);

    if (!normalize_with_limits(q_final, q_seed))
    {
        std::cout << "NO VALID IK SOLUTION FOUND FOR THIS LOCATION/ORIENTATION\n";
        return false;
    }

    std::cout << "Moving to target...\n";
    MoveAllActuatorsToAngles(
        q_final[0], q_final[1], q_final[2],
        q_final[3], q_final[4], q_final[5], q_final[6]
    );

    return true;
}

// Inverse kinematics with fixed orientation (orientation specified by R_des)
bool KinovaControl::MoveToTargetIKOrientFix(
    double x_w,
    double y_w,
    double z_w,
    const Matrix3& R_des
)
{
    std::cout << "\nIntermediate WORLD target: ("
              << x_w << ", " << y_w << ", " << z_w << ")\n";

    double x_ik, y_ik, z_ik;
    WorldToIK(x_w, y_w, z_w, x_ik, y_ik, z_ik);

    GraspWorkspaceType grasp_type = IdentifyGraspType(R_des);

    if (!WithinWorkspaceByGrasp(x_w, y_w, z_w, grasp_type))
    {
        std::cout << "Target unreachable for this grasp orientation\n";
        return false;
    }

    JointVector q_seed = HomePosition();   // or GetCurrentJointAngles()

    Vector3 p_des = { x_ik, y_ik, z_ik };

    JointVector q_final =
        inverse_kinematics_fixed_orientation(
            p_des,
            R_des,
            q_seed
        );

    if (!normalize_with_limits(q_final, q_seed))
    {
        std::cout << "NO VALID IK SOLUTION FOUND FOR THIS LOCATION/ORIENTATION\n";
        return false;
    }

    std::cout << "Moving to target (fixed orientation)...\n";

    MoveAllActuatorsToAngles(
        q_final[0], q_final[1], q_final[2],
        q_final[3], q_final[4], q_final[5], q_final[6]
    );
    
    return true;
}


////////////////////////// PICK & PLACE Algorithms ///////////////////////
// Pick & Place parameters
constexpr double x_place = 0.3;
constexpr double y_place = 0.6;

constexpr double z_safe = 0.30;

// Pick & Place with vertical grasp (fingers approach from above)
void KinovaControl::PickPlaceVertical(
    double x_target,
    double y_target,
    double z_target
)
{
 
    std::cout << "\n================ PICK & PLACE (VERTICAL) ================\n";
    std::cout << "Target WORLD: (" << x_target << ", "<< y_target << ", " << z_target << ")\n";
    std::cout << "Safe height: z = " << z_safe << "\n";

    if (!WithinWorkspaceVertical(x_target, y_target, z_target))
    {
        std::cout << "Target outside workspace for VERTICAL ORIENTATION\n";
        return;
    }
    std::cout << "\n[PRE] Move to pre-target (safe height)\n";
    if (!MoveToTargetIKOrientFix(x_target, y_target, z_safe,R_GRASP_VERTICAL))
    {
        std::cout << "Failed to move to pre-target. Aborting pick & place.\n";
        return;
    }

    std::cout << "\n[FINGERS] Open\n";
    Sleep(1500);
    MoveFingersNormalized(0.0, 0.0, 0.0);
    Sleep(1500);

    std::cout << "\n[REACH] Descend to target (fixed vertical grasp)\n";
    if (!MoveToTargetIKOrientFix(x_target, y_target, z_target,R_GRASP_VERTICAL))
    {
        std::cout << "Failed to move to target. Aborting pick & place.\n";
        return;
    }

    std::cout << "\n[GRASP] Closing fingers\n";
    Sleep(1500);
    MoveFingersNormalized(0.8, 0.8, 0.8);
    Sleep(1500);

    std::cout << "\n[LIFT] Raise object to safe height\n";
    if (!MoveToTargetIKOrientFix(x_target, y_target, z_safe,R_GRASP_VERTICAL))
    {
        std::cout << "Failed to move to safe height. Aborting pick & place.\n";
        return;
    }
    Sleep(1500);

    std::cout << "\n[TRANSFER] Move above place location\n";
    if (!MoveToTargetIKOrientFix(x_place, y_place, z_safe,R_GRASP_VERTICAL))
    {
        std::cout << "Failed to move to place location. Aborting pick & place.\n";
        return;
    }
    Sleep(1500);

    std::cout << "\n[PLACE] Descend to place location in 2 steps\n";
    if (!MoveToTargetIKOrientFix(x_place, y_place, z_target + 0.03, R_GRASP_VERTICAL))
    {
        std::cout << "Failed to move to place location (step 1). Aborting pick & place.\n";
        return;
    }
    Sleep(1000);
    if (!MoveToTargetIKOrientFix(x_place, y_place, z_target,R_GRASP_VERTICAL))
    {
        std::cout << "Failed to move to final place location. Aborting pick & place.\n";
        return;
    }

    std::cout << "\n[RELEASE] Opening fingers\n";
    Sleep(1500);
    MoveFingersNormalized(0.0, 0.0, 0.0);
    Sleep(1500);

    std::cout << "\n[RETREAT] Return to safe height\n";
    if (!MoveToTargetIKOrientFix(x_place, y_place, z_safe,R_GRASP_VERTICAL))
    {
        std::cout << "Failed to return to safe height. Aborting pick & place.\n";
        return;
    }
    
    std::cout<<"\nMove to home position\n";
    JointVector q_home=HomePosition();
    MoveAllActuatorsToAngles(q_home[0], q_home[1], q_home[2],q_home[3], q_home[4], q_home[5], q_home[6]);

    std::cout << "\n================ PICK & PLACE COMPLETE ==================\n";

}

// Pick & Place with horizontal grasp (fingers approach from the side, orientation chosen based on target location)
void KinovaControl::PickPlaceHorizontal(double x_target, 
    double y_target, 
    double z_target)
{
    const Matrix3* R_GRASP = nullptr;
    enum class GraspType { POS_X, NEG_X, Y };
    GraspType grasp_type;

    if (abs(x_target) > abs(y_target))
    {
        if (x_target >= 0.0)
        {
            R_GRASP = &R_GRASP_horizontal_pure_pos_X;
            grasp_type = GraspType::POS_X;
        }
        else
        {
            R_GRASP = &R_GRASP_horizontal_pure_neg_X;
            grasp_type = GraspType::NEG_X;
        }
    }
    else
    {
        R_GRASP = &R_GRASP_horizontal_pure_Y;
        grasp_type = GraspType::Y;
    }

    constexpr double approach_offset = 0.1;

    std::cout << "\n================ PICK & PLACE (HORIZONTAL) ================\n";
    std::cout << "Target WORLD: (" << x_target << ", " << y_target << ", " << z_target << ")\n";
    std::cout << "Safe height: z = " << z_safe << "\n";

    // Within Horizontal Workspace for selected Grasp orientation
    bool reachable = false;
    if (grasp_type == GraspType::Y)
    {
        reachable = WithinWorkspaceHorizontalPureY(
            x_target, y_target, z_target);
    }
    else
    {
        reachable = WithinWorkspaceHorizontalPureX(
            x_target, y_target, z_target);
    }

    if (!reachable)
    {
        std::cout
            << "Target unreachable for selected horizontal grasp\n";
        return;
    }

    std::cout << "Grasp orientation: ";
    if (grasp_type == GraspType::POS_X) std::cout << "PURE +X\n";
    else if (grasp_type == GraspType::NEG_X) std::cout << "PURE -X\n";
    else std::cout << "PURE Y\n";

    std::cout << "\n[APPROACH] Lateral approach to target\n";
    if (grasp_type == GraspType::POS_X)
        if (!MoveToTargetIKOrientFix(x_target - approach_offset, y_target, z_target, *R_GRASP))
        {
            std::cout << "Failed to approach target (POS_X). Aborting pick & place.\n";
            return;
        }
    else if (grasp_type == GraspType::NEG_X)
        if (!MoveToTargetIKOrientFix(x_target + approach_offset, y_target, z_target, *R_GRASP))
        {
            std::cout << "Failed to approach target (NEG_X). Aborting pick & place.\n";
            return;
        }
    else
        if (!MoveToTargetIKOrientFix(x_target, y_target - approach_offset, z_target, *R_GRASP))
        {
            std::cout << "Failed to approach target (Y). Aborting pick & place.\n";
            return;
        }

    Sleep(1500);
    Sleep(1500);
    std::cout << "\n[REACH] Final approach to grasp pose\n";
    if (!MoveToTargetIKOrientFix(x_target, y_target, z_target, *R_GRASP))
    {
        std::cout << "Failed to reach grasp pose. Aborting pick & place.\n";
        return;
    }

    Sleep(1500);
    Sleep(1500);
    std::cout << "\n[GRASP] Closing fingers\n";
    MoveFingersNormalized(0.6, 0.6, 0.6);
    Sleep(1500);
    Sleep(1500);
    std::cout << "\n[LIFT] Lift object to safe height\n";
    if (!MoveToTargetIKOrientFix(x_target, y_target, z_safe, *R_GRASP))
    {
        std::cout << "Failed to reach safe height. Aborting pick & place.\n";
        return;
    }

    Sleep(1500);
    Sleep(1500);
    std::cout << "\n[TRANSFER] Move above place location\n";
    if (!MoveToTargetIKOrientFix(x_place, y_place, z_safe, *R_GRASP))
    {
        std::cout << "Failed to reach place location. Aborting pick & place.\n";
        return;
    }
    Sleep(1500);
    Sleep(1500);
    std::cout << "\n[PLACE] Descend slowly to place\n";

    if (!MoveToTargetIKOrientFix(x_place, y_place, z_target, *R_GRASP))
    {
        std::cout << "Failed to reach place target. Aborting pick & place.\n";
        return;
    }
    Sleep(1500);
    Sleep(1500);
    std::cout << "\n[RELEASE] Opening fingers\n";
    MoveFingersNormalized(0.0, 0.0, 0.0);
    Sleep(1500);
    Sleep(1500);
    std::cout << "\n[RETREAT] Return to safe height\n";
    if (!MoveToTargetIKOrientFix(x_place, y_place, z_safe, *R_GRASP))
    {
        std::cout << "Failed to return to safe height. Aborting pick & place.\n";
        return;
    }
    Sleep(1500);
    Sleep(1500);
    std::cout<<"\nMove to home position\n";
    JointVector q_home=HomePosition();
    MoveAllActuatorsToAngles(q_home[0], q_home[1], q_home[2],q_home[3], q_home[4], q_home[5], q_home[6]);
    std::cout << "\n================ PICK & PLACE COMPLETE =====================\n";
}

