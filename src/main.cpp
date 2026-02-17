#include "move_to_point_angular_input.h"
#include <array>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>
#include <filesystem>
#include <windows.h>

namespace fs = std::filesystem;
// Get directory where the executable is located
fs::path getExecutableDir()
{
    char buffer[MAX_PATH];
    GetModuleFileNameA(NULL, buffer, MAX_PATH);
    return fs::path(buffer).parent_path();
}
// Define project root (two levels up from Debug/)
const fs::path PROJECT_ROOT = getExecutableDir().parent_path().parent_path();
const fs::path OUTPUT_PATH =PROJECT_ROOT /"data" /"homography_detected_bottle_centers.txt";

using JointVector = std::array<double, 7>;

bool ReadLatestDetection(
    const fs::path& filepath,
    double& x_m,
    double& y_m)
{
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open detection file\n";
        return false;
    }
    std::cout << "Found path: " << filepath << std::endl;
    bool frame_found = false;
    int max_frame = 0;
    double x_cm = 0.0, y_cm = 0.0;

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);

        int frame;
        std::string class_name;
        double px, py, X, Y;
        char comma;

        ss >> frame >> comma;
        ss >> class_name >> comma;
        ss >> px >> comma;
        ss >> py >> comma;
        ss >> X >> comma;
        ss >> Y;

        if (!frame_found || frame > max_frame) {
            frame_found = true;
            max_frame = frame;
            x_cm = X;
            y_cm = Y;
        }
    }

    file.close();

    if (!frame_found) {
        std::cerr << "No valid detections found\n";
        return false;
    }

    x_m = x_cm / 100.0; // in metres
    y_m = y_cm / 100.0; // in metres

    return true;
}



int main() {
    KinovaControl arm;
    JointVector qDeg;

    if (!arm.Initialize()) {
        return -1;
    }

    JointVector q_home = {
        4.710000,
        2.840000,
        0.000000,
        0.750000,
        4.620000,
        4.480000,
        4.880000
    };

    // Read latest detection
    double x_target_m, y_target_m;

    // READING FROM FILE OF BOTTLE TRACKING
    if (!ReadLatestDetection(OUTPUT_PATH, x_target_m, y_target_m)) {
        arm.Close();
        return -1;
    }

    std::cout << "Moving to target (meters): "<< x_target_m << ", "<< y_target_m << std::endl;

    // Pick Place Pipeline Run
    arm.PickPlaceVertical(
        x_target_m,   // x (meters)
        y_target_m,   // y (meters)
        0.05 // z (meters) 0.0 is 1cm off table with fingers closed
    );

    arm.Close();
    return 0;
}