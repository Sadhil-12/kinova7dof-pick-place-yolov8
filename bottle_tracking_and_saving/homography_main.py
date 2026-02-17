import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.append(str(Path(__file__).resolve().parent))

from calibration_homography_pixel_to_real_world_mapping import (
    run_calibration,
    load_calibration_points,
    compute_homography,
)

from homography_tracking_module import run_detection

# Parameters
CAMERA_INDEX = 0

MODEL_PATH = PROJECT_ROOT / "models" / "yolov8n.pt"
OUTPUT_PATH = PROJECT_ROOT / "data" / "homography_detected_bottle_centers.txt"
FINAL_CALIB_FILE = PROJECT_ROOT / "data" / "homography_calibration_pixel_to_real.txt"
TARGET_CLASS_ID = 39   # bottle
N_CALIB_POINTS = 5

# MAIN
if __name__ == "__main__":
    
    print("\n[INFO] Starting Bottle Detection Pipeline")
    print("-------------------------------------------")

    # 1. Calibration
    if not FINAL_CALIB_FILE.exists():
        print("[INFO] No calibration found — running manual calibration.")

        H = run_calibration(
            camera_index=CAMERA_INDEX,
            N=N_CALIB_POINTS,
            output_file=FINAL_CALIB_FILE,
        )

    else:
        choice = input("[?] Calibration file found. RECALIBRATE? (y/n): ").strip().lower()
        if choice == "y":
            H = run_calibration(
                camera_index=CAMERA_INDEX,
                N=N_CALIB_POINTS,
                output_file=FINAL_CALIB_FILE,
            )
        else:
            print("[INFO] Using existing calibration.")

            pixel_pts, real_pts = load_calibration_points(FINAL_CALIB_FILE)

            if pixel_pts is None or len(pixel_pts) < 4:
                print("[ERROR] Invalid calibration file.")
                sys.exit(1)

            H, _ = compute_homography(pixel_pts, real_pts)

    print("[INFO] Homography matrix:")
    print(H)

    # 2. Detection
    print("[INFO] Launching detection module...")

    run_detection(
        model_path=MODEL_PATH,
        calibration_matrix=H,
        output_path=OUTPUT_PATH,
        target_class_id=TARGET_CLASS_ID,
        conf_threshold=0.30,
        iou_threshold=0.4,
        camera_index=CAMERA_INDEX,
    )

    print("[INFO] Detection session complete.")
