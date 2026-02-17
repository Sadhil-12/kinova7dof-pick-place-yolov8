from pathlib import Path
import cv2
import numpy as np

# Anchor to project root
PROJECT_ROOT = Path(__file__).resolve().parent.parent

CALIBRATION_OUTPUT_FILE = (PROJECT_ROOT/ "data"/ "homography_calibration_points.txt")

# Live Feed Capture
def capture_frame(camera_index=0):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Failed to open camera")

    ret, frame = cap.read()
    cap.release()

    if not ret:
        raise RuntimeError("Failed to capture frame")

    return frame


# Manual point selection 
def select_pixel_points(image, N=5):
    selected = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(selected) < N:
                selected.append((x, y))
            else:
                print("[INFO] Point limit reached")

    cv2.namedWindow("Select Points")
    cv2.setMouseCallback("Select Points", on_mouse)

    while True:
        vis = image.copy()

        for i, p in enumerate(selected):
            cv2.circle(vis, p, 2, (255, 0, 0), -1)
            cv2.putText(
                vis,
                str(i),
                (p[0] + 6, p[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

        cv2.putText(
            vis,
            f"Click exactly {N} points | ENTER=confirm | ESC=cancel",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        if len(selected) == N:
            cv2.putText(
                vis,
                "Point limit reached",
                (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

        cv2.imshow("Select Points", vis)
        k = cv2.waitKey(1) & 0xFF

        if k == 13 and len(selected) == N:  # ENTER K=13
            break
        if k == 27:  # ESC K=27
            cv2.destroyAllWindows()
            return None

    cv2.destroyAllWindows()
    return np.array(selected, dtype=np.float32)



# Manual real-world coordinate input
def input_real_world_points(N):
    print(f"Enter real-world coordinates for {N} points")
    rw = []

    for i in range(N):
        x = float(input(f"Point {i} - X: "))
        y = float(input(f"Point {i} - Y: "))
        rw.append((x, y))

    return np.array(rw, dtype=np.float32)


# Homography computation
def compute_homography(pixel_pts, real_pts, ransac_thresh=0.05):
    if len(pixel_pts) < 4:
        raise ValueError("At least 4 points required")

    H, mask = cv2.findHomography(
        pixel_pts,
        real_pts,
        cv2.RANSAC,
        ransacReprojThreshold=ransac_thresh
    )

    if H is None:
        raise RuntimeError("Homography computation failed")

    return H, mask

# Load calibration points from file
def load_calibration_points(filepath):
    pixel_pts = []
    real_pts = []

    with open(filepath, "r") as f:
        for line in f:
            x, y, X, Y = map(float, line.strip().split(","))
            pixel_pts.append((x, y))
            real_pts.append((X, Y))

    return (
        np.array(pixel_pts, dtype=np.float32),
        np.array(real_pts, dtype=np.float32)
    )

# Save calibration points to file
def save_calibration_points(filepath, pixel_pts, real_pts):
    assert len(pixel_pts) == len(real_pts), "Point count mismatch"

    with open(filepath, "w") as f:
        for (x, y), (X, Y) in zip(pixel_pts, real_pts):
            f.write(f"{x:.4f}, {y:.4f}, {X:.4f}, {Y:.4f}\n")

    print(f"[INFO] Calibration points saved to:\n{filepath}")

# Pixel → real-world transform
def pixel_to_real(pt, H):
    pt = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
    rw = cv2.perspectiveTransform(pt, H)[0][0]
    return float(rw[0]), float(rw[1])

###############################################################
# Compiling all above functions into a single calibration run #
###############################################################
def run_calibration(
    camera_index=0,
    N=5,
    output_file=CALIBRATION_OUTPUT_FILE,
    ransac_thresh=1.0,):

    print("[INFO] Starting manual homography calibration")

    # 1. Capture live feed frame
    frame = capture_frame(camera_index)

    # 2. Select pixel points
    pixel_pts = select_pixel_points(frame, N=N)
    if pixel_pts is None:
        raise RuntimeError("Calibration aborted during point selection")

    # 3. Enter real-world coordinates
    real_pts = input_real_world_points(N)

    # 4. Compute homography
    H, mask = compute_homography(
        pixel_pts,
        real_pts,
        ransac_thresh=ransac_thresh
    )

    print("[INFO] Homography matrix:")
    print(H)

    # 5. Save calibration points for use later
    save_calibration_points(
        output_file,
        pixel_pts,
        real_pts
    )

    print("[INFO] Calibration completed successfully")

    # 6. Return homography
    return H
