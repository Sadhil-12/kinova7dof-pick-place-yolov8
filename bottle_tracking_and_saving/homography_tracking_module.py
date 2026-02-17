# tracking_module.py

import cv2
import torch
from ultralytics import YOLO

from calibration_homography_pixel_to_real_world_mapping import pixel_to_real


def run_detection(
    model_path,
    calibration_matrix,
    output_path,
    target_class_id=39,   # bottle
    conf_threshold=0.35,
    iou_threshold=0.4,
    camera_index=2,
):
    """
    YOLOv8 detection + pixel → real-world mapping. 
    - Homography maps (x_pixel, y_pixel) → (X_real, Y_real)
    - Bottom-centre of bounding box represents bottle-ground contact point
    - Press 's' to save detections
    """

    # Load model
    model = YOLO(model_path)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model.to(device)

    print(f"[INFO] YOLO running on {device}")

    # Camera
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    output_file = open(output_path, "w")
    frame_number = 0

    print("[INFO] Detection started")
    print("[INFO] Press 's' to SAVE, 'q' to quit")
    print("[INFO] Homography:\n", calibration_matrix)

    # Main loop
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(
            frame,
            conf=conf_threshold,
            iou=iou_threshold,
            verbose=False,
        )

        save_candidates = []

        for r in results:
            if r.boxes is None:
                continue

            for box, cls_id, conf in zip(
                r.boxes.xyxy,
                r.boxes.cls,
                r.boxes.conf,
            ):
                cls_id = int(cls_id)
                if cls_id != target_class_id:
                    continue

                x1, y1, x2, y2 = map(int, box.tolist())

                # Bottom-centre pixel of bottle
                px = (x1 + x2) // 2
                py = y2

                class_name = model.names[cls_id]

                # Pixel → real-world (direct homography)
                real_x, real_y = pixel_to_real((px, py), calibration_matrix)

                save_candidates.append(
                    (class_name, px, py, real_x, real_y)
                )

                # Visualization
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (px, py), 6, (0, 0, 255), -1)

                cv2.putText(
                    frame,
                    f"{class_name} ({real_x:.1f}, {real_y:.1f})",
                    (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                )

        cv2.imshow("Detection → World Mapping", frame)

        key = cv2.waitKey(1) & 0xFF

        # Press "s" to save current detections to file
        if key == ord("s"):
            for class_name, px, py, rx, ry in save_candidates:
                output_file.write(
                    f"{frame_number}, {class_name}, "
                    f"{px}, {py}, {rx:.2f}, {ry:.2f}\n"
                )
            print(f"[INFO] Saved {len(save_candidates)} detection(s)")

        elif key == ord("q"):
            break

        frame_number += 1

    cap.release()
    output_file.close()
    cv2.destroyAllWindows()

    print("[INFO] Detection complete. Output saved at:", output_path)
