#!/usr/bin/env python3

import cv2
import numpy as np
import argparse
import sys


# ==========================================================
# Argument Parsing
# ==========================================================


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Pixel → Ground (cm) Homography Calibration using ArUco"
    )

    parser.add_argument(
        "--camera", type=str, default="0", help="Camera index or video file path"
    )

    parser.add_argument(
        "--dict", type=str, default="DICT_4X4_50", help="ArUco dictionary name"
    )

    parser.add_argument(
        "--marker-id", type=int, default=None, help="Use only specific marker ID"
    )

    parser.add_argument(
        "--calibration",
        required=True,
        help="Path to calibration .npz containing camera matrix and distortion coefficients",
    )

    parser.add_argument(
        "--output", default="homography_cm.npy", help="Output homography filename"
    )

    parser.add_argument(
        "--min-points", type=int, default=4, help="Minimum calibration points"
    )

    return parser.parse_args()


# ==========================================================
# Camera Calibration Loading
# ==========================================================


def load_camera_calibration(calibration_path):
    try:
        calibration = np.load(calibration_path)

        camera_matrix_keys = ("camera_matrix", "cameraMatrix", "K")
        dist_coeffs_keys = ("distortion", "dist_coeffs", "distCoeffs", "D")

        K = next((calibration[k] for k in camera_matrix_keys if k in calibration), None)
        D = next((calibration[k] for k in dist_coeffs_keys if k in calibration), None)

        if K is None or D is None:
            raise KeyError(
                "Calibration file must contain camera matrix and distortion coefficients. "
                "Expected keys include: camera_matrix/cameraMatrix/K and "
                "distortion/dist_coeffs/distCoeffs/D"
            )
    except Exception as e:
        print(f"Error loading calibration files: {e}")
        sys.exit(1)

    return K, D


# ==========================================================
# ArUco Detector Setup
# ==========================================================


def create_aruco_detector(dict_name):
    if not hasattr(cv2.aruco, dict_name):
        print("Invalid ArUco dictionary name.")
        sys.exit(1)

    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))

    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    return detector


# ==========================================================
# Video Source Setup
# ==========================================================


def open_video_source(source):
    if source.isdigit():
        cap = cv2.VideoCapture(int(source))
    else:
        cap = cv2.VideoCapture(source)

    if not cap.isOpened():
        print("Could not open video source.")
        sys.exit(1)

    return cap


# ==========================================================
# Detect Marker Center
# ==========================================================


def detect_marker_center(frame, detector, marker_id_filter=None):
    corners, ids, _ = detector.detectMarkers(frame)

    center = None

    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id_filter is not None and marker_id != marker_id_filter:
                continue

            c = corners[i][0]
            center_x = np.mean(c[:, 0])
            center_y = np.mean(c[:, 1])
            center = (center_x, center_y)

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.circle(frame, (int(center_x), int(center_y)), 6, (0, 0, 255), -1)
            break

    return center, frame


# ==========================================================
# Compute Homography
# ==========================================================


def compute_homography(pixel_points, ground_points):
    pixel_pts = np.array(pixel_points, dtype=np.float32)
    ground_pts = np.array(ground_points, dtype=np.float32)

    H, status = cv2.findHomography(pixel_pts, ground_pts)
    return H


# ==========================================================
# Compute Reprojection Error
# ==========================================================


def compute_reprojection_error(H, pixel_points, ground_points):
    pixel_pts = np.array(pixel_points, dtype=np.float32).reshape(-1, 1, 2)
    ground_pts = np.array(ground_points, dtype=np.float32)

    projected = cv2.perspectiveTransform(pixel_pts, H).reshape(-1, 2)
    error = np.linalg.norm(projected - ground_pts, axis=1)

    return error, np.mean(error)


# ==========================================================
# Calibration Loop
# ==========================================================


def calibration_loop(cap, detector, K, D, marker_id_filter):
    pixel_points = []
    ground_points = []

    print("\nSPACE → record point")
    print("Q → quit and compute\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_undist = cv2.undistort(frame, K, D)

        center, display = detect_marker_center(frame_undist, detector, marker_id_filter)

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(" "):
            if center is None:
                print("No valid marker detected.")
                continue

            print(f"\nPixel: ({center[0]:.2f}, {center[1]:.2f})")

            try:
                Xg = float(input("Ground X (cm): "))
                Yg = float(input("Ground Y (cm): "))
            except ValueError:
                print("Invalid numeric input.")
                continue

            pixel_points.append([center[0], center[1]])
            ground_points.append([Xg, Yg])

            print(f"Stored point #{len(pixel_points)}")

        if key == ord("q"):
            break

    return pixel_points, ground_points


# ==========================================================
# Main
# ==========================================================


def main():
    args = parse_arguments()

    K, D = load_camera_calibration(args.calibration)

    detector = create_aruco_detector(args.dict)
    cap = open_video_source(args.camera)

    pixel_points, ground_points = calibration_loop(cap, detector, K, D, args.marker_id)

    cap.release()
    cv2.destroyAllWindows()

    if len(pixel_points) < args.min_points:
        print(f"\nNeed at least {args.min_points} points.")
        sys.exit(1)

    H = compute_homography(pixel_points, ground_points)
    np.save(args.output, H)

    print("\nHomography (pixel → cm):")
    print(H)
    print(f"\nSaved to {args.output}")

    error, mean_error = compute_reprojection_error(H, pixel_points, ground_points)

    print("\nReprojection error per point (cm):")
    print(error)
    print(f"Mean error: {mean_error:.4f} cm")


# ==========================================================

if __name__ == "__main__":
    main()
