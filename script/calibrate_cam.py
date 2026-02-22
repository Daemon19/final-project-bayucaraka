from __future__ import annotations
import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate camera using checkerboard images")
    parser.add_argument(
        "--images",
        type=str,
        required=True,
        help="Glob pattern for calibration images, e.g. 'dataset/calib/*.jpg'",
    )
    parser.add_argument(
        "--cols",
        type=int,
        required=True,
        help="Number of inner corners per checkerboard row (width).",
    )
    parser.add_argument(
        "--rows",
        type=int,
        required=True,
        help="Number of inner corners per checkerboard column (height).",
    )
    parser.add_argument(
        "--square-size",
        type=float,
        default=1.0,
        help="Checkerboard square size in real-world units (e.g. mm).",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Optional output file to save calibration as .npz",
    )
    parser.add_argument(
        "--show-corners",
        action="store_true",
        help="Show detected checkerboard corners while processing.",
    )
    return parser.parse_args()


def build_object_points(cols: int, rows: int, square_size: float) -> np.ndarray:
    objp = np.zeros((rows * cols, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp[:, :2] = grid
    objp *= square_size
    return objp


def main() -> None:
    args = parse_args()

    image_paths = sorted(Path().glob(args.images))
    if not image_paths:
        raise FileNotFoundError(f"No images found matching pattern: {args.images}")

    checkerboard_size = (args.cols, args.rows)
    objp = build_object_points(args.cols, args.rows, args.square_size)

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    for image_path in image_paths:
        image = cv2.imread(str(image_path))
        if image is None:
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])

        found, corners = cv2.findChessboardCorners(
            gray,
            checkerboard_size,
            None,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        if not found:
            continue

        refined_corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria,
        )

        object_points.append(objp.copy())
        image_points.append(refined_corners)

        if args.show_corners:
            vis = image.copy()
            cv2.drawChessboardCorners(vis, checkerboard_size, refined_corners, found)
            cv2.imshow("checkerboard", vis)
            cv2.waitKey(500)

    if args.show_corners:
        cv2.destroyAllWindows()

    if not object_points or image_size is None:
        raise RuntimeError(
            "Checkerboard corners were not detected in any image. "
            "Verify --cols/--rows and image quality."
        )

    camera_matrix_init = np.eye(3, dtype=np.float64)
    distortion_init = np.zeros((8, 1), dtype=np.float64)

    ret, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        camera_matrix_init,
        distortion_init,
    )

    total_error = 0.0
    for i in range(len(object_points)):
        projected, _ = cv2.projectPoints(
            object_points[i],
            rvecs[i],
            tvecs[i],
            camera_matrix,
            distortion,
        )
        error = cv2.norm(image_points[i], projected, cv2.NORM_L2) / len(projected)
        total_error += error

    mean_error = total_error / len(object_points)

    print(f"Images used for calibration: {len(object_points)} / {len(image_paths)}")
    print(f"RMS reprojection error: {ret:.6f}")
    print(f"Mean reprojection error: {mean_error:.6f}")
    print("cameraMatrix:")
    print(camera_matrix)
    print("distCoeffs:")
    print(distortion.ravel())

    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            args.save,
            camera_matrix=camera_matrix,
            distortion=distortion,
            rms=ret,
            mean_error=mean_error,
        )
        print(f"Saved calibration to: {args.save}")


if __name__ == "__main__":
    main()
