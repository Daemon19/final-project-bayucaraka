from __future__ import annotations

import argparse
import importlib
from pathlib import Path

import cv2
import numpy as np

YOLO = getattr(importlib.import_module("ultralytics"), "YOLO")


def load_calibration(calibration_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load camera matrix and distortion coefficients from NPZ file."""
    data = np.load(calibration_path)
    camera_matrix = data["camera_matrix"]
    distortion = data["distortion"]
    return camera_matrix, distortion


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run YOLO inference on undistorted video using camera calibration"
    )
    parser.add_argument(
        "--model",
        type=Path,
        required=True,
        help="Path to trained YOLO model (e.g., runs/yolo11_train/weights/best.pt)",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        required=True,
        help="Path to camera calibration NPZ file",
    )
    parser.add_argument(
        "--source",
        type=str,
        default="0",
        help="Video source: path to video file or camera index (0 for default webcam)",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="Confidence threshold for detections",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Optional: Save output video to this path",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display inference results in real-time",
    )
    parser.add_argument(
        "--no-undistort",
        action="store_true",
        help="Skip undistortion (use raw frames)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # Load model
    if not args.model.exists():
        raise FileNotFoundError(f"Model not found: {args.model}")

    print(f"Loading model from: {args.model}")
    model = YOLO(str(args.model))

    # Load calibration
    camera_matrix = None
    distortion = None
    if not args.no_undistort:
        if not args.calibration.exists():
            raise FileNotFoundError(f"Calibration file not found: {args.calibration}")

        camera_matrix, distortion = load_calibration(args.calibration)
        print(f"Loaded calibration from: {args.calibration}")
        print(f"Camera matrix shape: {camera_matrix.shape}")
        print(f"Distortion coeffs shape: {distortion.shape}")

    # Open video source
    source = args.source
    if source.isdigit():
        source = int(source)

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video source: {args.source}")

    # Get video properties
    fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"Video: {width}x{height} @ {fps} FPS")

    # Setup video writer if saving
    writer = None
    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter.fourcc(*"mp4v")  # type: ignore
        writer = cv2.VideoWriter(str(args.save), fourcc, fps, (width, height))
        print(f"Saving output to: {args.save}")

    # Compute optimal camera matrix for undistortion
    new_camera_matrix = None
    if not args.no_undistort and camera_matrix is not None and distortion is not None:
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix,
            distortion,
            (width, height),
            1,
            (width, height),
        )

    frame_count = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Undistort frame
            if (
                not args.no_undistort
                and camera_matrix is not None
                and distortion is not None
            ):
                frame = cv2.undistort(
                    frame,
                    camera_matrix,
                    distortion,
                    None,
                    new_camera_matrix,
                )

            # Run inference
            results = model(frame, conf=args.conf, verbose=False)

            # Annotate frame
            annotated_frame = results[0].plot()

            # Display
            if args.show:
                cv2.imshow("YOLO Inference (Undistorted)", annotated_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

            # Save
            if writer is not None:
                writer.write(annotated_frame)

            frame_count += 1
            if frame_count % 30 == 0:
                print(f"Processed {frame_count} frames...", end="\r")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
        print(f"\nTotal frames processed: {frame_count}")


if __name__ == "__main__":
    main()
