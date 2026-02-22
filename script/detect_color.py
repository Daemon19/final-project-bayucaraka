from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_triplet(text: str) -> tuple[int, int, int]:
    values = [value.strip() for value in text.split(",")]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("Expected format: H,S,V")
    try:
        h, s, v = (int(values[0]), int(values[1]), int(values[2]))
    except ValueError as error:
        raise argparse.ArgumentTypeError("H,S,V must be integers") from error

    if not (0 <= h <= 179 and 0 <= s <= 255 and 0 <= v <= 255):
        raise argparse.ArgumentTypeError(
            "HSV out of range: H[0-179], S[0-255], V[0-255]"
        )

    return h, s, v


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Open a video source, detect target color, convert contour to OBB, "
            "and estimate pose with solvePnP"
        )
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        required=True,
        help="Path to camera calibration .npz containing camera_matrix and distortion",
    )
    parser.add_argument(
        "--source",
        type=str,
        default="0",
        help="Video source: camera index (0,1,...) or video file path",
    )
    parser.add_argument(
        "--hsv-lower",
        type=parse_triplet,
        default=(35, 70, 70),
        help="Lower HSV threshold as H,S,V (default: 35,70,70)",
    )
    parser.add_argument(
        "--hsv-upper",
        type=parse_triplet,
        default=(90, 255, 255),
        help="Upper HSV threshold as H,S,V (default: 90,255,255)",
    )
    parser.add_argument(
        "--marker-width",
        type=float,
        default=10.0,
        help="Real target width in centimeters for solvePnP (default: 10.0)",
    )
    parser.add_argument(
        "--marker-height",
        type=float,
        default=6.0,
        help="Real target height in centimeters for solvePnP (default: 6.0)",
    )
    parser.add_argument(
        "--min-area",
        type=float,
        default=1200.0,
        help="Minimum contour area in pixels to be considered valid (default: 1200)",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Optional output video path (.mp4)",
    )
    parser.add_argument(
        "--show-mask",
        action="store_true",
        help="Display threshold mask window",
    )
    return parser.parse_args()


def load_calibration(calibration_path: Path) -> tuple[np.ndarray, np.ndarray]:
    if not calibration_path.exists():
        raise FileNotFoundError(f"Calibration file not found: {calibration_path}")

    data = np.load(calibration_path)
    required_keys = {"camera_matrix", "distortion"}
    if not required_keys.issubset(data.files):
        raise KeyError(
            "Calibration file must contain keys: camera_matrix and distortion"
        )

    camera_matrix = data["camera_matrix"].astype(np.float64)
    distortion = data["distortion"].astype(np.float64)
    return camera_matrix, distortion


def order_box_points(points: np.ndarray) -> np.ndarray:
    pts = points.astype(np.float32)

    sums = pts.sum(axis=1)
    diffs = np.diff(pts, axis=1).reshape(-1)

    top_left = pts[np.argmin(sums)]
    bottom_right = pts[np.argmax(sums)]
    top_right = pts[np.argmin(diffs)]
    bottom_left = pts[np.argmax(diffs)]

    ordered = np.array(
        [top_left, top_right, bottom_right, bottom_left], dtype=np.float32
    )
    return ordered


def create_object_points(width: float, height: float) -> np.ndarray:
    half_w = width / 2.0
    half_h = height / 2.0
    return np.array(
        [
            [-half_w, -half_h, 0.0],
            [half_w, -half_h, 0.0],
            [half_w, half_h, 0.0],
            [-half_w, half_h, 0.0],
        ],
        dtype=np.float32,
    )


def rvec_to_euler_deg(rvec: np.ndarray) -> tuple[float, float, float]:
    rotation_matrix, _ = cv2.Rodrigues(rvec)

    sy = float(np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2))
    singular = sy < 1e-6

    if not singular:
        x = float(np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2]))
        y = float(np.arctan2(-rotation_matrix[2, 0], sy))
        z = float(np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]))
    else:
        x = float(np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1]))
        y = float(np.arctan2(-rotation_matrix[2, 0], sy))
        z = 0.0

    return np.degrees([x, y, z]).tolist()


def draw_axes(
    frame: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    axis_length: float,
) -> None:
    axis_3d = np.array(
        [
            [0.0, 0.0, 0.0],
            [axis_length, 0.0, 0.0],
            [0.0, axis_length, 0.0],
            [0.0, 0.0, axis_length],
        ],
        dtype=np.float32,
    )
    projected, _ = cv2.projectPoints(axis_3d, rvec, tvec, camera_matrix, distortion)
    projected = projected.reshape(-1, 2)
    if projected.shape[0] < 4 or not np.isfinite(projected).all():
        return

    rounded = np.rint(projected[:4]).astype(np.int64)

    def to_point(point: np.ndarray) -> tuple[int, int] | None:
        x = int(point[0])
        y = int(point[1])
        if abs(x) > 1_000_000 or abs(y) > 1_000_000:
            return None
        return (x, y)

    points = [to_point(point) for point in rounded]
    if any(point is None for point in points):
        return

    origin = points[0]
    px = points[1]
    py = points[2]
    pz = points[3]
    if origin is None or px is None or py is None or pz is None:
        return
    cv2.line(frame, origin, px, (0, 0, 255), 2)
    cv2.line(frame, origin, py, (0, 255, 0), 2)
    cv2.line(frame, origin, pz, (255, 0, 0), 2)


def build_color_mask(
    frame: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    kernel: np.ndarray,
) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_target_obb(
    mask: np.ndarray, min_area: float
) -> tuple[np.ndarray, np.ndarray] | None:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < min_area:
        return None

    rect = cv2.minAreaRect(largest)
    box = cv2.boxPoints(rect)
    image_points = order_box_points(box)
    return image_points, box.astype(int)


def estimate_pose(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> tuple[bool, np.ndarray, np.ndarray]:
    solved, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    return solved, rvec, tvec


def annotate_pose(
    frame: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    marker_width: float,
    marker_height: float,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> str:
    distance = float(np.linalg.norm(tvec))
    roll, pitch, yaw = rvec_to_euler_deg(rvec)
    draw_axes(
        frame,
        camera_matrix,
        distortion,
        rvec,
        tvec,
        axis_length=min(marker_width, marker_height) * 0.6,
    )

    cx, cy = np.mean(image_points, axis=0).astype(int)
    cv2.circle(frame, (int(cx), int(cy)), 4, (255, 255, 0), -1)

    angle_text = f"rpy[deg]=({roll:.1f}, {pitch:.1f}, {yaw:.1f})"
    cv2.putText(
        frame,
        angle_text,
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    return (
        f"xyz[cm]=({tvec[0, 0]:.3f}, {tvec[1, 0]:.3f}, {tvec[2, 0]:.3f}) "
        f"dist={distance:.3f}cm"
    )


def process_frame(
    frame: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    kernel: np.ndarray,
    min_area: float,
    object_points: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    marker_width: float,
    marker_height: float,
) -> tuple[np.ndarray, str]:
    # frame = cv2.blur(frame, (10, 10))
    mask = build_color_mask(frame, lower, upper, kernel)
    status_text = "No valid target"

    target = find_target_obb(mask, min_area)
    if target is None:
        return mask, status_text

    image_points, box_int = target
    solved, rvec, tvec = estimate_pose(
        object_points,
        image_points,
        camera_matrix,
        distortion,
    )
    cv2.drawContours(frame, [box_int], 0, (0, 255, 255), 2)

    if not solved:
        return mask, status_text

    status_text = annotate_pose(
        frame,
        image_points,
        rvec,
        tvec,
        marker_width,
        marker_height,
        camera_matrix,
        distortion,
    )
    return mask, status_text


def draw_frame_overlay(frame: np.ndarray, status_text: str, frame_height: int) -> None:
    cv2.putText(
        frame,
        status_text,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        "Press q to quit",
        (10, frame_height - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (180, 180, 180),
        1,
        cv2.LINE_AA,
    )


def main() -> None:
    args = parse_args()

    camera_matrix, distortion = load_calibration(args.calibration)

    source_in: str = args.source
    if source_in.isdigit():
        source = int(source_in)
    else:
        source = source_in

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video source: {args.source}")

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30

    writer = None
    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter.fourcc(*"mp4v")  # type: ignore
        writer = cv2.VideoWriter(str(args.save), fourcc, fps, (width, height))

    lower = np.array(args.hsv_lower, dtype=np.uint8)
    upper = np.array(args.hsv_upper, dtype=np.uint8)
    object_points = create_object_points(args.marker_width, args.marker_height)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    frame_count = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            mask, status_text = process_frame(
                frame,
                lower,
                upper,
                kernel,
                args.min_area,
                object_points,
                camera_matrix,
                distortion,
                args.marker_width,
                args.marker_height,
            )
            draw_frame_overlay(frame, status_text, height)

            cv2.imshow("Color OBB Pose", frame)
            if args.show_mask:
                cv2.imshow("Color Mask", mask)

            if writer is not None:
                writer.write(frame)

            frame_count += 1
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
        print(f"Processed frames: {frame_count}")


if __name__ == "__main__":
    main()
