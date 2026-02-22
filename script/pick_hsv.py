# Utility script to select color ranges in an image using HSV color space

import argparse
from pathlib import Path

import cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="HSV picker for image or video capture input"
    )
    parser.add_argument(
        "--capture-type",
        choices=["videocapture", "image"],
        default="image",
        help="Input mode: 'image' for a single image or 'videocapture' for webcam/video",
    )
    parser.add_argument(
        "--source",
        type=str,
        required=True,
        help=(
            "Input source path or camera index. "
            "For image mode default is tugas/soal1.png, for videocapture use 0/1 or video path"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional output file path (image for image mode, video for videocapture mode)",
    )
    return parser.parse_args()


def create_trackbars() -> None:
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 640, 240)
    cv2.createTrackbar("HUE MIN", "Trackbars", 0, 179, empty)
    cv2.createTrackbar("HUE MAX", "Trackbars", 179, 179, empty)
    cv2.createTrackbar("SAT MIN", "Trackbars", 0, 255, empty)
    cv2.createTrackbar("SAT MAX", "Trackbars", 255, 255, empty)
    cv2.createTrackbar("VALUE MIN", "Trackbars", 0, 255, empty)
    cv2.createTrackbar("VALUE MAX", "Trackbars", 255, 255, empty)


def read_hsv_bounds() -> tuple[tuple[int, int, int], tuple[int, int, int]]:
    h_min = cv2.getTrackbarPos("HUE MIN", "Trackbars")
    h_max = cv2.getTrackbarPos("HUE MAX", "Trackbars")
    s_min = cv2.getTrackbarPos("SAT MIN", "Trackbars")
    s_max = cv2.getTrackbarPos("SAT MAX", "Trackbars")
    v_min = cv2.getTrackbarPos("VALUE MIN", "Trackbars")
    v_max = cv2.getTrackbarPos("VALUE MAX", "Trackbars")
    return (h_min, s_min, v_min), (h_max, s_max, v_max)


def apply_hsv_mask(frame):
    lower, upper = read_hsv_bounds()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)  # type: ignore
    result = cv2.bitwise_and(frame, frame, mask=mask)
    print(lower)
    print(upper)
    print()
    return result, mask


def run_image_mode(source: str, output: Path | None) -> None:
    src = cv2.imread(source)
    if src is None:
        raise FileNotFoundError(f"Failed to load image: {source}")

    last_result = src.copy()
    while True:
        result, _ = apply_hsv_mask(src)
        last_result = result
        cv2.imshow("Result", result)

        if cv2.waitKey(200) & 0xFF == ord("q"):
            break

    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output), last_result)
        print(f"Saved output image to: {output}")


def run_videocapture_mode(source: str, output: Path | None) -> None:
    if source.isdigit():
        cap_source: str | int = int(source)
    else:
        cap_source = source

    cap = cv2.VideoCapture(cap_source)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video source: {source}")

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30

    writer = None
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter.fourcc(*"mp4v")  # type: ignore
        writer = cv2.VideoWriter(str(output), fourcc, fps, (width, height))

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            result, _ = apply_hsv_mask(frame)
            cv2.imshow("Result", result)

            if writer is not None:
                writer.write(result)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        if writer is not None:
            writer.release()

    if output is not None:
        print(f"Saved output video to: {output}")


def main():
    args = parse_args()
    create_trackbars()

    try:
        if args.capture_type == "image":
            run_image_mode(args.source, args.output)
        else:
            run_videocapture_mode(args.source, args.output)
    finally:
        cv2.destroyAllWindows()


def empty(_):
    pass


if __name__ == "__main__":
    main()
