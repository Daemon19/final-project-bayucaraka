from pathlib import Path
from ultralytics import YOLO
import torch
import os


def get_project_root() -> Path:
    """Returns the root directory of the project."""
    return Path(os.path.abspath("."))


device_type = "cuda" if torch.cuda.is_available() else "cpu"

model = YOLO("yolov10n-obb.pt")
model.train(
    data=get_project_root() / "dataset" / "data.yaml",
    epochs=50,
    imgsz=640,
    batch=16,
    device=device_type,
    workers=4,
    project="final-project-bayucaraka",
    name="yolo10n-obb-payload",
    patience=10,
    verbose=True,
)
