from ultralytics import YOLO
import torch

print(torch.cuda.is_available())

model = YOLO("yolov8x-seg.pt")

results = model.train(
        batch=8,
        device="0",
        data="data.yaml",
        epochs=7
    )