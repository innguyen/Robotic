"""Optional ROS-free ONNX person detector using the repository's YOLOv5 format.

Adapted from jetauto_find_object.py:run_yolo preprocessing/YOLOv5 decoding.
No production module import, device initialization or network model download.
Checkpoint provenance/license must be verified before a public main-track run.
"""
import hashlib
from pathlib import Path


class OnnxPersonDetector:
    def __init__(self, model_path, threshold=.4):
        import cv2
        import numpy
        self.cv2, self.np = cv2, numpy
        path = Path(model_path)
        self.model_sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        self.net = cv2.dnn.readNetFromONNX(str(path))
        self.threshold = threshold

    def detect(self, rgb, width, height):
        np, cv2 = self.np, self.cv2
        # Input is already RGB; production input is BGR and uses swapRB=True.
        image = np.asarray(rgb, dtype=np.uint8).reshape(height, width, 3)
        self.net.setInput(cv2.dnn.blobFromImage(image, 1 / 255., (640, 640), swapRB=False, crop=False))
        rows = np.asarray(self.net.forward())
        if rows.ndim == 3 and rows.shape[0] == 1:
            rows = rows[0]
        if rows.ndim != 2 or rows.shape[1] != 85:
            raise ValueError("expected YOLOv5 COCO [N,85] output")
        boxes, scores = [], []
        for row in rows:
            score = float(row[4] * row[5])
            if int(np.argmax(row[5:])) != 0 or score < self.threshold:
                continue
            x, y, w, h = [float(v) for v in row[:4]]
            left, top = max(0., (x - w / 2) * width / 640), max(0., (y - h / 2) * height / 640)
            right, bottom = min(float(width), (x + w / 2) * width / 640), min(float(height), (y + h / 2) * height / 640)
            if right <= left or bottom <= top:
                continue
            boxes.append([left, top, right - left, bottom - top])
            scores.append(score)
        if not boxes:
            return []
        keep = cv2.dnn.NMSBoxes(boxes, scores, self.threshold, .45)
        return [{"box": [boxes[int(i)][0], boxes[int(i)][1], boxes[int(i)][0] + boxes[int(i)][2], boxes[int(i)][1] + boxes[int(i)][3]],
                 "confidence": scores[int(i)]} for i in np.asarray(keep).flatten()]
