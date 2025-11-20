#!/usr/bin/env python3
"""YOLO wrapper using ultralytics package.
It provides a simple API:
  model = YOLOWrapper(weights='yolov8n.pt')
  detections = model.detect(frame)
"""
import os
import numpy as np

try:
    from ultralytics import YOLO
    _HAS_ULTRALYTICS = True
except Exception:
    _HAS_ULTRALYTICS = False

class YOLOWrapper:
    def __init__(self, weights="yolov8n.pt", device=None, conf=0.25):
        if not _HAS_ULTRALYTICS:
            raise RuntimeError("ultralytics package not available. Install via `pip install ultralytics`.")
        self.weights = weights
        self.model = YOLO(weights)
        if device:
            self.model.to(device)
        self.conf = conf

    def detect(self, frame):
        """
        frame: ndarray BGR (as OpenCV) or RGB
        returns list of detections: [{'xyxy':[x1,y1,x2,y2],'conf':0.9,'class':0,'name':'person'}]
        """
        res = self.model(frame, imgsz=640, conf=self.conf)
        out = []
        for r in res:
            boxes = r.boxes
            for b in boxes:
                xyxy = b.xyxy[0].cpu().numpy().tolist()
                conf = float(b.conf[0].cpu().numpy())
                cls = int(b.cls[0].cpu().numpy())
                name = self.model.names[cls] if cls in self.model.names else str(cls)
                out.append({"xyxy": xyxy, "conf": conf, "class": cls, "name": name})
        return out

if __name__ == "__main__":
    # simple smoke test using webcam (index 0)
    import cv2
    w = YOLOWrapper()
    cap = cv2.VideoCapture(0)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        dets = w.detect(frame)
        for d in dets:
            x1,y1,x2,y2 = map(int, d['xyxy'])
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(frame, f"{d['name']}:{d['conf']:.2f}", (x1,y1-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        cv2.imshow("YOLO test", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
