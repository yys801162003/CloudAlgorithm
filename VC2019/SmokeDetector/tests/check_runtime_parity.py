"""Optional development check. The C++ application needs no Python runtime.

Requires opencv-python-headless 4.10.0.84, onnxruntime 1.23.2 and NumPy.
"""
from pathlib import Path
import hashlib
import cv2
import numpy as np
import onnxruntime as ort

root = Path(__file__).resolve().parents[1]
model = root / 'models/firewatch-yolo11s.onnx'
assert hashlib.sha256(model.read_bytes()).hexdigest() == '1ab25ae6a26e711542e5b5e9670611476719532bc00ef1bd1a1657c01f24c2b4'
network = cv2.dnn.readNetFromONNX(str(model))
network.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
network.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
session = ort.InferenceSession(str(model), providers=['CPUExecutionProvider'])
for name in ['upstream_smoke_annotated.jpg', 'upstream_mixed_missed_smoke.jpg']:
    image = cv2.imdecode(np.fromfile(root / 'samples' / name, dtype=np.uint8), cv2.IMREAD_COLOR)
    height, width = image.shape[:2]
    scale = min(640 / width, 640 / height)
    resized_width, resized_height = round(width * scale), round(height * scale)
    left, top = (640 - resized_width) // 2, (640 - resized_height) // 2
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(resized, top, 640-resized_height-top, left, 640-resized_width-left,
                               cv2.BORDER_CONSTANT, value=(114, 114, 114))
    blob = cv2.dnn.blobFromImage(padded, 1/255, (640, 640), swapRB=True)
    network.setInput(blob)
    actual = network.forward()
    reference = session.run(None, {'images': blob})[0]
    scores_error = float(np.max(np.abs(actual[:, 4:] - reference[:, 4:])))
    boxes_error = float(np.max(np.abs(actual[:, :4] - reference[:, :4])))
    print(name, 'score_max_abs_error=', scores_error, 'box_max_abs_error_px=', boxes_error)
    assert scores_error < 1e-4 and boxes_error < 0.02
print('Runtime parity: passed. This checks inference implementation, not smoke accuracy.')
