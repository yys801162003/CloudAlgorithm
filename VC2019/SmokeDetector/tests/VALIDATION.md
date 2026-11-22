# Validation record

Environment: Linux x86-64, GCC 14.2, native OpenCV 4.10.0 built with core/imgproc/imgcodecs/dnn, CPU inference. Windows resource and object-file checks used LLVM-MinGW. These are not MSVC or Windows desktop runtime tests.

## C++ functional test output

```text
Synthetic rule candidate area=24.695%
Real ONNX blank image: score=9.90986e-05, detected=0
Real smoke photo: score=0.676481, boxes=1, detected=1
Mixed fire/smoke regression photo: smoke score=0.0762374, smoke detected=0, fire=1
Image decoding, rule logic, YOLO postprocessing, real ONNX and exports: passed
```

The exact same test suite completed under AddressSanitizer and UndefinedBehaviorSanitizer for application/test code; the prebuilt native OpenCV static libraries were not instrumented.

The photo fixtures are `samples/upstream_smoke_annotated.jpg` and `samples/upstream_mixed_missed_smoke.jpg`. They are already-annotated upstream functional examples, not an independent accuracy test set. No overall accuracy/recall/false-positive claim is made.

## Checks completed

- PNG exact decode and JPEG decode, including Unicode file paths.
- Analysis-size limiting and original-size metadata.
- Damaged image, unsupported extension, invalid thresholds and missing-model errors.
- Rule positive construction, saturated background negative and uniform-gray negative.
- 640-square letterbox and padding, reverse box coordinates.
- Class-aware NMS, exact threshold boundary, invalid output shape and NaN filtering.
- Real trained ONNX forward pass, smoke example and mixed fire/smoke example (the latter is a documented smoke miss).
- Cancellation between processing stages, image/report export, existing-directory protection.
- Model SHA256 checked against upstream v0.1.0.
- Official OpenCV Windows installer SHA256 checked; archive contains required x64/vc16 Release and Debug import libraries and DLLs.
- PowerShell 7 syntax checks for all scripts; existing-OpenCV-directory setup with spaces/Unicode/ampersand and XML escaping; existing-model verification path.

Not exercised here: running the official extractor on Windows, MSVC v142 full build/link, GUI interaction on Windows, independent real-world smoke benchmark, or the script paths requiring a missing-model download. The README supplies Windows build and test commands.

The mixed-scene photograph contains visible/upstream-annotated smoke. The current default result misses that smoke. Its reported no-smoke result is a documented failure, not a verified negative or evidence of accuracy.


## Independent inference-backend comparison

OpenCV DNN 4.10.0 was compared with ONNX Runtime CPU 1.23.2 using identical inputs and the unmodified model:

| Input | Maximum class-score absolute error | Maximum box-coordinate absolute error in network pixels |
|---|---:|---:|
| upstream smoke example | 7.1525574e-7 | 0.003326416 |
| upstream mixed fire/smoke example | 1.4454126e-6 | 0.0010681152 |

The backends agree on the mixed-scene smoke miss. This rules out a large OpenCV-vs-ORT forward-pass discrepancy on these two examples; it does not validate the model's real-world accuracy or explain the upstream annotation/model difference. `tests/check_runtime_parity.py` reproduces this optional development comparison.
