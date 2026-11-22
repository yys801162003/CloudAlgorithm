# Third-party components and license scope

The new `VC2019/SmokeDetector` application and its included trained model are distributed under AGPL-3.0; see `LICENSE-AGPL-3.0.txt`. This declaration covers this smoke-detection module, not unrelated pre-existing repository modules.

## Trained firewatch YOLO11s model

- Author / copyright: Max Nikonov (leskoont), 2025, with Ultralytics YOLO11s architecture and pretrained lineage.
- Source: https://github.com/leskoont/firewatch
- Release: https://github.com/leskoont/firewatch/releases/tag/v0.1.0
- The included ONNX file is unmodified. Its SHA256 and input/output contract are recorded in `models/model.json` and `models/SHA256SUMS.txt`.
- Upstream identifies both the model and code using it as AGPL-3.0. The model's ONNX metadata also records AGPL-3.0 and Ultralytics version 8.4.183.
- Training/export scripts and configuration from upstream revision `b63a106e3554a8cfcdbd491fdde288ec80d3a759` are preserved in `model-source/`. The complete corresponding upstream source and package declarations are available at https://github.com/leskoont/firewatch/tree/b63a106e3554a8cfcdbd491fdde288ec80d3a759 . These scripts are supplied for provenance and are not executed by this C++ application.
- Ultralytics architecture/training source: https://github.com/ultralytics/ultralytics/tree/v8.4.183 ; licensed by its authors under AGPL-3.0 / their separately available commercial terms. We do not redistribute a Python runtime or claim a separate commercial grant.
- Training-data provenance is described by upstream as D-Fire (CC0 1.0): https://github.com/gaiasd/DFireDataset . D-Fire citation: de Venancio, Lisboa, Barbosa, Neural Computing and Applications, 2022. No raw training dataset is redistributed here.

## Example photos

`samples/upstream_smoke_annotated.jpg` and `samples/upstream_mixed_missed_smoke.jpg` are unmodified upstream `docs/example_2.jpg` and `docs/example_3.jpg` from the revision above, provided with the upstream repository's AGPL-3.0 notices. Upstream describes its documentation examples as AoF images from the authors' own cameras. They already contain upstream annotations. These are functional examples, not an independent held-out accuracy benchmark.

## OpenCV 4.10.0

OpenCV is Apache-2.0 licensed; see `licenses/OpenCV-4.10-LICENSE.txt` and https://github.com/opencv/opencv/tree/4.10.0 . This source ZIP does not bundle the OpenCV binaries. `scripts/Setup-OpenCV.ps1` fetches the official unmodified Windows package, verifies its SHA256, and extracts it under `third_party/`. Preserve the OpenCV package's own dependency notices when redistributing its runtime libraries.

The OpenCV-based heuristic in this application is separate from the YOLO neural inference, even though both use OpenCV as their C++ library.
