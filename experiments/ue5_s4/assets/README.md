# Human and detector provenance

**Cesium Man**, copyright 2017 Cesium, **CC BY 4.0**, from the official Khronos glTF Sample Assets repository at commit `e124e8d8d7330a5372bc2b4d00d39a4a9d7997ea`.

Source: <https://github.com/KhronosGroup/glTF-Sample-Assets/tree/e124e8d8d7330a5372bc2b4d00d39a4a9d7997ea/Models/CesiumMan>. License: <https://creativecommons.org/licenses/by/4.0/>. Original legal notice is retained in `source/CesiumMan-LICENSE-source.md`. The original source texture contains a Cesium logo with separate trademark terms; **the rendered derivatives replace that texture entirely** and imply no endorsement.

Changes: skinning/pose baking with natural arms, standing/seated/prone legs/body, glTF-to-UE coordinate conversion, normalization to a 1.75 m standing body, origin at torso, and original replacement clothing/skin materials. Derived OBJ files retain attribution. These are **stylized human meshes**, not scans of actual victims or a claim of photorealistic human appearance. One identity cannot establish demographic or held-out identity robustness.

**YOLOv5n ONNX**, official Ultralytics `v7.0` release, source commit `915bbf294bb74c859f0b41f1c23bc395014ea679`. The pinned source declares **GPL-3.0**, retained at `source/YOLOv5-v7-LICENSE.txt`. Download: <https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx>. Used for local COCO-person inference. No claim that the whole repository or all upstream dependencies share this license. Weights are ignored, not copied over production weights; `prepare_assets.py` restores the exact input and records SHA256.

Run `prepare_assets.py`, then `bake_humans.py`. `manifest.json` records upstream URLs, sizes and hashes; `derived/poses.json` records derived geometry hashes and torso heights. UE imports use content-derived asset names and validate OBJ hashes before use. Generated `.uasset`/shader caches stay ignored. Scene geometry, procedural PBR noise, clothing materials and robot visual recipe are original code in this repository. Engine Cube/Sphere/Cylinder remain in the licensed local UE installation and are not redistributed here.
