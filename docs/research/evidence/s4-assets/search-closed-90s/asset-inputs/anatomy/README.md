# Anatomical development avatar

The base OBJ, default rig and skin weights come from [MakeHuman source commit a8bc2d54ff0ac92e78ff71431b1023eda42bf482](https://github.com/makehumancommunity/makehuman/tree/a8bc2d54ff0ac92e78ff71431b1023eda42bf482). Their graphical-data license is **CC0-1.0**, verified against the [official license](https://static.makehumancommunity.org/about/license.html), the source headers, rig/weight license fields and the retained [asset legal statement](source/LICENSE.ASSETS.md). The application's program logic has a separate AGPL license; no MakeHuman application code is vendored or executed here.

[lock.json](lock.json) records exact URLs, SHA256 and sizes. Copyright notices in the source are retained: Data Collection AB, Joel Palmius and Jonas Hauquier, with the asset-specific dates stated in the headers. This does not relicense unrelated code in Robotic, Cesium Man, YOLO or UE.

`bake_anatomy.py` is an original offline skinning/baking implementation. It exports only body topology, excluding helper/genital/rig meshes, normalizes standing height to 1.75 m, shapes loose replacement work clothing, and bakes standing, seated and face-down prone poses. The body includes facial, hand and foot anatomy and has 13,380 vertices / 26,756 triangles. Seated arms are lowered toward the lap. Clothing and boot material regions are original; they are not scanned fabric or a complete garment model.

The origin remains the **posed torso joint**, not the body centroid or a detector prediction. [derived/poses.json](derived/poses.json) records exact mesh hashes, torso heights and bounds relative to the anchor; lower Z plus anchor height is zero. UE rotates the body about that origin, and private isolated-depth rendering uses the exact actor mesh. Public localization still uses RGB boxes, measured optical depth and estimated odometry; no joint/anchor metadata reaches the policy.

Restore inputs with `python experiments/ue5_s4/prepare_environment.py`, then bake with `python -m experiments.ue5_s4.bake_anatomy`. The legacy [Cesium Man derivatives](../README.md) remain unchanged and are used by `--scene-set legacy`. The new avatar is selected only with `--scene-set assets`.

This is a better anatomical **development avatar**, not a photorealistic scanned victim. It has one neutral identity, no skin/hair scan, no facial expression/injury corpus, no cloth/contact simulation and no held-out identity validation. Seated support and prone contact require visual review in the actual UE captures; anatomical mesh density alone is not an appearance acceptance test.
