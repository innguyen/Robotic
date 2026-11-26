# Anatomical development avatar

The base OBJ, default rig and skin weights come from [MakeHuman source commit a8bc2d54ff0ac92e78ff71431b1023eda42bf482](https://github.com/makehumancommunity/makehuman/tree/a8bc2d54ff0ac92e78ff71431b1023eda42bf482). Their graphical-data license is **CC0-1.0**, verified against the [official license](https://static.makehumancommunity.org/about/license.html), the source headers, rig/weight license fields and the retained [asset legal statement](source/LICENSE.ASSETS.md). The application's program logic has a separate AGPL license; no MakeHuman application code is vendored or executed here.

[lock.json](lock.json) records exact URLs, SHA256 and sizes. Copyright notices in the source are retained: Data Collection AB, Joel Palmius and Jonas Hauquier, with the asset-specific dates stated in the headers. This does not relicense unrelated code in Robotic, Cesium Man, YOLO or UE.

The first anatomical revision in `derived/` has 13,380 vertices and represents clothing with body material regions. In the current site revision, `dressed_human.py` fits separate shirt, trousers, loafers, hair and eye geometry to the HM08 body through the original bindings. The bake preserves garment and skin UVs across standing, seated and prone poses. Each exported mesh has 14,307 vertices and 27,966 triangles. [Dressed pose bounds and hashes](dressed/poses.json)

The five MakeHuman Community assets by MargaretToigo are Light Skin Male Bronze (1136), T-shirt Basic Tucked (1203), Wool Pants (1194), Men's Loafers (1732) and Blunt Bob (1669). The lock records the CC0 catalogue entries and exact per-file sources. The eye data comes from the pinned MakeHuman base commit. Garment candidates with conflicting file headers were excluded. UE samples the six material slots at a maximum texture dimension of 1024. [Community catalogue](https://files.makehumancommunity.org/sites/default/files/assets.json)

The origin remains the **posed torso joint**, not the body centroid or a detector prediction. [derived/poses.json](derived/poses.json) records exact mesh hashes, torso heights and bounds relative to the anchor; lower Z plus anchor height is zero. UE rotates the body about that origin, and private isolated-depth rendering uses the exact actor mesh. Public localization still uses RGB boxes, measured optical depth and estimated odometry; no joint/anchor metadata reaches the policy.

Restore inputs with `python experiments/ue5_s4/prepare_environment.py`, then bake with `python -m experiments.ue5_s4.bake_anatomy`. The legacy [Cesium Man derivatives](../README.md) remain unchanged and are used by `--scene-set legacy`. The new avatar is selected only with `--scene-set assets`.

The avatar represents one neutral identity with static poses. Facial expression, injury diversity, cloth dynamics and held-out identity validation remain outside the measured scope. Seated support, prone floor contact and visibility can be inspected in the native RGB/depth captures.
