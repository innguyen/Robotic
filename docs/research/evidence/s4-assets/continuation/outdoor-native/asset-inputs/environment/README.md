# Asset-based SAR development scenes

The input selection is **CC0-1.0**. License verified on 2026-11-26 against the [Poly Haven asset license](https://polyhaven.com/license). No Fab or UnrealZoo/RescueBench source-asset entitlement was established during this run; no cooked scene pack was downloaded or extracted. This selection contains three individual meshes and three PBR texture sets at **1K**, exactly 14,594,662 source bytes (13.92 MiB), with a 32 MiB downloader cap.

| Input | Author | Use |
| --- | --- | --- |
| [Concrete Road Barrier 02](https://polyhaven.com/a/concrete_road_barrier_02) | Amal Kumar | Chipped concrete/rebar courtyard barrier; source mesh, base color, DX normal, roughness |
| [Painted Wooden Chair 02](https://polyhaven.com/a/painted_wooden_chair_02) | Kirill Sannikov | Worn indoor furniture and seated support |
| [Wooden Crate 01](https://polyhaven.com/a/wooden_crate_01) | James Ray Cock | Salvage/loading clutter |
| [Concrete Debris](https://polyhaven.com/a/concrete_debris) | Amal Kumar | Rubble and outdoor ground base color/normal/roughness |
| [Broken Brick Wall](https://polyhaven.com/a/broken_brick_wall) | Amal Kumar | Damaged masonry surfaces |
| [Cracked Concrete](https://polyhaven.com/a/cracked_concrete) | Dimitrios Savva | Slabs and indoor floor |

[lock.json](lock.json) pins download URL, exact byte size, upstream MD5 and locally measured SHA256 for every input. [attribution.json](attribution.json) retains official author/dimension metadata. The API's advertised polycount can differ from the supplied glTF triangle count; [meshes.json](meshes.json) records **actual baked** vertices/triangles/bounds. No 4K/8K/16K texture set, archive or environment pack is downloaded by the restore command.

`bake_environment.py` applies glTF hierarchy transforms and UV conversion, recenters props, and emits OBJ bytes. It also authors original solid fractured wall profiles, bevelled rubble fragments, a fallen slab, bent steel bars and metric-UV structural panels. These original meshes and their recipes are dedicated to **CC0-1.0**. UVs on original structures repeat every two metres. Meshes have real thickness/fracture silhouettes; textures are external PBR maps, not procedural noise or generated images. No engine mesh is redistributed.

`asset_scene.py` imports content-addressed meshes/textures/materials and connects base color, DX tangent normal and roughness. Structural meshes use native complex rays for scan/depth agreement and imported convex collision for robot sweeps. Authoring/admission uses conservative AABBs. During asset-scene motion, [UE component overlap queries](https://dev.epicgames.com/documentation/en-us/unreal-engine/python-api/class/SystemLibrary?application_version=5.6) check the actual imported collision hull at the rotated chassis pose; SAT remains for solid cuboids. An irregular mesh's empty AABB corner is not itself occupied geometry. A deliberate embedded-mesh reset probe must be rejected, and a subsequent valid reset must work. Concave rubble/wall apertures remain **uncertified** traversable surfaces. Bent reinforcement and the victim's seat are explicitly visual only; their collision limitation is not hidden.

The indoor scene is a roof-damaged two-bay building with a search partition, ceiling joists, fallen slab, masonry piles, furniture and a collapsed entry. The semi-outdoor scene is an open service courtyard with a broken facade, loading island, barrier, retaining walls, debris and crates. They have different topology/target placement and share an asset library. They are **two authored development layouts**, not two statistically independent held-out asset lineages or a large disaster corpus. There is no dynamic collapse, smoke transport, vegetation simulation or destruction physics.

Run `python experiments/ue5_s4/prepare_environment.py`, then `python -m experiments.ue5_s4.bake_environment` from the repository root. Restoration validates the committed lock and also restores the [MakeHuman inputs](../anatomy/README.md). `--create-lock` is a maintainer operation that refuses to replace an existing lock. The runner never downloads assets.

Run input bytes and manifests are copied to each new run's `asset-inputs/` before UE starts; UE imports those frozen bytes. Generated UE `.uasset`, texture/shader/DDC caches remain ignored. Raw PNG/EXR evidence, lighting, performance and limitations are documented in the [S4 audit](../../../../docs/research/s4-audit.md).
