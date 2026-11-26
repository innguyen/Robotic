# Asset-based SAR development scenes

The **CC0-1.0** site library contains three meshes, six individual 1K PBR sets and a 1K daylight HDR panorama, with 34 locked input files. Local restoration enforces byte limits and verifies the hashes. We could establish redistribution rights for these individual assets. No source-asset entitlement for Fab, UnrealZoo or RescueBench was established. [Poly Haven license](https://polyhaven.com/license)

| Input | Author | Use |
| --- | --- | --- |
| [Concrete Road Barrier 02](https://polyhaven.com/a/concrete_road_barrier_02) | Amal Kumar | Chipped concrete/rebar courtyard barrier; source mesh, base color, DX normal, roughness |
| [Painted Wooden Chair 02](https://polyhaven.com/a/painted_wooden_chair_02) | Kirill Sannikov | Worn indoor furniture and seated support |
| [Wooden Crate 01](https://polyhaven.com/a/wooden_crate_01) | James Ray Cock | Salvage/loading clutter |
| [Concrete Debris](https://polyhaven.com/a/concrete_debris) | Amal Kumar | Rubble and outdoor ground base color/normal/roughness |
| [Broken Brick Wall](https://polyhaven.com/a/broken_brick_wall) | Amal Kumar | Damaged masonry surfaces |
| [Cracked Concrete](https://polyhaven.com/a/cracked_concrete) | Dimitrios Savva | Slabs and indoor floor |
| [Plastered Wall 02](https://polyhaven.com/a/plastered_wall_02) | Charlotte Baglioni | Interior partitions and office elevations |
| [Corrugated Iron 03](https://polyhaven.com/a/corrugated_iron_03) | Charlotte Baglioni | Loading doors, shed and canopy steel |
| [Asphalt 04](https://polyhaven.com/a/asphalt_04) | Sergej Majboroda, Jenelle van Heerden | Service apron and road |
| [Kloofendal 48d Partly Cloudy](https://polyhaven.com/a/kloofendal_48d_partly_cloudy_puresky) | Greg Zaal, Jarod Guest | HDR sky and fixed environment lighting |

[lock.json](lock.json) pins download URL, exact byte size, upstream MD5 and locally measured SHA256 for every input. [attribution.json](attribution.json) retains official author/dimension metadata. The API's advertised polycount can differ from the supplied glTF triangle count; [meshes.json](meshes.json) records **actual baked** vertices/triangles/bounds. No 4K/8K/16K texture set, archive or environment pack is downloaded by the restore command.

`bake_environment.py` applies glTF hierarchy transforms and UV conversion, recenters props, and emits OBJ bytes. It also authors original solid fractured wall profiles, bevelled rubble fragments, a fallen slab, bent steel bars and metric-UV structural panels. These original meshes and their recipes are dedicated to **CC0-1.0**. UVs on original structures repeat every two metres. Meshes have real thickness/fracture silhouettes; textures are external PBR maps, not procedural noise or generated images. No engine mesh is redistributed.

`asset_scene.py` imports content-addressed meshes/textures/materials and connects base color, DX tangent normal and roughness. Structural meshes use native complex rays for scan/depth agreement and imported convex collision for robot sweeps. Authoring/admission uses conservative AABBs. During asset-scene motion, [UE component overlap queries](https://dev.epicgames.com/documentation/en-us/unreal-engine/python-api/class/SystemLibrary?application_version=5.6) check the actual imported collision hull at the rotated chassis pose; SAT remains for solid cuboids. An irregular mesh's empty AABB corner is not itself occupied geometry. A deliberate embedded-mesh reset probe must be rejected, and a subsequent valid reset must work. Concave rubble/wall apertures remain **uncertified** traversable surfaces. Bent reinforcement and the victim's seat are explicitly visual only; their collision limitation is not hidden.

Site revision 2 adds an office wing, a fractured reinforced roof with loose cables, door and window reveals, remaining plaster and an annex around the indoor search rooms. A road, footways, sawtooth-roof shed, recessed loading doors, bent streetlamp, loading platforms and works offices surround the outdoor search area. Both layouts use the original CC0 metric meshes in `site_architecture.py`. Building assemblies keep triangle collision to preserve openings. The two layouts share materials and a single human identity. The benchmark models static damage. The prior revision's images and run snapshots remain available for comparison.

Run `python experiments/ue5_s4/prepare_environment.py`, then `python -m experiments.ue5_s4.bake_environment` from the repository root. Restoration validates the committed lock and also restores the [MakeHuman inputs](../anatomy/README.md). `--create-lock` is a maintainer operation that refuses to replace an existing lock. The runner never downloads assets.

Run input bytes and manifests are copied to each new run's `asset-inputs/` before UE starts; UE imports those frozen bytes. Generated UE `.uasset`, texture/shader/DDC caches remain ignored. Raw PNG/EXR evidence, lighting, performance and limitations are documented in the [S4 audit](../../../../docs/research/s4-audit.md).
