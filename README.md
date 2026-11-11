# Helix.Model

Model reading of the Helix engine: glTF 2.0 and GLB, Wavefront OBJ and MTL, read into one in memory scene that the
model importer turns into meshes, materials, skeletons and clips. All code is own; JSON comes from Helix.Serialization.

## Contents

| Path | Content |
|---|---|
| `Source/Helix/Model/Scene/Scene.h` | `Scene`: nodes, meshes, primitives, vertex streams, materials, textures, images, skins, clips, cameras, lights |
| `Source/Helix/Model/Gltf/Gltf.h` | `ReadGltf`, `IsGlb`, `ReadOptions` (the loader of external files), `SupportedGltfExtensions` |
| `Source/Helix/Model/Obj/Obj.h` | `ReadObj`, `ReadMtl` |
| `Tests/Support/GltfWriter.h` | Builds glTF and GLB files for the tests: binary data, buffer views, accessors, the JSON |
| `Tests/`, `Benchmarks/`, `Fuzz/` | Every accessor form, extension and OBJ statement, errors with their paths and lines; a large grid |

## The scene

Everything is a flat array; objects refer to each other by index and keep lists as ranges into pools of the scene.
Every vertex attribute is float (`Scene::floats`), whatever the file stored: normalized integers are scaled to their
range, quantized attributes keep their values. A primitive has its streams (semantic, set, components, offset), its
indices (`Scene::indices`), its material and morph targets (delta streams). Materials carry the metallic roughness
model, the KHR_materials_* parameters and, for formats with one map per parameter, separate roughness, metallic,
alpha, bump, displacement and classic specular maps. Images stay as URIs or embedded bytes. Nothing is converted to the
engine's axes or units: `Scene::handedness`, `upAxis` and `unitScale` tell what the source used, and readers record
what they skipped in `Scene::warnings`.

## glTF

```cpp
using namespace helix::model;

ReadOptions options;
options.context = &folder;
options.loader = [](void* context, helix::StringView uri, helix::Array<u8>& out) { return ReadFileNextTo(context, uri, out); };
Scene scene;
helix::String error;
if (!ReadGltf(fileBytes, scene, error, options))
    Log(error);   // "meshes[0].primitives[1].indices: index 7 of 3 vertices"
```

- Buffers: the GLB BIN chunk, base64 data URIs, or the loader for external files (URIs percent decoded).
- Accessors: every component type, normalized, strided, sparse, without a buffer view, matrix column padding,
  KHR_mesh_quantization. Indices are read as exact integers.
- Materials: metallic roughness, normal, occlusion, emissive, alpha mode, double sided, KHR_texture_transform,
  KHR_materials_emissive_strength, ior, transmission, volume, clearcoat, sheen, specular, iridescence, anisotropy,
  dispersion, unlit and variants.
- Nodes (TRS, or a matrix with its decomposition), the default scene, skins with inverse bind matrices, morph targets
  with target names and weights, animations (step, linear, cubic spline; translation, rotation, scale, weights),
  cameras and KHR_lights_punctual.
- A file that requires an extension outside `SupportedGltfExtensions()` is refused, naming it. Every count, index,
  offset and size is checked; a node with two parents or a cycle of parents is an error.

## OBJ and MTL

`ReadObj` reads positions (with vertex colors), texture coordinates, normals, faces of any size (fans), negative
indices, lines and points; each `o` or `g` becomes a node and mesh, each `usemtl` a primitive, and vertices are shared by
their index triples. `mtllib` files come through the loader and `ReadMtl`: colors, Ns (as roughness unless Pr is
given), Ni, d / Tr, the PBR keys (Pr, Pm, Ps, Pc, Pcr, aniso, anisor) and the maps with their -s, -o, -bm and -clamp
options. A missing library is a warning, materials then keep only their names. Errors name the line.

## Speed

Release, one thread, a grid of 131072 triangles with positions, normals and texture coordinates: GLB in about 6 ms
(45 ns a triangle), OBJ (7 MB of text) in about 75 ms.

## Building

The repository builds with the presets of Helix.CMake (Windows, Web, Android); the submodules in `External/` are
Helix.CMake, Helix.Testing, Helix.Core and Helix.Serialization.

## License

GPL-3.0, see `LICENSE`.
