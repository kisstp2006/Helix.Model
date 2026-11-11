# Helix.Model

Model reading of the Helix engine: glTF 2.0 and GLB, FBX (binary and ASCII), Wavefront OBJ and MTL, read into one in
memory scene that the model importer turns into meshes, materials, skeletons and clips, the conversion of that scene
to the engine's axes and units, and the mesh processing the importer runs on it. All code is own; JSON comes from Helix.Serialization, inflate from Helix.Image.

## Contents

| Path | Content |
|---|---|
| `Source/Helix/Model/Scene/Scene.h` | `Scene`: nodes, meshes, primitives, vertex streams, materials, textures, images, skins, clips, cameras, lights |
| `Source/Helix/Model/Gltf/Gltf.h` | `ReadGltf`, `IsGlb`, `ReadOptions` (the loader of external files), `SupportedGltfExtensions` |
| `Source/Helix/Model/Obj/Obj.h` | `ReadObj`, `ReadMtl` |
| `Source/Helix/Model/Fbx/Fbx.h` | `ReadFbx`, `IsBinaryFbx` |
| `Source/Helix/Model/Fbx/Document.h` | `fbx::Document`: the node tree of a binary or ASCII FBX file |
| `Source/Helix/Model/Scene/Convert.h` | `ConvertScene`, `ConvertOptions`, `kEngineAxes` |
| `Source/Helix/Model/Mesh/Mesh.h` | Welding, normals, tangents, cache / overdraw / fetch optimization, simplification, meshlets, blend shape deltas |
| `Tests/Support/GltfWriter.h` | Builds glTF and GLB files for the tests: binary data, buffer views, accessors, the JSON |
| `Tests/Support/FbxWriter.h` | Builds FBX files for the tests: a node tree as binary 7400 / 7500 (raw or zlib arrays) or ASCII |
| `Tests/Support/SampleMeshes.h` | Generated grids, UV spheres (with their seams) and cubes for the mesh processing |
| `Tests/`, `Benchmarks/`, `Fuzz/` | Every accessor form, extension and OBJ statement, errors with their paths and lines; a large grid |

## The scene

Everything is a flat array; objects refer to each other by index and keep lists as ranges into pools of the scene.
Every vertex attribute is float (`Scene::floats`), whatever the file stored: normalized integers are scaled to their
range, quantized attributes keep their values. A primitive has its streams (semantic, set, components, offset), its
indices (`Scene::indices`), its material and morph targets (delta streams). Materials carry the metallic roughness
model, the KHR_materials_* parameters and, for formats with one map per parameter, separate roughness, metallic,
alpha, bump, displacement and classic specular maps. Images stay as URIs or embedded bytes. Nothing is converted to the
engine's axes or units by the readers: `Scene::axes` (up, front and right with their signs, as FBX names them),
`unitScale` and `frameRate` tell what the source used, and readers record what they skipped in `Scene::warnings`.

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

## FBX

`ReadFbx` reads binary files of versions 7100 to 7700 (32 bit records, 64 bit from 7500; arrays raw or zlib) and ASCII
7.x files into `fbx::Document`, then the object graph and its connections into the scene:

- Models become nodes. Their whole transform (translation, rotation in its rotation order, scaling, pre and post
  rotation, rotation and scaling pivots and offsets) is reduced to translation, rotation and scale.
- Meshes come from the geometry with every layer element mapping (by polygon vertex, by vertex, by polygon, by edge,
  all same; direct or indexed): normals, tangents (handedness from the binormals), every UV set by name, colors, and
  one primitive per material of the model. Polygons become fans; the geometric transform is baked into the vertices.
- Materials take the Lambert and Phong properties (diffuse, emissive with its factor, specular, shininess as roughness,
  opacity or transparency) and their textures by property. Embedded video content (raw or base64) becomes image bytes.
- Skins: clusters give joints and weights streams, four influences a set, as many sets as a vertex needs, normalized.
  Bind matrices come from TransformLink and Transform.
- Blend shapes give morph targets with their channel and in-between full weights; DeformPercent gives the default
  weights.
- Animation stacks become clips: the base layer's curves (constant, linear, cubic with their tangents), evaluated
  through the whole transform and resampled at the file's frame rate (TimeMode or the custom rate).
- Cameras and lights come from their node attributes.

## Axes and units

```cpp
ConvertOptions options;
options.scaleFactor = 1;     // the importer's Scale Factor
options.convertUnits = true; // Convert Units
ConvertScene(scene, options);
```

The engine is y up with a model's front toward +z and its right toward -x as seen from the front (`kEngineAxes`).
Sources are mapped by their up, front and right axes, so right handed sources (glTF, OBJ, Maya and Blender FBX) are
mirrored. Positions, normals, tangents and their handedness, winding (triangles, strips, fans), node transforms, inverse
bind matrices, morph deltas, animation keys, camera planes and light ranges follow; units become meters times the scale
factor.

## Mesh processing

Every function works on a primitive of the scene and keeps all its streams and morph targets in step; new stream data
is appended to `Scene::floats` and `Compact` drops what nothing uses any more.

```cpp
WeldVertices(scene, p);                        // vertices equal in every stream merged
CalculateNormals(scene, p, 60);                // smoothing angle, area and angle weighted
CalculateTangents(scene, p);                   // MikkTSpace, bitangent sign in w
OptimizeVertexCache(scene, p);                 // or OptimizeOverdraw(scene, p, 1.05f)
OptimizeVertexFetch(scene, p);
Simplify(scene, p, 0.25f, 0.01f);              // a quarter of the triangles, at most 1 % of the size away
Compact(scene);
```

- Strips and fans become lists; triangles that use indices outside the vertices are dropped by the functions that
  need triangles.
- Normals sum the face normals around each position within the smoothing angle (area and angle, area, angle or
  unweighted); vertices split where their corners differ, so seams of other streams keep one smooth normal.
- Tangents follow MikkTSpace: per triangle derivatives along u signed by the UV orientation, projected into each
  corner's normal plane and summed by angle over the corners of one vertex and orientation.
- The vertex cache order is Forsyth's linear speed algorithm; the overdraw order splits it into clusters and draws
  them outside in, within the given loss of cache efficiency. `AnalyzeVertexCache` (FIFO ACMR and ATVR) and
  `AnalyzeOverdraw` (a software rasterizer from six directions) measure both.
- Simplification collapses edges by quadric error onto existing vertices: borders only along themselves, vertices on
  attribute seams and non-manifold edges stay, collapses that would turn a triangle are refused. `SimplifyIndices`
  works on plain arrays for LODs.
- `BuildMeshlets` grows meshlets of up to 64 vertices and 124 triangles along shared vertices, with bounding spheres
  and normal cones.
- `CompressDeltas` keeps the vertices a blend shape moves, 16 bits a component with one scale, within half a step.

On a scrambled grid the cache order brings a 16 entry FIFO from 3.0 to 0.68 vertices per triangle; the overdraw
order of four overlapping spheres shades 1.09 pixels per covered pixel against 1.24 for the cache order alone. A UV
sphere of 11904 indices reaches a quarter at 0.4 % of its size.

## Speed

Release, one thread, a grid of 131072 triangles with positions, normals and texture coordinates: GLB in about 6 ms
(45 ns a triangle), OBJ (7 MB of text) in about 70 ms, FBX 7500 with zlib arrays in about 85 ms.

Mesh processing of a UV sphere of 130560 triangles: welding about 8 ms, normals and tangents about 40 to 60 ms each,
vertex cache order about 90 ms, overdraw order about 100 ms, fetch order about 10 ms, simplification to a tenth about
140 ms, meshlets about 50 ms.

## Building

The repository builds with the presets of Helix.CMake (Windows, Web, Android); the submodules in `External/` are
Helix.CMake, Helix.Testing, Helix.Core, Helix.Serialization, Helix.Image and the libraries Helix.Image uses
(Helix.Platform, Helix.Profiler, Helix.Jobs).

## License

GPL-3.0, see `LICENSE`.
