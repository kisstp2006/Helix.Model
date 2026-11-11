#pragma once

#include <Helix/Model/Gltf/Gltf.h>

namespace helix::model
{
    // Reads Wavefront OBJ into `out` (cleared first): v (with vertex colors), vt, vn, f (any size, triangulated as fans,
    // negative indices), l, p, o and g (each a node and mesh), usemtl (a primitive per material), mtllib through the
    // loader. Vertices are shared by their position, texture and normal indices. Statements of free form geometry are
    // skipped. Errors name the line.
    bool ReadObj(Span<const u8> data, Scene& out, String& error, const ReadOptions& options = {});

    // Reads the materials of an MTL file into `scene` (appending; materials of a name already there are replaced) with
    // its textures and images: Ka / Kd / Ks / Ke, Ns, Ni, d / Tr, illum, the maps with -s, -o, -bm, -clamp and the
    // other options, and the PBR keys (Pr, Pm, Ps, Pc, Pcr, aniso, anisor, map_Pr, map_Pm, map_Ps, norm).
    bool ReadMtl(StringView text, Scene& scene, String& error);
}
