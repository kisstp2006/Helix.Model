#pragma once

#include <Helix/Model/Gltf/Gltf.h>

namespace helix::model
{
    bool IsBinaryFbx(Span<const u8> data);

    // Reads FBX (binary 7100 to 7700, ASCII 7.x) into `out` (cleared first):
    // - models as nodes, their whole transform (translation, rotation in its order, scaling, pre and post rotation,
    //   rotation and scaling pivots and offsets) reduced to local translation, rotation and scale;
    // - meshes from the geometry and every layer element mapping, one primitive per material of the model, the
    //   geometric transform baked into the vertices;
    // - materials (Lambert and Phong properties) with textures by property and embedded video content;
    // - skins (clusters to joints and weights streams, four a set) and blend shapes with in-between shapes;
    // - animation stacks as clips, the base layer resampled at the file's frame rate through the whole transform;
    // - cameras and lights.
    // The axes and unit scale of GlobalSettings are recorded in the scene; ConvertScene turns them into the engine's.
    // Errors name the byte offset or line of the document, or the object.
    bool ReadFbx(Span<const u8> data, Scene& out, String& error, const ReadOptions& options = {});
}
