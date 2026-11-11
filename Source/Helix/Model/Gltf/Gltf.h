#pragma once

#include <Helix/Core/Basics/Span.h>
#include <Helix/Model/Scene/Scene.h>

namespace helix::model
{
    // Loads a file a model refers to, by its URI relative to the model (already percent decoded). False when it
    // cannot.
    using ResourceLoader = bool (*)(void* context, StringView uri, Array<u8>& out);

    struct ReadOptions
    {
        ResourceLoader loader = nullptr;
        void* context = nullptr;
    };

    // The extensions ReadGltf understands; a file that requires any other is refused.
    Span<const StringView> SupportedGltfExtensions();

    bool IsGlb(Span<const u8> data);

    // Reads glTF 2.0 JSON or GLB into `out` (cleared first): buffers from the GLB BIN chunk, data URIs or the loader;
    // every accessor form (normalized, sparse, strided, KHR_mesh_quantization), materials with KHR_materials_* and
    // KHR_texture_transform, skins, morph targets, animations, cameras, KHR_lights_punctual, KHR_materials_variants.
    // Images stay as URIs or embedded bytes. Errors name the JSON path, as "meshes[0].primitives[1].indices".
    bool ReadGltf(Span<const u8> data, Scene& out, String& error, const ReadOptions& options = {});
}
