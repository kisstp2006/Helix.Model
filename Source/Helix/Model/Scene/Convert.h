#pragma once

#include <Helix/Model/Scene/Scene.h>

namespace helix::model
{
    // The engine's frame: y up, a model's front facing +z, its right toward -x as seen from the front (left handed,
    // x right, y up, z forward).
    inline constexpr AxisSystem kEngineAxes = {1, 2, 0, 1, 1, -1};

    struct ConvertOptions
    {
        // The importer's Scale Factor, applied on top of the unit conversion.
        float scaleFactor = 1;
        // Convert Units: the source's units to meters (scenes that do not say keep theirs).
        bool convertUnits = true;
    };

    // Turns a scene into the engine's frame: up, front and right of the source onto the engine's (a mirror for the right
    // handed sources of every format), and its units to meters. Positions, normals, tangents (and their handedness),
    // winding, node transforms, inverse bind matrices, morph deltas, animation keys and tangents, camera planes and
    // light ranges all follow. Cameras and lights keep the direction they look in their source's convention (glTF -z,
    // FBX +x). Converting a converted scene again only applies the scale factor.
    void ConvertScene(Scene& scene, const ConvertOptions& options = {});
}
