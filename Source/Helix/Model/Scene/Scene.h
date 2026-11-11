#pragma once

#include <Helix/Core/Containers/Array.h>
#include <Helix/Core/Strings/String.h>

// The in memory scene every model reader fills (glTF, OBJ, FBX): flat arrays that refer to each other by index, every
// list a range into a pool of the scene, every vertex attribute float. Nothing is converted to the engine's axes or
// units here; the source's are recorded for the importer.
namespace helix::model
{
    inline constexpr u32 kNone = ~0u;

    struct Range
    {
        u32 begin = 0;
        u32 count = 0;
    };

    struct Node
    {
        String name;
        u32 parent = kNone;
        // Into Scene::nodeChildren.
        Range children;
        float translation[3] = {0, 0, 0};
        // x, y, z, w.
        float rotation[4] = {0, 0, 0, 1};
        float scale[3] = {1, 1, 1};
        // The node gave a matrix (column major); translation, rotation and scale are its decomposition.
        bool hasMatrix = false;
        float matrix[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        u32 mesh = kNone, skin = kNone, camera = kNone, light = kNone;
        // Morph weights of the node (into Scene::floats), empty for the mesh's defaults.
        Range weights;
    };

    enum class Semantic : u8
    {
        Position,
        Normal,
        Tangent,
        TexCoord,
        Color,
        Joints,
        Weights,
        // Attributes of other names (glTF "_CUSTOM"), named in VertexStream::name.
        Custom,
    };

    // One attribute of a primitive: `components` floats per vertex in Scene::floats from `offset`.
    struct VertexStream
    {
        Semantic semantic = Semantic::Position;
        u32 set = 0;
        u32 components = 3;
        u32 offset = 0;
        String name;
    };

    enum class Topology : u8
    {
        Points,
        Lines,
        LineLoop,
        LineStrip,
        Triangles,
        TriangleStrip,
        TriangleFan,
    };

    struct MorphTarget
    {
        String name;
        // Deltas of position, normal and tangent (into Scene::streams).
        Range streams;
    };

    struct Primitive
    {
        Topology topology = Topology::Triangles;
        u32 vertexCount = 0;
        // Into Scene::streams.
        Range streams;
        // Into Scene::indices; empty when the vertices are used in order.
        Range indices;
        u32 material = kNone;
        // Into Scene::morphTargets.
        Range targets;
        // KHR_materials_variants: pairs of variant and material (into Scene::variantMappings).
        Range variants;
    };

    struct Mesh
    {
        String name;
        // Into Scene::primitives.
        Range primitives;
        // Default morph weights (into Scene::floats).
        Range weights;
    };

    enum class Filter : u8
    {
        Unset,
        Nearest,
        Linear,
        NearestMipmapNearest,
        LinearMipmapNearest,
        NearestMipmapLinear,
        LinearMipmapLinear,
    };

    enum class Wrap : u8
    {
        Repeat,
        ClampToEdge,
        MirroredRepeat,
    };

    struct Image
    {
        String name;
        // A path relative to the model file, empty for embedded images.
        String uri;
        String mimeType;
        // Embedded bytes (a buffer view or a data URI) in Scene::bytes.
        Range bytes;
    };

    struct Texture
    {
        String name;
        u32 image = kNone;
        Filter magFilter = Filter::Unset, minFilter = Filter::Unset;
        Wrap wrapS = Wrap::Repeat, wrapT = Wrap::Repeat;
    };

    // A texture slot of a material: the texture, its UV set and the KHR_texture_transform.
    struct TextureRef
    {
        u32 texture = kNone;
        u32 texCoord = 0;
        float offset[2] = {0, 0};
        float scale[2] = {1, 1};
        float rotation = 0;
        // The normal scale, the occlusion strength or the bump multiplier.
        float strength = 1;
        bool Present() const { return texture != kNone; }
    };

    enum class AlphaMode : u8
    {
        Opaque,
        Mask,
        Blend,
    };

    struct Material
    {
        String name;
        float baseColor[4] = {1, 1, 1, 1};
        TextureRef baseColorTexture;
        float metallic = 1, roughness = 1;
        TextureRef metallicRoughnessTexture;
        TextureRef normalTexture;
        TextureRef occlusionTexture;
        float emissive[3] = {0, 0, 0};
        TextureRef emissiveTexture;
        AlphaMode alphaMode = AlphaMode::Opaque;
        float alphaCutoff = 0.5f;
        bool doubleSided = false;
        bool unlit = false;
        // KHR_materials_*: each present only when the file uses it.
        float emissiveStrength = 1;
        float ior = 1.5f;
        bool hasTransmission = false;
        float transmission = 0;
        TextureRef transmissionTexture;
        bool hasVolume = false;
        float thickness = 0;
        TextureRef thicknessTexture;
        float attenuationDistance = 0;
        float attenuationColor[3] = {1, 1, 1};
        bool hasClearcoat = false;
        float clearcoat = 0, clearcoatRoughness = 0;
        TextureRef clearcoatTexture, clearcoatRoughnessTexture, clearcoatNormalTexture;
        bool hasSheen = false;
        float sheenColor[3] = {0, 0, 0};
        float sheenRoughness = 0;
        TextureRef sheenColorTexture, sheenRoughnessTexture;
        bool hasSpecular = false;
        float specular = 1;
        float specularColor[3] = {1, 1, 1};
        TextureRef specularTexture, specularColorTexture;
        bool hasIridescence = false;
        float iridescence = 0, iridescenceIor = 1.3f, iridescenceThicknessMin = 100, iridescenceThicknessMax = 400;
        TextureRef iridescenceTexture, iridescenceThicknessTexture;
        bool hasAnisotropy = false;
        float anisotropyStrength = 0, anisotropyRotation = 0;
        TextureRef anisotropyTexture;
        float dispersion = 0;
        // Maps of formats with one texture per parameter (OBJ, FBX), and the classic specular color and exponent.
        TextureRef roughnessTexture, metallicTexture, alphaTexture, bumpTexture, displacementTexture;
        float shininess = 0;
        float specularColorClassic[3] = {0, 0, 0};
        TextureRef specularColorClassicTexture, shininessTexture;
    };

    struct Skin
    {
        String name;
        // Nodes, into Scene::skinJoints.
        Range joints;
        // 16 floats per joint (column major) in Scene::floats; empty for identities.
        Range inverseBindMatrices;
        u32 skeleton = kNone;
    };

    enum class AnimationPath : u8
    {
        Translation,
        Rotation,
        Scale,
        Weights,
    };

    enum class Interpolation : u8
    {
        Step,
        Linear,
        CubicSpline,
    };

    struct AnimationChannel
    {
        u32 node = kNone;
        AnimationPath path = AnimationPath::Translation;
        Interpolation interpolation = Interpolation::Linear;
        // Key times in seconds and values in Scene::floats; cubic splines hold in tangent, value, out tangent per key.
        Range times;
        Range values;
        // Floats per value (3, 4, or the morph target count).
        u32 components = 3;
    };

    struct AnimationClip
    {
        String name;
        // Into Scene::channels.
        Range channels;
        float duration = 0;
    };

    struct Camera
    {
        String name;
        bool orthographic = false;
        // Radians; aspect 0 when the file leaves it to the viewport.
        float yfov = 0.8f, aspectRatio = 0;
        float xmag = 1, ymag = 1;
        // zfar 0: infinite.
        float znear = 0.01f, zfar = 0;
    };

    enum class LightType : u8
    {
        Directional,
        Point,
        Spot,
    };

    struct Light
    {
        String name;
        LightType type = LightType::Point;
        float color[3] = {1, 1, 1};
        float intensity = 1;
        // 0: no range.
        float range = 0;
        float innerConeAngle = 0, outerConeAngle = 0.7853982f;
    };

    struct Scene
    {
        // Where the scene came from, for the importer's conversion.
        enum class Handedness : u8
        {
            Right,
            Left,
        };
        Handedness handedness = Handedness::Right;
        // 1 = y up, 2 = z up.
        u8 upAxis = 1;
        // Meters per unit; 0 when the format does not say.
        float unitScale = 1;

        Array<Node> nodes;
        Array<u32> roots;
        Array<Mesh> meshes;
        Array<Primitive> primitives;
        Array<VertexStream> streams;
        Array<MorphTarget> morphTargets;
        Array<Material> materials;
        Array<String> variants;
        // Pairs of variant and material index.
        Array<u32> variantMappings;
        Array<Texture> textures;
        Array<Image> images;
        Array<Skin> skins;
        Array<AnimationClip> clips;
        Array<AnimationChannel> channels;
        Array<Camera> cameras;
        Array<Light> lights;

        // What the reader skipped or could not load, one line each (missing material libraries, unknown statements).
        Array<String> warnings;

        Array<u32> nodeChildren;
        Array<u32> skinJoints;
        Array<u32> indices;
        Array<float> floats;
        Array<u8> bytes;

        void Clear() { *this = Scene(); }
        // The stream of a primitive, null when it has none.
        const VertexStream* Find(const Primitive& p, Semantic semantic, u32 set = 0) const;
        const float* Data(const VertexStream& s) const { return floats.Data() + s.offset; }
        // Triangles of all triangle primitives (strips and fans unrolled).
        u64 TriangleCount() const;
    };
}
