#include <Support/GltfWriter.h>

#include <Helix/Model/Gltf/Gltf.h>
#include <Helix/Testing/Test.h>

#include <cmath>
#include <cstdio>

using namespace helix;
using namespace helix::model;
using namespace helix::model::testing;

namespace
{
    bool Read(Span<const u8> bytes, Scene& s, const ReadOptions& options = {})
    {
        String error;
        if (!ReadGltf(bytes, s, error, options))
        {
            std::printf("    %s\n", error.CStr());
            return false;
        }
        return true;
    }

    bool Refused(StringView text, StringView expected)
    {
        Scene s;
        String error;
        if (ReadGltf(AsBytes(String(text)), s, error) || !error.View().StartsWith(expected))
        {
            std::printf("    %s (expected %.*s)\n", error.Empty() ? "read" : error.CStr(), int(expected.Size()), expected.Data());
            return false;
        }
        return true;
    }

    bool Near(float a, float b, float e = 1e-5f) { return std::fabs(a - b) <= e; }

    const float* Stream(const Scene& s, const Primitive& p, Semantic semantic, u32 set = 0)
    {
        const VertexStream* st = s.Find(p, semantic, set);
        return st ? s.Data(*st) : nullptr;
    }
}

HX_TEST(Gltf, EveryAccessorForm)
{
    GltfWriter w;
    // Interleaved float positions and byte normals (quantized: KHR_mesh_quantization), stride 16.
    struct Vertex
    {
        float p[3];
        i8 n[4];
    };
    Vertex vs[3] = {{{0, 0, 0}, {0, 0, 127, 0}}, {{1, 0, 0}, {0, -128, 0, 0}}, {{0, 1, 0}, {127, 0, 0, 0}}};
    u32 view = w.View(vs, sizeof(vs), 16);
    u32 pos = w.Accessor(view, 5126, 3, "VEC3");
    u32 nrm = w.Accessor(view, 5120, 3, "VEC3", true, 12);
    u32 uv = w.Data<u16>({0, 0, 65535, 0, 0, 32768}, 5123, "VEC2", 2, true);
    u32 color = w.Data<u8>({255, 0, 0, 255, 0, 255, 0, 128, 0, 0, 255, 0}, 5121, "VEC4", 4, true);
    u32 joints = w.Data<u8>({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 5121, "VEC4", 4);
    u32 weights = w.Data<u8>({255, 0, 0, 0, 128, 127, 0, 0, 85, 85, 85, 0}, 5121, "VEC4", 4, true);
    u32 i8s = w.Data<u8>({0, 1, 2}, 5121, "SCALAR", 1);
    u32 i16s = w.Data<u16>({2, 1, 0}, 5123, "SCALAR", 1);
    u32 i32s = w.Data<u32>({1, 2, 0}, 5125, "SCALAR", 1);
    // A sparse accessor over the positions: vertex 2 moves to (5, 6, 7).
    u32 si = w.View(std::initializer_list<u16>({2}).begin(), 2);
    u32 sv = w.View(std::initializer_list<float>({5, 6, 7}).begin(), 12);
    u32 moved = w.Accessor(view, 5126, 3, "VEC3", false, 0, Format("\"sparse\":{{\"count\":1,\"indices\":{{\"bufferView\":{},\"componentType\":5123}},\"values\":{{\"bufferView\":{}}}}}", si, sv));
    // No buffer view: zeros, with a sparse value.
    u32 zeros = w.Accessor(~0u, 5126, 3, "VEC3", false, 0, Format("\"sparse\":{{\"count\":1,\"indices\":{{\"bufferView\":{},\"componentType\":5123}},\"values\":{{\"bufferView\":{}}}}}", si, sv));
    // A MAT2 of bytes: columns padded to 4 bytes.
    u32 mat = w.Data<i8>({1, 2, 0, 0, 3, 4, 0, 0}, 5120, "MAT2", 8);
    String body = Format(
        "\"extensionsUsed\":[\"KHR_mesh_quantization\"],\"extensionsRequired\":[\"KHR_mesh_quantization\"],"
        "\"meshes\":[{{\"name\":\"tri\",\"primitives\":["
        "{{\"attributes\":{{\"POSITION\":{},\"NORMAL\":{},\"TEXCOORD_0\":{},\"COLOR_0\":{},\"JOINTS_0\":{},\"WEIGHTS_0\":{},\"_TEMPERATURE\":{}}},\"indices\":{}}},"
        "{{\"attributes\":{{\"POSITION\":{}}},\"indices\":{},\"mode\":5}},"
        "{{\"attributes\":{{\"POSITION\":{},\"TEXCOORD_1\":{}}},\"indices\":{},\"mode\":0}}]}}],"
        "\"nodes\":[{{\"mesh\":0}}],\"scenes\":[{{\"nodes\":[0]}}],\"scene\":0,\"extras\":{{\"matrix\":{}}}",
        pos, nrm, uv, color, joints, weights, zeros, i8s, moved, i16s, pos, uv, i32s, mat);
    Scene s;
    HX_REQUIRE(Read(AsBytes(w.Text(body)), s));
    HX_REQUIRE(s.meshes.Size() == 1u && s.primitives.Size() == 3u && s.roots.Size() == 1u);
    const Primitive& p = s.primitives[0];
    HX_CHECK(p.vertexCount == 3u && p.streams.count == 7u && p.indices.count == 3u && p.topology == Topology::Triangles);
    const float* position = Stream(s, p, Semantic::Position);
    const float* normal = Stream(s, p, Semantic::Normal);
    const float* texcoord = Stream(s, p, Semantic::TexCoord);
    const float* col = Stream(s, p, Semantic::Color);
    const float* joint = Stream(s, p, Semantic::Joints);
    const float* weight = Stream(s, p, Semantic::Weights);
    HX_REQUIRE(position && normal && texcoord && col && joint && weight);
    HX_CHECK(position[3] == 1 && position[7] == 1);
    // Signed normalized: -128 is -1 as 127 is.
    HX_CHECK(normal[2] == 1 && normal[4] == -1 && normal[6] == 1);
    HX_CHECK(texcoord[2] == 1 && Near(texcoord[5], 32768.0f / 65535));
    HX_CHECK(col[0] == 1 && Near(col[7], 128.0f / 255) && s.Find(p, Semantic::Color)->components == 4u);
    HX_CHECK(joint[11] == 11 && weight[0] == 1 && Near(weight[8] + weight[9] + weight[10], 1, 0.01f));
    const VertexStream* custom = s.Find(p, Semantic::Custom);
    HX_CHECK(custom && custom->name == StringView("_TEMPERATURE") && s.Data(*custom)[6] == 5 && s.Data(*custom)[0] == 0);
    HX_CHECK(s.indices[p.indices.begin + 2] == 2u);

    const Primitive& strip = s.primitives[1];
    const float* sparse = Stream(s, strip, Semantic::Position);
    HX_CHECK(strip.topology == Topology::TriangleStrip && sparse[6] == 5 && sparse[8] == 7 && sparse[3] == 1 && s.indices[strip.indices.begin] == 2u);
    const Primitive& points = s.primitives[2];
    HX_CHECK(points.topology == Topology::Points && s.indices[points.indices.begin] == 1u && s.Find(points, Semantic::TexCoord, 1) != nullptr);
    HX_CHECK(s.TriangleCount() == 2u);
}

HX_TEST(Gltf, MaterialsTexturesAndImages)
{
    GltfWriter w;
    u32 png = w.View("PNGDATA!", 8);
    String body = Format(
        "\"extensionsUsed\":[\"KHR_texture_transform\",\"KHR_materials_variants\"],"
        "\"images\":[{{\"uri\":\"textures/albedo%20map.png\"}},{{\"bufferView\":{},\"mimeType\":\"image/png\"}},{{\"uri\":\"data:image/jpeg;base64,SGVsaXg=\"}}],"
        "\"samplers\":[{{\"magFilter\":9728,\"minFilter\":9987,\"wrapS\":33071,\"wrapT\":33648}}],"
        "\"textures\":[{{\"source\":0,\"sampler\":0,\"name\":\"albedo\"}},{{\"source\":1}},{{\"source\":2}}],"
        "\"materials\":[{{\"name\":\"glass\",\"pbrMetallicRoughness\":{{\"baseColorFactor\":[0.5,0.25,1,0.75],\"metallicFactor\":0.1,\"roughnessFactor\":0.2,"
        "\"baseColorTexture\":{{\"index\":0,\"texCoord\":1,\"extensions\":{{\"KHR_texture_transform\":{{\"offset\":[0.5,0.25],\"scale\":[2,3],\"rotation\":0.5}}}}}}}},"
        "\"normalTexture\":{{\"index\":1,\"scale\":0.5}},\"occlusionTexture\":{{\"index\":2,\"strength\":0.25}},\"emissiveFactor\":[1,0.5,0],"
        "\"alphaMode\":\"BLEND\",\"doubleSided\":true,\"extensions\":{{"
        "\"KHR_materials_emissive_strength\":{{\"emissiveStrength\":5}},\"KHR_materials_ior\":{{\"ior\":1.33}},"
        "\"KHR_materials_transmission\":{{\"transmissionFactor\":0.9,\"transmissionTexture\":{{\"index\":2}}}},"
        "\"KHR_materials_volume\":{{\"thicknessFactor\":0.3,\"attenuationDistance\":2,\"attenuationColor\":[1,0.5,0.25]}},"
        "\"KHR_materials_clearcoat\":{{\"clearcoatFactor\":1,\"clearcoatRoughnessFactor\":0.1,\"clearcoatNormalTexture\":{{\"index\":1,\"scale\":2}}}},"
        "\"KHR_materials_sheen\":{{\"sheenColorFactor\":[0.1,0.2,0.3],\"sheenRoughnessFactor\":0.4}},"
        "\"KHR_materials_specular\":{{\"specularFactor\":0.5,\"specularColorFactor\":[1,0.9,0.8]}},"
        "\"KHR_materials_iridescence\":{{\"iridescenceFactor\":1,\"iridescenceIor\":1.5,\"iridescenceThicknessMaximum\":500}},"
        "\"KHR_materials_anisotropy\":{{\"anisotropyStrength\":0.6,\"anisotropyRotation\":1.5}},\"KHR_materials_dispersion\":{{\"dispersion\":0.1}}}}}},"
        "{{\"name\":\"flat\",\"alphaMode\":\"MASK\",\"alphaCutoff\":0.3,\"extensions\":{{\"KHR_materials_unlit\":{{}}}}}}],"
        "\"extensions\":{{\"KHR_materials_variants\":{{\"variants\":[{{\"name\":\"red\"}},{{\"name\":\"blue\"}}]}}}}",
        png);
    Scene s;
    HX_REQUIRE(Read(AsBytes(w.Text(body)), s));
    HX_REQUIRE(s.images.Size() == 3u && s.textures.Size() == 3u && s.materials.Size() == 2u);
    HX_CHECK(s.images[0].uri == StringView("textures/albedo map.png") && s.images[0].bytes.count == 0u);
    HX_CHECK(s.images[1].bytes.count == 8u && s.images[1].mimeType == StringView("image/png") && s.bytes[s.images[1].bytes.begin] == 'P');
    HX_CHECK(s.images[2].mimeType == StringView("image/jpeg") && s.images[2].bytes.count == 5u && s.bytes[s.images[2].bytes.begin + 4] == 'x');
    const Texture& t = s.textures[0];
    HX_CHECK(t.name == StringView("albedo") && t.magFilter == Filter::Nearest && t.minFilter == Filter::LinearMipmapLinear && t.wrapS == Wrap::ClampToEdge &&
             t.wrapT == Wrap::MirroredRepeat && s.textures[1].wrapS == Wrap::Repeat);
    const Material& m = s.materials[0];
    HX_CHECK(m.name == StringView("glass") && m.baseColor[3] == 0.75f && Near(m.metallic, 0.1f) && Near(m.roughness, 0.2f));
    const TextureRef& b = m.baseColorTexture;
    HX_CHECK(b.texture == 0u && b.texCoord == 1u && b.offset[0] == 0.5f && b.scale[1] == 3 && b.rotation == 0.5f);
    HX_CHECK(m.normalTexture.texture == 1u && m.normalTexture.strength == 0.5f && m.occlusionTexture.strength == 0.25f && m.emissive[1] == 0.5f);
    HX_CHECK(m.alphaMode == AlphaMode::Blend && m.doubleSided && !m.unlit && m.emissiveStrength == 5 && Near(m.ior, 1.33f));
    HX_CHECK(m.hasTransmission && Near(m.transmission, 0.9f) && m.transmissionTexture.texture == 2u);
    HX_CHECK(m.hasVolume && Near(m.thickness, 0.3f) && m.attenuationDistance == 2 && m.attenuationColor[2] == 0.25f);
    HX_CHECK(m.hasClearcoat && m.clearcoat == 1 && m.clearcoatNormalTexture.strength == 2);
    HX_CHECK(m.hasSheen && Near(m.sheenColor[2], 0.3f) && Near(m.sheenRoughness, 0.4f));
    HX_CHECK(m.hasSpecular && m.specular == 0.5f && Near(m.specularColor[2], 0.8f));
    HX_CHECK(m.hasIridescence && m.iridescenceIor == 1.5f && m.iridescenceThicknessMin == 100 && m.iridescenceThicknessMax == 500);
    HX_CHECK(m.hasAnisotropy && Near(m.anisotropyStrength, 0.6f) && m.anisotropyRotation == 1.5f && Near(m.dispersion, 0.1f));
    const Material& flat = s.materials[1];
    HX_CHECK(flat.unlit && flat.alphaMode == AlphaMode::Mask && Near(flat.alphaCutoff, 0.3f) && !flat.hasClearcoat && flat.metallic == 1);
    HX_CHECK(s.variants.Size() == 2u && s.variants[1] == StringView("blue"));
}

HX_TEST(Gltf, NodesSkinsMorphsAnimationsCamerasLights)
{
    GltfWriter w;
    u32 pos = w.Data<float>({0, 0, 0, 1, 0, 0, 0, 1, 0}, 5126, "VEC3", 3);
    u32 delta = w.Data<float>({0, 0, 1, 0, 0, 1, 0, 0, 1}, 5126, "VEC3", 3);
    u32 ibm = w.Data<float>({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -1, -2, -3, 1, 2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 1}, 5126, "MAT4", 16);
    u32 times = w.Data<float>({0, 1, 2}, 5126, "SCALAR", 1);
    u32 moves = w.Data<float>({0, 0, 0, 1, 0, 0, 2, 0, 0}, 5126, "VEC3", 3);
    // Cubic spline rotation: in tangent, value, out tangent per key.
    u32 turns = w.Data<float>({0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0}, 5126, "VEC4", 4);
    u32 morph = w.Data<float>({0, 0.5f, 1}, 5126, "SCALAR", 1);
    String body = Format(
        "\"extensionsUsed\":[\"KHR_lights_punctual\"],"
        "\"meshes\":[{{\"primitives\":[{{\"attributes\":{{\"POSITION\":{}}},\"targets\":[{{\"POSITION\":{}}}]}}],\"weights\":[0.25],"
        "\"extras\":{{\"targetNames\":[\"lift\"]}}}}],"
        "\"skins\":[{{\"name\":\"rig\",\"joints\":[1,2],\"inverseBindMatrices\":{},\"skeleton\":1}}],"
        "\"cameras\":[{{\"type\":\"perspective\",\"perspective\":{{\"yfov\":0.5,\"aspectRatio\":1.5,\"znear\":0.1}}}},"
        "{{\"type\":\"orthographic\",\"orthographic\":{{\"xmag\":2,\"ymag\":3,\"znear\":0.5,\"zfar\":50}}}}],"
        "\"extensions\":{{\"KHR_lights_punctual\":{{\"lights\":[{{\"type\":\"spot\",\"color\":[1,0.5,0],\"intensity\":10,\"range\":5,"
        "\"spot\":{{\"innerConeAngle\":0.1,\"outerConeAngle\":0.5}}}},{{\"type\":\"directional\"}}]}}}},"
        "\"nodes\":[{{\"name\":\"root\",\"children\":[1,3],\"mesh\":0,\"skin\":0,\"weights\":[0.75]}},"
        "{{\"name\":\"hip\",\"children\":[2],\"matrix\":[0,2,0,0,-2,0,0,0,0,0,2,0,1,2,3,1]}},{{\"name\":\"knee\",\"translation\":[0,-1,0],"
        "\"rotation\":[0,0,0.7071068,0.7071068],\"scale\":[1,2,1]}},{{\"camera\":1,\"extensions\":{{\"KHR_lights_punctual\":{{\"light\":0}}}}}}],"
        "\"scenes\":[{{\"nodes\":[0]}}],"
        "\"animations\":[{{\"name\":\"walk\",\"samplers\":[{{\"input\":{},\"output\":{}}},{{\"input\":{},\"output\":{},\"interpolation\":\"CUBICSPLINE\"}},"
        "{{\"input\":{},\"output\":{},\"interpolation\":\"STEP\"}}],\"channels\":[{{\"sampler\":0,\"target\":{{\"node\":1,\"path\":\"translation\"}}}},"
        "{{\"sampler\":1,\"target\":{{\"node\":2,\"path\":\"rotation\"}}}},{{\"sampler\":2,\"target\":{{\"node\":0,\"path\":\"weights\"}}}},"
        "{{\"sampler\":0,\"target\":{{\"path\":\"translation\"}}}}]}}]",
        pos, delta, ibm, times, moves, times, turns, times, morph);
    Scene s;
    HX_REQUIRE(Read(AsBytes(w.Text(body)), s));
    HX_REQUIRE(s.nodes.Size() == 4u && s.roots.Size() == 1u && s.roots[0] == 0u);
    const Node& root = s.nodes[0];
    HX_CHECK(root.children.count == 2u && s.nodeChildren[root.children.begin + 1] == 3u && root.mesh == 0u && root.skin == 0u && root.weights.count == 1u);
    HX_CHECK(s.nodes[1].parent == 0u && s.nodes[2].parent == 1u && s.nodes[3].parent == 0u);
    // The matrix: a quarter turn about z, scale 2, translation (1, 2, 3).
    const Node& hip = s.nodes[1];
    HX_CHECK(hip.hasMatrix && hip.translation[2] == 3 && Near(hip.scale[0], 2) && Near(hip.scale[1], 2) && Near(hip.scale[2], 2));
    HX_CHECK(Near(hip.rotation[2], 0.7071068f) && Near(hip.rotation[3], 0.7071068f) && Near(hip.rotation[0], 0));
    const Node& knee = s.nodes[2];
    HX_CHECK(!knee.hasMatrix && knee.translation[1] == -1 && knee.scale[1] == 2 && Near(knee.rotation[2], 0.7071068f));
    HX_CHECK(s.nodes[3].camera == 1u && s.nodes[3].light == 0u);

    const Skin& skin = s.skins[0];
    HX_CHECK(skin.name == StringView("rig") && skin.joints.count == 2u && s.skinJoints[skin.joints.begin + 1] == 2u && skin.skeleton == 1u);
    HX_CHECK(skin.inverseBindMatrices.count == 32u && s.floats[skin.inverseBindMatrices.begin + 12] == -1 && s.floats[skin.inverseBindMatrices.begin + 16] == 2);

    const Primitive& p = s.primitives[0];
    HX_REQUIRE(p.targets.count == 1u);
    const MorphTarget& t = s.morphTargets[p.targets.begin];
    HX_CHECK(t.name == StringView("lift") && t.streams.count == 1u && s.Data(s.streams[t.streams.begin])[8] == 1);
    HX_CHECK(s.meshes[0].weights.count == 1u && s.floats[s.meshes[0].weights.begin] == 0.25f && s.floats[root.weights.begin] == 0.75f);

    HX_REQUIRE(s.clips.Size() == 1u);
    const AnimationClip& clip = s.clips[0];
    // The channel without a node is left out.
    HX_CHECK(clip.name == StringView("walk") && clip.channels.count == 3u && clip.duration == 2);
    const AnimationChannel& move = s.channels[clip.channels.begin];
    HX_CHECK(move.node == 1u && move.path == AnimationPath::Translation && move.interpolation == Interpolation::Linear && move.times.count == 3u &&
             move.components == 3u && s.floats[move.values.begin + 6] == 2);
    const AnimationChannel& turn = s.channels[clip.channels.begin + 1];
    HX_CHECK(turn.interpolation == Interpolation::CubicSpline && turn.components == 4u && turn.values.count == 36u);
    const AnimationChannel& weights = s.channels[clip.channels.begin + 2];
    HX_CHECK(weights.path == AnimationPath::Weights && weights.interpolation == Interpolation::Step && weights.components == 1u);

    HX_REQUIRE(s.cameras.Size() == 2u && s.lights.Size() == 2u);
    HX_CHECK(!s.cameras[0].orthographic && s.cameras[0].yfov == 0.5f && s.cameras[0].aspectRatio == 1.5f && s.cameras[0].zfar == 0);
    HX_CHECK(s.cameras[1].orthographic && s.cameras[1].ymag == 3 && s.cameras[1].zfar == 50);
    const Light& spot = s.lights[0];
    HX_CHECK(spot.type == LightType::Spot && spot.color[1] == 0.5f && spot.intensity == 10 && spot.range == 5 && Near(spot.outerConeAngle, 0.5f));
    HX_CHECK(s.lights[1].type == LightType::Directional && s.lights[1].intensity == 1);
}

HX_TEST(Gltf, GlbAndExternalBuffers)
{
    GltfWriter w;
    u32 pos = w.Data<float>({0, 0, 0, 1, 0, 0, 0, 1, 0}, 5126, "VEC3", 3);
    String body = Format("\"meshes\":[{{\"primitives\":[{{\"attributes\":{{\"POSITION\":{}}}}}]}}],\"nodes\":[{{\"mesh\":0}},{{}}]", pos);
    Array<u8> glb = w.Glb(body);
    HX_CHECK(IsGlb(glb));
    Scene s;
    HX_REQUIRE(Read(Span<const u8>(glb), s));
    // Without scenes every node without a parent is a root.
    HX_CHECK(s.primitives.Size() == 1u && s.Data(s.streams[0])[3] == 1 && s.roots.Size() == 2u && s.unitScale == 1 && s.axes.up == 1 && s.axes.front == 2);

    struct Context
    {
        const Array<u8>* bin;
        String asked;
    } context{&w.bin, String()};
    ReadOptions options;
    options.context = &context;
    options.loader = [](void* c, StringView uri, Array<u8>& out) {
        Context& ctx = *static_cast<Context*>(c);
        ctx.asked = uri;
        if (uri != StringView("data/mesh bin.bin"))
            return false;
        out = *ctx.bin;
        return true;
    };
    HX_REQUIRE(Read(AsBytes(w.External(body, "data/mesh%20bin.bin")), s, options));
    HX_CHECK(context.asked == StringView("data/mesh bin.bin") && s.Data(s.streams[0])[7] == 1);
    String error;
    HX_CHECK(!ReadGltf(AsBytes(w.External(body, "missing.bin")), s, error, options) && error == StringView("buffers[0].uri: 'missing.bin' could not be loaded"));
}

HX_TEST(Gltf, DamagedFilesAreErrorsWithTheirPaths)
{
    HX_CHECK(Refused("{}", "asset: missing"));
    HX_CHECK(Refused("{\"asset\":{\"version\":\"1.0\"}}", "asset.version: '1.0'"));
    HX_CHECK(Refused("[1]", "the document is not a JSON object"));
    HX_CHECK(Refused("{\"asset\":", "not glTF JSON"));
    HX_CHECK(Refused("{\"asset\":{\"version\":\"2.0\"},\"extensionsRequired\":[\"KHR_draco_mesh_compression\"]}",
                     "extensionsRequired: KHR_draco_mesh_compression is not supported"));
    GltfWriter w;
    u32 pos = w.Data<float>({0, 0, 0, 1, 0, 0, 0, 1, 0}, 5126, "VEC3", 3);
    u32 bad = w.Data<u16>({0, 1, 7}, 5123, "SCALAR", 1);
    u32 two = w.Data<float>({0, 0, 1, 0, 0, 1}, 5126, "VEC3", 3);
    auto refused = [&](StringView body, StringView expected) { return Refused(w.Text(body), expected); };
    HX_CHECK(refused(Format("\"meshes\":[{{\"primitives\":[{{\"attributes\":{{\"POSITION\":{}}},\"indices\":{}}}]}}]", pos, bad),
                     "meshes[0].primitives[0].indices: index 7 of 3 vertices"));
    HX_CHECK(refused("\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":9}}]}]", "meshes[0].primitives[0].attributes.POSITION: accessor 9 of 3"));
    HX_CHECK(refused(Format("\"meshes\":[{{\"primitives\":[{{\"attributes\":{{\"POSITION\":{},\"NORMAL\":{}}}}}]}}]", pos, two),
                     "meshes[0].primitives[0].attributes.NORMAL: 2 elements where the primitive has 3 vertices"));
    HX_CHECK(refused("\"meshes\":[{\"primitives\":[]}]", "meshes[0].primitives: missing"));
    HX_CHECK(refused("\"nodes\":[{\"children\":[1]},{\"children\":[0]}]", "nodes[0]: its parents form a cycle"));
    HX_CHECK(refused("\"nodes\":[{\"children\":[2]},{\"children\":[2]},{}]", "nodes[2]: a child of nodes 0 and 1"));
    HX_CHECK(refused("\"nodes\":[{\"mesh\":3}]", "nodes[0].mesh: 3 of 0"));
    HX_CHECK(refused("\"nodes\":[{}],\"scenes\":[{\"nodes\":[4]}]", "scenes[0].nodes[0]: not a root node"));
    HX_CHECK(Refused("{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":3,\"uri\":\"data:application/octet-stream;base64,AAAA\"}],"
                     "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":2,\"byteLength\":4}]}",
                     "bufferViews[0]: bytes 2 to 6 of a buffer of 3"));
    HX_CHECK(Refused("{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":3,\"uri\":\"data:application/octet-stream;base64,AAAA\"}],"
                     "\"bufferViews\":[{\"buffer\":0,\"byteLength\":3}],\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":1,\"type\":\"VEC3\"}],"
                     "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0}}]}]}",
                     "accessors[0]: 1 elements of 12 bytes from 0 do not fit bufferView 0"));

    // Every cut of a GLB reads or is refused, never outside.
    String body = Format("\"meshes\":[{{\"primitives\":[{{\"attributes\":{{\"POSITION\":{}}}}}]}}]", pos);
    Array<u8> glb = w.Glb(body);
    for (usize n = 0; n < glb.Size(); n++)
    {
        Scene s;
        String e;
        ReadGltf(Span<const u8>(glb.Data(), n), s, e);
    }
}
