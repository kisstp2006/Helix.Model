#include <Support/GltfWriter.h>
#include <Support/SampleFbx.h>

#include <Helix/Model/Fbx/Document.h>
#include <Helix/Model/Fbx/Fbx.h>
#include <Helix/Model/Scene/Convert.h>
#include <Helix/Testing/Test.h>

#include <cmath>
#include <cstdio>

using namespace helix;
using namespace helix::model;
using namespace helix::model::testing;

namespace
{
    bool Read(Span<const u8> bytes, Scene& s)
    {
        String error;
        if (!ReadFbx(bytes, s, error))
        {
            std::printf("    %s\n", error.CStr());
            return false;
        }
        return true;
    }

    bool Near(float a, float b, float e = 1e-4f) { return std::fabs(a - b) <= e; }

    bool Vec(const float* v, float x, float y, float z, float e = 1e-4f) { return Near(v[0], x, e) && Near(v[1], y, e) && Near(v[2], z, e); }

    const VertexStream* Find(const Scene& s, const Primitive& p, Semantic semantic, u32 set = 0) { return s.Find(p, semantic, set); }

    // The first vertex of a primitive at a position.
    u32 VertexAt(const Scene& s, const Primitive& p, float x, float y, float z)
    {
        const float* pos = s.Data(*Find(s, p, Semantic::Position));
        for (u32 v = 0; v < p.vertexCount; v++)
            if (Vec(pos + 3 * v, x, y, z))
                return v;
        return kNone;
    }

    // A vector turned by a quaternion.
    void Rotate(const float* q, const float* v, float* out)
    {
        float x = q[0], y = q[1], z = q[2], w = q[3];
        float tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
        out[0] = v[0] + w * tx + (y * tz - z * ty);
        out[1] = v[1] + w * ty + (z * tx - x * tz);
        out[2] = v[2] + w * tz + (x * ty - y * tx);
    }

    void CheckSample(const Scene& s)
    {
        HX_CHECK(s.axes.up == 2 && s.axes.front == 1 && s.axes.frontSign == -1 && s.axes.coord == 0 && Near(s.unitScale, 0.01f) && s.frameRate == 30);
        HX_REQUIRE(s.nodes.Size() == 6u && s.roots.Size() == 4u);
        HX_CHECK(s.nodes[0].name == StringView("Cube") && s.nodes[2].name == StringView("Bone") && s.nodes[2].parent == 1u && s.nodes[3].parent == 2u);
        // Translation, rotation 90 about x, scale 2.
        const Node& cube = s.nodes[0];
        HX_CHECK(Vec(cube.translation, 1, 2, 3) && Near(cube.rotation[0], 0.7071068f) && Near(cube.rotation[3], 0.7071068f) && Vec(cube.scale, 2, 2, 2));
        // Pre rotation 90 about z.
        const Node& bone = s.nodes[2];
        HX_CHECK(Vec(bone.translation, 0, 1, 0) && Near(bone.rotation[2], 0.7071068f) && Near(bone.rotation[3], 0.7071068f));
        // Rotation about the pivot (1, 0, 0): the pivot stays.
        const Node& tip = s.nodes[3];
        HX_CHECK(Vec(tip.translation, 1, -1, 0) && Near(tip.rotation[2], 0.7071068f));

        HX_REQUIRE(cube.mesh == 0u && s.meshes.Size() == 1u && s.meshes[0].primitives.count == 2u);
        const Primitive& red = s.primitives[0];
        const Primitive& blue = s.primitives[1];
        HX_CHECK(red.vertexCount == 12u && red.indices.count == 18u && blue.vertexCount == 12u && red.material == 0u && blue.material == 1u);
        // The geometric translation (0, 0, 1) is in the vertices.
        u32 v0 = VertexAt(s, red, -1, -1, 0);
        HX_REQUIRE(v0 != kNone);
        const float* n = s.Data(*Find(s, red, Semantic::Normal));
        const VertexStream* uv0 = Find(s, red, Semantic::TexCoord, 0);
        const VertexStream* uv1 = Find(s, red, Semantic::TexCoord, 1);
        const VertexStream* color = Find(s, red, Semantic::Color);
        HX_REQUIRE(uv0 && uv1 && color && uv0->name == StringView("map") && uv1->name == StringView("second"));
        HX_CHECK(red.streams.count == 7u);
        // Several vertices share control point 0; one of them is the corner of face 0 (normal -z, uv 0 0, color 0).
        bool found = false;
        const float* pos = s.Data(*Find(s, red, Semantic::Position));
        for (u32 v = 0; v < red.vertexCount; v++)
            if (Vec(pos + 3 * v, -1, -1, 0) && Vec(n + 3 * v, 0, 0, -1))
                found = found || (s.Data(*uv0)[2 * v] == 0 && s.Data(*uv0)[2 * v + 1] == 0 && s.Data(*color)[4 * v] == 0);
        HX_CHECK(found);
        // Skin: control point 0 fully on the bone, 2 half on each.
        const float* joints = s.Data(*Find(s, red, Semantic::Joints));
        const float* weights = s.Data(*Find(s, red, Semantic::Weights));
        HX_CHECK(joints[4 * v0] == 0 && weights[4 * v0] == 1 && weights[4 * v0 + 1] == 0);
        u32 v2 = VertexAt(s, red, 1, 1, 0);
        HX_REQUIRE(v2 != kNone);
        HX_CHECK(Near(weights[4 * v2], 0.5f) && Near(weights[4 * v2 + 1], 0.5f) && joints[4 * v2] + joints[4 * v2 + 1] == 1);
        HX_REQUIRE(cube.skin == 0u && s.skins.Size() == 1u);
        const Skin& skin = s.skins[0];
        HX_CHECK(skin.joints.count == 2u && s.skinJoints[skin.joints.begin] == 2u && s.skinJoints[skin.joints.begin + 1] == 3u);
        const float* ibm = s.floats.Data() + skin.inverseBindMatrices.begin;
        HX_CHECK(Vec(ibm + 12, 0, 0, -5) && Vec(ibm + 28, -1, 0, 0));
        // Blend shapes: two targets of one channel, at 50 and 100.
        HX_REQUIRE(red.targets.count == 2u);
        const MorphTarget& half = s.morphTargets[red.targets.begin];
        const MorphTarget& full = s.morphTargets[red.targets.begin + 1];
        HX_CHECK(half.channel == 0u && half.fullWeight == 50 && full.fullWeight == 100 && half.name == StringView("Bulge_0") && half.streams.count == 2u);
        HX_CHECK(Vec(s.Data(s.streams[full.streams.begin]) + 3 * v0, 0, 0, 2) && Vec(s.Data(s.streams[full.streams.begin]) + 3 * v2, 0, 0, 0));
        HX_CHECK(s.meshes[0].weights.count == 1u && Near(s.floats[s.meshes[0].weights.begin], 0.25f));

        HX_REQUIRE(s.materials.Size() == 2u);
        const Material& m = s.materials[0];
        HX_CHECK(m.name == StringView("Red") && Vec(m.baseColor, 1, 0, 0) && m.baseColor[3] == 0.5f && m.alphaMode == AlphaMode::Blend);
        HX_CHECK(Vec(m.emissive, 0, 0.5f, 0) && Near(m.roughness, std::sqrt(2.0f / 22)) && m.shininess == 20);
        HX_REQUIRE(m.baseColorTexture.Present());
        HX_CHECK(m.baseColorTexture.scale[0] == 2 && m.baseColorTexture.scale[1] == 3);
        const Image& image = s.images[s.textures[m.baseColorTexture.texture].image];
        HX_CHECK(image.uri == StringView("tex/red.png") && image.bytes.count == 4u && s.bytes[image.bytes.begin] == 'P');
        HX_CHECK(s.materials[1].baseColor[3] == 1 && s.materials[1].alphaMode == AlphaMode::Opaque);

        HX_REQUIRE(s.cameras.Size() == 1u && s.lights.Size() == 1u && s.nodes[4].camera == 0u && s.nodes[5].light == 0u);
        HX_CHECK(Near(s.cameras[0].yfov, 60 * 3.14159265f / 180) && Near(s.cameras[0].aspectRatio, 1920.0f / 1080) && s.cameras[0].zfar == 1000);
        HX_CHECK(s.lights[0].type == LightType::Spot && s.lights[0].intensity == 2 && Near(s.lights[0].outerConeAngle, 30 * 3.14159265f / 180) && s.lights[0].color[2] == 0.25f);

        HX_REQUIRE(s.clips.Size() == 1u);
        const AnimationClip& clip = s.clips[0];
        HX_CHECK(clip.name == StringView("Take") && Near(clip.duration, 1) && clip.channels.count == 4u);
        const AnimationChannel& t = s.channels[clip.channels.begin];
        HX_CHECK(t.node == 2u && t.path == AnimationPath::Translation && t.times.count == 31u && Near(s.floats[t.times.begin + 15], 0.5f));
        // Linear x from 0 to 10, y held at 3 (a constant key), z from the curve node.
        HX_CHECK(Vec(s.floats.Data() + t.values.begin + 15 * 3, 5, 3, 0) && Vec(s.floats.Data() + t.values.begin + 30 * 3, 10, 7, 0));
        // The rotation channel keeps the pre rotation.
        const AnimationChannel& r = s.channels[clip.channels.begin + 1];
        HX_CHECK(r.path == AnimationPath::Rotation && Near(s.floats[r.values.begin + 2], 0.7071068f));
        const AnimationChannel& w = s.channels[clip.channels.begin + 3];
        HX_CHECK(w.path == AnimationPath::Weights && w.node == 0u && w.components == 1u && Near(s.floats[w.values.begin + 15], 0.5f) && Near(s.floats[w.values.begin + 30], 1));
    }
}

HX_TEST(Fbx, BinaryDocument)
{
    FbxFile f = SampleFbx();
    for (u32 version : {7400u, 7500u})
        for (bool zlib : {false, true})
        {
            Array<u8> bytes = f.Binary(version, zlib);
            fbx::Document d;
            String error;
            HX_REQUIRE(d.Read(Span<const u8>(bytes), error));
            HX_CHECK(d.binary && d.version == version && d.ChildCount(0) == 3u);
            u32 objects = d.Find(0, "Objects");
            HX_REQUIRE(objects != fbx::kNoNode);
            u32 g = d.Child(objects, 0);
            HX_CHECK(d.Name(g) == StringView("Geometry") && d.Prop(g, 0).integer == 100 && fbx::ObjectName(d.Text(d.Prop(g, 1))) == StringView("CubeMesh"));
            Array<double> v;
            HX_CHECK(d.Numbers(d.Prop(d.Find(g, "Vertices"), 0), v) && v.Size() == 24u && v[23] == 1);
            Array<i64> pvi;
            HX_CHECK(d.Integers(d.Prop(d.Find(g, "PolygonVertexIndex"), 0), pvi) && pvi[3] == -2 && pvi.Size() == 24u);
        }
}

HX_TEST(Fbx, AsciiDocument)
{
    String text = SampleFbx().Ascii();
    fbx::Document d;
    String error;
    HX_REQUIRE(d.Read(AsBytes(text), error));
    HX_CHECK(!d.binary && d.version == 7400u);
    u32 g = d.Child(d.Find(0, "Objects"), 0);
    HX_CHECK(fbx::ObjectName(d.Text(d.Prop(g, 1))) == StringView("CubeMesh") && d.Text(d.Prop(g, 2)) == StringView("Mesh"));
    Array<double> v;
    HX_CHECK(d.Numbers(d.Prop(d.Find(g, "Vertices"), 0), v) && v.Size() == 24u && v[0] == -1);
    // Comments, bare words, and errors with their lines.
    HX_CHECK(d.Read(AsBytes(String("; a comment\nA: 1, \"x\", T {\n\tB: *2 {\n\t\ta: 1.5,2\n\t}\n}\n")), error) && d.ChildCount(0) == 1u);
    u32 a = d.Child(0, 0);
    HX_CHECK(d.PropertyCount(a) == 3u && d.Text(d.Prop(a, 2)) == StringView("T") && d.Prop(d.Child(a, 0), 0).count == 2u);
    HX_CHECK(!d.Read(AsBytes(String("A: 1 {\nB: 2\n")), error) && error == StringView("line 3: a block without its closing brace"));
    HX_CHECK(!d.Read(AsBytes(String("A: *3 {\na: 1,2\n}\n")), error) && error == StringView("line 3: 2 values where the array says 3"));
    HX_CHECK(!d.Read(AsBytes(String("A 1\n")), error) && error == StringView("line 1: 'A' without a colon"));
}

HX_TEST(Fbx, SceneFromEveryEncoding)
{
    FbxFile f = SampleFbx();
    Scene s;
    HX_REQUIRE(Read(f.Binary(7400), s));
    CheckSample(s);
    HX_REQUIRE(Read(f.Binary(7500, true), s));
    CheckSample(s);
    String text = f.Ascii();
    HX_REQUIRE(Read(AsBytes(text), s));
    CheckSample(s);
}

// Models that use one geometry with the same materials and no geometric transform share one mesh and its skin, as
// instances (found with Blender's FBX exports in the corpus); a model with a geometric transform keeps its own.
HX_TEST(Fbx, SharedGeometryIsOneMesh)
{
    FbxFile f = SampleFbx();
    for (FbxNode& top : f.top)
    {
        if (top.name == StringView("Objects"))
            for (i64 id : {210, 211})
            {
                FbxNode m = Object("Model", id, "Model", id == 210 ? "CopyA" : "CopyB", "Mesh");
                m.Add(FbxNode("Properties70"));
                top.Add(Move(m));
            }
        if (top.name == StringView("Connections"))
            for (i64 id : {210, 211})
                top.Add(Connect(id, 0)).Add(Connect(100, id)).Add(Connect(300, id)).Add(Connect(301, id));
    }
    Scene s;
    HX_REQUIRE(Read(f.Binary(7400), s));
    u32 cube = kNone, a = kNone, b = kNone;
    for (u32 i = 0; i < s.nodes.Size(); i++)
    {
        StringView name = s.nodes[i].name;
        cube = name == StringView("Cube") ? i : cube;
        a = name == StringView("CopyA") ? i : a;
        b = name == StringView("CopyB") ? i : b;
    }
    HX_REQUIRE(cube != kNone && a != kNone && b != kNone);
    HX_CHECK(s.nodes[a].mesh != kNone && s.nodes[a].mesh == s.nodes[b].mesh);
    HX_CHECK(s.nodes[cube].mesh != s.nodes[a].mesh);
    HX_CHECK(s.nodes[a].skin != kNone && s.nodes[a].skin == s.nodes[b].skin);
    HX_CHECK_EQ(s.meshes.Size(), usize(2));
}

HX_TEST(Fbx, ConversionToTheEngineFrame)
{
    Scene s;
    HX_REQUIRE(Read(SampleFbx().Binary(7400), s));
    Scene before;
    HX_REQUIRE(Read(SampleFbx().Binary(7400), before));
    ConvertScene(s);
    // Z up, front -y, centimeters: engine (x, y, z) = (-x, z, -y) / 100.
    HX_CHECK(s.axes.up == 1 && s.axes.front == 2 && s.axes.coord == 0 && s.axes.coordSign == -1 && s.unitScale == 1);
    HX_CHECK(Vec(s.nodes[0].translation, -0.01f, 0.03f, -0.02f, 1e-6f) && Vec(s.nodes[0].scale, 2, 2, 2));
    const Primitive& red = s.primitives[0];
    HX_CHECK(VertexAt(s, red, 0.01f, 0, 0.01f) != kNone);
    // A mirror: every triangle's winding turns.
    HX_CHECK(s.indices[red.indices.begin + 1] == before.indices[red.indices.begin + 2] && s.indices[red.indices.begin + 2] == before.indices[red.indices.begin + 1]);
    // Rotations turn the converted vectors as the source turned the source vectors.
    for (const Node* n : {&s.nodes[0], &s.nodes[2], &s.nodes[3]})
    {
        const Node& b = before.nodes[usize(n - s.nodes.Data())];
        const float probe[3] = {0.3f, 0.5f, 0.7f};
        float turned[3], converted[3], probeConverted[3] = {-probe[0], probe[2], -probe[1]}, turnedConverted[3];
        Rotate(b.rotation, probe, turned);
        Rotate(n->rotation, probeConverted, converted);
        turnedConverted[0] = -turned[0], turnedConverted[1] = turned[2], turnedConverted[2] = -turned[1];
        HX_CHECK(Vec(converted, turnedConverted[0], turnedConverted[1], turnedConverted[2]));
    }
    const float* ibm = s.floats.Data() + s.skins[0].inverseBindMatrices.begin;
    HX_CHECK(Vec(ibm + 12, 0, -0.05f, 0, 1e-6f));
    const AnimationChannel& t = s.channels[s.clips[0].channels.begin];
    HX_CHECK(Vec(s.floats.Data() + t.values.begin + 15 * 3, -0.05f, 0, -0.03f, 1e-6f));
    HX_CHECK(Near(s.cameras[0].zfar, 10));
    // Converting again only scales.
    ConvertScene(s, {2, true});
    HX_CHECK(Vec(s.nodes[0].translation, -0.02f, 0.06f, -0.04f, 1e-6f));

    // glTF's frame: x mirrored, nothing else.
    Scene g;
    g.nodes.Emplace();
    g.nodes[0].translation[0] = 1, g.nodes[0].translation[2] = 2;
    ConvertScene(g);
    HX_CHECK(Vec(g.nodes[0].translation, -1, 0, 2));
}

HX_TEST(Fbx, DamagedFilesAreErrors)
{
    Array<u8> good = SampleFbx().Binary(7400, true);
    auto refused = [&](auto change, StringView expected) {
        Array<u8> bytes = good;
        change(bytes);
        Scene s;
        String error;
        if (ReadFbx(Span<const u8>(bytes), s, error) || !error.View().StartsWith(expected))
        {
            std::printf("    %s (expected %.*s)\n", error.Empty() ? "read" : error.CStr(), int(expected.Size()), expected.Data());
            return false;
        }
        return true;
    };
    HX_CHECK(refused([](Array<u8>& b) { b[23] = 0x10, b[24] = 0x27; }, "byte 23: FBX version 10000"));
    HX_CHECK(refused([](Array<u8>& b) { b.Resize(40); }, "byte 27: a record ending at"));
    HX_CHECK(refused([](Array<u8>& b) { b[27] = 0xFF, b[28] = 0xFF, b[29] = 0xFF; }, "byte 27: a record ending at"));
    Scene s;
    String error;
    HX_CHECK(!ReadFbx(AsBytes(String("Nothing: 1\n")), s, error) && error == StringView("no Objects"));
    // Every cut of the binary and the ASCII file reads or is refused, never outside.
    for (usize n = 0; n < good.Size(); n += 5)
    {
        Scene cut;
        String e;
        ReadFbx(Span<const u8>(good.Data(), n), cut, e);
    }
    String text = SampleFbx().Ascii();
    for (usize n = 0; n < text.Size(); n += 37)
    {
        Scene cut;
        String e;
        ReadFbx(Span<const u8>(reinterpret_cast<const u8*>(text.Data()), n), cut, e);
    }
}
