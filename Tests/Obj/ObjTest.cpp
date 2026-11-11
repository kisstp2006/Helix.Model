#include <Support/GltfWriter.h>

#include <Helix/Model/Obj/Obj.h>
#include <Helix/Testing/Test.h>

#include <cmath>
#include <cstdio>

using namespace helix;
using namespace helix::model;
using namespace helix::model::testing;

namespace
{
    constexpr const char* kCube = R"(# a cube with two materials
mtllib cube.mtl
o Cube
v -1 -1 -1
v 1 -1 -1
v 1 1 -1
v -1 1 -1
v -1 -1 1
v 1 -1 1
v 1 1 1
v -1 1 1
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn 0 0 -1
vn 0 0 1
usemtl red
f 1/1/1 4/4/1 3/3/1 2/2/1
f -4/1/2 -3/2/2 -2/3/2 -1/4/2
usemtl blue
f 1/1 2/2 6/3 5/4
s off
g Lines
l 1 2 3
p 7 8
o Empty
o Colored
v 0 0 0 1 0 0
v 1 0 0 0 1 0
v 0 1 0 0 0 1
f 9//1 10//1 \
11//1
)";

    constexpr const char* kMtl = R"(newmtl red
Kd 1 0 0
Ks 0.5 0.5 0.5
Ns 98
Ke 0.1 0.2 0.3
d 0.5
Ni 1.45
map_Kd -s 2 3 -o 0.5 0.25 -clamp on textures/red brick.png
map_bump -bm 0.75 textures/bump.png
norm normal.png
map_d alpha.png

newmtl blue
Kd 0 0 1
Pr 0.3
Pm 1
Pc 0.5
Pcr 0.25
Ps 0.1 0.2 0.3
aniso 0.4
anisor 0.2
map_Pr rough.png
map_Pm metal.png
map_Kd textures/red brick.png
unknownkey 1
)";

    bool Read(StringView text, Scene& s, const ReadOptions& options = {})
    {
        String error;
        if (!ReadObj(AsBytes(String(text)), s, error, options))
        {
            std::printf("    %s\n", error.CStr());
            return false;
        }
        return true;
    }

    ReadOptions Loader()
    {
        ReadOptions o;
        o.loader = [](void*, StringView uri, Array<u8>& out) {
            if (uri != StringView("cube.mtl"))
                return false;
            out.Clear();
            out.Append(reinterpret_cast<const u8*>(kMtl), std::strlen(kMtl));
            return true;
        };
        return o;
    }
}

HX_TEST(Obj, Geometry)
{
    Scene s;
    HX_REQUIRE(Read(kCube, s, Loader()));
    HX_CHECK(s.unitScale == 0 && s.axes.up == 1);
    // Cube, Lines, Colored; the empty object is left out.
    HX_REQUIRE(s.meshes.Size() == 3u && s.nodes.Size() == 3u && s.roots.Size() == 3u);
    HX_CHECK(s.meshes[0].name == StringView("Cube") && s.meshes[1].name == StringView("Lines") && s.meshes[2].name == StringView("Colored"));
    HX_CHECK(s.nodes[2].mesh == 2u && s.nodes[1].name == StringView("Lines"));
    const Mesh& cube = s.meshes[0];
    HX_REQUIRE(cube.primitives.count == 2u);
    const Primitive& red = s.primitives[cube.primitives.begin];
    // Two quads: four triangles; 8 corners with distinct position, uv and normal triples.
    HX_CHECK(red.topology == Topology::Triangles && red.indices.count == 12u && red.vertexCount == 8u);
    HX_CHECK(s.materials[red.material].name == StringView("red"));
    const float* p = s.Data(*s.Find(red, Semantic::Position));
    const float* n = s.Data(*s.Find(red, Semantic::Normal));
    const float* uv = s.Data(*s.Find(red, Semantic::TexCoord));
    // The first fan triangle: corners 1, 4, 3.
    u32 a = s.indices[red.indices.begin], b = s.indices[red.indices.begin + 1];
    HX_CHECK(p[a * 3] == -1 && p[b * 3 + 1] == 1 && n[a * 3 + 2] == -1 && uv[b * 2 + 1] == 1);
    // Negative indices count back from the last vertex: -4 is vertex 5.
    u32 back = s.indices[red.indices.begin + 6];
    HX_CHECK(p[back * 3 + 2] == 1 && n[back * 3 + 2] == 1);
    const Primitive& blue = s.primitives[cube.primitives.begin + 1];
    HX_CHECK(s.materials[blue.material].name == StringView("blue") && s.Find(blue, Semantic::Normal) == nullptr && s.Find(blue, Semantic::TexCoord) != nullptr);

    const Mesh& lines = s.meshes[1];
    HX_REQUIRE(lines.primitives.count == 2u);
    const Primitive& l = s.primitives[lines.primitives.begin];
    const Primitive& pts = s.primitives[lines.primitives.begin + 1];
    HX_CHECK(l.topology == Topology::Lines && l.indices.count == 4u && l.vertexCount == 3u);
    HX_CHECK(pts.topology == Topology::Points && pts.indices.count == 2u);

    // Vertex colors, and a face continued on the next line.
    const Primitive& colored = s.primitives[s.meshes[2].primitives.begin];
    const VertexStream* c = s.Find(colored, Semantic::Color);
    HX_REQUIRE(c && c->components == 3u && colored.indices.count == 3u);
    HX_CHECK(s.Data(*c)[0] == 1 && s.Data(*c)[4] == 1 && s.Data(*c)[8] == 1);
    // Every primitive of the file has colors now; the cube's vertices without them are white.
    HX_CHECK(s.Data(*s.Find(red, Semantic::Color))[0] == 1 && s.Data(*s.Find(red, Semantic::Color))[1] == 1);
    HX_CHECK(s.TriangleCount() == 7u);
}

HX_TEST(Obj, Materials)
{
    Scene s;
    HX_REQUIRE(Read(kCube, s, Loader()));
    HX_REQUIRE(s.materials.Size() == 2u);
    const Material& red = s.materials[0];
    HX_CHECK(red.baseColor[0] == 1 && red.baseColor[3] == 0.5f && red.alphaMode == AlphaMode::Blend && red.metallic == 0);
    HX_CHECK(std::fabs(red.roughness - std::sqrt(2.0f / 100)) < 1e-6f && red.shininess == 98 && red.specularColorClassic[1] == 0.5f);
    HX_CHECK(red.emissive[2] == 0.3f && std::fabs(red.ior - 1.45f) < 1e-6f);
    const TextureRef& kd = red.baseColorTexture;
    HX_REQUIRE(kd.Present());
    HX_CHECK(kd.scale[0] == 2 && kd.scale[1] == 3 && kd.offset[0] == 0.5f && kd.offset[1] == 0.25f);
    const Texture& t = s.textures[kd.texture];
    HX_CHECK(t.wrapS == Wrap::ClampToEdge && s.images[t.image].uri == StringView("textures/red brick.png"));
    HX_CHECK(red.bumpTexture.Present() && red.bumpTexture.strength == 0.75f && s.images[s.textures[red.bumpTexture.texture].image].uri == StringView("textures/bump.png"));
    HX_CHECK(red.normalTexture.Present() && red.alphaTexture.Present());

    const Material& blue = s.materials[1];
    HX_CHECK(blue.roughness == 0.3f && blue.metallic == 1 && blue.hasClearcoat && blue.clearcoat == 0.5f && blue.clearcoatRoughness == 0.25f);
    HX_CHECK(blue.hasSheen && blue.sheenColor[2] == 0.3f && blue.hasAnisotropy && blue.anisotropyStrength == 0.4f && blue.anisotropyRotation == 0.2f);
    HX_CHECK(blue.roughnessTexture.Present() && blue.metallicTexture.Present());
    // The same file with another wrap is another texture of the same image.
    HX_CHECK(blue.baseColorTexture.texture != red.baseColorTexture.texture &&
             s.textures[blue.baseColorTexture.texture].image == s.textures[red.baseColorTexture.texture].image);
    bool warned = false;
    for (const String& w : s.warnings)
        warned = warned || w == StringView("MTL line 25: 'unknownkey' skipped");
    HX_CHECK(warned);

    // Without the loader the materials are placeholders named by usemtl, with a warning.
    HX_REQUIRE(Read(kCube, s));
    HX_CHECK(s.materials.Size() == 2u && s.materials[0].name == StringView("red") && !s.materials[0].baseColorTexture.Present());
    HX_CHECK(!s.warnings.Empty() && s.warnings[0] == StringView("OBJ line 2: material library 'cube.mtl' could not be loaded"));
}

HX_TEST(Obj, ErrorsNameTheLine)
{
    auto refused = [](StringView text, StringView expected) {
        Scene s;
        String error;
        if (ReadObj(AsBytes(String(text)), s, error) || error != expected)
        {
            std::printf("    %s (expected %.*s)\n", error.Empty() ? "read" : error.CStr(), int(expected.Size()), expected.Data());
            return false;
        }
        return true;
    };
    HX_CHECK(refused("v 0 0 0\nv 1 0 0\nf 1 2 3\n", "line 3: index 3 of 2 in '3'"));
    HX_CHECK(refused("v 0 0\n", "line 1: v needs three coordinates"));
    HX_CHECK(refused("v 0 0 x\n", "line 1: 'x' is not a number"));
    HX_CHECK(refused("v 0 0 0\nf 1 1\n", "line 2: f needs 3 vertices"));
    HX_CHECK(refused("v 0 0 0\nv 0 0 0\nv 0 0 0\nf 1/1 2 3\n", "line 4: index 1 of 0 in '1/1'"));
    HX_CHECK(refused("v 0 0 0\nf 0 1 1\n", "line 2: '0' is not a vertex"));
    HX_CHECK(refused("v 0 0 0\nf /1 1 1\n", "line 2: '/1' has no position"));
    Scene s;
    String error;
    HX_CHECK(!ReadMtl("newmtl a\nKd 1\nNs\n", s, error) && error == StringView("MTL line 3: 'Ns' without its values"));
    // A gray Kd of one value is fine.
    HX_CHECK(s.materials.Size() == 1u && s.materials[0].baseColor[2] == 1);
    // Every cut reads or is refused.
    StringView cube(kCube);
    for (usize n = 0; n < cube.Size(); n++)
    {
        Scene cut;
        String e;
        ReadObj(Span<const u8>(reinterpret_cast<const u8*>(cube.Data()), n), cut, e);
    }
}
