#include <Support/GltfWriter.h>

#include <Helix/Model/Gltf/Gltf.h>
#include <Helix/Model/Obj/Obj.h>
#include <Helix/Testing/Bench.h>

#include <cmath>

using namespace helix;
using namespace helix::model;
using namespace helix::model::testing;

namespace
{
    constexpr u32 kSide = 256;

    // A grid of kSide x kSide quads: positions, normals, texture coordinates and 32 bit indices.
    const Array<u8>& LargeGlb()
    {
        static const Array<u8> bytes = [] {
            Array<float> p, n, uv;
            Array<u32> idx;
            for (u32 y = 0; y <= kSide; y++)
                for (u32 x = 0; x <= kSide; x++)
                {
                    p.Push(float(x)), p.Push(std::sin(float(x + y) * 0.1f)), p.Push(float(y));
                    n.Push(0), n.Push(1), n.Push(0);
                    uv.Push(float(x) / kSide), uv.Push(float(y) / kSide);
                }
            for (u32 y = 0; y < kSide; y++)
                for (u32 x = 0; x < kSide; x++)
                {
                    u32 a = y * (kSide + 1) + x, b = a + 1, c = a + kSide + 1, d = c + 1;
                    idx.Push(a), idx.Push(c), idx.Push(b), idx.Push(b), idx.Push(c), idx.Push(d);
                }
            GltfWriter w;
            u32 vp = w.Accessor(w.View(p.Data(), p.Size() * 4), 5126, u32(p.Size() / 3), "VEC3");
            u32 vn = w.Accessor(w.View(n.Data(), n.Size() * 4), 5126, u32(n.Size() / 3), "VEC3");
            u32 vt = w.Accessor(w.View(uv.Data(), uv.Size() * 4), 5126, u32(uv.Size() / 2), "VEC2");
            u32 vi = w.Accessor(w.View(idx.Data(), idx.Size() * 4), 5125, u32(idx.Size()), "SCALAR");
            return w.Glb(Format("\"meshes\":[{{\"primitives\":[{{\"attributes\":{{\"POSITION\":{},\"NORMAL\":{},\"TEXCOORD_0\":{}}},\"indices\":{}}}]}}],"
                                "\"nodes\":[{{\"mesh\":0}}]",
                                vp, vn, vt, vi));
        }();
        return bytes;
    }

    const String& LargeObj()
    {
        static const String text = [] {
            String s;
            for (u32 y = 0; y <= kSide; y++)
                for (u32 x = 0; x <= kSide; x++)
                    FormatAppend(s, "v {} {} {}\nvt {} {}\n", x, std::sin(float(x + y) * 0.1f), y, float(x) / kSide, float(y) / kSide);
            s.Append("vn 0 1 0\n");
            for (u32 y = 0; y < kSide; y++)
                for (u32 x = 0; x < kSide; x++)
                {
                    u32 a = y * (kSide + 1) + x + 1, b = a + 1, c = a + kSide + 1, d = c + 1;
                    FormatAppend(s, "f {}/{}/1 {}/{}/1 {}/{}/1 {}/{}/1\n", a, a, c, c, d, d, b, b);
                }
            return s;
        }();
        return text;
    }
}

// Items are triangles.
HX_BENCH(ReadGlbGrid)
{
    const Array<u8>& bytes = LargeGlb();
    state.SetItemsPerIteration(u64(kSide) * kSide * 2);
    Scene s;
    String error;
    while (state.KeepRunning())
    {
        ReadGltf(Span<const u8>(bytes), s, error);
        helix::testing::DoNotOptimize(s.floats.Data());
    }
}

HX_BENCH(ReadObjGrid)
{
    const String& text = LargeObj();
    state.SetItemsPerIteration(u64(kSide) * kSide * 2);
    Scene s;
    String error;
    while (state.KeepRunning())
    {
        ReadObj(AsBytes(text), s, error);
        helix::testing::DoNotOptimize(s.floats.Data());
    }
}
