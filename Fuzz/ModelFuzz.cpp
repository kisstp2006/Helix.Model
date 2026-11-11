#include <Support/SampleFbx.h>
#include <Support/SampleModels.h>

#include <Helix/Model/Fbx/Fbx.h>
#include <Helix/Model/Scene/Convert.h>

#include <Helix/Model/Obj/Obj.h>
#include <Helix/Testing/Fuzz.h>

#include <cstdlib>
#include <cstring>

using namespace helix;
using namespace helix::model;
using namespace helix::model::testing;

namespace
{
    Array<u8> Mutated(helix::testing::FuzzRound& round, const u8* data, usize size)
    {
        Array<u8> bytes;
        bytes.Append(data, size);
        bytes.Resize(size + 64);
        bytes.Resize(round.Mutate(bytes.Data(), size, bytes.Size()));
        return bytes;
    }

    Array<u8> Copy(const char* text)
    {
        Array<u8> b;
        b.Append(reinterpret_cast<const u8*>(text), std::strlen(text));
        return b;
    }

    // A walk over everything the scene refers to: a reader that left an index out of range stops here.
    u64 Check(const Scene& s)
    {
        u64 n = 0;
        for (const Primitive& p : s.primitives)
        {
            for (u32 i = 0; i < p.streams.count; i++)
            {
                const VertexStream& st = s.streams[p.streams.begin + i];
                if (st.offset + u64(st.components) * p.vertexCount > s.floats.Size())
                    std::abort();
            }
            for (u32 i = 0; i < p.indices.count; i++)
                if (s.indices[p.indices.begin + i] >= p.vertexCount)
                    std::abort();
            if (p.material != kNone && p.material >= s.materials.Size())
                std::abort();
            n += p.vertexCount;
        }
        for (const Node& node : s.nodes)
            if ((node.mesh != kNone && node.mesh >= s.meshes.Size()) || (node.parent != kNone && node.parent >= s.nodes.Size()))
                std::abort();
        for (const Texture& t : s.textures)
            if (t.image != kNone && t.image >= s.images.Size())
                std::abort();
        return n + 1;
    }

    const Array<u8>& Gltf()
    {
        static const Array<u8> bytes = [] {
            String body;
            GltfWriter w = SampleWriter(body);
            String text = w.Text(body);
            Array<u8> b;
            b.Append(reinterpret_cast<const u8*>(text.Data()), text.Size());
            return b;
        }();
        return bytes;
    }

    const Array<u8>& Glb()
    {
        static const Array<u8> bytes = [] {
            String body;
            GltfWriter w = SampleWriter(body);
            return w.Glb(body);
        }();
        return bytes;
    }

    // The MTL the loader hands out in this round.
    thread_local Array<u8> mtl;
}

HX_FUZZ(GltfJson)
{
    Array<u8> bytes = Mutated(round, Gltf().Data(), Gltf().Size());
    Scene s;
    String error;
    round.Step("read");
    return ReadGltf(Span<const u8>(bytes), s, error) ? Check(s) : 0;
}

HX_FUZZ(Glb)
{
    Array<u8> bytes = Mutated(round, Glb().Data(), Glb().Size());
    Scene s;
    String error;
    round.Step("read");
    return ReadGltf(Span<const u8>(bytes), s, error) ? Check(s) : 0;
}

// The OBJ or its MTL mutated.
HX_FUZZ(Obj)
{
    Array<u8> obj = Copy(kSampleObj);
    mtl = Copy(kSampleMtl);
    if (round.Below(2) == 0)
        obj = Mutated(round, obj.Data(), obj.Size());
    else
        mtl = Mutated(round, mtl.Data(), mtl.Size());
    ReadOptions options;
    options.loader = [](void*, StringView, Array<u8>& out) {
        out = mtl;
        return true;
    };
    Scene s;
    String error;
    round.Step("read");
    return ReadObj(Span<const u8>(obj), s, error, options) ? Check(s) : 0;
}


namespace
{
    const Array<u8>& Fbx(int which)
    {
        static const Array<u8> files[3] = {SampleFbx().Binary(7400), SampleFbx().Binary(7500, true), [] {
                                               String text = SampleFbx().Ascii();
                                               Array<u8> b;
                                               b.Append(reinterpret_cast<const u8*>(text.Data()), text.Size());
                                               return b;
                                           }()};
        return files[which];
    }

    u64 ReadFbxRound(helix::testing::FuzzRound& round, const Array<u8>& seed)
    {
        Array<u8> bytes = Mutated(round, seed.Data(), seed.Size());
        Scene s;
        String error;
        round.Step("read");
        if (!ReadFbx(Span<const u8>(bytes), s, error))
            return 0;
        u64 n = Check(s);
        round.Step("convert");
        ConvertScene(s);
        return n + Check(s);
    }
}

// Binary FBX, raw and zlib arrays, records of 32 and 64 bit offsets.
HX_FUZZ(FbxBinary)
{
    return ReadFbxRound(round, Fbx(int(round.Below(2))));
}

HX_FUZZ(FbxAscii)
{
    return ReadFbxRound(round, Fbx(2));
}
