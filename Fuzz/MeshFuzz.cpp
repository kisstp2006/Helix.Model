#include <Helix/Model/Mesh/Mesh.h>
#include <Helix/Testing/Fuzz.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>

using namespace helix;
using namespace helix::model;
using helix::testing::FuzzRound;

namespace
{
    // Mostly a coarse lattice, so vertices coincide; now and then a value no mesh should have.
    float Value(FuzzRound& r)
    {
        switch (r.Below(24))
        {
        case 0:
            return std::numeric_limits<float>::quiet_NaN();
        case 1:
            return std::numeric_limits<float>::infinity();
        case 2:
            return -0.0f;
        case 3:
            return 1e30f;
        case 4:
            return 1e-30f;
        default:
            return (float(r.Below(5)) - 2) * 0.5f;
        }
    }

    void AddStream(FuzzRound& r, Scene& s, Range& range, Semantic semantic, u32 set, u32 components, u32 vertices)
    {
        VertexStream st;
        st.semantic = semantic;
        st.set = set;
        st.components = components;
        st.offset = u32(s.floats.Size());
        for (u32 i = 0; i < vertices * components; i++)
            s.floats.Push(Value(r));
        if (range.count == 0)
            range.begin = u32(s.streams.Size());
        s.streams.Push(Move(st));
        range.count++;
    }

    // A small random primitive: streams of every kind, maybe indices (a few outside the vertices), a topology, morph
    // targets. A second primitive shares nothing, so its data must come out unchanged.
    Scene RandomScene(FuzzRound& r)
    {
        Scene s;
        for (u32 n = 0; n < 2; n++)
        {
            Primitive p;
            p.vertexCount = r.Below(40);
            u32 t = r.Below(10);
            p.topology = t < 7 ? Topology::Triangles : t == 7 ? Topology::TriangleStrip : t == 8 ? Topology::TriangleFan : Topology::Lines;
            AddStream(r, s, p.streams, Semantic::Position, 0, r.Below(8) ? 3 : 2, p.vertexCount);
            if (r.Below(2))
                AddStream(r, s, p.streams, Semantic::Normal, 0, 3, p.vertexCount);
            if (r.Below(2))
                AddStream(r, s, p.streams, Semantic::TexCoord, 0, 2, p.vertexCount);
            if (r.Below(4) == 0)
                AddStream(r, s, p.streams, Semantic::TexCoord, 1, 2, p.vertexCount);
            if (r.Below(4) == 0)
                AddStream(r, s, p.streams, Semantic::Color, 0, 4, p.vertexCount);
            if (r.Below(2))
            {
                u32 count = r.Below(100);
                p.indices.begin = u32(s.indices.Size());
                for (u32 i = 0; i < count; i++)
                    s.indices.Push(r.Below(32) == 0 ? p.vertexCount + r.Below(3) : r.Below(p.vertexCount + 1) % (p.vertexCount ? p.vertexCount : 1));
                p.indices.count = count;
            }
            for (u32 m = r.Below(3); m > 0; m--)
            {
                MorphTarget target;
                AddStream(r, s, target.streams, Semantic::Position, 0, 3, p.vertexCount);
                if (r.Below(2))
                    AddStream(r, s, target.streams, Semantic::Normal, 0, 3, p.vertexCount);
                if (p.targets.count == 0)
                    p.targets.begin = u32(s.morphTargets.Size());
                s.morphTargets.Push(Move(target));
                p.targets.count++;
            }
            s.primitives.Push(p);
        }
        // Weights in the pool between the streams, for Compact.
        Node node;
        node.weights = {u32(s.floats.Size()), 2};
        s.floats.Push(0.25f), s.floats.Push(0.5f);
        s.nodes.Push(Move(node));
        return s;
    }

    void Check(const Scene& s)
    {
        auto streams = [&](Range r, u32 vertices) {
            if (u64(r.begin) + r.count > s.streams.Size())
                std::abort();
            for (u32 i = 0; i < r.count; i++)
            {
                const VertexStream& st = s.streams[r.begin + i];
                if (st.offset + u64(st.components) * vertices > s.floats.Size())
                    std::abort();
            }
        };
        for (const Primitive& p : s.primitives)
        {
            streams(p.streams, p.vertexCount);
            if (u64(p.targets.begin) + p.targets.count > s.morphTargets.Size() || u64(p.indices.begin) + p.indices.count > s.indices.Size())
                std::abort();
            for (u32 t = 0; t < p.targets.count; t++)
                streams(s.morphTargets[p.targets.begin + t].streams, p.vertexCount);
        }
        const Node& n = s.nodes[0];
        if (n.weights.begin + 2u > s.floats.Size() || s.floats[n.weights.begin] != 0.25f || s.floats[n.weights.begin + 1] != 0.5f)
            std::abort();
    }

    // The triangles of a primitive that use only its vertices, as position bits (-0 as 0), each rotated to its smallest
    // corner and sorted: what reordering, welding and splitting must keep.
    struct Tri
    {
        u32 v[9];
        bool operator<(const Tri& o) const { return std::memcmp(v, o.v, sizeof(v)) < 0; }
        bool operator!=(const Tri& o) const { return std::memcmp(v, o.v, sizeof(v)) != 0; }
    };

    Array<Tri> Geometry(const Scene& s, u32 primitive)
    {
        Array<Tri> out;
        const Primitive& p = s.primitives[primitive];
        const VertexStream* st = s.Find(p, Semantic::Position);
        if (p.topology != Topology::Triangles || !st || st->components < 3)
            return out;
        u32 n = p.indices.count ? p.indices.count : p.vertexCount;
        for (u32 i = 0; i + 2 < n; i += 3)
        {
            u32 idx[3];
            bool valid = true;
            for (u32 k = 0; k < 3; k++)
            {
                idx[k] = p.indices.count ? s.indices[p.indices.begin + i + k] : i + k;
                valid = valid && idx[k] < p.vertexCount;
            }
            if (!valid)
                continue;
            u32 c[3][3];
            for (u32 k = 0; k < 3; k++)
                for (u32 a = 0; a < 3; a++)
                {
                    float f = s.Data(*st)[usize(idx[k]) * st->components + a];
                    f = f == 0 ? 0.0f : f;
                    std::memcpy(&c[k][a], &f, 4);
                }
            u32 first = 0;
            for (u32 k = 1; k < 3; k++)
                if (std::memcmp(c[k], c[first], sizeof(c[k])) < 0)
                    first = k;
            Tri t;
            for (u32 k = 0; k < 3; k++)
                std::memcpy(t.v + k * 3, c[(first + k) % 3], sizeof(c[0]));
            out.Push(t);
        }
        std::sort(out.begin(), out.end());
        return out;
    }

    void Same(const Array<Tri>& a, const Array<Tri>& b)
    {
        if (a.Size() != b.Size())
            std::abort();
        for (usize i = 0; i < a.Size(); i++)
            if (a[i] != b[i])
                std::abort();
    }
}

// Random sequences of the mesh functions on random primitives, the scene checked after each and the geometry compared
// where the function must keep it.
HX_FUZZ(MeshProcessing)
{
    Scene s = RandomScene(round);
    u64 items = 0;
    for (u32 step = round.Below(6) + 1; step > 0; step--)
    {
        u32 primitive = round.Below(2);
        u32 op = round.Below(10);
        Array<Tri> before = Geometry(s, primitive);
        bool keeps = true;
        switch (op)
        {
        case 0:
            round.Step("triangle list");
            ToTriangleList(s, primitive);
            keeps = false;
            break;
        case 1:
            round.Step("weld");
            WeldVertices(s, primitive);
            break;
        case 2:
        {
            round.Step("gather");
            Array<u32> order;
            for (u32 i = round.Below(48); i > 0; i--)
                order.Push(round.Below(s.primitives[primitive].vertexCount + 2));
            GatherVertices(s, primitive, order);
            keeps = false;
            break;
        }
        case 3:
            round.Step("compact");
            Compact(s);
            break;
        case 4:
            round.Step("normals");
            CalculateNormals(s, primitive, float(round.Below(200)), NormalWeighting(round.Below(4)));
            break;
        case 5:
            round.Step("tangents");
            CalculateTangents(s, primitive, round.Below(2));
            break;
        case 6:
            round.Step("vertex cache");
            OptimizeVertexCache(s, primitive);
            break;
        case 7:
            round.Step("overdraw");
            OptimizeOverdraw(s, primitive, 1 + float(round.Below(100)) / 100);
            break;
        case 8:
            round.Step("vertex fetch");
            OptimizeVertexFetch(s, primitive);
            break;
        default:
            round.Step("simplify");
            Simplify(s, primitive, float(round.Below(11)) / 10, float(round.Below(5)) / 10);
            keeps = false;
            break;
        }
        Check(s);
        // Strips and fans become lists in the functions that need triangles: their geometry is compared from then on.
        if (keeps && !before.Empty())
            Same(Geometry(s, primitive), before);
        items += s.primitives[primitive].vertexCount + 1;
    }
    return items;
}

// The index level functions on random triangle lists: their outputs stay in range and within their limits.
HX_FUZZ(MeshAlgorithms)
{
    u32 vertices = round.Below(64) + 1;
    Array<float> positions;
    for (u32 i = 0; i < vertices * 3; i++)
        positions.Push(Value(round));
    Array<u32> indices;
    for (u32 i = round.Below(64) * 3; i > 0; i--)
        indices.Push(round.Below(vertices));

    round.Step("analyze");
    AnalyzeVertexCache(indices.Data(), indices.Size(), vertices, round.Below(40));
    AnalyzeOverdraw(positions.Data(), indices.Data(), indices.Size());

    round.Step("simplify");
    Array<u32> out;
    SimplifyResult r = SimplifyIndices(positions.Data(), vertices, indices.Data(), indices.Size(), round.Below(u32(indices.Size()) + 1), float(round.Below(4)) / 4, out);
    if (r.indexCount != out.Size() || out.Size() % 3 || out.Size() > indices.Size() || !(r.error >= 0))
        std::abort();
    for (u32 v : out)
        if (v >= vertices)
            std::abort();

    round.Step("meshlets");
    Meshlets ml;
    u32 maxVertices = round.Below(80), maxTriangles = round.Below(140);
    BuildMeshlets(positions.Data(), vertices, indices.Data(), indices.Size(), ml, maxVertices, maxTriangles);
    usize triangles = 0;
    for (const Meshlet& m : ml.meshlets)
    {
        if (m.vertexCount > (maxVertices < 3 ? 3 : maxVertices) || m.triangleCount > (maxTriangles < 1 ? 1 : maxTriangles) || m.triangleCount == 0)
            std::abort();
        if (u64(m.vertexOffset) + m.vertexCount > ml.vertices.Size() || u64(m.triangleOffset) + m.triangleCount * 3u > ml.triangles.Size())
            std::abort();
        for (u32 i = 0; i < m.triangleCount * 3; i++)
            if (ml.triangles[m.triangleOffset + i] >= m.vertexCount)
                std::abort();
        triangles += m.triangleCount;
    }
    if (triangles != indices.Size() / 3)
        std::abort();

    round.Step("deltas");
    CompressedDeltas packed;
    float threshold = float(round.Below(3)) * 0.25f;
    CompressDeltas(positions.Data(), vertices, 3, threshold, packed);
    Array<float> back;
    DecompressDeltas(packed, vertices, back);
    if (back.Size() != positions.Size())
        std::abort();
    for (usize i = 0; i < back.Size(); i++)
    {
        float d = std::isfinite(positions[i]) ? positions[i] : 0;
        // Kept values within half a step, dropped ones below the threshold.
        if (!(std::fabs(back[i] - d) <= std::fmax(packed.scale * 0.51f, threshold) + 1e-30f))
            std::abort();
    }
    return indices.Size() / 3 + 1;
}
