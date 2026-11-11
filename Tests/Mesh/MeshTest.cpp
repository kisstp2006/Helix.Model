#include <Support/SampleMeshes.h>

#include <Helix/Model/Mesh/Mesh.h>
#include <Helix/Testing/Test.h>

#include <algorithm>
#include <cmath>

using namespace helix;
using namespace helix::model;
using namespace helix::model::testing;

namespace
{
    const float* Stream(const Scene& s, Semantic semantic, u32 primitive = 0)
    {
        const VertexStream* st = s.Find(s.primitives[primitive], semantic);
        return st ? s.Data(*st) : nullptr;
    }

    Array<u32> Indices(const Scene& s, u32 primitive = 0)
    {
        const Primitive& p = s.primitives[primitive];
        Array<u32> out;
        out.Append(s.indices.Data() + p.indices.begin, p.indices.count);
        return out;
    }

    // The triangles as position triples (each rotated to start at its smallest position, winding kept), sorted: two
    // meshes with the same geometry give the same list whatever their vertices and triangle order.
    struct Corner
    {
        float p[3];
        bool operator<(const Corner& o) const { return p[0] != o.p[0] ? p[0] < o.p[0] : p[1] != o.p[1] ? p[1] < o.p[1] : p[2] < o.p[2]; }
        bool operator==(const Corner& o) const { return p[0] == o.p[0] && p[1] == o.p[1] && p[2] == o.p[2]; }
    };
    struct Tri
    {
        Corner c[3];
        bool operator<(const Tri& o) const
        {
            for (u32 k = 0; k < 3; k++)
                if (!(c[k] == o.c[k]))
                    return c[k] < o.c[k];
            return false;
        }
        bool operator==(const Tri& o) const { return c[0] == o.c[0] && c[1] == o.c[1] && c[2] == o.c[2]; }
    };

    Array<Tri> Geometry(const float* positions, const Array<u32>& indices)
    {
        Array<Tri> out;
        for (usize i = 0; i + 2 < indices.Size(); i += 3)
        {
            Tri t;
            for (u32 k = 0; k < 3; k++)
                for (u32 a = 0; a < 3; a++)
                    t.c[k].p[a] = positions[usize(indices[i + k]) * 3 + a];
            u32 first = t.c[1] < t.c[0] ? (t.c[2] < t.c[1] ? 2 : 1) : (t.c[2] < t.c[0] ? 2 : 0);
            Tri r;
            for (u32 k = 0; k < 3; k++)
                r.c[k] = t.c[(first + k) % 3];
            out.Push(r);
        }
        std::sort(out.begin(), out.end());
        return out;
    }

    Array<Tri> Geometry(const Scene& s, u32 primitive = 0) { return Geometry(Stream(s, Semantic::Position, primitive), Indices(s, primitive)); }

    bool Same(const Array<Tri>& a, const Array<Tri>& b)
    {
        if (a.Size() != b.Size())
            return false;
        for (usize i = 0; i < a.Size(); i++)
            if (!(a[i] == b[i]))
                return false;
        return true;
    }

    float Area(const float* positions, const Array<u32>& indices)
    {
        double sum = 0;
        for (usize i = 0; i + 2 < indices.Size(); i += 3)
        {
            const float* a = positions + usize(indices[i]) * 3;
            const float* b = positions + usize(indices[i + 1]) * 3;
            const float* c = positions + usize(indices[i + 2]) * 3;
            double e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
            double n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
            sum += std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]) / 2;
        }
        return float(sum);
    }

    // The cube as 36 vertices, three per triangle, as a file without indices gives them.
    Scene Unindexed(const MeshData& m, bool normals)
    {
        MeshData flat;
        for (u32 i : m.indices)
            flat.Vertex(m.positions[i * 3], m.positions[i * 3 + 1], m.positions[i * 3 + 2], m.normals[i * 3], m.normals[i * 3 + 1], m.normals[i * 3 + 2], m.uvs[i * 2],
                        m.uvs[i * 2 + 1]);
        Scene s = ToScene(flat, normals, false);
        s.primitives[0].indices = {};
        s.indices.Clear();
        return s;
    }
}

HX_TEST(Mesh, StripsAndFans)
{
    // A strip of two quads joined to a third by repeated vertices.
    MeshData m = Grid(3);
    Scene s = ToScene(m);
    s.primitives[0].topology = Topology::TriangleStrip;
    s.indices = Array<u32>();
    for (u32 i : {0u, 4u, 1u, 5u, 2u, 6u, 6u, 9u, 9u, 13u, 10u, 14u})
        s.indices.Push(i);
    s.primitives[0].indices = {0, u32(s.indices.Size())};
    ToTriangleList(s, 0);
    HX_CHECK(s.primitives[0].topology == Topology::Triangles);
    Array<u32> list = Indices(s);
    const u32 expected[] = {0, 4, 1, 1, 4, 5, 1, 5, 2, 2, 5, 6, 9, 13, 10, 10, 13, 14};
    HX_REQUIRE(list.Size() == 18u);
    for (u32 i = 0; i < 18; i++)
        HX_CHECK_EQ(list[i], expected[i]);

    // A fan without indices.
    Scene f = ToScene(m);
    f.primitives[0].topology = Topology::TriangleFan;
    f.primitives[0].indices = {};
    f.primitives[0].vertexCount = 5;
    ToTriangleList(f, 0);
    Array<u32> fan = Indices(f);
    const u32 fanExpected[] = {0, 1, 2, 0, 2, 3, 0, 3, 4};
    HX_REQUIRE(fan.Size() == 9u);
    for (u32 i = 0; i < 9; i++)
        HX_CHECK_EQ(fan[i], fanExpected[i]);

    // Lists are left alone.
    Scene l = ToScene(m);
    ToTriangleList(l, 0);
    HX_CHECK_EQ(l.primitives[0].indices.count, u32(m.indices.Size()));
}

HX_TEST(Mesh, WeldGatherCompact)
{
    MeshData cube = Cube();
    // Positions only: the 8 corners.
    Scene s = Unindexed(cube, false);
    Array<Tri> before = Geometry(s.floats.Data(), [] {
        Array<u32> i;
        for (u32 k = 0; k < 36; k++)
            i.Push(k);
        return i;
    }());
    HX_CHECK_EQ(WeldVertices(s, 0), 8u);
    HX_CHECK_EQ(s.primitives[0].indices.count, 36u);
    HX_CHECK(Same(Geometry(s), before));
    // With normals: 4 per face.
    Scene n = Unindexed(cube, true);
    HX_CHECK_EQ(WeldVertices(n, 0), 24u);
    // Welding again changes nothing.
    HX_CHECK_EQ(WeldVertices(n, 0), 24u);

    // A morph target that moves one of two otherwise equal vertices keeps them apart; -0 and 0 are equal.
    Scene m = Unindexed(cube, false);
    MorphTarget target;
    target.streams = {u32(m.streams.Size()), 1};
    VertexStream delta;
    delta.offset = u32(m.floats.Size());
    m.streams.Push(delta);
    for (u32 v = 0; v < 36; v++)
        m.floats.Push(v == 0 ? 0.5f : 0), m.floats.Push(v % 2 ? -0.0f : 0.0f), m.floats.Push(0);
    m.morphTargets.Push(target);
    m.primitives[0].targets = {0, 1};
    HX_CHECK_EQ(WeldVertices(m, 0), 9u);
    const float* moved = m.floats.Data() + m.streams[m.morphTargets[0].streams.begin].offset;
    u32 movedCount = 0;
    for (u32 v = 0; v < 9; v++)
        movedCount += moved[v * 3] == 0.5f;
    HX_CHECK_EQ(movedCount, 1u);

    // Gather: repeated and dropped vertices, every stream along.
    Scene g = ToScene(Grid(1));
    Array<u32> order;
    order.Push(3), order.Push(0), order.Push(0);
    GatherVertices(g, 0, order);
    HX_CHECK_EQ(g.primitives[0].vertexCount, 3u);
    const float* gp = Stream(g, Semantic::Position);
    const float* guv = Stream(g, Semantic::TexCoord);
    HX_CHECK(gp[0] == 1 && gp[1] == 1 && gp[3] == 0 && gp[6] == 0);
    HX_CHECK(guv[0] == 1 && guv[1] == 1 && guv[2] == 0 && guv[5] == 0);

    // Compact drops what no stream uses and keeps the rest where the ranges point.
    usize grown = n.floats.Size();
    Array<float> positions;
    positions.Append(Stream(n, Semantic::Position), 24 * 3);
    n.nodes[0].weights = {u32(n.floats.Size()), 2};
    n.floats.Push(0.25f), n.floats.Push(0.75f);
    Compact(n);
    HX_CHECK(n.floats.Size() < grown);
    HX_CHECK_EQ(n.floats.Size(), usize(24 * 6 + 2));
    const float* np = Stream(n, Semantic::Position);
    bool same = true;
    for (u32 i = 0; i < 24 * 3; i++)
        same = same && np[i] == positions[i];
    HX_CHECK(same);
    HX_CHECK(n.floats[n.nodes[0].weights.begin] == 0.25f && n.floats[n.nodes[0].weights.begin + 1] == 0.75f);
}

HX_TEST(Mesh, Normals)
{
    // The 8 corner cube: faceted below 90 degrees, smooth at 180.
    Scene s = Unindexed(Cube(), false);
    WeldVertices(s, 0);
    Scene faceted = s;
    CalculateNormals(faceted, 0, 60);
    HX_CHECK_EQ(faceted.primitives[0].vertexCount, 24u);
    {
        const float* nrm = Stream(faceted, Semantic::Normal);
        const float* pos = Stream(faceted, Semantic::Position);
        HX_REQUIRE(nrm);
        Array<u32> idx = Indices(faceted);
        bool ok = true;
        for (usize t = 0; t < idx.Size(); t += 3)
        {
            // Every corner's normal is its face's.
            const float* a = pos + idx[t] * 3;
            const float* b = pos + idx[t + 1] * 3;
            const float* c = pos + idx[t + 2] * 3;
            float e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
            float f[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
            float l = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
            for (u32 k = 0; k < 3; k++)
                for (u32 a2 = 0; a2 < 3; a2++)
                    ok = ok && std::fabs(nrm[idx[t + k] * 3 + a2] - f[a2] / l) < 1e-6f;
        }
        HX_CHECK(ok);
    }
    Scene smooth = s;
    CalculateNormals(smooth, 0, 180);
    HX_CHECK_EQ(smooth.primitives[0].vertexCount, 8u);
    {
        const float* nrm = Stream(smooth, Semantic::Normal);
        const float* pos = Stream(smooth, Semantic::Position);
        bool ok = true;
        for (u32 v = 0; v < 8; v++)
            for (u32 a = 0; a < 3; a++)
                ok = ok && std::fabs(nrm[v * 3 + a] - pos[v * 3 + a] / std::sqrt(3.0f)) < 1e-4f;
        HX_CHECK(ok);
    }

    // A sphere: smooth normals point outward and the seam does not show (twins get the same normal).
    for (NormalWeighting w : {NormalWeighting::AreaAndAngle, NormalWeighting::Area, NormalWeighting::Angle, NormalWeighting::Unweighted})
    {
        Scene sp = ToScene(Sphere(16, 32), false, true);
        u32 vertices = sp.primitives[0].vertexCount;
        CalculateNormals(sp, 0, 80, w);
        // The pole rows and the seam stay separate vertices for their UVs, nothing else splits.
        HX_CHECK(sp.primitives[0].vertexCount <= vertices);
        const float* nrm = Stream(sp, Semantic::Normal);
        const float* pos = Stream(sp, Semantic::Position);
        float worst = 1;
        for (u32 v = 0; v < sp.primitives[0].vertexCount; v++)
        {
            float d = nrm[v * 3] * pos[v * 3] + nrm[v * 3 + 1] * pos[v * 3 + 1] + nrm[v * 3 + 2] * pos[v * 3 + 2];
            worst = d < worst ? d : worst;
        }
        HX_CHECK(worst > 0.99f);
        // Seam twins: equal positions, equal normals.
        bool twins = true;
        for (u32 a = 0; a < sp.primitives[0].vertexCount; a++)
            for (u32 b = a + 1; b < sp.primitives[0].vertexCount; b++)
                if (pos[a * 3] == pos[b * 3] && pos[a * 3 + 1] == pos[b * 3 + 1] && pos[a * 3 + 2] == pos[b * 3 + 2])
                    twins = twins && nrm[a * 3] == nrm[b * 3] && nrm[a * 3 + 1] == nrm[b * 3 + 1] && nrm[a * 3 + 2] == nrm[b * 3 + 2];
        HX_CHECK(twins);
    }

    // Weighting matters where faces differ: a corner of one large and two small faces.
    MeshData fan;
    fan.Vertex(0, 0, 0, 0, 0, 1, 0, 0);
    fan.Vertex(4, 0, 0, 0, 0, 1, 0, 0);
    fan.Vertex(0, 4, 0, 0, 0, 1, 0, 0);
    fan.Vertex(0, 0, 1, 0, 0, 1, 0, 0);
    fan.Vertex(0.5f, 0, 1, 0, 0, 1, 0, 0);
    fan.Triangle(0, 1, 2), fan.Triangle(0, 3, 1), fan.Triangle(0, 2, 3);
    Scene area = ToScene(fan, false, false), unweighted = ToScene(fan, false, false);
    CalculateNormals(area, 0, 180, NormalWeighting::Area);
    CalculateNormals(unweighted, 0, 180, NormalWeighting::Unweighted);
    // The large face is z: area weighting leans toward it.
    HX_CHECK(Stream(area, Semantic::Normal)[2] > Stream(unweighted, Semantic::Normal)[2] + 0.1f);

    // A flat grid at angle 0 stays one smooth surface.
    Scene grid = ToScene(Grid(4), false, true);
    CalculateNormals(grid, 0, 0);
    HX_CHECK_EQ(grid.primitives[0].vertexCount, 25u);
    HX_CHECK(Stream(grid, Semantic::Normal)[2] == 1);
}

HX_TEST(Mesh, Tangents)
{
    // A flat grid with u along x: tangents +x and bitangents along +v (w = 1); mirrored u gives -x and w = -1.
    for (int mirror = 0; mirror < 2; mirror++)
    {
        MeshData m = Grid(4);
        if (mirror)
            for (usize i = 0; i < m.uvs.Size(); i += 2)
                m.uvs[i] = 1 - m.uvs[i];
        Scene s = ToScene(m);
        HX_REQUIRE(CalculateTangents(s, 0));
        HX_CHECK_EQ(s.primitives[0].vertexCount, 25u);
        const VertexStream* t = s.Find(s.primitives[0], Semantic::Tangent);
        HX_REQUIRE(t && t->components == 4);
        bool ok = true;
        for (u32 v = 0; v < 25; v++)
        {
            const float* x = s.Data(*t) + v * 4;
            ok = ok && std::fabs(x[0] - (mirror ? -1.0f : 1.0f)) < 1e-6f && std::fabs(x[1]) < 1e-6f && x[2] == 0 && x[3] == (mirror ? -1.0f : 1.0f);
        }
        HX_CHECK(ok);
    }

    // A sphere: unit tangents perpendicular to the normals, along increasing u.
    Scene sp = ToScene(Sphere(16, 32));
    HX_REQUIRE(CalculateTangents(sp, 0));
    const VertexStream* ts = sp.Find(sp.primitives[0], Semantic::Tangent);
    const float* nrm = Stream(sp, Semantic::Normal);
    bool ok = true;
    for (u32 v = 0; v < sp.primitives[0].vertexCount; v++)
    {
        const float* x = sp.Data(*ts) + v * 4;
        const float* n = nrm + v * 3;
        float len = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
        float dot = x[0] * n[0] + x[1] * n[1] + x[2] * n[2];
        ok = ok && std::fabs(len - 1) < 1e-4f && std::fabs(dot) < 1e-4f && (x[3] == 1 || x[3] == -1);
    }
    HX_CHECK(ok);

    // A cube with a quad per face needs no split.
    Scene cube = ToScene(Cube());
    HX_REQUIRE(CalculateTangents(cube, 0));
    HX_CHECK_EQ(cube.primitives[0].vertexCount, 24u);

    // Without UVs or normals there is nothing to build from.
    Scene bare = ToScene(Grid(2), true, false);
    HX_CHECK(!CalculateTangents(bare, 0));
    HX_CHECK(!CalculateTangents(bare, 0, 1));
}

HX_TEST(Mesh, VertexCache)
{
    const u32 one[] = {0, 1, 2};
    VertexCacheStats single = AnalyzeVertexCache(one, 3, 3);
    HX_CHECK(single.acmr == 3 && single.atvr == 1);

    MeshData m = Grid(64);
    Shuffle(m);
    Scene s = ToScene(m);
    Array<Tri> before = Geometry(s);
    VertexCacheStats scrambled = AnalyzeVertexCache(s.indices.Data(), s.indices.Size(), m.VertexCount());
    OptimizeVertexCache(s, 0);
    Array<u32> idx = Indices(s);
    VertexCacheStats optimized = AnalyzeVertexCache(idx.Data(), idx.Size(), m.VertexCount());
    HX_CHECK(Same(Geometry(s), before));
    HX_CHECK(scrambled.acmr > 2);
    HX_CHECK(optimized.acmr < 0.8f);
    HX_CHECK(optimized.atvr < 1.6f);

    // Fetch order: indices count up as vertices are first used, unused vertices go.
    // The last row of quads left out: its top vertices are unused.
    Scene f = ToScene(Grid(64));
    f.primitives[0].indices.count -= 3 * 128;
    Array<Tri> kept = Geometry(f);
    OptimizeVertexFetch(f, 0);
    Array<u32> fi = Indices(f);
    u32 next = 0;
    bool ordered = true;
    for (u32 i : fi)
    {
        ordered = ordered && i <= next;
        next = i == next ? next + 1 : next;
    }
    HX_CHECK(ordered);
    HX_CHECK_EQ(f.primitives[0].vertexCount, next);
    HX_CHECK(next < m.VertexCount());
    HX_CHECK(Same(Geometry(f), kept));
}

HX_TEST(Mesh, Overdraw)
{
    // Two quads facing +z, one behind the other: far first shades every covered pixel twice, near first once.
    MeshData q;
    for (float z : {0.0f, 1.0f})
    {
        u32 b = q.VertexCount();
        q.Vertex(0, 0, z, 0, 0, 1, 0, 0), q.Vertex(1, 0, z, 0, 0, 1, 0, 0), q.Vertex(1, 1, z, 0, 0, 1, 0, 0), q.Vertex(0, 1, z, 0, 0, 1, 0, 0);
        q.Triangle(b, b + 1, b + 2), q.Triangle(b, b + 2, b + 3);
    }
    OverdrawStats farFirst = AnalyzeOverdraw(q.positions.Data(), q.indices.Data(), 12);
    HX_CHECK_NEAR(farFirst.overdraw, 2.0f, 1e-6f);
    Array<u32> nearFirst;
    for (u32 i = 6; i < 12; i++)
        nearFirst.Push(q.indices[i]);
    for (u32 i = 0; i < 6; i++)
        nearFirst.Push(q.indices[i]);
    HX_CHECK_NEAR(AnalyzeOverdraw(q.positions.Data(), nearFirst.Data(), 12).overdraw, 1.0f, 1e-6f);

    // Spheres in a row, triangles scrambled: the reordering draws less and keeps the cache efficiency.
    MeshData row;
    for (u32 k = 0; k < 4; k++)
    {
        MeshData one = Sphere(16, 32, float(k) * 1.5f);
        u32 base = row.VertexCount();
        row.positions.Append(one.positions.Data(), one.positions.Size());
        row.normals.Append(one.normals.Data(), one.normals.Size());
        row.uvs.Append(one.uvs.Data(), one.uvs.Size());
        for (u32 i : one.indices)
            row.indices.Push(base + i);
    }
    Shuffle(row, 7);
    Scene s = ToScene(row);
    Array<Tri> before = Geometry(s);
    Scene cached = s;
    OptimizeVertexCache(cached, 0);
    Array<u32> ci = Indices(cached);
    float cacheOnly = AnalyzeOverdraw(row.positions.Data(), ci.Data(), ci.Size()).overdraw;
    VertexCacheStats cacheStats = AnalyzeVertexCache(ci.Data(), ci.Size(), row.VertexCount());
    OptimizeOverdraw(s, 0, 1.05f);
    Array<u32> oi = Indices(s);
    float reordered = AnalyzeOverdraw(row.positions.Data(), oi.Data(), oi.Size()).overdraw;
    VertexCacheStats reorderedStats = AnalyzeVertexCache(oi.Data(), oi.Size(), row.VertexCount());
    HX_CHECK(Same(Geometry(s), before));
    HX_CHECK(reordered < cacheOnly);
    HX_CHECK(reorderedStats.acmr < cacheStats.acmr * 1.1f);
}

HX_TEST(Mesh, Simplify)
{
    // A flat grid collapses to its target without error, its outline and area kept.
    MeshData g = Grid(32);
    Array<u32> out;
    SimplifyResult r = SimplifyIndices(g.positions.Data(), g.VertexCount(), g.indices.Data(), g.indices.Size(), g.indices.Size() / 10, 0.001f, out);
    HX_CHECK_EQ(r.indexCount, u32(out.Size()));
    HX_CHECK(out.Size() <= g.indices.Size() / 10 + 30 && out.Size() >= 6);
    HX_CHECK(r.error < 1e-6f);
    HX_CHECK_NEAR(Area(g.positions.Data(), out), 1.0f, 1e-4f);
    bool corners[4] = {};
    for (u32 v : out)
        for (u32 c = 0; c < 4; c++)
            corners[c] = corners[c] || v == (c & 1 ? 32u : 0u) + (c & 2 ? 33u * 32 : 0u);
    HX_CHECK(corners[0] && corners[1] && corners[2] && corners[3]);
    // Everything faces +z still.
    bool facing = true;
    for (usize i = 0; i < out.Size(); i += 3)
    {
        const float* a = g.positions.Data() + out[i] * 3;
        const float* b = g.positions.Data() + out[i + 1] * 3;
        const float* c = g.positions.Data() + out[i + 2] * 3;
        facing = facing && (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0;
    }
    HX_CHECK(facing);

    // A sphere: the error bound holds and limits how far it goes.
    MeshData sp = Sphere(32, 64);
    Scene s = ToScene(sp);
    SimplifyResult half = Simplify(s, 0, 0.25f, 0.05f);
    HX_CHECK(half.error <= 0.05f && half.error > 0);
    HX_CHECK(half.indexCount < sp.indices.Size() / 2);
    HX_CHECK_EQ(s.primitives[0].indices.count, half.indexCount);
    SimplifyResult tight = SimplifyIndices(sp.positions.Data(), sp.VertexCount(), sp.indices.Data(), sp.indices.Size(), 0, 1e-5f, out);
    HX_CHECK(tight.indexCount > sp.indices.Size() * 9 / 10);
    HX_CHECK(tight.error <= 1e-5f);

    // Every vertex of a cube with a quad per face is on a seam: nothing moves.
    MeshData cube = Cube();
    SimplifyResult locked = SimplifyIndices(cube.positions.Data(), 24, cube.indices.Data(), 36, 0, 1, out);
    HX_CHECK_EQ(locked.indexCount, 36u);

    // Out of range and degenerate triangles are dropped, a target above the count keeps the rest.
    const u32 bad[] = {0, 1, 2, 0, 0, 1, 0, 1, 99999};
    SimplifyResult cleaned = SimplifyIndices(g.positions.Data(), g.VertexCount(), bad, 9, 9, 0, out);
    HX_CHECK_EQ(cleaned.indexCount, 3u);
}

HX_TEST(Mesh, Meshlets)
{
    MeshData sp = Sphere(32, 64);
    Meshlets ml;
    BuildMeshlets(sp.positions.Data(), sp.VertexCount(), sp.indices.Data(), sp.indices.Size(), ml);
    HX_REQUIRE(!ml.meshlets.Empty());
    u32 triangles = 0;
    bool limits = true, bounds = true, cones = true;
    Array<u32> rebuilt;
    for (const Meshlet& m : ml.meshlets)
    {
        limits = limits && m.vertexCount <= kMeshletVertices && m.triangleCount <= kMeshletTriangles && m.triangleCount > 0;
        triangles += m.triangleCount;
        for (u32 i = 0; i < m.vertexCount; i++)
        {
            const float* p = sp.positions.Data() + ml.vertices[m.vertexOffset + i] * 3;
            float d = std::sqrt((p[0] - m.center[0]) * (p[0] - m.center[0]) + (p[1] - m.center[1]) * (p[1] - m.center[1]) + (p[2] - m.center[2]) * (p[2] - m.center[2]));
            bounds = bounds && d <= m.radius * 1.0001f;
        }
        for (u32 t = 0; t < m.triangleCount; t++)
        {
            u32 v[3];
            for (u32 k = 0; k < 3; k++)
            {
                u8 local = ml.triangles[m.triangleOffset + t * 3 + k];
                limits = limits && local < m.vertexCount;
                v[k] = ml.vertices[m.vertexOffset + local];
                rebuilt.Push(v[k]);
            }
            // The cone: viewed along its axis, inside the cutoff, every triangle faces away.
            if (m.coneCutoff <= 1)
            {
                const float* a = sp.positions.Data() + v[0] * 3;
                const float* b = sp.positions.Data() + v[1] * 3;
                const float* c = sp.positions.Data() + v[2] * 3;
                float e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
                float n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
                cones = cones && n[0] * m.coneAxis[0] + n[1] * m.coneAxis[1] + n[2] * m.coneAxis[2] > 0;
            }
        }
    }
    HX_CHECK(limits && bounds && cones);
    HX_CHECK_EQ(triangles, u32(sp.indices.Size() / 3));
    HX_CHECK(Same(Geometry(sp.positions.Data(), rebuilt), Geometry(sp.positions.Data(), sp.indices)));
    // Grown along shared vertices, meshlets fill up.
    HX_CHECK(float(triangles) / float(ml.meshlets.Size()) > 80);

    // A flat grid: one direction, a narrow cone.
    MeshData g = Grid(16);
    BuildMeshlets(g.positions.Data(), g.VertexCount(), g.indices.Data(), g.indices.Size(), ml, 32, 40);
    bool flat = true;
    for (const Meshlet& m : ml.meshlets)
        flat = flat && m.vertexCount <= 32 && m.triangleCount <= 40 && std::fabs(m.coneAxis[2] - 1) < 1e-6f && m.coneCutoff < 0.01f;
    HX_CHECK(flat);
}

HX_TEST(Mesh, BlendShapes)
{
    const u32 vertices = 1000;
    Array<float> deltas;
    deltas.Resize(vertices * 3, 0.0f);
    u32 moving = 0;
    for (u32 v = 0; v < vertices; v++)
        if (v % 10 == 3)
        {
            moving++;
            for (u32 c = 0; c < 3; c++)
                deltas[v * 3 + c] = std::sin(float(v * 3 + c)) * 0.2f;
        }
        else
            deltas[v * 3 + 1] = 1e-5f;
    CompressedDeltas packed;
    CompressDeltas(deltas.Data(), vertices, 3, 1e-4f, packed);
    HX_CHECK_EQ(u32(packed.vertices.Size()), moving);
    HX_CHECK_EQ(packed.values.Size(), usize(moving) * 3);
    HX_CHECK(packed.scale > 0 && packed.scale <= 0.2f / 32767 * 1.0001f);
    Array<float> back;
    DecompressDeltas(packed, vertices, back);
    HX_REQUIRE(back.Size() == deltas.Size());
    float worst = 0;
    for (usize i = 0; i < deltas.Size(); i++)
    {
        float e = std::fabs(back[i] - deltas[i]);
        bool kept = (i / 3) % 10 == 3;
        HX_CHECK(kept ? e <= packed.scale * 0.5f + 1e-9f : e <= 1e-4f);
        worst = e > worst ? e : worst;
    }
    HX_CHECK(worst > 0);

    // Nothing moving: an empty shape.
    Array<float> still;
    still.Resize(30, 0.0f);
    CompressDeltas(still.Data(), 10, 3, 0, packed);
    HX_CHECK(packed.vertices.Empty() && packed.scale == 0);
}
