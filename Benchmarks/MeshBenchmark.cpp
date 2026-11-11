#include <Support/SampleMeshes.h>

#include <Helix/Model/Mesh/Mesh.h>
#include <Helix/Testing/Bench.h>

#include <cmath>

using namespace helix;
using namespace helix::model;
using namespace helix::model::testing;

namespace
{
    // A UV sphere of 256 x 256 quads (130560 triangles) in row order, and with its triangles scrambled for the
    // optimizers.
    const MeshData& Large(bool scrambled = false)
    {
        static const MeshData rows = Sphere(256, 256);
        static const MeshData mixed = [] {
            MeshData d = Sphere(256, 256);
            Shuffle(d, 3);
            return d;
        }();
        return scrambled ? mixed : rows;
    }

    u64 Triangles() { return Large().indices.Size() / 3; }

    // Each round works on a fresh copy of the scene; the copy is a small part of the time.
    template <typename F>
    void Run(helix::testing::BenchState& state, F&& f, bool scrambled = false)
    {
        const Scene base = ToScene(Large(scrambled));
        state.SetItemsPerIteration(Triangles());
        while (state.KeepRunning())
        {
            Scene s = base;
            f(s);
            helix::testing::DoNotOptimize(s.indices.Data());
        }
    }
}

// Items are triangles.
HX_BENCH(MeshWeld)
{
    Run(state, [](Scene& s) { WeldVertices(s, 0); });
}

HX_BENCH(MeshNormals)
{
    Run(state, [](Scene& s) { CalculateNormals(s, 0, 60); });
}

HX_BENCH(MeshTangents)
{
    Run(state, [](Scene& s) { CalculateTangents(s, 0); });
}

HX_BENCH(MeshVertexCache)
{
    Run(state, [](Scene& s) { OptimizeVertexCache(s, 0); }, true);
}

HX_BENCH(MeshOverdraw)
{
    Run(state, [](Scene& s) { OptimizeOverdraw(s, 0); }, true);
}

HX_BENCH(MeshVertexFetch)
{
    Run(state, [](Scene& s) { OptimizeVertexFetch(s, 0); }, true);
}

HX_BENCH(MeshSimplify)
{
    Run(state, [](Scene& s) { Simplify(s, 0, 0.1f, 0.05f); });
}

HX_BENCH(MeshMeshlets)
{
    const MeshData& m = Large();
    state.SetItemsPerIteration(Triangles());
    Meshlets out;
    while (state.KeepRunning())
    {
        BuildMeshlets(m.positions.Data(), m.VertexCount(), m.indices.Data(), m.indices.Size(), out);
        helix::testing::DoNotOptimize(out.meshlets.Data());
    }
}

// Items are vertices.
HX_BENCH(MeshCompressDeltas)
{
    const MeshData& m = Large();
    Array<float> deltas;
    for (usize i = 0; i < m.positions.Size(); i++)
        deltas.Push(m.positions[i] > 0.5f ? m.positions[i] * 0.1f : 0);
    state.SetItemsPerIteration(m.VertexCount());
    CompressedDeltas out;
    while (state.KeepRunning())
    {
        CompressDeltas(deltas.Data(), m.VertexCount(), 3, 1e-4f, out);
        helix::testing::DoNotOptimize(out.values.Data());
    }
}
