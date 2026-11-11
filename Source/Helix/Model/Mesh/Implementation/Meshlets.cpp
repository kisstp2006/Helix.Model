#include "Common.h"

namespace helix::model
{
    using namespace detail;

    namespace
    {
        void Bounds(const float* positions, const Meshlets& out, Meshlet& m)
        {
            const u32* vertices = out.vertices.Data() + m.vertexOffset;
            float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
            for (u32 i = 0; i < m.vertexCount; i++)
                for (u32 k = 0; k < 3; k++)
                {
                    float f = positions[usize(vertices[i]) * 3 + k];
                    lo[k] = f < lo[k] ? f : lo[k], hi[k] = f > hi[k] ? f : hi[k];
                }
            Vec3 center = {(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2};
            float radius = 0;
            for (u32 i = 0; i < m.vertexCount; i++)
            {
                float d = Length(Load(positions + usize(vertices[i]) * 3) - center);
                radius = d > radius ? d : radius;
            }
            m.center[0] = center.x, m.center[1] = center.y, m.center[2] = center.z;
            m.radius = radius;

            // The cone around the mean of the unit face normals; half angle t gives cutoff sin(t) while t < 90 degrees.
            const u8* tris = out.triangles.Data() + m.triangleOffset;
            Array<Vec3> normals;
            Vec3 sum;
            for (u32 t = 0; t < m.triangleCount; t++)
            {
                Vec3 p0 = Load(positions + usize(vertices[tris[t * 3]]) * 3), p1 = Load(positions + usize(vertices[tris[t * 3 + 1]]) * 3),
                     p2 = Load(positions + usize(vertices[tris[t * 3 + 2]]) * 3);
                Vec3 n = Normalize(Cross(p1 - p0, p2 - p0));
                if (Dot(n, n) == 0)
                    continue;
                normals.Push(n);
                sum = sum + n;
            }
            Vec3 axis = Normalize(sum);
            m.coneCutoff = 2;
            if (normals.Empty() || Dot(axis, axis) == 0)
                return;
            m.coneAxis[0] = axis.x, m.coneAxis[1] = axis.y, m.coneAxis[2] = axis.z;
            float least = 1;
            for (Vec3 n : normals)
            {
                float d = Dot(n, axis);
                least = d < least ? d : least;
            }
            if (least > 0)
            {
                // Rounded up a little so the test stays conservative.
                float cutoff = std::sqrt(1 - least * least) * 1.0001f + 1e-4f;
                m.coneCutoff = cutoff < 1 ? cutoff : 1;
            }
        }
    }

    void BuildMeshlets(const float* positions, u32 vertexCount, const u32* indices, usize indexCount, Meshlets& out, u32 maxVertices, u32 maxTriangles)
    {
        out.meshlets.Clear(), out.vertices.Clear(), out.triangles.Clear();
        maxVertices = maxVertices < 3 ? 3 : maxVertices > 256 ? 256 : maxVertices;
        maxTriangles = maxTriangles < 1 ? 1 : maxTriangles;
        Array<u32> list;
        for (usize i = 0; i + 2 < indexCount; i += 3)
            if (indices[i] < vertexCount && indices[i + 1] < vertexCount && indices[i + 2] < vertexCount)
                list.Push(indices[i]), list.Push(indices[i + 1]), list.Push(indices[i + 2]);
        usize triangles = list.Size() / 3;
        if (triangles == 0)
            return;

        // The triangles of every vertex.
        Array<u32> start, adjacency;
        start.Resize(usize(vertexCount) + 1, 0u);
        for (u32 v : list)
            start[v + 1]++;
        for (u32 v = 0; v < vertexCount; v++)
            start[v + 1] += start[v];
        adjacency.Resize(list.Size());
        {
            Array<u32> fill;
            fill.Resize(vertexCount);
            std::memcpy(fill.Data(), start.Data(), vertexCount * sizeof(u32));
            for (usize i = 0; i < list.Size(); i++)
                adjacency[fill[list[i]]++] = u32(i / 3);
        }

        Array<u8> emitted;
        emitted.Resize(triangles, u8(0));
        // The local index of each vertex in the meshlet being built.
        Array<u32> local;
        local.Resize(vertexCount, kNone);
        usize cursor = 0;
        Meshlet m;
        auto fresh = [&](u32 t) {
            const u32* tri = list.Data() + usize(t) * 3;
            u32 n = 0;
            for (u32 k = 0; k < 3; k++)
                if (local[tri[k]] == kNone && (k == 0 || tri[k] != tri[0]) && (k < 2 || tri[k] != tri[1]))
                    n++;
            return n;
        };
        auto finish = [&]() {
            if (m.triangleCount == 0)
                return;
            Bounds(positions, out, m);
            for (u32 i = 0; i < m.vertexCount; i++)
                local[out.vertices[m.vertexOffset + i]] = kNone;
            out.meshlets.Push(m);
            m = Meshlet();
            m.vertexOffset = u32(out.vertices.Size());
            m.triangleOffset = u32(out.triangles.Size());
        };
        for (;;)
        {
            // The triangle around the meshlet's vertices that adds the fewest vertices.
            u32 best = kNone, bestFresh = 4;
            for (u32 i = 0; i < m.vertexCount && bestFresh > 0; i++)
            {
                u32 v = out.vertices[m.vertexOffset + i];
                for (u32 a = start[v]; a < start[v + 1]; a++)
                {
                    u32 t = adjacency[a];
                    if (emitted[t])
                        continue;
                    u32 n = fresh(t);
                    if (n < bestFresh)
                        bestFresh = n, best = t;
                }
            }
            if (best == kNone)
            {
                // Nothing connected: the next triangle in order.
                while (cursor < triangles && emitted[cursor])
                    cursor++;
                if (cursor == triangles)
                    break;
                best = u32(cursor);
                bestFresh = fresh(best);
            }
            if (m.vertexCount + bestFresh > maxVertices || m.triangleCount + 1 > maxTriangles)
            {
                finish();
                continue;
            }
            emitted[best] = 1;
            const u32* tri = list.Data() + usize(best) * 3;
            for (u32 k = 0; k < 3; k++)
            {
                u32 v = tri[k];
                if (local[v] == kNone)
                {
                    local[v] = m.vertexCount++;
                    out.vertices.Push(v);
                }
                out.triangles.Push(u8(local[v]));
            }
            m.triangleCount++;
        }
        finish();
    }
}
