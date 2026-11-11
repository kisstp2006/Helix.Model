#include "Common.h"

#include <algorithm>

namespace helix::model
{
    using namespace detail;

    namespace
    {
        float Min3(const float* v) { return v[0] < v[1] ? (v[0] < v[2] ? v[0] : v[2]) : (v[1] < v[2] ? v[1] : v[2]); }
        float Max3(const float* v) { return v[0] > v[1] ? (v[0] > v[2] ? v[0] : v[2]) : (v[1] > v[2] ? v[1] : v[2]); }
    }

    VertexCacheStats AnalyzeVertexCache(const u32* indices, usize indexCount, u32 vertexCount, u32 cacheSize)
    {
        VertexCacheStats stats;
        if (cacheSize == 0 || indexCount < 3)
            return stats;
        // A vertex is in the FIFO while fewer than cacheSize misses came after its own.
        Array<u64> stamp;
        stamp.Resize(vertexCount, u64(0));
        u64 misses = 0;
        usize triangles = 0;
        for (usize i = 0; i + 2 < indexCount; i += 3)
        {
            if (indices[i] >= vertexCount || indices[i + 1] >= vertexCount || indices[i + 2] >= vertexCount)
                continue;
            triangles++;
            for (u32 k = 0; k < 3; k++)
            {
                u32 v = indices[i + k];
                if (stamp[v] == 0 || misses + 1 - stamp[v] >= cacheSize)
                    stamp[v] = ++misses;
            }
        }
        stats.acmr = triangles ? float(double(misses) / double(triangles)) : 0;
        stats.atvr = vertexCount ? float(double(misses) / double(vertexCount)) : 0;
        return stats;
    }

    OverdrawStats AnalyzeOverdraw(const float* positions, const u32* indices, usize indexCount)
    {
        OverdrawStats stats;
        float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
        for (usize i = 0; i < indexCount; i++)
            for (u32 a = 0; a < 3; a++)
            {
                float f = positions[usize(indices[i]) * 3 + a];
                if (std::isfinite(f))
                    lo[a] = f < lo[a] ? f : lo[a], hi[a] = f > hi[a] ? f : hi[a];
            }
        float extent = 0;
        for (u32 a = 0; a < 3; a++)
            extent = hi[a] - lo[a] > extent ? hi[a] - lo[a] : extent;
        if (!(extent > 0) || !std::isfinite(extent))
            return stats;

        constexpr i32 kSize = 256;
        Array<float> depth;
        Array<u8> covered;
        depth.Resize(usize(kSize) * kSize);
        covered.Resize(usize(kSize) * kSize);
        u64 shaded = 0, pixels = 0;
        float scale = (kSize - 1) / extent;
        for (u32 axis = 0; axis < 3; axis++)
            for (u32 side = 0; side < 2; side++)
            {
                u32 ua = (axis + 1) % 3, va = (axis + 2) % 3;
                float flip = side ? -1.0f : 1.0f;
                for (usize i = 0; i < depth.Size(); i++)
                    depth[i] = 1e30f, covered[i] = 0;
                for (usize i = 0; i + 2 < indexCount; i += 3)
                {
                    float x[3], y[3], z[3];
                    bool finite = true;
                    for (u32 k = 0; k < 3; k++)
                    {
                        const float* p = positions + usize(indices[i + k]) * 3;
                        // Looking along -axis from the + side or along +axis from the - side; u flips to keep the
                        // winding of front faces.
                        x[k] = side ? (hi[ua] - p[ua]) * scale : (p[ua] - lo[ua]) * scale;
                        y[k] = (p[va] - lo[va]) * scale;
                        z[k] = (p[axis] - lo[axis]) * scale * -flip;
                        finite = finite && std::isfinite(x[k]) && std::isfinite(y[k]) && std::isfinite(z[k]);
                    }
                    float area = (x[1] - x[0]) * (y[2] - y[0]) - (y[1] - y[0]) * (x[2] - x[0]);
                    if (!finite || !(area > 0))
                        continue;
                    auto clamp = [](float f) { return i32(f < 0 ? 0 : f > kSize ? kSize : f); };
                    i32 x0 = clamp(Min3(x)), x1 = clamp(Max3(x) + 1);
                    i32 y0 = clamp(Min3(y)), y1 = clamp(Max3(y) + 1);
                    float inv = 1 / area;
                    bool owns[3];
                    for (u32 k = 0; k < 3; k++)
                    {
                        // Edge k runs from corner k + 1 to corner k + 2.
                        float dx = x[(k + 2) % 3] - x[(k + 1) % 3], dy = y[(k + 2) % 3] - y[(k + 1) % 3];
                        owns[k] = dy > 0 || (dy == 0 && dx < 0);
                    }
                    for (i32 py = y0; py < y1; py++)
                        for (i32 px = x0; px < x1; px++)
                        {
                            float cx = float(px) + 0.5f, cy = float(py) + 0.5f;
                            float w0 = (x[2] - x[1]) * (cy - y[1]) - (y[2] - y[1]) * (cx - x[1]);
                            float w1 = (x[0] - x[2]) * (cy - y[2]) - (y[0] - y[2]) * (cx - x[2]);
                            float w2 = (x[1] - x[0]) * (cy - y[0]) - (y[1] - y[0]) * (cx - x[0]);
                            // Pixels on an edge belong to one of the two triangles sharing it.
                            if (!(w0 > 0 || (w0 == 0 && owns[0])) || !(w1 > 0 || (w1 == 0 && owns[1])) || !(w2 > 0 || (w2 == 0 && owns[2])))
                                continue;
                            float d = (w0 * z[0] + w1 * z[1] + w2 * z[2]) * inv;
                            usize at = usize(py) * kSize + usize(px);
                            if (d <= depth[at])
                            {
                                depth[at] = d;
                                shaded++;
                                if (!covered[at])
                                    covered[at] = 1, pixels++;
                            }
                        }
                }
            }
        stats.overdraw = pixels ? float(double(shaded) / double(pixels)) : 0;
        return stats;
    }

    namespace detail
    {
        namespace
        {
            constexpr u32 kCacheSize = 32;

            float VertexScore(i32 cachePosition, u32 remaining)
            {
                if (remaining == 0)
                    return -1;
                float score = 0;
                if (cachePosition >= 0)
                {
                    if (cachePosition < 3)
                        score = 0.75f;
                    else
                        score = std::pow(1 - float(cachePosition - 3) / float(kCacheSize - 3), 1.5f);
                }
                return score + 2 / std::sqrt(float(remaining));
            }
        }

        void OptimizeCacheIndices(const u32* indices, usize indexCount, u32 vertexCount, u32* out)
        {
            usize triangles = indexCount / 3;
            if (triangles == 0)
                return;
            // The triangles of every vertex not yet emitted (CSR, shrunk as triangles are emitted).
            Array<u32> start, count, adjacency;
            start.Resize(usize(vertexCount) + 1, 0u);
            count.Resize(vertexCount, 0u);
            for (usize i = 0; i < triangles * 3; i++)
                count[indices[i]]++;
            for (u32 v = 0; v < vertexCount; v++)
                start[v + 1] = start[v] + count[v];
            adjacency.Resize(triangles * 3);
            {
                Array<u32> fill;
                fill.Resize(vertexCount, 0u);
                for (usize i = 0; i < triangles * 3; i++)
                {
                    u32 v = indices[i];
                    adjacency[start[v] + fill[v]++] = u32(i / 3);
                }
            }
            Array<i32> position;
            position.Resize(vertexCount, -1);
            Array<float> vertexScore, triangleScore;
            vertexScore.Resize(vertexCount);
            for (u32 v = 0; v < vertexCount; v++)
                vertexScore[v] = VertexScore(-1, count[v]);
            triangleScore.Resize(triangles);
            for (usize t = 0; t < triangles; t++)
                triangleScore[t] = vertexScore[indices[t * 3]] + vertexScore[indices[t * 3 + 1]] + vertexScore[indices[t * 3 + 2]];
            Array<u8> emitted;
            emitted.Resize(triangles, u8(0));

            u32 cache[kCacheSize + 3], next[kCacheSize + 3];
            u32 cached = 0;
            usize cursor = 0, written = 0;
            u32 best = 0;
            for (usize t = 1; t < triangles; t++)
                if (triangleScore[t] > triangleScore[best])
                    best = u32(t);
            while (best != kNone)
            {
                emitted[best] = 1;
                const u32* tri = indices + usize(best) * 3;
                for (u32 k = 0; k < 3; k++)
                {
                    u32 v = tri[k];
                    out[written++] = v;
                    // Drop the triangle from the vertex's list (once even for a repeated index).
                    u32* list = adjacency.Data() + start[v];
                    for (u32 j = 0; j < count[v]; j++)
                        if (list[j] == best)
                        {
                            list[j] = list[--count[v]];
                            break;
                        }
                }
                // The triangle's vertices to the front of the cache, the others behind them.
                u32 made = 0;
                for (u32 k = 0; k < 3; k++)
                {
                    bool seen = false;
                    for (u32 j = 0; j < made; j++)
                        seen = seen || next[j] == tri[k];
                    if (!seen)
                        next[made++] = tri[k];
                }
                for (u32 j = 0; j < cached; j++)
                    if (cache[j] != tri[0] && cache[j] != tri[1] && cache[j] != tri[2])
                        next[made++] = cache[j];
                // Vertices pushed out lose their position.
                for (u32 j = kCacheSize; j < made; j++)
                    position[next[j]] = -1;
                cached = made < kCacheSize ? made : kCacheSize;
                for (u32 j = 0; j < made; j++)
                    cache[j] = next[j];
                for (u32 j = 0; j < made; j++)
                {
                    u32 v = cache[j];
                    position[v] = j < kCacheSize ? i32(j) : -1;
                    vertexScore[v] = VertexScore(position[v], count[v]);
                }
                best = kNone;
                float bestScore = -1e30f;
                for (u32 j = 0; j < made; j++)
                {
                    u32 v = cache[j];
                    for (u32 a = 0; a < count[v]; a++)
                    {
                        u32 t = adjacency[start[v] + a];
                        const u32* o = indices + usize(t) * 3;
                        triangleScore[t] = vertexScore[o[0]] + vertexScore[o[1]] + vertexScore[o[2]];
                        if (triangleScore[t] > bestScore)
                            bestScore = triangleScore[t], best = t;
                    }
                }
                if (best == kNone)
                {
                    // A dead end: the next triangle in the input order.
                    while (cursor < triangles && emitted[cursor])
                        cursor++;
                    best = cursor < triangles ? u32(cursor) : kNone;
                }
            }
        }
    }

    void OptimizeVertexCache(Scene& s, u32 primitive)
    {
        Array<u32> indices;
        if (!TriangleIndices(s, primitive, indices))
            return;
        Array<u32> out;
        out.Resize(indices.Size());
        OptimizeCacheIndices(indices.Data(), indices.Size(), s.primitives[primitive].vertexCount, out.Data());
        SetIndices(s, primitive, out);
    }

    void OptimizeOverdraw(Scene& s, u32 primitive, float threshold)
    {
        Array<u32> input;
        Array<float> positions;
        if (!TriangleIndices(s, primitive, input) || !Positions(s, primitive, positions))
            return;
        u32 vertexCount = s.primitives[primitive].vertexCount;
        usize triangles = input.Size() / 3;
        if (triangles == 0)
            return;
        Array<u32> indices;
        indices.Resize(input.Size());
        OptimizeCacheIndices(input.Data(), input.Size(), vertexCount, indices.Data());

        // Hard boundaries: triangles whose three vertices all miss a 16 entry FIFO.
        constexpr u32 kFifo = 16;
        Array<u32> clusters;
        {
            Array<u64> stamp;
            stamp.Resize(vertexCount, u64(0));
            u64 misses = 0;
            for (usize t = 0; t < triangles; t++)
            {
                u32 m = 0;
                for (u32 k = 0; k < 3; k++)
                {
                    u32 v = indices[t * 3 + k];
                    if (stamp[v] == 0 || misses + 1 - stamp[v] >= kFifo)
                        stamp[v] = ++misses, m++;
                }
                if (t == 0 || m == 3)
                    clusters.Push(u32(t));
            }
        }
        clusters.Push(u32(triangles));

        // Soft boundaries: each cluster split where its own cache efficiency, from a cold cache, gets within the threshold
        // of the whole cluster's.
        Array<u32> split;
        {
            Array<u64> stamp;
            stamp.Resize(vertexCount, u64(0));
            u64 misses = 0;
            auto run = [&](usize from, usize to) {
                // Cold: every stamp older than the cache.
                misses += kFifo + 1;
                u64 before = misses;
                for (usize t = from; t < to; t++)
                    for (u32 k = 0; k < 3; k++)
                    {
                        u32 v = indices[t * 3 + k];
                        if (stamp[v] == 0 || misses + 1 - stamp[v] >= kFifo)
                            stamp[v] = ++misses;
                    }
                return misses - before;
            };
            for (usize c = 0; c + 1 < clusters.Size(); c++)
            {
                usize from = clusters[c], to = clusters[c + 1];
                double acmr = double(run(from, to)) / double(to - from);
                usize begin = from;
                misses += kFifo + 1;
                u64 before = misses;
                split.Push(u32(from));
                for (usize t = from; t < to; t++)
                {
                    for (u32 k = 0; k < 3; k++)
                    {
                        u32 v = indices[t * 3 + k];
                        if (stamp[v] == 0 || misses + 1 - stamp[v] >= kFifo)
                            stamp[v] = ++misses;
                    }
                    if (t + 1 < to && double(misses - before) / double(t + 1 - begin) <= acmr * threshold)
                    {
                        split.Push(u32(t + 1));
                        begin = t + 1;
                        misses += kFifo + 1;
                        before = misses;
                    }
                }
            }
        }
        split.Push(u32(triangles));

        // Clusters ordered by how far out they face: dot(centroid - mesh centroid, cluster normal), largest first.
        Vec3 meshCenter;
        float meshArea = 0;
        struct Cluster
        {
            u32 begin, end;
            Vec3 center, normal;
            float area;
            float key;
        };
        Array<Cluster> list;
        for (usize c = 0; c + 1 < split.Size(); c++)
        {
            Cluster cl{split[c], split[c + 1], {}, {}, 0, 0};
            for (u32 t = cl.begin; t < cl.end; t++)
            {
                Vec3 p0 = Load(positions.Data() + usize(indices[t * 3]) * 3), p1 = Load(positions.Data() + usize(indices[t * 3 + 1]) * 3),
                     p2 = Load(positions.Data() + usize(indices[t * 3 + 2]) * 3);
                Vec3 n = Cross(p1 - p0, p2 - p0);
                float area = Length(n);
                cl.normal = cl.normal + n;
                cl.center = cl.center + (p0 + p1 + p2) * (area / 3);
                cl.area += area;
            }
            meshCenter = meshCenter + cl.center;
            meshArea += cl.area;
            list.Push(cl);
        }
        meshCenter = meshArea > 0 ? meshCenter * (1 / meshArea) : Vec3{};
        for (Cluster& cl : list)
        {
            Vec3 center = cl.area > 0 ? cl.center * (1 / cl.area) : meshCenter;
            float key = Dot(center - meshCenter, Normalize(cl.normal));
            cl.key = std::isfinite(key) ? key : 0;
        }
        std::stable_sort(list.begin(), list.end(), [](const Cluster& a, const Cluster& b) { return a.key > b.key; });
        Array<u32> out;
        out.Reserve(indices.Size());
        for (const Cluster& cl : list)
            out.Append(indices.Data() + usize(cl.begin) * 3, usize(cl.end - cl.begin) * 3);
        SetIndices(s, primitive, out);
    }

    void OptimizeVertexFetch(Scene& s, u32 primitive)
    {
        Primitive& p = s.primitives[primitive];
        if (p.indices.count == 0)
            return;
        Array<u32> remap, order, indices;
        remap.Resize(p.vertexCount, kNone);
        for (u32 i = 0; i < p.indices.count; i++)
        {
            u32 idx = s.indices[p.indices.begin + i];
            if (idx < p.vertexCount && remap[idx] == kNone)
            {
                remap[idx] = u32(order.Size());
                order.Push(idx);
            }
            // Indices outside the vertices stay outside.
            indices.Push(idx < p.vertexCount ? remap[idx] : idx);
        }
        GatherVertices(s, primitive, order);
        SetIndices(s, primitive, indices);
    }
}
