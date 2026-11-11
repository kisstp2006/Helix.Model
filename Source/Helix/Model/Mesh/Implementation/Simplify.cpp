#include "Common.h"

#include <algorithm>

// Edge collapse driven by quadric error metrics (Garland and Heckbert, "Surface Simplification Using Quadric Error
// Metrics"): every vertex sums the planes of its triangles weighted by area, border edges add a plane through the edge
// perpendicular to its triangle; a vertex collapses onto a neighbor at the cost of the summed quadrics at the neighbor's
// position. Collapses run in passes of independent edges, cheapest first.
namespace helix::model
{
    using namespace detail;

    namespace
    {
        struct Quadric
        {
            // xx, xy, xz, yy, yz, zz, then b and c, and the weight they were summed with.
            double a[6] = {}, b[3] = {}, c = 0, w = 0;
        };

        struct Point
        {
            double x = 0, y = 0, z = 0;
        };

        Point operator-(Point a, Point b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
        Point Cross(Point a, Point b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
        double Dot(Point a, Point b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
        double Length(Point a) { return std::sqrt(Dot(a, a)); }

        void AddPlane(Quadric& q, Point n, double d, double weight)
        {
            q.a[0] += weight * n.x * n.x, q.a[1] += weight * n.x * n.y, q.a[2] += weight * n.x * n.z;
            q.a[3] += weight * n.y * n.y, q.a[4] += weight * n.y * n.z, q.a[5] += weight * n.z * n.z;
            q.b[0] += weight * d * n.x, q.b[1] += weight * d * n.y, q.b[2] += weight * d * n.z;
            q.c += weight * d * d;
            q.w += weight;
        }

        void Add(Quadric& q, const Quadric& o)
        {
            for (u32 i = 0; i < 6; i++)
                q.a[i] += o.a[i];
            for (u32 i = 0; i < 3; i++)
                q.b[i] += o.b[i];
            q.c += o.c, q.w += o.w;
        }

        // The summed squared distances to the planes, by their weights.
        double Distance(const Quadric& q, Point p)
        {
            return q.a[0] * p.x * p.x + 2 * q.a[1] * p.x * p.y + 2 * q.a[2] * p.x * p.z + q.a[3] * p.y * p.y + 2 * q.a[4] * p.y * p.z + q.a[5] * p.z * p.z +
                   2 * (q.b[0] * p.x + q.b[1] * p.y + q.b[2] * p.z) + q.c;
        }

        // The mean squared distance to the planes of two quadrics together.
        double Error(const Quadric& a, const Quadric& b, Point p)
        {
            double w = a.w + b.w;
            double e = w > 0 ? (Distance(a, p) + Distance(b, p)) / w : 0;
            return e > 0 ? e : 0;
        }

        enum class Kind : u8
        {
            Interior,
            Border,
            Locked,
        };

        struct Collapse
        {
            u32 from, to;
            double cost;
        };

        // Border edges weigh like a triangle of this many times their squared length.
        constexpr double kBorderWeight = 10;
    }

    SimplifyResult SimplifyIndices(const float* positions, u32 vertexCount, const u32* indices, usize indexCount, usize targetIndexCount, float targetError,
                                   Array<u32>& out)
    {
        SimplifyResult result;
        out.Clear();
        for (usize i = 0; i + 2 < indexCount; i += 3)
        {
            u32 a = indices[i], b = indices[i + 1], c = indices[i + 2];
            if (a < vertexCount && b < vertexCount && c < vertexCount && a != b && b != c && a != c)
                out.Push(a), out.Push(b), out.Push(c);
        }
        result.indexCount = u32(out.Size());
        if (targetIndexCount >= out.Size())
            return result;

        // Positions relative to the mesh's size, so errors are too.
        double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
        for (u32 v : out)
            for (u32 k = 0; k < 3; k++)
            {
                double f = positions[usize(v) * 3 + k];
                lo[k] = f < lo[k] ? f : lo[k], hi[k] = f > hi[k] ? f : hi[k];
            }
        double extent = 0;
        for (u32 k = 0; k < 3; k++)
            extent = hi[k] - lo[k] > extent ? hi[k] - lo[k] : extent;
        if (!(extent > 0) || !std::isfinite(extent))
            return result;
        Array<Point> pos;
        pos.Resize(vertexCount);
        for (u32 v = 0; v < vertexCount; v++)
        {
            const float* p = positions + usize(v) * 3;
            pos[v] = {(p[0] - lo[0]) / extent, (p[1] - lo[1]) / extent, (p[2] - lo[2]) / extent};
        }

        // Vertices sharing a position with another used vertex lie on an attribute seam and stay.
        Array<u32> group;
        u32 groups = GroupEqual(positions, vertexCount, 3, group);
        Array<u32> usedInGroup;
        usedInGroup.Resize(groups, 0u);
        {
            Array<u8> used;
            used.Resize(vertexCount, u8(0));
            for (u32 v : out)
                if (!used[v])
                    used[v] = 1, usedInGroup[group[v]]++;
        }
        auto seam = [&](u32 v) { return usedInGroup[group[v]] > 1; };

        Array<Quadric> quadrics;
        quadrics.Resize(vertexCount);
        Array<Kind> kind;
        Array<u32> borderOut, borderIn;
        Array<u32> start, adjacency;
        // Per corner: the edge to the next corner of its triangle has its opposite.
        Array<u8> opposite;
        Array<u8> touched;
        Array<Collapse> candidates;
        // Cost (as float bits, which order like the costs) and candidate, sorted.
        Array<u64> order;
        // The position of every corner.
        Array<u32> corner;
        Array<u32> collapse;
        collapse.Resize(vertexCount, kNone);
        double limit = double(targetError) * double(targetError);
        double reached = 0;
        // Triangles around position ga with an edge from ga to gb.
        auto edges = [&](u32 ga, u32 gb) {
            u32 n = 0;
            for (u32 i = start[ga]; i < start[ga + 1]; i++)
            {
                const u32* tri = corner.Data() + usize(adjacency[i]) * 3;
                n += (tri[0] == ga && tri[1] == gb) + (tri[1] == ga && tri[2] == gb) + (tri[2] == ga && tri[0] == gb);
            }
            return n;
        };

        for (bool first = true;; first = false)
        {
            usize triangles = out.Size() / 3;
            corner.Resize(out.Size());
            for (usize i = 0; i < out.Size(); i++)
                corner[i] = group[out[i]];
            // The triangles around every position.
            start.Clear(), adjacency.Clear();
            start.Resize(usize(groups) + 1, 0u);
            for (u32 v : out)
                start[group[v] + 1]++;
            for (u32 g = 0; g < groups; g++)
                start[g + 1] += start[g];
            adjacency.Resize(out.Size());
            {
                Array<u32> fill;
                fill.Resize(groups);
                std::memcpy(fill.Data(), start.Data(), groups * sizeof(u32));
                for (usize i = 0; i < out.Size(); i++)
                    adjacency[fill[group[out[i]]]++] = u32(i / 3);
            }

            // Topology by position: an edge is a border without its opposite, non-manifold when used twice one way.
            opposite.Clear();
            opposite.Resize(out.Size(), u8(0));
            kind.Clear();
            kind.Resize(groups, Kind::Interior);
            borderOut.Clear(), borderIn.Clear();
            borderOut.Resize(groups, kNone), borderIn.Resize(groups, kNone);
            for (usize i = 0; i < out.Size(); i++)
            {
                u32 a = out[i], b = out[i - i % 3 + (i + 1) % 3];
                u32 ga = group[a], gb = group[b];
                if (edges(ga, gb) > 1)
                {
                    kind[ga] = kind[gb] = Kind::Locked;
                    opposite[i] = 1;
                    continue;
                }
                opposite[i] = edges(gb, ga) > 0;
                if (opposite[i])
                    continue;
                if (first)
                {
                    // The plane through the edge, perpendicular to its triangle.
                    u32 c = out[i - i % 3 + (i + 2) % 3];
                    Point edge = pos[b] - pos[a];
                    Point normal = Cross(edge, Cross(edge, pos[c] - pos[a]));
                    double l = Length(normal);
                    if (l > 0)
                    {
                        Point n = {normal.x / l, normal.y / l, normal.z / l};
                        double weight = Dot(edge, edge) * kBorderWeight;
                        AddPlane(quadrics[a], n, -Dot(n, pos[a]), weight);
                        AddPlane(quadrics[b], n, -Dot(n, pos[a]), weight);
                    }
                }
                // Two border edges leaving or entering one position: a vertex where borders meet.
                kind[ga] = kind[ga] == Kind::Locked || borderOut[ga] != kNone ? Kind::Locked : Kind::Border;
                kind[gb] = kind[gb] == Kind::Locked || borderIn[gb] != kNone ? Kind::Locked : Kind::Border;
                borderOut[ga] = gb, borderIn[gb] = ga;
            }
            if (first)
                for (usize t = 0; t < triangles; t++)
                {
                    u32 a = out[t * 3], b = out[t * 3 + 1], c = out[t * 3 + 2];
                    Point n = Cross(pos[b] - pos[a], pos[c] - pos[a]);
                    double area = Length(n);
                    if (!(area > 0))
                        continue;
                    n = {n.x / area, n.y / area, n.z / area};
                    double d = -Dot(n, pos[a]);
                    AddPlane(quadrics[a], n, d, area * 0.5);
                    AddPlane(quadrics[b], n, d, area * 0.5);
                    AddPlane(quadrics[c], n, d, area * 0.5);
                }

            // The cheaper allowed direction of every edge (interior edges once, from the side where they run up),
            // cheapest first.
            candidates.Clear();
            for (usize i = 0; i < out.Size(); i++)
            {
                u32 a = out[i], b = out[i - i % 3 + (i + 1) % 3];
                if (a > b && opposite[i])
                    continue;
                Collapse best = {kNone, kNone, 1e300};
                for (u32 dir = 0; dir < 2; dir++)
                {
                    u32 from = dir ? b : a, to = dir ? a : b;
                    u32 gf = group[from], gt = group[to];
                    if (gf == gt || seam(from) || seam(to) || kind[gf] == Kind::Locked)
                        continue;
                    if (kind[gf] == Kind::Border && borderOut[gf] != gt && borderIn[gf] != gt)
                        continue;
                    double cost = Error(quadrics[from], quadrics[to], pos[to]);
                    cost = std::isfinite(cost) ? cost : 1e300;
                    if (best.from == kNone || cost < best.cost)
                        best = {from, to, cost};
                }
                if (best.from != kNone)
                    candidates.Push(best);
            }
            order.Resize(candidates.Size());
            for (usize i = 0; i < candidates.Size(); i++)
            {
                float f = float(candidates[i].cost);
                u32 bits;
                std::memcpy(&bits, &f, 4);
                order[i] = (u64(bits) << 32) | u32(i);
            }
            std::sort(order.begin(), order.end());

            // This pass goes up to one and a half times the cost of the collapse that would reach the goal (two
            // triangles each), so the cheap collapses of the next pass are not taken over by expensive ones now.
            usize goal = targetIndexCount / 3, left = triangles;
            double passLimit = limit;
            {
                usize needed = (left - goal + 1) / 2;
                if (needed < candidates.Size())
                {
                    double at = candidates[u32(order[needed])].cost * 1.5;
                    passLimit = at < limit ? at : limit;
                }
            }
            touched.Clear();
            touched.Resize(vertexCount, u8(0));
            auto moved = [&](u32 v) { return collapse[v] != kNone ? collapse[v] : v; };
            u32 done = 0;
            for (u64 o : order)
            {
                const Collapse& c = candidates[u32(o)];
                if (c.cost > passLimit || left <= goal)
                    break;
                if (touched[c.from] || touched[c.to])
                    continue;
                // Triangles of the vertex as this pass left them: those with the target go, the others must not turn.
                usize removed = 0;
                bool flips = false;
                for (u32 a = start[group[c.from]]; a < start[group[c.from] + 1] && !flips; a++)
                {
                    const u32* tri = out.Data() + usize(adjacency[a]) * 3;
                    u32 v[3] = {moved(tri[0]), moved(tri[1]), moved(tri[2])};
                    if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2])
                        continue;
                    if (v[0] == c.to || v[1] == c.to || v[2] == c.to)
                    {
                        removed++;
                        continue;
                    }
                    Point p[3], q[3];
                    for (u32 k = 0; k < 3; k++)
                        p[k] = pos[v[k]], q[k] = v[k] == c.from ? pos[c.to] : pos[v[k]];
                    Point n0 = Cross(p[1] - p[0], p[2] - p[0]), n1 = Cross(q[1] - q[0], q[2] - q[0]);
                    flips = Dot(n0, n1) <= 0.25 * Length(n0) * Length(n1);
                }
                if (flips)
                    continue;
                collapse[c.from] = c.to;
                Add(quadrics[c.to], quadrics[c.from]);
                touched[c.from] = touched[c.to] = 1;
                left -= removed < left ? removed : left;
                reached = c.cost > reached ? c.cost : reached;
                done++;
            }
            if (done == 0)
                break;
            usize written = 0;
            for (usize t = 0; t < triangles; t++)
            {
                u32 v[3] = {moved(out[t * 3]), moved(out[t * 3 + 1]), moved(out[t * 3 + 2])};
                if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2])
                    continue;
                out[written++] = v[0], out[written++] = v[1], out[written++] = v[2];
            }
            out.Resize(written);
            for (u32& c : collapse)
                c = kNone;
            if (out.Size() <= targetIndexCount)
                break;
        }
        result.indexCount = u32(out.Size());
        result.error = float(std::sqrt(reached));
        return result;
    }

    SimplifyResult Simplify(Scene& s, u32 primitive, float ratio, float targetError)
    {
        Array<u32> indices, out;
        Array<float> positions;
        if (!TriangleIndices(s, primitive, indices) || !Positions(s, primitive, positions))
            return {};
        ratio = ratio < 0 ? 0 : ratio > 1 ? 1 : ratio;
        usize target = usize(double(indices.Size() / 3) * ratio) * 3;
        SimplifyResult result = SimplifyIndices(positions.Data(), s.primitives[primitive].vertexCount, indices.Data(), indices.Size(), target, targetError, out);
        SetIndices(s, primitive, out);
        return result;
    }
}
