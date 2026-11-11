#include "Common.h"

#include <Helix/Core/Containers/HashMap.h>

// Tangent spaces as Morten Mikkelsen describes them in "Simulation of Wrinkled Surfaces Revisited" (MikkTSpace): the
// per triangle derivatives of position along u, normalized and signed by the triangle's UV orientation, projected into
// the plane of each corner's normal and summed with the corner's angle over the corners that share the vertex
// (position, normal and UV) and the orientation and are joined around it by edges; corners of triangles without UV
// area take the tangent space of another corner of their vertex.
namespace helix::model
{
    using namespace detail;

    namespace
    {
        Vec3 Project(Vec3 v, Vec3 n) { return Normalize(v - n * Dot(n, v)); }

        // Any unit vector perpendicular to n.
        Vec3 Perpendicular(Vec3 n)
        {
            Vec3 axis = std::fabs(n.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
            Vec3 t = Project(axis, n);
            return Dot(t, t) > 0 ? t : Vec3{1, 0, 0};
        }
    }

    bool CalculateTangents(Scene& s, u32 primitive, u32 uvSet)
    {
        const Primitive& p = s.primitives[primitive];
        const VertexStream* normalStream = s.Find(p, Semantic::Normal);
        const VertexStream* uvStream = s.Find(p, Semantic::TexCoord, uvSet);
        if (!normalStream || normalStream->components < 3 || !uvStream || uvStream->components < 2)
            return false;
        Array<u32> indices;
        Array<float> positions;
        if (!TriangleIndices(s, primitive, indices) || !Positions(s, primitive, positions))
            return false;
        u32 vertexCount = s.primitives[primitive].vertexCount;
        // TriangleIndices may have added indices, the streams are where they were.
        normalStream = s.Find(s.primitives[primitive], Semantic::Normal);
        uvStream = s.Find(s.primitives[primitive], Semantic::TexCoord, uvSet);

        // Position, normal and UV of every vertex; corners of equal vertices share a tangent space.
        Array<float> vertex;
        vertex.Resize(usize(vertexCount) * 8);
        for (u32 v = 0; v < vertexCount; v++)
        {
            float* d = vertex.Data() + usize(v) * 8;
            std::memcpy(d, positions.Data() + usize(v) * 3, 3 * sizeof(float));
            Vec3 n = Normalize(Load(s.Data(*normalStream) + usize(v) * normalStream->components));
            d[3] = n.x, d[4] = n.y, d[5] = n.z;
            std::memcpy(d + 6, s.Data(*uvStream) + usize(v) * uvStream->components, 2 * sizeof(float));
        }
        Array<u32> identity;
        u32 identities = GroupEqual(vertex.Data(), vertexCount, 8, identity);

        usize triangles = indices.Size() / 3;
        // Per triangle: its UV orientation (2: no UV area) and its unit derivative along u, signed by the orientation.
        Array<u8> orientation;
        orientation.Resize(triangles);
        Array<Vec3> derivative;
        derivative.Resize(triangles);
        for (usize t = 0; t < triangles; t++)
        {
            const float* v[3];
            for (u32 k = 0; k < 3; k++)
                v[k] = vertex.Data() + usize(indices[t * 3 + k]) * 8;
            Vec3 d1 = Load(v[1]) - Load(v[0]), d2 = Load(v[2]) - Load(v[0]);
            float s1 = v[1][6] - v[0][6], t1 = v[1][7] - v[0][7];
            float s2 = v[2][6] - v[0][6], t2 = v[2][7] - v[0][7];
            float area = s1 * t2 - t1 * s2;
            orientation[t] = area > 0 ? 1 : area < 0 ? 0 : 2;
            derivative[t] = Normalize(d1 * t2 - d2 * t1) * (area > 0 ? 1.0f : -1.0f);
        }

        // Corners share a tangent space when they are one vertex, of one orientation, and joined around that vertex by
        // edges of triangles of that orientation (MikkTSpace's groups): union find over the corners, joined through
        // the edges (vertex, neighbor) they have.
        usize corners = indices.Size();
        Array<u32> parent;
        parent.Resize(corners);
        for (usize c = 0; c < corners; c++)
            parent[c] = u32(c);
        auto find = [&](u32 c) {
            while (parent[c] != c)
                c = parent[c] = parent[parent[c]];
            return c;
        };
        HashMap<u64, u32> around;
        around.Reserve(corners * 2);
        for (usize c = 0; c < corners; c++)
        {
            usize t = c / 3, k = c % 3;
            if (orientation[t] == 2)
                continue;
            u32 v = identity[indices[c]];
            for (usize n : {t * 3 + (k + 1) % 3, t * 3 + (k + 2) % 3})
            {
                u64 key = (u64(v) << 33) | (u64(identity[indices[n]]) << 1) | orientation[t];
                if (u32* other = around.Find(key))
                {
                    u32 a = find(u32(c)), b = find(*other);
                    if (a != b)
                        parent[a] = b;
                }
                else
                    around.Set(key, u32(c));
            }
        }

        // The sum of each group: derivatives projected into each corner's normal plane, weighted by the corner's angle.
        Array<Vec3> sums;
        sums.Resize(corners);
        // A corner of each vertex in a triangle with UV area, for the corners of triangles without it.
        Array<u32> anyCorner;
        anyCorner.Resize(identities, kNone);
        for (usize c = 0; c < corners; c++)
        {
            usize t = c / 3, k = c % 3;
            if (orientation[t] == 2)
                continue;
            const float* v = vertex.Data() + usize(indices[c]) * 8;
            const float* a = vertex.Data() + usize(indices[t * 3 + (k + 1) % 3]) * 8;
            const float* b = vertex.Data() + usize(indices[t * 3 + (k + 2) % 3]) * 8;
            Vec3 n = Load(v + 3);
            Vec3 e1 = Project(Load(a) - Load(v), n), e2 = Project(Load(b) - Load(v), n);
            float angle = FastAcos(Dot(e1, e2));
            u32 g = find(u32(c));
            sums[g] = sums[g] + Project(derivative[t], n) * angle;
            u32& any = anyCorner[identity[indices[c]]];
            any = any == kNone ? u32(c) : any;
        }

        Array<float> tangents;
        tangents.Resize(corners * 4);
        for (usize c = 0; c < corners; c++)
        {
            const float* v = vertex.Data() + usize(indices[c]) * 8;
            Vec3 n = Load(v + 3);
            u32 from = orientation[c / 3] == 2 ? anyCorner[identity[indices[c]]] : u32(c);
            Vec3 tangent = from != kNone ? Project(sums[find(from)], n) : Vec3{};
            bool preserving = from == kNone || orientation[from / 3] == 1;
            if (Dot(tangent, tangent) == 0)
                tangent = Perpendicular(n);
            float* out = tangents.Data() + c * 4;
            out[0] = tangent.x, out[1] = tangent.y, out[2] = tangent.z, out[3] = preserving ? 1.0f : -1.0f;
        }
        SplitByCorners(s, primitive, indices, tangents.Data(), 4, Semantic::Tangent, 0);
        return true;
    }
}
