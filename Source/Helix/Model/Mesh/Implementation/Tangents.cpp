#include "Common.h"

// Tangent spaces as Morten Mikkelsen describes them in "Simulation of Wrinkled Surfaces Revisited" (MikkTSpace): the
// per triangle derivatives of position along u and v, normalized and signed by the triangle's UV orientation, projected
// into the plane of each corner's normal and summed with the corner's angle over the corners that share the vertex
// (position, normal and UV) and the orientation.
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
        // Per group (identity * 2 + orientation): the summed tangent.
        Array<Vec3> sums;
        sums.Resize(usize(identities) * 2);
        Array<u8> orientation;
        orientation.Resize(triangles);
        for (usize t = 0; t < triangles; t++)
        {
            const float* v[3];
            for (u32 k = 0; k < 3; k++)
                v[k] = vertex.Data() + usize(indices[t * 3 + k]) * 8;
            Vec3 d1 = Load(v[1]) - Load(v[0]), d2 = Load(v[2]) - Load(v[0]);
            float s1 = v[1][6] - v[0][6], t1 = v[1][7] - v[0][7];
            float s2 = v[2][6] - v[0][6], t2 = v[2][7] - v[0][7];
            float area = s1 * t2 - t1 * s2;
            bool preserving = area > 0;
            orientation[t] = preserving ? 1 : 0;
            // Without UV area the triangle gives no direction; its corners take their neighbors'.
            if (!(area != 0))
            {
                orientation[t] = 2;
                continue;
            }
            Vec3 os = d1 * t2 - d2 * t1;
            os = Normalize(os) * (preserving ? 1.0f : -1.0f);
            for (u32 k = 0; k < 3; k++)
            {
                Vec3 n = Load(v[k] + 3);
                Vec3 e1 = Project(Load(v[(k + 1) % 3]) - Load(v[k]), n), e2 = Project(Load(v[(k + 2) % 3]) - Load(v[k]), n);
                float angle = FastAcos(Dot(e1, e2));
                u32 g = identity[indices[t * 3 + k]] * 2 + orientation[t];
                sums[g] = sums[g] + Project(os, n) * angle;
            }
        }

        Array<float> tangents;
        tangents.Resize(indices.Size() * 4);
        for (usize c = 0; c < indices.Size(); c++)
        {
            u32 id = identity[indices[c]];
            const float* v = vertex.Data() + usize(indices[c]) * 8;
            Vec3 n = Load(v + 3);
            u8 o = orientation[c / 3];
            if (o == 2)
                o = Dot(sums[id * 2 + 1], sums[id * 2 + 1]) > 0 || Dot(sums[id * 2], sums[id * 2]) == 0 ? 1 : 0;
            Vec3 tangent = Project(sums[id * 2 + o], n);
            if (Dot(tangent, tangent) == 0)
                tangent = Perpendicular(n);
            float* out = tangents.Data() + c * 4;
            out[0] = tangent.x, out[1] = tangent.y, out[2] = tangent.z, out[3] = o ? 1.0f : -1.0f;
        }
        SplitByCorners(s, primitive, indices, tangents.Data(), 4, Semantic::Tangent, 0);
        return true;
    }
}
