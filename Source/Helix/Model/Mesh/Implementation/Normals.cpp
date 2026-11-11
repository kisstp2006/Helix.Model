#include "Common.h"

namespace helix::model
{
    using namespace detail;

    namespace
    {
        // The angle at corner a of triangle a, b, c.
        float CornerAngle(Vec3 a, Vec3 b, Vec3 c)
        {
            Vec3 e1 = b - a, e2 = c - a;
            float l = Dot(e1, e1) * Dot(e2, e2);
            return l > 0 ? FastAcos(Dot(e1, e2) / std::sqrt(l)) : 0;
        }
    }

    void CalculateNormals(Scene& s, u32 primitive, float smoothingAngle, NormalWeighting weighting)
    {
        Array<u32> indices;
        Array<float> positions;
        if (!TriangleIndices(s, primitive, indices) || !Positions(s, primitive, positions))
            return;
        u32 vertexCount = s.primitives[primitive].vertexCount;
        usize triangles = indices.Size() / 3;

        Array<u32> group;
        u32 groups = GroupEqual(positions.Data(), vertexCount, 3, group);

        // Face normals of unit length (zero for degenerate faces) and the weight of each corner.
        Array<Vec3> faces;
        faces.Resize(triangles);
        Array<float> weights;
        weights.Resize(indices.Size());
        for (usize t = 0; t < triangles; t++)
        {
            Vec3 p[3];
            for (u32 k = 0; k < 3; k++)
                p[k] = Load(positions.Data() + usize(indices[t * 3 + k]) * 3);
            Vec3 n = Cross(p[1] - p[0], p[2] - p[0]);
            float area = Length(n);
            faces[t] = area > 0 ? n * (1 / area) : Vec3{};
            for (u32 k = 0; k < 3; k++)
            {
                float angle = weighting == NormalWeighting::Area || weighting == NormalWeighting::Unweighted ? 1 : CornerAngle(p[k], p[(k + 1) % 3], p[(k + 2) % 3]);
                float w = weighting == NormalWeighting::Angle || weighting == NormalWeighting::Unweighted ? 1 : area;
                weights[t * 3 + k] = angle * w;
            }
        }

        // The corners around each position.
        Array<u32> start, corners;
        start.Resize(usize(groups) + 1, 0u);
        for (u32 v : indices)
            start[group[v] + 1]++;
        for (u32 g = 0; g < groups; g++)
            start[g + 1] += start[g];
        corners.Resize(indices.Size());
        {
            Array<u32> fill;
            fill.Resize(groups);
            std::memcpy(fill.Data(), start.Data(), groups * sizeof(u32));
            for (usize c = 0; c < indices.Size(); c++)
                corners[fill[group[indices[c]]]++] = u32(c);
        }

        // The face normal and weight of every corner in the order of the positions, so the sums read them in a row.
        Array<Vec3> face;
        Array<float> weight;
        face.Resize(indices.Size());
        weight.Resize(indices.Size());
        for (usize i = 0; i < indices.Size(); i++)
            face[i] = faces[corners[i] / 3], weight[i] = weights[corners[i]];

        bool smoothAll = smoothingAngle >= 180;
        float limit = std::cos(smoothingAngle * 3.14159265358979f / 180) - 1e-5f;
        Array<float> normals;
        normals.Resize(indices.Size() * 3);
        for (u32 g = 0; g < groups; g++)
        {
            // The sum over all faces of the position, for corners that take every face.
            Vec3 whole, mean;
            for (u32 i = start[g]; i < start[g + 1]; i++)
                whole = whole + face[i] * weight[i], mean = mean + face[i];
            // When the faces lie within half the angle of their mean, every pair is within the angle: each corner
            // takes them all.
            Vec3 axis = Normalize(mean);
            float least = 1;
            for (u32 i = start[g]; i < start[g + 1]; i++)
            {
                float d = Dot(face[i], axis);
                least = Dot(face[i], face[i]) > 0 && d < least ? d : least;
            }
            bool together = smoothAll || (least > 0 && 2 * least * least - 1 >= limit + 1e-4f);
            for (u32 i = start[g]; i < start[g + 1]; i++)
            {
                Vec3 own = face[i];
                bool degenerate = Dot(own, own) == 0;
                Vec3 sum;
                if (together || degenerate)
                    sum = whole;
                else
                    for (u32 j = start[g]; j < start[g + 1]; j++)
                        if (Dot(own, face[j]) >= limit)
                            sum = sum + face[j] * weight[j];
                Vec3 n = Normalize(sum);
                if (Dot(n, n) == 0)
                    n = degenerate ? Vec3{0, 0, 1} : own;
                usize c = corners[i];
                normals[c * 3] = n.x, normals[c * 3 + 1] = n.y, normals[c * 3 + 2] = n.z;
            }
        }
        SplitByCorners(s, primitive, indices, normals.Data(), 3, Semantic::Normal, 0);
    }
}
