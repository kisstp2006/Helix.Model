#pragma once

#include <Helix/Model/Mesh/Mesh.h>

#include <cmath>
#include <cstring>

namespace helix::model::detail
{
    // The bits of a float for exact comparison and hashing, -0 taken as 0.
    inline u32 FloatBits(float f)
    {
        u32 b;
        f = f == 0 ? 0.0f : f;
        std::memcpy(&b, &f, 4);
        return b;
    }

    struct Vec3
    {
        float x = 0, y = 0, z = 0;
    };

    inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
    inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
    inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
    inline float Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline Vec3 Cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
    inline float Length(Vec3 a) { return std::sqrt(Dot(a, a)); }
    inline Vec3 Normalize(Vec3 a)
    {
        float l = Length(a);
        return l > 0 ? a * (1 / l) : Vec3{};
    }
    inline Vec3 Load(const float* p) { return {p[0], p[1], p[2]}; }

    // acos within 7e-5 radians (Abramowitz and Stegun 4.4.45), for angle weights.
    inline float FastAcos(float x)
    {
        x = x < -1 ? -1 : x > 1 ? 1 : x;
        float a = x < 0 ? -x : x;
        float r = std::sqrt(1 - a) * (1.5707288f + a * (-0.2121144f + a * (0.0742610f - 0.0187293f * a)));
        return x < 0 ? 3.14159265f - r : r;
    }

    // The triangle list of a primitive: strips and fans are turned into lists first, triangles with an index outside the
    // vertices are dropped. False when the primitive is not made of triangles.
    bool TriangleIndices(Scene& scene, u32 primitive, Array<u32>& out);
    // The primitive's index range replaced by `indices`.
    void SetIndices(Scene& scene, u32 primitive, const Array<u32>& indices);
    // `components` floats per vertex of the primitive written to the stream, which is added when missing.
    void SetStream(Scene& scene, u32 primitive, Semantic semantic, u32 set, u32 components, const float* data);
    // The positions of a primitive, 3 floats a vertex; false when it has none.
    bool Positions(const Scene& scene, u32 primitive, Array<float>& out);
    // Groups of vertices equal in all `components` floats: group[v] is the group, numbered in first appearance; returns the
    // group count.
    u32 GroupEqual(const float* data, u32 count, u32 components, Array<u32>& group);
    // Corner i of `indices` takes `values + i * components`; the vertices are split where the corners of one vertex take
    // different values, unused vertices dropped, and the indices and the stream replaced.
    void SplitByCorners(Scene& scene, u32 primitive, const Array<u32>& indices, const float* values, u32 components, Semantic semantic, u32 set);
    // Triangles reordered for a post transform cache.
    void OptimizeCacheIndices(const u32* indices, usize indexCount, u32 vertexCount, u32* out);
}
