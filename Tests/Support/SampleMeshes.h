#pragma once

#include <Helix/Model/Scene/Scene.h>

#include <cmath>

// Generated meshes for the mesh processing tests, fuzz targets and benchmarks.
namespace helix::model::testing
{
    struct MeshData
    {
        Array<float> positions, normals, uvs;
        Array<u32> indices;

        u32 VertexCount() const { return u32(positions.Size() / 3); }
        void Vertex(float x, float y, float z, float nx, float ny, float nz, float u, float v)
        {
            positions.Push(x), positions.Push(y), positions.Push(z);
            normals.Push(nx), normals.Push(ny), normals.Push(nz);
            uvs.Push(u), uvs.Push(v);
        }
        void Triangle(u32 a, u32 b, u32 c) { indices.Push(a), indices.Push(b), indices.Push(c); }
    };

    // side x side quads in the xy plane from (0, 0) to (size, size), facing +z, UVs from 0 to 1.
    inline MeshData Grid(u32 side, float size = 1)
    {
        MeshData m;
        for (u32 y = 0; y <= side; y++)
            for (u32 x = 0; x <= side; x++)
                m.Vertex(size * float(x) / float(side), size * float(y) / float(side), 0, 0, 0, 1, float(x) / float(side), float(y) / float(side));
        for (u32 y = 0; y < side; y++)
            for (u32 x = 0; x < side; x++)
            {
                u32 a = y * (side + 1) + x, b = a + 1, c = a + side + 1, d = c + 1;
                m.Triangle(a, b, d), m.Triangle(a, d, c);
            }
        return m;
    }

    // A unit UV sphere around `center`: the seam column and the pole rows are separate vertices (rings + 1 rows of
    // segments + 1), the triangles at the poles that would have no area left out.
    inline MeshData Sphere(u32 rings, u32 segments, float cx = 0, float cy = 0, float cz = 0)
    {
        MeshData m;
        const float pi = 3.14159265358979f;
        for (u32 i = 0; i <= rings; i++)
            for (u32 j = 0; j <= segments; j++)
            {
                float theta = pi * float(i) / float(rings), phi = 2 * pi * float(j) / float(segments);
                // The seam and the poles exactly equal to their twins.
                if (j == segments)
                    phi = 0;
                float st = i == 0 || i == rings ? 0 : std::sin(theta), ct = i == 0 ? 1 : i == rings ? -1 : std::cos(theta);
                float x = st * std::cos(phi), y = ct, z = st * std::sin(phi);
                m.Vertex(cx + x, cy + y, cz + z, x, y, z, float(j) / float(segments), float(i) / float(rings));
            }
        for (u32 i = 0; i < rings; i++)
            for (u32 j = 0; j < segments; j++)
            {
                u32 a = i * (segments + 1) + j, b = a + 1, c = a + segments + 1, d = c + 1;
                if (i != 0)
                    m.Triangle(a, b, c);
                if (i + 1 != rings)
                    m.Triangle(b, d, c);
            }
        return m;
    }

    // A cube from -1 to 1 with a separate quad per face (24 vertices), face normals and UVs from 0 to 1 per face.
    inline MeshData Cube()
    {
        MeshData m;
        // Normal, u and v axes with cross(u, v) = normal.
        const float faces[6][9] = {{1, 0, 0, 0, 1, 0, 0, 0, 1},  {-1, 0, 0, 0, 0, 1, 0, 1, 0}, {0, 1, 0, 0, 0, 1, 1, 0, 0},
                                   {0, -1, 0, 1, 0, 0, 0, 0, 1}, {0, 0, 1, 1, 0, 0, 0, 1, 0},  {0, 0, -1, 0, 1, 0, 1, 0, 0}};
        const float corners[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
        for (const auto& f : faces)
        {
            u32 base = m.VertexCount();
            for (const auto& c : corners)
            {
                float p[3];
                for (u32 k = 0; k < 3; k++)
                    p[k] = f[k] + c[0] * f[3 + k] + c[1] * f[6 + k];
                m.Vertex(p[0], p[1], p[2], f[0], f[1], f[2], (c[0] + 1) / 2, (c[1] + 1) / 2);
            }
            m.Triangle(base, base + 1, base + 2), m.Triangle(base, base + 2, base + 3);
        }
        return m;
    }

    // Rows of triangles in the mesh in a scrambled order (a fixed shuffle), for the optimizers.
    inline void Shuffle(MeshData& m, u32 seed = 1)
    {
        usize triangles = m.indices.Size() / 3;
        u32 state = seed * 2654435761u + 1;
        for (usize t = triangles; t > 1; t--)
        {
            state = state * 1664525u + 1013904223u;
            usize o = usize(state >> 8) % t;
            for (u32 k = 0; k < 3; k++)
            {
                u32 x = m.indices[(t - 1) * 3 + k];
                m.indices[(t - 1) * 3 + k] = m.indices[o * 3 + k];
                m.indices[o * 3 + k] = x;
            }
        }
    }

    // One mesh, node and indexed triangle primitive with the chosen streams.
    inline Scene ToScene(const MeshData& m, bool normals = true, bool uvs = true)
    {
        Scene s;
        Primitive p;
        p.vertexCount = m.VertexCount();
        p.streams.begin = 0;
        auto stream = [&](Semantic semantic, u32 components, const Array<float>& data) {
            VertexStream st;
            st.semantic = semantic;
            st.components = components;
            st.offset = u32(s.floats.Size());
            s.floats.Append(data.Data(), data.Size());
            s.streams.Push(Move(st));
            p.streams.count++;
        };
        stream(Semantic::Position, 3, m.positions);
        if (normals)
            stream(Semantic::Normal, 3, m.normals);
        if (uvs)
            stream(Semantic::TexCoord, 2, m.uvs);
        p.indices = {0, u32(m.indices.Size())};
        s.indices = m.indices;
        s.primitives.Push(p);
        Mesh mesh;
        mesh.primitives = {0, 1};
        s.meshes.Push(Move(mesh));
        Node node;
        node.mesh = 0;
        s.nodes.Push(Move(node));
        s.roots.Push(0);
        return s;
    }
}
