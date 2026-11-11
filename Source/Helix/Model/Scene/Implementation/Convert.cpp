#include <Helix/Model/Scene/Convert.h>
#include <Helix/Model/Scene/Implementation/Transform.h>

#include <cmath>

namespace helix::model
{
    namespace
    {
        // A signed permutation: engine axis `to[i]` with sign `sign[i]` for source axis i.
        struct Map
        {
            int to[3];
            float sign[3];
            float det;

            void Vector(float* v) const
            {
                float s[3] = {v[0], v[1], v[2]};
                for (int i = 0; i < 3; i++)
                    v[to[i]] = s[i] * sign[i];
            }
            // The quaternion of M R M^T: the vector part like a vector, times the determinant.
            void Rotation(float* q) const
            {
                Vector(q);
                for (int i = 0; i < 3; i++)
                    q[i] *= det;
            }
            void Scale(float* s) const
            {
                float o[3] = {s[0], s[1], s[2]};
                for (int i = 0; i < 3; i++)
                    s[to[i]] = o[i];
            }
        };

        Map Build(const AxisSystem& from)
        {
            // Engine directions of the source's up, front and right.
            struct Target
            {
                int axis;
                float sign;
            };
            // The engine axis of each role (0 up, 1 front, 2 right) and the sign that carries the source's direction there.
            auto engine = [](const AxisSystem& a, int role) {
                if (role == 0)
                    return Target{1, float(a.upSign) * kEngineAxes.upSign};
                if (role == 1)
                    return Target{2, float(a.frontSign) * kEngineAxes.frontSign};
                return Target{0, float(a.coordSign) * kEngineAxes.coordSign};
            };
            Map m{};
            u8 axes[3] = {from.up, from.front, from.coord};
            for (int role = 0; role < 3; role++)
            {
                Target t = engine(from, role);
                m.to[axes[role]] = t.axis;
                m.sign[axes[role]] = t.sign;
            }
            // The determinant of a signed permutation: the signs times the parity.
            float parity = 1;
            int p[3] = {m.to[0], m.to[1], m.to[2]};
            for (int i = 0; i < 3; i++)
                for (int j = i + 1; j < 3; j++)
                    if (p[i] > p[j])
                        parity = -parity;
            m.det = parity * m.sign[0] * m.sign[1] * m.sign[2];
            return m;
        }

        void Streams(Scene& s, Range streams, u32 vertices, const Map& m, float u, bool deltas)
        {
            for (u32 i = 0; i < streams.count; i++)
            {
                VertexStream& st = s.streams[streams.begin + i];
                float* d = s.floats.Data() + st.offset;
                if (st.components < 3)
                    continue;
                for (u32 v = 0; v < vertices; v++)
                {
                    float* x = d + usize(v) * st.components;
                    switch (st.semantic)
                    {
                    case Semantic::Position:
                        m.Vector(x);
                        x[0] *= u, x[1] *= u, x[2] *= u;
                        break;
                    case Semantic::Normal: m.Vector(x); break;
                    case Semantic::Tangent:
                        m.Vector(x);
                        if (st.components == 4 && !deltas)
                            x[3] *= m.det;
                        break;
                    default: break;
                    }
                }
            }
        }

        // Winding of a primitive reversed: triangles swap two corners, fans keep their hub, strips gain a corner.
        void Flip(Scene& s, Primitive& p)
        {
            if (p.topology != Topology::Triangles && p.topology != Topology::TriangleStrip && p.topology != Topology::TriangleFan)
                return;
            if (p.indices.count == 0)
            {
                p.indices = {u32(s.indices.Size()), p.vertexCount};
                for (u32 i = 0; i < p.vertexCount; i++)
                    s.indices.Push(i);
            }
            u32* idx = s.indices.Data() + p.indices.begin;
            u32 n = p.indices.count;
            if (p.topology == Topology::Triangles)
                for (u32 i = 0; i + 2 < n; i += 3)
                {
                    u32 t = idx[i + 1];
                    idx[i + 1] = idx[i + 2];
                    idx[i + 2] = t;
                }
            else if (p.topology == Topology::TriangleFan)
                for (u32 a = 1, b = n - 1; a < b; a++, b--)
                {
                    u32 t = idx[a];
                    idx[a] = idx[b];
                    idx[b] = t;
                }
            else if (n > 0)
            {
                // A repeated first corner shifts every triangle's parity.
                Array<u32> copy;
                copy.Push(idx[0]);
                copy.Append(idx, n);
                p.indices = {u32(s.indices.Size()), n + 1};
                s.indices.Append(copy.Data(), copy.Size());
            }
        }
    }

    void ConvertScene(Scene& s, const ConvertOptions& options)
    {
        Map m = Build(s.axes);
        float u = options.scaleFactor * (options.convertUnits && s.unitScale > 0 ? s.unitScale : 1.0f);
        for (Primitive& p : s.primitives)
        {
            Streams(s, p.streams, p.vertexCount, m, u, false);
            for (u32 t = 0; t < p.targets.count; t++)
                Streams(s, s.morphTargets[p.targets.begin + t].streams, p.vertexCount, m, u, true);
            if (m.det < 0)
                Flip(s, p);
        }
        for (Node& n : s.nodes)
        {
            m.Vector(n.translation);
            for (float& t : n.translation)
                t *= u;
            m.Rotation(n.rotation);
            m.Scale(n.scale);
            if (n.hasMatrix)
            {
                detail::Mat4 r = detail::Mat4::Quaternion(n.rotation[0], n.rotation[1], n.rotation[2], n.rotation[3]);
                detail::Mat4 full = detail::Mat4::Translation(n.translation[0], n.translation[1], n.translation[2]) * r *
                                    detail::Mat4::Scale(n.scale[0], n.scale[1], n.scale[2]);
                for (int i = 0; i < 16; i++)
                    n.matrix[i] = float(full.m[i]);
            }
        }
        // Inverse bind matrices: C B C^-1, C the mirror and the scale.
        for (const Skin& skin : s.skins)
            for (u32 j = 0; j < skin.inverseBindMatrices.count / 16; j++)
            {
                float* b = s.floats.Data() + skin.inverseBindMatrices.begin + 16 * usize(j);
                float out[16];
                for (int c = 0; c < 4; c++)
                    for (int r = 0; r < 4; r++)
                    {
                        // Row r, column c of the result from the source's row and column of the same axes.
                        int sr = r, sc = c;
                        float sign = 1;
                        if (r < 3)
                            for (int i = 0; i < 3; i++)
                                if (m.to[i] == r)
                                    sr = i, sign *= m.sign[i];
                        if (c < 3)
                            for (int i = 0; i < 3; i++)
                                if (m.to[i] == c)
                                    sc = i, sign *= m.sign[i];
                        float v = b[sc * 4 + sr] * sign;
                        if (c == 3 && r < 3)
                            v *= u;
                        out[c * 4 + r] = v;
                    }
                for (int i = 0; i < 16; i++)
                    b[i] = out[i];
            }
        for (const AnimationChannel& ch : s.channels)
        {
            if (ch.path == AnimationPath::Weights)
                continue;
            float* v = s.floats.Data() + ch.values.begin;
            u32 stride = ch.path == AnimationPath::Rotation ? 4 : 3;
            for (u32 k = 0; k + stride <= ch.values.count; k += stride)
            {
                float* x = v + k;
                if (ch.path == AnimationPath::Translation)
                {
                    m.Vector(x);
                    x[0] *= u, x[1] *= u, x[2] *= u;
                }
                else if (ch.path == AnimationPath::Rotation)
                    m.Rotation(x);
                else
                    m.Scale(x);
            }
        }
        for (Camera& c : s.cameras)
        {
            c.znear *= u, c.zfar *= u;
            c.xmag *= u, c.ymag *= u;
        }
        for (Light& l : s.lights)
            l.range *= u;
        s.axes = kEngineAxes;
        if (options.convertUnits && s.unitScale > 0)
            s.unitScale = 1;
    }
}
