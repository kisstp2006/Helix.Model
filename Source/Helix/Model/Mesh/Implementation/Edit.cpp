#include <Helix/Model/Mesh/Mesh.h>

#include <Helix/Core/Containers/HashMap.h>

#include <cstring>

namespace helix::model
{
    void ToTriangleList(Scene& s, u32 primitive)
    {
        Primitive& p = s.primitives[primitive];
        if (p.topology != Topology::TriangleStrip && p.topology != Topology::TriangleFan)
            return;
        u32 n = p.indices.count ? p.indices.count : p.vertexCount;
        auto at = [&](u32 i) { return p.indices.count ? s.indices[p.indices.begin + i] : i; };
        Array<u32> list;
        for (u32 i = 0; i + 2 < n; i++)
        {
            u32 a, b, c;
            if (p.topology == Topology::TriangleFan)
                a = at(0), b = at(i + 1), c = at(i + 2);
            else if (i % 2 == 0)
                a = at(i), b = at(i + 1), c = at(i + 2);
            else
                a = at(i + 1), b = at(i), c = at(i + 2);
            // Strips join with repeated vertices: those triangles have no area.
            if (a == b || b == c || a == c)
                continue;
            list.Push(a), list.Push(b), list.Push(c);
        }
        p.topology = Topology::Triangles;
        p.indices = {u32(s.indices.Size()), u32(list.Size())};
        s.indices.Append(list.Data(), list.Size());
    }

    namespace
    {
        void GatherRange(Scene& s, Range streams, u32 oldCount, const Array<u32>& order)
        {
            for (u32 i = 0; i < streams.count; i++)
            {
                VertexStream& st = s.streams[streams.begin + i];
                Array<float> data;
                data.Resize(usize(order.Size()) * st.components);
                const float* src = s.floats.Data() + st.offset;
                for (usize v = 0; v < order.Size(); v++)
                    if (order[v] < oldCount)
                        std::memcpy(data.Data() + v * st.components, src + usize(order[v]) * st.components, st.components * sizeof(float));
                st.offset = u32(s.floats.Size());
                s.floats.Append(data.Data(), data.Size());
            }
        }
    }

    void GatherVertices(Scene& s, u32 primitive, const Array<u32>& order)
    {
        Primitive& p = s.primitives[primitive];
        GatherRange(s, p.streams, p.vertexCount, order);
        for (u32 t = 0; t < p.targets.count; t++)
            GatherRange(s, s.morphTargets[p.targets.begin + t].streams, p.vertexCount, order);
        p.vertexCount = u32(order.Size());
    }

    u32 WeldVertices(Scene& s, u32 primitive)
    {
        Primitive& p = s.primitives[primitive];
        // Every stream the vertex has, primitive and morph targets.
        Array<const VertexStream*> streams;
        for (u32 i = 0; i < p.streams.count; i++)
            streams.Push(&s.streams[p.streams.begin + i]);
        for (u32 t = 0; t < p.targets.count; t++)
        {
            const MorphTarget& m = s.morphTargets[p.targets.begin + t];
            for (u32 i = 0; i < m.streams.count; i++)
                streams.Push(&s.streams[m.streams.begin + i]);
        }
        auto bits = [](float f) {
            u32 b;
            f = f == 0 ? 0.0f : f;
            std::memcpy(&b, &f, 4);
            return b;
        };
        auto same = [&](u32 a, u32 b) {
            for (const VertexStream* st : streams)
                for (u32 c = 0; c < st->components; c++)
                    if (bits(s.floats[st->offset + usize(a) * st->components + c]) != bits(s.floats[st->offset + usize(b) * st->components + c]))
                        return false;
            return true;
        };
        HashMap<u64, u32> first;
        // Vertices of one hash, chained.
        Array<u32> next, remap, order;
        next.Resize(p.vertexCount, kNone);
        remap.Resize(p.vertexCount, kNone);
        for (u32 v = 0; v < p.vertexCount; v++)
        {
            u64 h = 1469598103934665603ull;
            for (const VertexStream* st : streams)
                for (u32 c = 0; c < st->components; c++)
                    h = (h ^ bits(s.floats[st->offset + usize(v) * st->components + c])) * 1099511628211ull;
            u32* head = first.Find(h);
            u32 found = kNone;
            for (u32 o = head ? *head : kNone; o != kNone; o = next[o])
                if (same(o, v))
                {
                    found = o;
                    break;
                }
            if (found != kNone)
            {
                remap[v] = remap[found];
                continue;
            }
            remap[v] = u32(order.Size());
            order.Push(v);
            if (head)
            {
                next[v] = *head;
                *head = v;
            }
            else
                first.Set(h, v);
        }
        if (order.Size() == p.vertexCount)
            return p.vertexCount;
        // New indices, as index ranges may be shared.
        Array<u32> indices;
        if (p.indices.count == 0)
            for (u32 v = 0; v < p.vertexCount; v++)
                indices.Push(remap[v]);
        for (u32 i = 0; i < p.indices.count; i++)
        {
            u32 idx = s.indices[p.indices.begin + i];
            indices.Push(idx < p.vertexCount ? remap[idx] : idx);
        }
        p.indices = {u32(s.indices.Size()), u32(indices.Size())};
        s.indices.Append(indices.Data(), indices.Size());
        GatherVertices(s, primitive, order);
        return p.vertexCount;
    }

    void Compact(Scene& s)
    {
        Array<u8> used;
        used.Resize(s.floats.Size(), u8(0));
        auto mark = [&](u32 begin, u64 count) {
            for (u64 i = 0; i < count && begin + i < used.Size(); i++)
                used[usize(begin + i)] = 1;
        };
        auto streams = [&](Range r, u32 vertices) {
            for (u32 i = 0; i < r.count; i++)
            {
                const VertexStream& st = s.streams[r.begin + i];
                mark(st.offset, u64(st.components) * vertices);
            }
        };
        for (const Primitive& p : s.primitives)
        {
            streams(p.streams, p.vertexCount);
            for (u32 t = 0; t < p.targets.count; t++)
                streams(s.morphTargets[p.targets.begin + t].streams, p.vertexCount);
        }
        for (const Node& n : s.nodes)
            mark(n.weights.begin, n.weights.count);
        for (const Mesh& m : s.meshes)
            mark(m.weights.begin, m.weights.count);
        for (const Skin& k : s.skins)
            mark(k.inverseBindMatrices.begin, k.inverseBindMatrices.count);
        for (const AnimationChannel& c : s.channels)
            mark(c.times.begin, c.times.count), mark(c.values.begin, c.values.count);
        Array<u32> moved;
        moved.Resize(s.floats.Size() + 1);
        Array<float> floats;
        for (usize i = 0; i < s.floats.Size(); i++)
        {
            moved[i] = u32(floats.Size());
            if (used[i])
                floats.Push(s.floats[i]);
        }
        moved[s.floats.Size()] = u32(floats.Size());
        auto fix = [&](u32& offset) { offset = moved[offset < moved.Size() ? offset : moved.Size() - 1]; };
        for (VertexStream& st : s.streams)
            fix(st.offset);
        for (Node& n : s.nodes)
            fix(n.weights.begin);
        for (Mesh& m : s.meshes)
            fix(m.weights.begin);
        for (Skin& k : s.skins)
            fix(k.inverseBindMatrices.begin);
        for (AnimationChannel& c : s.channels)
            fix(c.times.begin), fix(c.values.begin);
        s.floats = Move(floats);
    }
}
