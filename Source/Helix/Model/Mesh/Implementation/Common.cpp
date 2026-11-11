#include "Common.h"

#include <Helix/Core/Containers/HashMap.h>

namespace helix::model::detail
{
    bool TriangleIndices(Scene& s, u32 primitive, Array<u32>& out)
    {
        ToTriangleList(s, primitive);
        const Primitive& p = s.primitives[primitive];
        out.Clear();
        if (p.topology != Topology::Triangles)
            return false;
        u32 n = p.indices.count ? p.indices.count : p.vertexCount;
        out.Reserve(n);
        for (u32 i = 0; i + 2 < n; i += 3)
        {
            u32 a = p.indices.count ? s.indices[p.indices.begin + i] : i;
            u32 b = p.indices.count ? s.indices[p.indices.begin + i + 1] : i + 1;
            u32 c = p.indices.count ? s.indices[p.indices.begin + i + 2] : i + 2;
            if (a < p.vertexCount && b < p.vertexCount && c < p.vertexCount)
                out.Push(a), out.Push(b), out.Push(c);
        }
        return true;
    }

    void SetIndices(Scene& s, u32 primitive, const Array<u32>& indices)
    {
        Primitive& p = s.primitives[primitive];
        p.indices = {u32(s.indices.Size()), u32(indices.Size())};
        s.indices.Append(indices.Data(), indices.Size());
    }

    void SetStream(Scene& s, u32 primitive, Semantic semantic, u32 set, u32 components, const float* data)
    {
        Primitive& p = s.primitives[primitive];
        VertexStream* found = nullptr;
        for (u32 i = 0; i < p.streams.count; i++)
        {
            VertexStream& st = s.streams[p.streams.begin + i];
            if (st.semantic == semantic && st.set == set)
                found = &st;
        }
        if (!found)
        {
            // The primitive's streams must stay one range: moved to the end unless they are there already.
            if (p.streams.begin + p.streams.count != s.streams.Size())
            {
                Array<VertexStream> copy;
                for (u32 i = 0; i < p.streams.count; i++)
                    copy.Push(s.streams[p.streams.begin + i]);
                p.streams.begin = u32(s.streams.Size());
                for (VertexStream& st : copy)
                    s.streams.Push(Move(st));
            }
            if (p.streams.count == 0)
                p.streams.begin = u32(s.streams.Size());
            VertexStream st;
            st.semantic = semantic;
            st.set = set;
            s.streams.Push(Move(st));
            p.streams.count++;
            found = &s.streams.Back();
        }
        found->components = components;
        found->offset = u32(s.floats.Size());
        s.floats.Append(data, usize(p.vertexCount) * components);
    }

    bool Positions(const Scene& s, u32 primitive, Array<float>& out)
    {
        const Primitive& p = s.primitives[primitive];
        const VertexStream* st = s.Find(p, Semantic::Position);
        out.Clear();
        if (!st || st->components < 3)
            return false;
        out.Resize(usize(p.vertexCount) * 3);
        const float* src = s.Data(*st);
        for (u32 v = 0; v < p.vertexCount; v++)
            std::memcpy(out.Data() + usize(v) * 3, src + usize(v) * st->components, 3 * sizeof(float));
        return true;
    }

    u32 GroupEqual(const float* data, u32 count, u32 components, Array<u32>& group)
    {
        HashMap<u64, u32> first;
        first.Reserve(count);
        Array<u32> next;
        next.Resize(count, kNone);
        group.Clear();
        group.Resize(count, kNone);
        u32 groups = 0;
        for (u32 v = 0; v < count; v++)
        {
            const float* a = data + usize(v) * components;
            u64 h = 1469598103934665603ull;
            for (u32 c = 0; c < components; c++)
                h = (h ^ FloatBits(a[c])) * 1099511628211ull;
            u32* head = first.Find(h);
            for (u32 o = head ? *head : kNone; o != kNone; o = next[o])
            {
                const float* b = data + usize(o) * components;
                u32 c = 0;
                while (c < components && FloatBits(a[c]) == FloatBits(b[c]))
                    c++;
                if (c == components)
                {
                    group[v] = group[o];
                    break;
                }
            }
            if (group[v] != kNone)
                continue;
            group[v] = groups++;
            if (head)
            {
                next[v] = *head;
                *head = v;
            }
            else
                first.Set(h, v);
        }
        return groups;
    }

    void SplitByCorners(Scene& s, u32 primitive, const Array<u32>& indices, const float* values, u32 components, Semantic semantic, u32 set)
    {
        u32 vertexCount = s.primitives[primitive].vertexCount;
        // The new vertices made from each old one, chained.
        Array<u32> head, next, order, remapped;
        head.Resize(vertexCount, kNone);
        remapped.Resize(indices.Size());
        Array<float> data;
        for (usize i = 0; i < indices.Size(); i++)
        {
            u32 v = indices[i];
            const float* value = values + i * components;
            u32 found = kNone;
            for (u32 n = head[v]; n != kNone; n = next[n])
            {
                const float* d = data.Data() + usize(n) * components;
                u32 c = 0;
                while (c < components && FloatBits(d[c]) == FloatBits(value[c]))
                    c++;
                if (c == components)
                {
                    found = n;
                    break;
                }
            }
            if (found == kNone)
            {
                found = u32(order.Size());
                order.Push(v);
                data.Append(value, components);
                next.Push(head[v]);
                head[v] = found;
            }
            remapped[i] = found;
        }
        GatherVertices(s, primitive, order);
        SetIndices(s, primitive, remapped);
        SetStream(s, primitive, semantic, set, components, data.Data());
    }
}
