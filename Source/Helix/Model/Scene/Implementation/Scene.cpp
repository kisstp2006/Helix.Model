#include <Helix/Model/Scene/Scene.h>

namespace helix::model
{
    const VertexStream* Scene::Find(const Primitive& p, Semantic semantic, u32 set) const
    {
        for (u32 i = 0; i < p.streams.count; i++)
        {
            const VertexStream& s = streams[p.streams.begin + i];
            if (s.semantic == semantic && s.set == set)
                return &s;
        }
        return nullptr;
    }

    u64 Scene::TriangleCount() const
    {
        u64 n = 0;
        for (const Primitive& p : primitives)
        {
            u64 count = p.indices.count ? p.indices.count : p.vertexCount;
            switch (p.topology)
            {
            case Topology::Triangles: n += count / 3; break;
            case Topology::TriangleStrip:
            case Topology::TriangleFan: n += count >= 3 ? count - 2 : 0; break;
            default: break;
            }
        }
        return n;
    }
}
