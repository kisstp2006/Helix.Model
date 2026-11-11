#include "Common.h"

namespace helix::model
{
    void CompressDeltas(const float* deltas, u32 vertexCount, u32 components, float threshold, CompressedDeltas& out)
    {
        out.vertices.Clear(), out.values.Clear();
        out.components = components;
        out.scale = 0;
        auto value = [&](u32 v, u32 c) {
            float f = deltas[usize(v) * components + c];
            return std::isfinite(f) ? f : 0.0f;
        };
        float largest = 0;
        for (u32 v = 0; v < vertexCount; v++)
        {
            bool moves = false;
            for (u32 c = 0; c < components; c++)
                moves = moves || std::fabs(value(v, c)) > threshold;
            if (!moves)
                continue;
            out.vertices.Push(v);
            for (u32 c = 0; c < components; c++)
                largest = std::fabs(value(v, c)) > largest ? std::fabs(value(v, c)) : largest;
        }
        out.scale = largest / 32767;
        out.values.Reserve(out.vertices.Size() * components);
        for (u32 v : out.vertices)
            for (u32 c = 0; c < components; c++)
            {
                float q = out.scale > 0 ? value(v, c) / out.scale : 0;
                q = q < -32767 ? -32767 : q > 32767 ? 32767 : q;
                out.values.Push(i16(std::lround(q)));
            }
    }

    void DecompressDeltas(const CompressedDeltas& in, u32 vertexCount, Array<float>& out)
    {
        out.Clear();
        out.Resize(usize(vertexCount) * in.components, 0.0f);
        for (usize i = 0; i < in.vertices.Size(); i++)
        {
            u32 v = in.vertices[i];
            if (v >= vertexCount)
                continue;
            for (u32 c = 0; c < in.components && i * in.components + c < in.values.Size(); c++)
                out[usize(v) * in.components + c] = float(in.values[i * in.components + c]) * in.scale;
        }
    }
}
