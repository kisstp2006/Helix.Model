#pragma once

#include <Helix/Core/Containers/Array.h>
#include <Helix/Core/Strings/Format.h>
#include <Helix/Core/Strings/String.h>

#include <cstring>

// Builds glTF files for the tests, fuzz targets and benchmarks: binary data into one buffer, buffer views and accessors
// as JSON, and the whole file as .gltf (a data URI buffer) or GLB.
namespace helix::model::testing
{
    inline String Base64(const u8* data, usize size)
    {
        static constexpr char kDigits[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        String s;
        for (usize i = 0; i < size; i += 3)
        {
            u32 v = u32(data[i]) << 16 | (i + 1 < size ? u32(data[i + 1]) << 8 : 0) | (i + 2 < size ? data[i + 2] : 0);
            s.Append(kDigits[v >> 18 & 63]);
            s.Append(kDigits[v >> 12 & 63]);
            s.Append(i + 1 < size ? kDigits[v >> 6 & 63] : '=');
            s.Append(i + 2 < size ? kDigits[v & 63] : '=');
        }
        return s;
    }

    struct GltfWriter
    {
        Array<u8> bin;
        Array<String> views, accessors;

        // Bytes into the buffer at a multiple of 4, as a buffer view.
        u32 View(const void* data, usize size, u32 stride = 0)
        {
            while (bin.Size() % 4)
                bin.Push(u8(0));
            usize at = bin.Size();
            bin.Append(static_cast<const u8*>(data), size);
            String v = Format("{{\"buffer\":0,\"byteOffset\":{},\"byteLength\":{}", at, size);
            if (stride)
                FormatAppend(v, ",\"byteStride\":{}", stride);
            v.Append('}');
            views.Push(v);
            return u32(views.Size() - 1);
        }

        // An accessor of a view (or none: ~0u), with optional JSON members added (as "\"sparse\":{...}").
        u32 Accessor(u32 view, u32 componentType, u32 count, StringView type, bool normalized = false, usize offset = 0, StringView extra = {})
        {
            String a = Format("{{\"componentType\":{},\"count\":{},\"type\":\"{}\"", componentType, count, type);
            if (view != ~0u)
                FormatAppend(a, ",\"bufferView\":{},\"byteOffset\":{}", view, offset);
            if (normalized)
                a.Append(",\"normalized\":true");
            if (!extra.Empty())
            {
                a.Append(',');
                a.Append(extra);
            }
            a.Append('}');
            accessors.Push(a);
            return u32(accessors.Size() - 1);
        }

        template <class T>
        u32 Data(std::initializer_list<T> values, u32 componentType, StringView type, u32 components, bool normalized = false)
        {
            u32 view = View(values.begin(), values.size() * sizeof(T));
            return Accessor(view, componentType, u32(values.size() / components), type, normalized);
        }

        static String Join(const Array<String>& items)
        {
            String s = "[";
            for (usize i = 0; i < items.Size(); i++)
            {
                if (i)
                    s.Append(',');
                s.Append(items[i]);
            }
            s.Append(']');
            return s;
        }

        // The JSON members before `body` (which holds meshes, nodes and the rest, without the outer braces).
        String Head(StringView buffer) const
        {
            String s = "{\"asset\":{\"version\":\"2.0\",\"generator\":\"Helix tests\"}";
            if (!bin.Empty())
            {
                FormatAppend(s, ",\"buffers\":[{}]", buffer);
                FormatAppend(s, ",\"bufferViews\":{}", Join(views));
            }
            if (!accessors.Empty())
                FormatAppend(s, ",\"accessors\":{}", Join(accessors));
            return s;
        }

        // A .gltf with the buffer as a data URI.
        String Text(StringView body) const
        {
            String buffer = Format("{{\"byteLength\":{},\"uri\":\"data:application/octet-stream;base64,{}\"}}", bin.Size(), Base64(bin.Data(), bin.Size()));
            String s = Head(buffer);
            if (!body.Empty())
            {
                s.Append(',');
                s.Append(body);
            }
            s.Append('}');
            return s;
        }

        // A .gltf whose buffer is the external file `uri`.
        String External(StringView body, StringView uri) const
        {
            String s = Head(Format("{{\"byteLength\":{},\"uri\":\"{}\"}}", bin.Size(), uri));
            if (!body.Empty())
            {
                s.Append(',');
                s.Append(body);
            }
            s.Append('}');
            return s;
        }

        Array<u8> Glb(StringView body) const
        {
            String json = Head(Format("{{\"byteLength\":{}}}", bin.Size()));
            if (!body.Empty())
            {
                json.Append(',');
                json.Append(body);
            }
            json.Append('}');
            while (json.Size() % 4)
                json.Append(' ');
            Array<u8> out;
            auto put = [&](u32 v) { out.Append(reinterpret_cast<const u8*>(&v), 4); };
            usize binSize = (bin.Size() + 3) & ~usize(3);
            put(0x46546C67);
            put(2);
            put(u32(12 + 8 + json.Size() + (bin.Empty() ? 0 : 8 + binSize)));
            put(u32(json.Size()));
            put(0x4E4F534A);
            out.Append(reinterpret_cast<const u8*>(json.Data()), json.Size());
            if (!bin.Empty())
            {
                put(u32(binSize));
                put(0x004E4942);
                out.Append(bin.Data(), bin.Size());
                while (out.Size() % 4)
                    out.Push(u8(0));
            }
            return out;
        }
    };

    inline Span<const u8> AsBytes(const String& s) { return Span<const u8>(reinterpret_cast<const u8*>(s.Data()), s.Size()); }
}
