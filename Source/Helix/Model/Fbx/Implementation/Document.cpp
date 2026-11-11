#include <Helix/Model/Fbx/Document.h>

#include <Helix/Core/Strings/Format.h>
#include <Helix/Image/Compression/Deflate.h>

#include <charconv>
#include <cstring>

namespace helix::model::fbx
{
    namespace
    {
        constexpr char kMagic[] = "Kaydara FBX Binary  ";
        constexpr u32 kMaxDepth = 64;
        // Decoded array bytes of one file: more is damage, not a model.
        constexpr usize kMaxArrayBytes = usize(1) << 31;

        u32 ElementSize(char type)
        {
            switch (type)
            {
            case 'b': return 1;
            case 'i':
            case 'f': return 4;
            case 'l':
            case 'd': return 8;
            default: return 0;
            }
        }

        PropertyType ArrayType(char type)
        {
            switch (type)
            {
            case 'b': return PropertyType::BoolArray;
            case 'i': return PropertyType::Int32Array;
            case 'l': return PropertyType::Int64Array;
            case 'f': return PropertyType::FloatArray;
            default: return PropertyType::DoubleArray;
            }
        }
    }

    // ---------------------------------------------------------------------- binary

    struct BinaryReader
    {
        Document& d;
        Span<const u8> data;
        String& error;
        bool wide = false;

        bool Fail(usize at, StringView what)
        {
            error = Format("byte {}: {}", at, what);
            return false;
        }

        template <class T>
        T Get(usize at) const
        {
            T v;
            std::memcpy(&v, data.Data() + at, sizeof(T));
            return v;
        }

        // One record at `at`: its node index, or kNoNode for the null record that ends a list.
        bool Record(usize& at, usize limit, u32 depth, u32& node)
        {
            usize header = wide ? 25 : 13;
            if (limit < at || limit - at < header)
                return Fail(at, "a record header past the end");
            u64 end = wide ? Get<u64>(at) : Get<u32>(at);
            u64 count = wide ? Get<u64>(at + 8) : Get<u32>(at + 4);
            u64 listLength = wide ? Get<u64>(at + 16) : Get<u32>(at + 8);
            u8 nameLength = data[at + header - 1];
            if (end == 0 && count == 0 && listLength == 0 && nameLength == 0)
            {
                node = kNoNode;
                at += header;
                return true;
            }
            if (end > limit || end < at + header + nameLength + listLength)
                return Fail(at, Format("a record ending at {} outside {} to {}", end, at, limit));
            if (depth >= kMaxDepth)
                return Fail(at, "records nested too deep");
            node = u32(d.nodes.Size());
            Node n;
            n.nameOffset = u32(d.bytes.Size());
            n.nameSize = nameLength;
            d.bytes.Append(data.Data() + at + header, nameLength);
            n.firstProperty = u32(d.properties.Size());
            d.nodes.Push(n);
            usize p = at + header + nameLength;
            usize propsEnd = p + usize(listLength);
            for (u64 i = 0; i < count; i++)
                if (!Value(p, propsEnd))
                    return false;
            if (p != propsEnd)
                return Fail(p, Format("properties end at {}, the record says {}", p, propsEnd));
            d.nodes[node].propertyCount = u32(d.properties.Size()) - d.nodes[node].firstProperty;
            // Children up to the end, closed by a null record when there are any.
            Array<u32> kids;
            while (p < end)
            {
                u32 child;
                if (!Record(p, usize(end), depth + 1, child))
                    return false;
                if (child == kNoNode)
                    break;
                kids.Push(child);
            }
            if (p != end)
                return Fail(p, Format("children end at {}, the record says {}", p, end));
            d.nodes[node].firstChild = u32(d.children.Size());
            d.nodes[node].childCount = u32(kids.Size());
            d.children.Append(kids.Data(), kids.Size());
            at = usize(end);
            return true;
        }

        bool Value(usize& p, usize end)
        {
            if (p >= end)
                return Fail(p, "a property past the end of its list");
            char type = char(data[p++]);
            Property prop;
            auto need = [&](usize n) { return end - p >= n; };
            switch (type)
            {
            case 'Y':
                if (!need(2))
                    return Fail(p, "a property past its list");
                prop.type = PropertyType::Int16, prop.integer = Get<i16>(p), p += 2;
                break;
            case 'C':
                if (!need(1))
                    return Fail(p, "a property past its list");
                prop.type = PropertyType::Bool, prop.integer = data[p] != 0, p += 1;
                break;
            case 'I':
                if (!need(4))
                    return Fail(p, "a property past its list");
                prop.type = PropertyType::Int32, prop.integer = Get<i32>(p), p += 4;
                break;
            case 'L':
                if (!need(8))
                    return Fail(p, "a property past its list");
                prop.type = PropertyType::Int64, prop.integer = Get<i64>(p), p += 8;
                break;
            case 'F':
                if (!need(4))
                    return Fail(p, "a property past its list");
                prop.type = PropertyType::Float, prop.number = Get<float>(p), p += 4;
                break;
            case 'D':
                if (!need(8))
                    return Fail(p, "a property past its list");
                prop.type = PropertyType::Double, prop.number = Get<double>(p), p += 8;
                break;
            case 'S':
            case 'R':
            {
                if (!need(4))
                    return Fail(p, "a property past its list");
                u32 n = Get<u32>(p);
                p += 4;
                if (!need(n))
                    return Fail(p, Format("a string of {} bytes past its list", n));
                prop.type = type == 'S' ? PropertyType::String : PropertyType::Raw;
                prop.offset = u32(d.bytes.Size());
                prop.count = n;
                d.bytes.Append(data.Data() + p, n);
                p += n;
                break;
            }
            case 'b':
            case 'i':
            case 'l':
            case 'f':
            case 'd':
            {
                if (!need(12))
                    return Fail(p, "an array header past its list");
                u32 length = Get<u32>(p), encoding = Get<u32>(p + 4), stored = Get<u32>(p + 8);
                p += 12;
                if (!need(stored))
                    return Fail(p, Format("an array of {} bytes past its list", stored));
                usize bytes = usize(length) * ElementSize(type);
                if (bytes > kMaxArrayBytes || d.arrays.Size() + bytes > kMaxArrayBytes)
                    return Fail(p, Format("an array of {} elements", length));
                while (d.arrays.Size() % 8)
                    d.arrays.Push(u8(0));
                prop.type = ArrayType(type);
                prop.offset = u32(d.arrays.Size());
                prop.count = length;
                if (encoding == 0)
                {
                    if (stored != bytes)
                        return Fail(p, Format("an array of {} elements in {} bytes", length, stored));
                    d.arrays.Append(data.Data() + p, bytes);
                }
                else if (encoding == 1)
                {
                    Array<u8> out;
                    if (!image::ZlibDecompress(Span<const u8>(data.Data() + p, stored), out) || out.Size() != bytes)
                        return Fail(p, Format("a zlib array that does not inflate to {} elements", length));
                    d.arrays.Append(out.Data(), out.Size());
                }
                else
                    return Fail(p, Format("array encoding {}", encoding));
                p += stored;
                break;
            }
            default: return Fail(p - 1, Format("property type {}", u32(u8(type))));
            }
            d.properties.Push(prop);
            return true;
        }

        bool Run()
        {
            if (data.Size() < 27)
                return Fail(0, "a file shorter than the FBX header");
            d.version = Get<u32>(23);
            if (d.version < 7100 || d.version > 7700)
                return Fail(23, Format("FBX version {}", d.version));
            wide = d.version >= 7500;
            d.nodes.Push(Node());
            usize at = 27;
            Array<u32> top;
            usize header = wide ? 25 : 13;
            while (at + header <= data.Size())
            {
                u32 node;
                if (!Record(at, data.Size(), 0, node))
                    return false;
                if (node == kNoNode)
                    break;
                top.Push(node);
            }
            d.nodes[0].firstChild = u32(d.children.Size());
            d.nodes[0].childCount = u32(top.Size());
            d.children.Append(top.Data(), top.Size());
            return true;
        }
    };

    // ---------------------------------------------------------------------- ASCII

    struct AsciiReader
    {
        Document& d;
        StringView t;
        String& error;
        usize at = 0;
        u32 line = 1;

        bool Fail(StringView what)
        {
            error = Format("line {}: {}", line, what);
            return false;
        }

        void Skip()
        {
            while (at < t.Size())
            {
                char c = t[at];
                if (c == '\n')
                    line++, at++;
                else if (c == ' ' || c == '\t' || c == '\r')
                    at++;
                else if (c == ';')
                    while (at < t.Size() && t[at] != '\n')
                        at++;
                else
                    break;
            }
        }

        static bool Word(char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '|' || c == '-'; }

        // A run of word characters.
        StringView Ident()
        {
            usize start = at;
            while (at < t.Size() && Word(t[at]))
                at++;
            return t.Substr(start, at - start);
        }

        u32 Name(StringView name)
        {
            u32 offset = u32(d.bytes.Size());
            d.bytes.Append(reinterpret_cast<const u8*>(name.Data()), name.Size());
            return offset;
        }

        bool Number(StringView s, Property& p)
        {
            bool real = false;
            for (char c : s)
                real = real || c == '.' || c == 'e' || c == 'E';
            const char* b = s.Data();
            if (!s.Empty() && s[0] == '+')
                b++;
            if (real)
            {
                double v;
                auto r = std::from_chars(b, s.Data() + s.Size(), v);
                if (r.ec != std::errc() || r.ptr != s.Data() + s.Size())
                    return false;
                p.type = PropertyType::Double, p.number = v, p.integer = i64(v);
                return true;
            }
            i64 v;
            auto r = std::from_chars(b, s.Data() + s.Size(), v);
            if (r.ec != std::errc() || r.ptr != s.Data() + s.Size())
                return false;
            p.type = PropertyType::Int64, p.integer = v, p.number = double(v);
            return true;
        }

        // `*N { a: v, v, ... }`.
        bool ArrayValue(Property& p)
        {
            at++;
            StringView n = Ident();
            u64 count = 0;
            for (char c : n)
            {
                if (c < '0' || c > '9' || count > (u64(1) << 32))
                    return Fail("an array count that is not a number");
                count = count * 10 + u64(c - '0');
            }
            Skip();
            if (at >= t.Size() || t[at] != '{')
                return Fail("an array without its block");
            at++;
            Skip();
            Array<double> values;
            bool real = false;
            if (at < t.Size() && t[at] == 'a')
            {
                at++;
                Skip();
                if (at >= t.Size() || t[at] != ':')
                    return Fail("an array without 'a:'");
                at++;
                while (true)
                {
                    Skip();
                    if (at < t.Size() && t[at] == '}')
                        break;
                    usize start = at;
                    while (at < t.Size() && (Word(t[at]) || t[at] == '.' || t[at] == '+'))
                        at++;
                    StringView s = t.Substr(start, at - start);
                    Property v;
                    if (s.Empty() || !Number(s, v))
                        return Fail(Format("'{}' in an array", s));
                    real = real || v.type == PropertyType::Double;
                    values.Push(v.number);
                    if (values.Size() > count)
                        return Fail(Format("more than the {} values the array says", count));
                    Skip();
                    if (at < t.Size() && t[at] == ',')
                        at++;
                }
            }
            Skip();
            if (at >= t.Size() || t[at] != '}')
                return Fail("an array without its closing brace");
            at++;
            if (values.Size() != count)
                return Fail(Format("{} values where the array says {}", values.Size(), count));
            while (d.arrays.Size() % 8)
                d.arrays.Push(u8(0));
            p.offset = u32(d.arrays.Size());
            p.count = u32(count);
            if (real)
            {
                p.type = PropertyType::DoubleArray;
                d.arrays.Append(reinterpret_cast<const u8*>(values.Data()), values.Size() * 8);
            }
            else
            {
                p.type = PropertyType::Int64Array;
                for (double v : values)
                {
                    i64 i = i64(v);
                    d.arrays.Append(reinterpret_cast<const u8*>(&i), 8);
                }
            }
            return true;
        }

        bool Values(u32 node)
        {
            // An empty first value, as embedded content is written: "Content: , \"...\"".
            Skip();
            if (at < t.Size() && t[at] == ',')
                at++;
            while (true)
            {
                Skip();
                if (at >= t.Size())
                    return true;
                char c = t[at];
                Property p;
                if (c == '"')
                {
                    usize end = t.Find('"', at + 1);
                    if (end == StringView::npos)
                        return Fail("a string without its closing quote");
                    for (usize i = at; i < end; i++)
                        line += t[i] == '\n';
                    p.type = PropertyType::String;
                    p.offset = Name(t.Substr(at + 1, end - at - 1));
                    p.count = u32(end - at - 1);
                    at = end + 1;
                }
                else if (c == '*')
                {
                    if (!ArrayValue(p))
                        return false;
                }
                else if (Word(c) || c == '.' || c == '+')
                {
                    usize start = at;
                    while (at < t.Size() && (Word(t[at]) || t[at] == '.' || t[at] == '+'))
                        at++;
                    StringView s = t.Substr(start, at - start);
                    if (!Number(s, p))
                    {
                        // A bare word value (Shading: T): a string; but a word followed by ':' starts the next node.
                        usize save = at;
                        Skip();
                        if (at < t.Size() && t[at] == ':')
                        {
                            at = start;
                            return true;
                        }
                        at = save;
                        p.type = PropertyType::String;
                        p.offset = Name(s);
                        p.count = u32(s.Size());
                    }
                }
                else
                    return true;
                d.properties.Push(p);
                d.nodes[node].propertyCount++;
                Skip();
                if (at < t.Size() && t[at] == ',')
                {
                    at++;
                    continue;
                }
                return true;
            }
        }

        bool List(u32 parent, u32 depth, bool braced)
        {
            if (depth >= kMaxDepth)
                return Fail("nodes nested too deep");
            Array<u32> kids;
            while (true)
            {
                Skip();
                if (at >= t.Size())
                {
                    if (braced)
                        return Fail("a block without its closing brace");
                    break;
                }
                if (t[at] == '}')
                {
                    if (!braced)
                        return Fail("a closing brace without its block");
                    at++;
                    break;
                }
                StringView name = Ident();
                if (name.Empty())
                    return Fail(Format("'{}' where a node name is expected", t.Substr(at, 1)));
                Skip();
                if (at >= t.Size() || t[at] != ':')
                    return Fail(Format("'{}' without a colon", name));
                at++;
                u32 node = u32(d.nodes.Size());
                Node n;
                n.nameOffset = Name(name);
                n.nameSize = u32(name.Size());
                n.firstProperty = u32(d.properties.Size());
                d.nodes.Push(n);
                if (!Values(node))
                    return false;
                Skip();
                if (at < t.Size() && t[at] == '{')
                {
                    at++;
                    if (!List(node, depth + 1, true))
                        return false;
                }
                kids.Push(node);
            }
            d.nodes[parent].firstChild = u32(d.children.Size());
            d.nodes[parent].childCount = u32(kids.Size());
            d.children.Append(kids.Data(), kids.Size());
            return true;
        }

        bool Run()
        {
            d.nodes.Push(Node());
            if (!List(0, 0, false))
                return false;
            const Property* v = d.Value(d.Find(0, "FBXHeaderExtension"), "FBXVersion");
            d.version = v && v->IsNumber() ? u32(v->integer) : 0;
            return true;
        }
    };

    bool Document::Read(Span<const u8> data, String& error)
    {
        *this = Document();
        binary = data.Size() >= 21 && std::memcmp(data.Data(), kMagic, 20) == 0;
        if (binary)
        {
            BinaryReader r{*this, data, error};
            if (r.Run())
                return true;
        }
        else
        {
            AsciiReader r{*this, StringView(reinterpret_cast<const char*>(data.Data()), data.Size()), error};
            if (r.Run())
                return true;
        }
        *this = Document();
        return false;
    }

    u32 Document::Find(u32 node, StringView name) const
    {
        if (node >= nodes.Size())
            return kNoNode;
        for (u32 i = 0; i < nodes[node].childCount; i++)
        {
            u32 c = children[nodes[node].firstChild + i];
            if (Name(c) == name)
                return c;
        }
        return kNoNode;
    }

    const Property* Document::Value(u32 node, StringView child, u32 index) const
    {
        u32 c = Find(node, child);
        if (c == kNoNode || index >= nodes[c].propertyCount)
            return nullptr;
        return &properties[nodes[c].firstProperty + index];
    }

    StringView Document::Text(const Property& p) const
    {
        if (p.type != PropertyType::String && p.type != PropertyType::Raw)
            return StringView();
        return StringView(reinterpret_cast<const char*>(bytes.Data()) + p.offset, p.count);
    }

    bool Document::Numbers(const Property& p, Array<double>& out) const
    {
        out.Clear();
        if (!p.IsArray())
        {
            if (!p.IsNumber())
                return false;
            out.Push(p.type == PropertyType::Float || p.type == PropertyType::Double ? p.number : double(p.integer));
            return true;
        }
        out.Resize(p.count);
        const u8* a = arrays.Data() + p.offset;
        for (u32 i = 0; i < p.count; i++)
        {
            switch (p.type)
            {
            case PropertyType::BoolArray: out[i] = a[i] != 0; break;
            case PropertyType::Int32Array:
            {
                i32 v;
                std::memcpy(&v, a + 4 * usize(i), 4);
                out[i] = v;
                break;
            }
            case PropertyType::Int64Array:
            {
                i64 v;
                std::memcpy(&v, a + 8 * usize(i), 8);
                out[i] = double(v);
                break;
            }
            case PropertyType::FloatArray:
            {
                float v;
                std::memcpy(&v, a + 4 * usize(i), 4);
                out[i] = v;
                break;
            }
            default:
            {
                double v;
                std::memcpy(&v, a + 8 * usize(i), 8);
                out[i] = v;
                break;
            }
            }
        }
        return true;
    }

    bool Document::Integers(const Property& p, Array<i64>& out) const
    {
        out.Clear();
        if (!p.IsArray())
        {
            if (!p.IsNumber())
                return false;
            out.Push(p.type == PropertyType::Float || p.type == PropertyType::Double ? i64(p.number) : p.integer);
            return true;
        }
        out.Resize(p.count);
        const u8* a = arrays.Data() + p.offset;
        for (u32 i = 0; i < p.count; i++)
        {
            switch (p.type)
            {
            case PropertyType::BoolArray: out[i] = a[i] != 0; break;
            case PropertyType::Int32Array:
            {
                i32 v;
                std::memcpy(&v, a + 4 * usize(i), 4);
                out[i] = v;
                break;
            }
            case PropertyType::Int64Array: std::memcpy(&out[i], a + 8 * usize(i), 8); break;
            case PropertyType::FloatArray:
            {
                float v;
                std::memcpy(&v, a + 4 * usize(i), 4);
                out[i] = i64(v);
                break;
            }
            default:
            {
                double v;
                std::memcpy(&v, a + 8 * usize(i), 8);
                out[i] = v >= -9.2e18 && v <= 9.2e18 ? i64(v) : 0;
                break;
            }
            }
        }
        return true;
    }

    StringView ObjectName(StringView full)
    {
        for (usize i = 0; i + 1 < full.Size(); i++)
            if (full[i] == '\0' && full[i + 1] == '\x01')
                return full.Substr(0, i);
        usize colons = full.Find("::");
        return colons == StringView::npos ? full : full.Substr(colons + 2);
    }
}
