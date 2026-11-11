#pragma once

#include <Helix/Core/Containers/Array.h>
#include <Helix/Core/Strings/Format.h>
#include <Helix/Core/Strings/String.h>
#include <Helix/Image/Compression/Deflate.h>

#include <cstring>

// Builds FBX files for the tests, fuzz targets and benchmarks: a node tree written as binary (7400 or 7500, arrays
// raw or zlib) or ASCII. Written from the format as it is documented by its readers; nothing here comes from the SDK.
namespace helix::model::testing
{
    inline String Base64Text(StringView data)
    {
        static constexpr char kDigits[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        String s;
        const u8* d = reinterpret_cast<const u8*>(data.Data());
        usize size = data.Size();
        for (usize i = 0; i < size; i += 3)
        {
            u32 v = u32(d[i]) << 16 | (i + 1 < size ? u32(d[i + 1]) << 8 : 0) | (i + 2 < size ? d[i + 2] : 0);
            s.Append(kDigits[v >> 18 & 63]);
            s.Append(kDigits[v >> 12 & 63]);
            s.Append(i + 1 < size ? kDigits[v >> 6 & 63] : '=');
            s.Append(i + 2 < size ? kDigits[v & 63] : '=');
        }
        return s;
    }

    struct FbxValue
    {
        // 'I', 'L', 'D', 'S', 'R', 'N' (an object name: class and name), 'i' / 'l' / 'd' / 'f' arrays.
        char type = 'I';
        i64 i = 0;
        double d = 0;
        String s, cls;
        Array<i64> ints;
        Array<double> doubles;
    };

    struct FbxNode
    {
        String name;
        Array<FbxValue> props;
        Array<FbxNode> children;

        explicit FbxNode(StringView n = {}) : name(n) {}

        FbxNode& I(i64 v)
        {
            FbxValue& x = props.Emplace();
            x.type = 'I', x.i = v;
            return *this;
        }
        FbxNode& L(i64 v)
        {
            FbxValue& x = props.Emplace();
            x.type = 'L', x.i = v;
            return *this;
        }
        FbxNode& D(double v)
        {
            FbxValue& x = props.Emplace();
            x.type = 'D', x.d = v;
            return *this;
        }
        FbxNode& S(StringView v)
        {
            FbxValue& x = props.Emplace();
            x.type = 'S', x.s = v;
            return *this;
        }
        FbxNode& R(const Array<u8>& v)
        {
            FbxValue& x = props.Emplace();
            x.type = 'R';
            x.s = StringView(reinterpret_cast<const char*>(v.Data()), v.Size());
            return *this;
        }
        FbxNode& Name(StringView cls, StringView object)
        {
            FbxValue& x = props.Emplace();
            x.type = 'N', x.cls = cls, x.s = object;
            return *this;
        }
        FbxNode& Ints(std::initializer_list<i64> v, char type = 'i')
        {
            FbxValue& x = props.Emplace();
            x.type = type;
            for (i64 e : v)
                x.ints.Push(e);
            return *this;
        }
        FbxNode& Doubles(std::initializer_list<double> v, char type = 'd')
        {
            FbxValue& x = props.Emplace();
            x.type = type;
            for (double e : v)
                x.doubles.Push(e);
            return *this;
        }
        FbxNode& DoubleArray(const Array<double>& v)
        {
            FbxValue& x = props.Emplace();
            x.type = 'd';
            x.doubles = v;
            return *this;
        }
        FbxNode& IntArray(const Array<i64>& v)
        {
            FbxValue& x = props.Emplace();
            x.type = 'i';
            x.ints = v;
            return *this;
        }
        FbxNode& Add(FbxNode child)
        {
            children.Push(Move(child));
            return *this;
        }
        // A child holding one array, as FBX writes Vertices, Normals and the like.
        FbxNode& Numbers(StringView child, std::initializer_list<double> v)
        {
            FbxNode n(child);
            n.Doubles(v);
            return Add(Move(n));
        }
        FbxNode& IntArrayChild(StringView child, std::initializer_list<i64> v)
        {
            FbxNode n(child);
            n.Ints(v);
            return Add(Move(n));
        }
        FbxNode& Text(StringView child, StringView v)
        {
            FbxNode n(child);
            n.S(v);
            return Add(Move(n));
        }
        FbxNode& Int(StringView child, i64 v)
        {
            FbxNode n(child);
            n.I(v);
            return Add(Move(n));
        }
    };

    // A Properties70 entry: P: name, type, label, flags, values.
    inline FbxNode P(StringView name, StringView type, std::initializer_list<double> values)
    {
        FbxNode p("P");
        p.S(name).S(type).S("").S("A");
        for (double v : values)
            p.D(v);
        return p;
    }
    inline FbxNode PInt(StringView name, i64 value)
    {
        FbxNode p("P");
        p.S(name).S("int").S("Integer").S("").I(value);
        return p;
    }
    inline FbxNode PTime(StringView name, i64 ticks)
    {
        FbxNode p("P");
        p.S(name).S("KTime").S("Time").S("").L(ticks);
        return p;
    }

    // An object node: Kind: id, "Class::Name", "Subclass".
    inline FbxNode Object(StringView kind, i64 id, StringView cls, StringView name, StringView subclass)
    {
        FbxNode n(kind);
        n.L(id).Name(cls, name).S(subclass);
        return n;
    }

    inline FbxNode Connect(i64 child, i64 parent, StringView property = {})
    {
        FbxNode c("C");
        c.S(property.Empty() ? "OO" : "OP").L(child).L(parent);
        if (!property.Empty())
            c.S(property);
        return c;
    }

    struct FbxFile
    {
        Array<FbxNode> top;

        Array<u8> Binary(u32 version = 7400, bool zlib = false) const
        {
            Array<u8> out;
            const char magic[] = "Kaydara FBX Binary  ";
            out.Append(reinterpret_cast<const u8*>(magic), 21);
            out.Push(u8(0x1A));
            out.Push(u8(0));
            Put(out, version, 4);
            bool wide = version >= 7500;
            for (const FbxNode& n : top)
                Record(out, n, wide, zlib);
            for (int i = 0; i < (wide ? 25 : 13); i++)
                out.Push(u8(0));
            return out;
        }

        String Ascii() const
        {
            String s = "; FBX 7.4.0 project file\n; ----------------------------------------------------\n\n";
            s.Append("FBXHeaderExtension:  {\n\tFBXHeaderVersion: 1003\n\tFBXVersion: 7400\n}\n");
            for (const FbxNode& n : top)
                Text(s, n, 0);
            return s;
        }

    private:
        static void Put(Array<u8>& out, u64 v, int bytes)
        {
            for (int i = 0; i < bytes; i++)
                out.Push(u8(v >> (8 * i)));
        }

        static void Patch(Array<u8>& out, usize at, u64 v, int bytes)
        {
            for (int i = 0; i < bytes; i++)
                out[at + i] = u8(v >> (8 * i));
        }

        static void Value(Array<u8>& out, const FbxValue& v, bool zlib)
        {
            auto raw = [&](StringView s, char type) {
                out.Push(u8(type));
                Put(out, s.Size(), 4);
                out.Append(reinterpret_cast<const u8*>(s.Data()), s.Size());
            };
            switch (v.type)
            {
            case 'I':
                out.Push(u8('I'));
                Put(out, u64(v.i), 4);
                break;
            case 'L':
                out.Push(u8('L'));
                Put(out, u64(v.i), 8);
                break;
            case 'D':
            {
                out.Push(u8('D'));
                u64 b;
                std::memcpy(&b, &v.d, 8);
                Put(out, b, 8);
                break;
            }
            case 'S': raw(v.s, 'S'); break;
            case 'R': raw(v.s, 'R'); break;
            case 'N':
            {
                String full = v.s;
                full.Append('\0');
                full.Append('\x01');
                full.Append(v.cls);
                raw(full, 'S');
                break;
            }
            default:
            {
                Array<u8> data;
                u32 count = 0;
                if (v.type == 'd' || v.type == 'f')
                {
                    count = u32(v.doubles.Size());
                    for (double d : v.doubles)
                    {
                        if (v.type == 'd')
                            data.Append(reinterpret_cast<const u8*>(&d), 8);
                        else
                        {
                            float f = float(d);
                            data.Append(reinterpret_cast<const u8*>(&f), 4);
                        }
                    }
                }
                else
                {
                    count = u32(v.ints.Size());
                    for (i64 i : v.ints)
                    {
                        if (v.type == 'l')
                            data.Append(reinterpret_cast<const u8*>(&i), 8);
                        else
                        {
                            i32 x = i32(i);
                            data.Append(reinterpret_cast<const u8*>(&x), 4);
                        }
                    }
                }
                out.Push(u8(v.type));
                Put(out, count, 4);
                if (zlib)
                {
                    Array<u8> packed;
                    image::ZlibCompress(Span<const u8>(data.Data(), data.Size()), packed);
                    Put(out, 1, 4);
                    Put(out, packed.Size(), 4);
                    out.Append(packed.Data(), packed.Size());
                }
                else
                {
                    Put(out, 0, 4);
                    Put(out, data.Size(), 4);
                    out.Append(data.Data(), data.Size());
                }
                break;
            }
            }
        }

        static void Record(Array<u8>& out, const FbxNode& n, bool wide, bool zlib)
        {
            int w = wide ? 8 : 4;
            usize start = out.Size();
            Put(out, 0, w);
            Put(out, n.props.Size(), w);
            Put(out, 0, w);
            out.Push(u8(n.name.Size()));
            out.Append(reinterpret_cast<const u8*>(n.name.Data()), n.name.Size());
            usize props = out.Size();
            for (const FbxValue& v : n.props)
                Value(out, v, zlib);
            Patch(out, start + 2 * w, out.Size() - props, w);
            if (!n.children.Empty())
            {
                for (const FbxNode& c : n.children)
                    Record(out, c, wide, zlib);
                for (int i = 0; i < (wide ? 25 : 13); i++)
                    out.Push(u8(0));
            }
            Patch(out, start, out.Size(), w);
        }

        static void Text(String& s, const FbxNode& n, int depth)
        {
            s.Append('\t', usize(depth));
            s.Append(n.name);
            s.Append(": ");
            for (usize i = 0; i < n.props.Size(); i++)
            {
                const FbxValue& v = n.props[i];
                if (i)
                    s.Append(", ");
                switch (v.type)
                {
                case 'I':
                case 'L': FormatAppend(s, "{}", v.i); break;
                case 'D': FormatAppend(s, "{}", v.d); break;
                case 'S': FormatAppend(s, "\"{}\"", v.s); break;
                case 'N': FormatAppend(s, "\"{}::{}\"", v.cls, v.s); break;
                case 'R': FormatAppend(s, ", \"{}\"", Base64Text(v.s)); break;
                default:
                {
                    usize count = v.doubles.Size() + v.ints.Size();
                    FormatAppend(s, "*{} {{\n", count);
                    s.Append('\t', usize(depth + 1));
                    s.Append("a: ");
                    for (usize k = 0; k < v.doubles.Size(); k++)
                        FormatAppend(s, k ? ",{}" : "{}", v.doubles[k]);
                    for (usize k = 0; k < v.ints.Size(); k++)
                        FormatAppend(s, k ? ",{}" : "{}", v.ints[k]);
                    s.Append('\n');
                    s.Append('\t', usize(depth));
                    s.Append('}');
                    break;
                }
                }
            }
            if (!n.children.Empty())
            {
                s.Append(" {\n");
                for (const FbxNode& c : n.children)
                    Text(s, c, depth + 1);
                s.Append('\t', usize(depth));
                s.Append("}\n");
            }
            else
                s.Append('\n');
        }
    };
}
