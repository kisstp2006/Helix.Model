#include <Helix/Model/Obj/Obj.h>

#include <Helix/Core/Containers/HashMap.h>
#include <Helix/Core/Strings/Format.h>

#include <charconv>
#include <cmath>

namespace helix::model
{
    namespace
    {
        // A text split into lines, continuation backslashes joined, comments cut. Lines are views into the text; only a
        // continued line is copied.
        class Lines
        {
        public:
            explicit Lines(StringView text) : text_(text) {}

            bool Next(StringView& line, u32& number)
            {
                if (at_ >= text_.Size())
                    return false;
                number = ++line_;
                StringView part = Take();
                if (part.Empty() || part[part.Size() - 1] != '\\')
                    line = part;
                else
                {
                    joined_.Clear();
                    while (!part.Empty() && part[part.Size() - 1] == '\\')
                    {
                        joined_.Append(part.Substr(0, part.Size() - 1));
                        joined_.Append(' ');
                        if (at_ >= text_.Size())
                        {
                            part = StringView();
                            break;
                        }
                        part = Take();
                        line_++;
                    }
                    joined_.Append(part);
                    line = joined_.View();
                }
                usize hash = line.Find('#');
                if (hash != StringView::npos)
                    line = line.Substr(0, hash);
                return true;
            }

        private:
            StringView Take()
            {
                usize end = text_.Find('\n', at_);
                if (end == StringView::npos)
                    end = text_.Size();
                StringView part = text_.Substr(at_, end - at_);
                at_ = end + 1;
                if (!part.Empty() && part[part.Size() - 1] == '\r')
                    part = part.Substr(0, part.Size() - 1);
                return part;
            }

            StringView text_;
            usize at_ = 0;
            u32 line_ = 0;
            String joined_;
        };

        bool Space(char c) { return c == ' ' || c == '\t' || c == '\v' || c == '\f'; }

        // Whitespace separated words.
        void Split(StringView line, Array<StringView>& out)
        {
            out.Clear();
            usize i = 0;
            while (i < line.Size())
            {
                while (i < line.Size() && Space(line[i]))
                    i++;
                usize start = i;
                while (i < line.Size() && !Space(line[i]))
                    i++;
                if (i > start)
                    out.Push(line.Substr(start, i - start));
            }
        }

        // The rest of a line after its keyword, trimmed.
        StringView Rest(StringView line, StringView keyword)
        {
            usize at = line.Find(keyword);
            StringView r = line.Substr(at + keyword.Size());
            while (!r.Empty() && Space(r[0]))
                r = r.Substr(1);
            while (!r.Empty() && Space(r[r.Size() - 1]))
                r = r.Substr(0, r.Size() - 1);
            return r;
        }

        bool ParseFloat(StringView t, float& out)
        {
            if (!t.Empty() && t[0] == '+')
                t = t.Substr(1);
            // The common short decimals exactly through double arithmetic (at most 15 digits and a power of ten up to
            // 22 are exact in a double); the rest through from_chars.
            static constexpr double kPowers[] = {1e0,  1e1,  1e2,  1e3,  1e4,  1e5,  1e6,  1e7,  1e8,  1e9,  1e10, 1e11,
                                                 1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22};
            const char* p = t.Data();
            const char* end = p + t.Size();
            bool negative = p < end && *p == '-';
            p += negative;
            u64 mantissa = 0;
            int digits = 0, exponent = 0;
            bool any = false;
            while (p < end && *p >= '0' && *p <= '9')
            {
                if (digits < 18)
                    mantissa = mantissa * 10 + u64(*p - '0'), digits += mantissa != 0;
                else
                    exponent++;
                p++, any = true;
            }
            if (p < end && *p == '.')
            {
                p++;
                while (p < end && *p >= '0' && *p <= '9')
                {
                    if (digits < 18)
                        mantissa = mantissa * 10 + u64(*p - '0'), digits += mantissa != 0, exponent--;
                    p++, any = true;
                }
            }
            if (any && p < end && (*p == 'e' || *p == 'E'))
            {
                const char* q = p + 1;
                bool eneg = q < end && *q == '-';
                q += (q < end && (*q == '-' || *q == '+'));
                int e = 0;
                bool edigits = false;
                while (q < end && *q >= '0' && *q <= '9' && e < 10000)
                    e = e * 10 + (*q++ - '0'), edigits = true;
                if (edigits)
                    exponent += eneg ? -e : e, p = q;
            }
            if (any && p == end && digits <= 15 && exponent >= -22 && exponent <= 22)
            {
                double v = double(mantissa);
                v = exponent < 0 ? v / kPowers[-exponent] : v * kPowers[exponent];
                out = float(negative ? -v : v);
                return std::isfinite(out);
            }
            auto r = std::from_chars(t.Data(), t.Data() + t.Size(), out);
            return r.ec == std::errc() && r.ptr == t.Data() + t.Size() && std::isfinite(out);
        }

        bool ParseInt(StringView t, i64& out)
        {
            if (!t.Empty() && t[0] == '+')
                t = t.Substr(1);
            auto r = std::from_chars(t.Data(), t.Data() + t.Size(), out);
            return r.ec == std::errc() && r.ptr == t.Data() + t.Size();
        }

        // ------------------------------------------------------------------ MTL

        class MtlReader
        {
        public:
            MtlReader(Scene& s, String& error) : s_(s), error_(error) {}

            bool Run(StringView text)
            {
                Lines lines(text);
                StringView line;
                u32 number;
                Array<StringView> w;
                Material* m = nullptr;
                usize current = 0;
                while (lines.Next(line, number))
                {
                    Split(line, w);
                    if (w.Empty())
                        continue;
                    StringView key = w[0];
                    if (key == StringView("newmtl"))
                    {
                        StringView name = Rest(line, "newmtl");
                        current = s_.materials.Size();
                        for (usize i = 0; i < s_.materials.Size(); i++)
                            if (s_.materials[i].name == name)
                                current = i;
                        if (current == s_.materials.Size())
                            s_.materials.Emplace();
                        s_.materials[current] = Material();
                        s_.materials[current].name = name;
                        // OBJ materials are not metals unless they say so.
                        s_.materials[current].metallic = 0;
                        roughnessSet_ = false;
                        m = &s_.materials[current];
                        continue;
                    }
                    if (!m)
                        continue;
                    m = &s_.materials[current];
                    auto color = [&](float* out) {
                        if (w.Size() >= 4)
                            return ParseFloat(w[1], out[0]) && ParseFloat(w[2], out[1]) && ParseFloat(w[3], out[2]);
                        // One value: gray.
                        if (w.Size() == 2 && ParseFloat(w[1], out[0]))
                            return out[1] = out[2] = out[0], true;
                        return false;
                    };
                    auto scalar = [&](float& out) { return w.Size() >= 2 && ParseFloat(w[w.Size() - 1], out); };
                    bool ok = true;
                    if (key == StringView("Kd"))
                        ok = color(m->baseColor);
                    else if (key == StringView("Ks"))
                        ok = color(m->specularColorClassic);
                    else if (key == StringView("Ke"))
                        ok = color(m->emissive);
                    else if (key == StringView("Ka") || key == StringView("Tf") || key == StringView("illum") || key == StringView("sharpness"))
                        ok = true;
                    else if (key == StringView("Ns"))
                    {
                        ok = scalar(m->shininess);
                        if (ok && !roughnessSet_)
                            m->roughness = std::sqrt(2.0f / (std::fmax(m->shininess, 0.0f) + 2.0f));
                    }
                    else if (key == StringView("Ni"))
                        ok = scalar(m->ior);
                    else if (key == StringView("d") || key == StringView("Tr"))
                    {
                        float v;
                        ok = scalar(v);
                        if (ok)
                        {
                            m->baseColor[3] = key == StringView("d") ? v : 1 - v;
                            if (m->baseColor[3] < 1)
                                m->alphaMode = AlphaMode::Blend;
                        }
                    }
                    else if (key == StringView("Pr"))
                        ok = scalar(m->roughness), roughnessSet_ = true;
                    else if (key == StringView("Pm"))
                        ok = scalar(m->metallic);
                    else if (key == StringView("Ps"))
                        ok = color(m->sheenColor), m->hasSheen = true;
                    else if (key == StringView("Pc"))
                        ok = scalar(m->clearcoat), m->hasClearcoat = true;
                    else if (key == StringView("Pcr"))
                        ok = scalar(m->clearcoatRoughness), m->hasClearcoat = true;
                    else if (key == StringView("aniso"))
                        ok = scalar(m->anisotropyStrength), m->hasAnisotropy = true;
                    else if (key == StringView("anisor"))
                        ok = scalar(m->anisotropyRotation), m->hasAnisotropy = true;
                    else if (TextureRef* slot = Slot(*m, key))
                        ok = Map(line, key, *slot, number);
                    else
                    {
                        s_.warnings.Push(Format("MTL line {}: '{}' skipped", number, key));
                        continue;
                    }
                    if (!ok)
                        return error_ = Format("MTL line {}: '{}' without its values", number, key), false;
                    if (key == StringView("map_d"))
                        m->alphaMode = AlphaMode::Blend;
                }
                return true;
            }

        private:
            TextureRef* Slot(Material& m, StringView key)
            {
                if (key == StringView("map_Kd"))
                    return &m.baseColorTexture;
                if (key == StringView("map_Ks"))
                    return &m.specularColorClassicTexture;
                if (key == StringView("map_Ns"))
                    return &m.shininessTexture;
                if (key == StringView("map_Ke"))
                    return &m.emissiveTexture;
                if (key == StringView("map_d"))
                    return &m.alphaTexture;
                if (key == StringView("map_bump") || key == StringView("map_Bump") || key == StringView("bump"))
                    return &m.bumpTexture;
                if (key == StringView("norm") || key == StringView("map_Kn"))
                    return &m.normalTexture;
                if (key == StringView("disp"))
                    return &m.displacementTexture;
                if (key == StringView("map_Pr"))
                    return &m.roughnessTexture;
                if (key == StringView("map_Pm"))
                    return &m.metallicTexture;
                if (key == StringView("map_Ps"))
                    return &m.sheenColorTexture;
                if (key == StringView("map_Ka") || key == StringView("refl"))
                    return &scratch_;
                return nullptr;
            }

            // A map statement: options, then the file name (which may hold spaces).
            bool Map(StringView line, StringView key, TextureRef& slot, u32 number)
            {
                StringView rest = Rest(line, key);
                Array<StringView> w;
                Split(rest, w);
                TextureRef ref;
                bool clamp = false;
                usize i = 0;
                auto numbers = [&](float* out, u32 most) {
                    u32 n = 0;
                    float v;
                    while (n < most && i < w.Size() && ParseFloat(w[i], v))
                        out[n++] = v, i++;
                    return n;
                };
                while (i < w.Size() && w[i].Size() > 1 && w[i][0] == '-')
                {
                    StringView opt = w[i++];
                    float values[3];
                    if (opt == StringView("-o"))
                    {
                        u32 n = numbers(values, 3);
                        if (n >= 1)
                            ref.offset[0] = values[0];
                        if (n >= 2)
                            ref.offset[1] = values[1];
                    }
                    else if (opt == StringView("-s"))
                    {
                        u32 n = numbers(values, 3);
                        if (n >= 1)
                            ref.scale[0] = values[0];
                        if (n >= 2)
                            ref.scale[1] = values[1];
                    }
                    else if (opt == StringView("-t"))
                        numbers(values, 3);
                    else if (opt == StringView("-bm"))
                    {
                        if (numbers(values, 1) == 1)
                            ref.strength = values[0];
                    }
                    else if (opt == StringView("-mm"))
                        numbers(values, 2);
                    else if (opt == StringView("-clamp"))
                        clamp = i < w.Size() && w[i++] == StringView("on");
                    else if (opt == StringView("-blendu") || opt == StringView("-blendv") || opt == StringView("-cc") || opt == StringView("-imfchan") ||
                             opt == StringView("-type") || opt == StringView("-texres") || opt == StringView("-boost"))
                        i++;
                    else
                        s_.warnings.Push(Format("MTL line {}: map option '{}' skipped", number, opt));
                }
                if (i >= w.Size())
                    return false;
                // The name: from the first word left to the end of the line.
                usize from = usize(w[i].Data() - rest.Data());
                StringView file = rest.Substr(from);
                ref.texture = Texture(file, clamp);
                slot = ref;
                return true;
            }

            u32 Texture(StringView file, bool clamp)
            {
                u32 image = kNone;
                for (u32 i = 0; i < s_.images.Size(); i++)
                    if (s_.images[i].uri == file)
                        image = i;
                if (image == kNone)
                {
                    image = u32(s_.images.Size());
                    s_.images.Emplace().uri = file;
                }
                for (u32 i = 0; i < s_.textures.Size(); i++)
                    if (s_.textures[i].image == image && (s_.textures[i].wrapS == Wrap::ClampToEdge) == clamp)
                        return i;
                ::helix::model::Texture& t = s_.textures.Emplace();
                t.image = image;
                t.wrapS = t.wrapT = clamp ? Wrap::ClampToEdge : Wrap::Repeat;
                return u32(s_.textures.Size() - 1);
            }

            Scene& s_;
            String& error_;
            bool roughnessSet_ = false;
            TextureRef scratch_;
        };

        // ------------------------------------------------------------------ OBJ

        struct Part
        {
            u32 material;
            Topology topology;
            // Position, texture coordinate and normal index per corner (-1 when missing).
            Array<i32> corners;
        };

        struct ObjMesh
        {
            String name;
            Array<Part> parts;
        };

        class ObjReader
        {
        public:
            ObjReader(Scene& s, String& error, const ReadOptions& options) : s_(s), error_(error), options_(options) {}

            bool Run(StringView text)
            {
                s_.upAxis = 1;
                s_.unitScale = 0;
                Lines lines(text);
                StringView line;
                u32 number;
                Array<StringView> w;
                meshes_.Emplace().name = "default";
                while (lines.Next(line, number))
                {
                    Split(line, w);
                    if (w.Empty())
                        continue;
                    StringView key = w[0];
                    if (key == StringView("v"))
                    {
                        float v[7];
                        if (w.Size() < 4)
                            return Fail(number, "v needs three coordinates");
                        for (usize i = 1; i < w.Size() && i < 8; i++)
                            if (!ParseFloat(w[i], v[i - 1]))
                                return Fail(number, Format("'{}' is not a number", w[i]));
                        positions_.Push(v[0]), positions_.Push(v[1]), positions_.Push(v[2]);
                        // x y z r g b, or x y z w r g b.
                        bool rgb = w.Size() == 7 || w.Size() == 8;
                        usize c = w.Size() == 8 ? 4 : 3;
                        if (rgb)
                            hasColors_ = true;
                        colors_.Push(rgb ? v[c] : 1), colors_.Push(rgb ? v[c + 1] : 1), colors_.Push(rgb ? v[c + 2] : 1);
                    }
                    else if (key == StringView("vt"))
                    {
                        float u = 0, v = 0;
                        if (w.Size() < 2 || !ParseFloat(w[1], u) || (w.Size() >= 3 && !ParseFloat(w[2], v)))
                            return Fail(number, "vt needs a number");
                        texcoords_.Push(u), texcoords_.Push(v);
                    }
                    else if (key == StringView("vn"))
                    {
                        float n[3];
                        if (w.Size() < 4 || !ParseFloat(w[1], n[0]) || !ParseFloat(w[2], n[1]) || !ParseFloat(w[3], n[2]))
                            return Fail(number, "vn needs three numbers");
                        normals_.Push(n[0]), normals_.Push(n[1]), normals_.Push(n[2]);
                    }
                    else if (key == StringView("f") || key == StringView("l") || key == StringView("p"))
                    {
                        Topology topology = key == StringView("f") ? Topology::Triangles : key == StringView("l") ? Topology::Lines : Topology::Points;
                        usize need = topology == Topology::Triangles ? 3 : topology == Topology::Lines ? 2 : 1;
                        if (w.Size() < need + 1)
                            return Fail(number, Format("{} needs {} vertices", key, need));
                        corner_.Clear();
                        for (usize i = 1; i < w.Size(); i++)
                            if (!Corner(w[i], number))
                                return false;
                        Array<i32>& out = Current(topology).corners;
                        usize n = corner_.Size() / 3;
                        auto put = [&](usize k) { out.Append(corner_.Data() + 3 * k, 3); };
                        if (topology == Topology::Triangles)
                            for (usize k = 1; k + 1 < n; k++)
                                put(0), put(k), put(k + 1);
                        else if (topology == Topology::Lines)
                            for (usize k = 0; k + 1 < n; k++)
                                put(k), put(k + 1);
                        else
                            for (usize k = 0; k < n; k++)
                                put(k);
                    }
                    else if (key == StringView("o") || key == StringView("g"))
                    {
                        StringView name = w.Size() > 1 ? Rest(line, key) : StringView("default");
                        ObjMesh& last = meshes_.Back();
                        bool empty = true;
                        for (const Part& p : last.parts)
                            empty = empty && p.corners.Empty();
                        if (empty)
                            last.name = name;
                        else
                            meshes_.Emplace().name = name;
                    }
                    else if (key == StringView("usemtl"))
                        material_ = MaterialNamed(w.Size() > 1 ? Rest(line, "usemtl") : StringView("default"));
                    else if (key == StringView("mtllib"))
                    {
                        if (!Library(Rest(line, "mtllib"), number))
                            return false;
                    }
                    else if (key == StringView("s") || key == StringView("vp") || key == StringView("cstype") || key == StringView("deg") ||
                             key == StringView("curv") || key == StringView("curv2") || key == StringView("surf") || key == StringView("parm") ||
                             key == StringView("trim") || key == StringView("hole") || key == StringView("end") || key == StringView("bmat") ||
                             key == StringView("step") || key == StringView("con") || key == StringView("mg") || key == StringView("lod") ||
                             key == StringView("usemap") || key == StringView("maplib") || key == StringView("shadow_obj") || key == StringView("trace_obj"))
                        continue;
                    else
                        s_.warnings.Push(Format("OBJ line {}: '{}' skipped", number, key));
                }
                return Build();
            }

        private:
            bool Fail(u32 line, StringView what)
            {
                error_ = Format("line {}: {}", line, what);
                return false;
            }

            // One corner "v", "v/t", "v//n" or "v/t/n" into corner_.
            bool Corner(StringView token, u32 line)
            {
                i32 index[3] = {-1, -1, -1};
                usize counts[3] = {positions_.Size() / 3, texcoords_.Size() / 2, normals_.Size() / 3};
                usize at = 0;
                for (int part = 0; part < 3 && at <= token.Size(); part++)
                {
                    usize slash = token.Find('/', at);
                    StringView t = token.Substr(at, (slash == StringView::npos ? token.Size() : slash) - at);
                    if (!t.Empty())
                    {
                        i64 v;
                        if (!ParseInt(t, v) || v == 0)
                            return Fail(line, Format("'{}' is not a vertex", token));
                        i64 resolved = v > 0 ? v - 1 : i64(counts[part]) + v;
                        if (resolved < 0 || resolved >= i64(counts[part]))
                            return Fail(line, Format("index {} of {} in '{}'", v, counts[part], token));
                        index[part] = i32(resolved);
                    }
                    else if (part == 0)
                        return Fail(line, Format("'{}' has no position", token));
                    if (slash == StringView::npos)
                        break;
                    at = slash + 1;
                }
                corner_.Append(index, 3);
                return true;
            }

            Part& Current(Topology topology)
            {
                ObjMesh& m = meshes_.Back();
                for (Part& p : m.parts)
                    if (p.material == material_ && p.topology == topology)
                        return p;
                Part& p = m.parts.Emplace();
                p.material = material_;
                p.topology = topology;
                return p;
            }

            u32 MaterialNamed(StringView name)
            {
                for (u32 i = 0; i < s_.materials.Size(); i++)
                    if (s_.materials[i].name == name)
                        return i;
                Material& m = s_.materials.Emplace();
                m.name = name;
                m.metallic = 0;
                return u32(s_.materials.Size() - 1);
            }

            bool Library(StringView names, u32 line)
            {
                // One name with spaces when it ends in .mtl, else names separated by spaces.
                Array<StringView> files;
                if (names.Size() > 4 && names.Substr(names.Size() - 4) == StringView(".mtl") && names.Find(".mtl ") == StringView::npos)
                    files.Push(names);
                else
                    Split(names, files);
                for (StringView f : files)
                {
                    Array<u8> bytes;
                    if (!options_.loader || !options_.loader(options_.context, f, bytes))
                    {
                        s_.warnings.Push(Format("OBJ line {}: material library '{}' could not be loaded", line, f));
                        continue;
                    }
                    String error;
                    if (!ReadMtl(StringView(reinterpret_cast<const char*>(bytes.Data()), bytes.Size()), s_, error))
                        return Fail(line, Format("{}: {}", f, error));
                }
                return true;
            }

            bool Build()
            {
                for (ObjMesh& om : meshes_)
                {
                    bool any = false;
                    for (const Part& p : om.parts)
                        any = any || !p.corners.Empty();
                    if (!any)
                        continue;
                    Mesh mesh;
                    mesh.name = om.name;
                    mesh.primitives.begin = u32(s_.primitives.Size());
                    for (const Part& part : om.parts)
                        if (!part.corners.Empty())
                            Emit(part);
                    mesh.primitives.count = u32(s_.primitives.Size()) - mesh.primitives.begin;
                    u32 meshIndex = u32(s_.meshes.Size());
                    s_.meshes.Push(mesh);
                    Node& node = s_.nodes.Emplace();
                    node.name = om.name;
                    node.mesh = meshIndex;
                    s_.roots.Push(u32(s_.nodes.Size() - 1));
                }
                return true;
            }

            // A primitive of the corners, vertices shared by their index triples.
            void Emit(const Part& part)
            {
                usize n = part.corners.Size() / 3;
                bool texcoords = false, normals = false;
                for (usize k = 0; k < n; k++)
                {
                    texcoords = texcoords || part.corners[3 * k + 1] >= 0;
                    normals = normals || part.corners[3 * k + 2] >= 0;
                }
                HashMap<u64, u32> seen;
                Array<u32> indices;
                indices.Reserve(n);
                Array<u32> unique;
                bool packable = positions_.Size() / 3 < (1u << 21) && texcoords_.Size() / 2 < (1u << 21) && normals_.Size() / 3 < (1u << 21);
                for (usize k = 0; k < n; k++)
                {
                    const i32* c = part.corners.Data() + 3 * k;
                    if (packable)
                    {
                        u64 key = u64(c[0] + 1) | u64(c[1] + 1) << 21 | u64(c[2] + 1) << 42;
                        if (const u32* known = seen.Find(key))
                        {
                            indices.Push(*known);
                            continue;
                        }
                        seen.Set(key, u32(unique.Size() / 3));
                    }
                    indices.Push(u32(unique.Size() / 3));
                    unique.Append(reinterpret_cast<const u32*>(c), 3);
                }
                u32 vertices = u32(unique.Size() / 3);
                Primitive p;
                p.topology = part.topology;
                p.material = part.material;
                p.vertexCount = vertices;
                p.streams.begin = u32(s_.streams.Size());
                auto stream = [&](Semantic semantic, u32 components, const Array<float>& source, int which, bool fill) {
                    VertexStream st;
                    st.semantic = semantic;
                    st.components = components;
                    st.offset = u32(s_.floats.Size());
                    for (u32 v = 0; v < vertices; v++)
                    {
                        i32 i = i32(unique[usize(v) * 3 + usize(which)]);
                        for (u32 c = 0; c < components; c++)
                            s_.floats.Push(i >= 0 ? source[usize(i) * components + c] : (fill ? 1.0f : 0.0f));
                    }
                    s_.streams.Push(st);
                };
                stream(Semantic::Position, 3, positions_, 0, false);
                if (normals)
                    stream(Semantic::Normal, 3, normals_, 2, false);
                if (texcoords)
                    stream(Semantic::TexCoord, 2, texcoords_, 1, false);
                if (hasColors_)
                    stream(Semantic::Color, 3, colors_, 0, true);
                p.streams.count = u32(s_.streams.Size()) - p.streams.begin;
                p.indices = {u32(s_.indices.Size()), u32(indices.Size())};
                s_.indices.Append(indices.Data(), indices.Size());
                s_.primitives.Push(p);
            }

            Scene& s_;
            String& error_;
            const ReadOptions& options_;
            Array<float> positions_, colors_, texcoords_, normals_;
            bool hasColors_ = false;
            Array<ObjMesh> meshes_;
            u32 material_ = kNone;
            Array<i32> corner_;
        };
    }

    bool ReadMtl(StringView text, Scene& scene, String& error)
    {
        MtlReader r(scene, error);
        return r.Run(text);
    }

    bool ReadObj(Span<const u8> data, Scene& out, String& error, const ReadOptions& options)
    {
        out.Clear();
        ObjReader r(out, error, options);
        if (r.Run(StringView(reinterpret_cast<const char*>(data.Data()), data.Size())))
            return true;
        out.Clear();
        return false;
    }
}
