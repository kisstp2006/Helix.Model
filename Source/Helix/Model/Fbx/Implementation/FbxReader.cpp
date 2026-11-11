#include <Helix/Model/Fbx/Document.h>
#include <Helix/Model/Fbx/Fbx.h>
#include <Helix/Model/Scene/Implementation/Transform.h>

#include <Helix/Core/Containers/HashMap.h>
#include <Helix/Core/Strings/Format.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace helix::model
{
    namespace
    {
        using detail::Mat4;
        using fbx::Document;
        using fbx::Property;
        using fbx::PropertyType;

        constexpr double kTicksPerSecond = 46186158000.0;

        bool Base64(StringView text, Array<u8>& out)
        {
            u32 bits = 0;
            int have = 0;
            for (char c : text)
            {
                int v = c >= 'A' && c <= 'Z' ? c - 'A' : c >= 'a' && c <= 'z' ? c - 'a' + 26 : c >= '0' && c <= '9' ? c - '0' + 52 : c == '+' ? 62 : c == '/' ? 63 : -1;
                if (c == '=')
                    break;
                if (v < 0)
                    return false;
                bits = bits << 6 | u32(v);
                have += 6;
                if (have >= 8)
                {
                    have -= 8;
                    out.Push(u8(bits >> have));
                }
            }
            return true;
        }
        // Frames one clip is resampled to at most.
        constexpr u64 kMaxFrames = 100000;

        struct Object
        {
            i64 id = 0;
            u32 node = 0;
            StringView kind, subclass, name;
        };

        struct Link
        {
            i64 child = 0, parent = 0;
            StringView property;
            // Order in the file.
            u32 order = 0;
        };

        // The FBX transform of a model, in its own pieces.
        struct Xform
        {
            double t[3] = {0, 0, 0}, r[3] = {0, 0, 0}, s[3] = {1, 1, 1};
            double pre[3] = {0, 0, 0}, post[3] = {0, 0, 0};
            double rotationOffset[3] = {0, 0, 0}, rotationPivot[3] = {0, 0, 0};
            double scalingOffset[3] = {0, 0, 0}, scalingPivot[3] = {0, 0, 0};
            double gt[3] = {0, 0, 0}, gr[3] = {0, 0, 0}, gs[3] = {1, 1, 1};
            int order = 0;
            bool rotationActive = false;
        };

        // Euler degrees in an FBX rotation order (0 XYZ ... 5 ZYX): the first axis turns first.
        Mat4 Euler(const double* d, int order)
        {
            Mat4 x = Mat4::Axis(0, d[0]), y = Mat4::Axis(1, d[1]), z = Mat4::Axis(2, d[2]);
            switch (order)
            {
            case 1: return y * z * x;
            case 2: return x * z * y;
            case 3: return z * x * y;
            case 4: return y * x * z;
            case 5: return x * y * z;
            default: return z * y * x;
            }
        }

        Mat4 Local(const Xform& x, const double* t, const double* r, const double* s)
        {
            auto T = [](const double* v) { return Mat4::Translation(v[0], v[1], v[2]); };
            auto N = [](const double* v) { return Mat4::Translation(-v[0], -v[1], -v[2]); };
            Mat4 pre = x.rotationActive ? Euler(x.pre, 0) : Mat4();
            Mat4 post = x.rotationActive ? detail::Inverse(Euler(x.post, 0)) : Mat4();
            return T(t) * T(x.rotationOffset) * T(x.rotationPivot) * pre * Euler(r, x.order) * post * N(x.rotationPivot) * T(x.scalingOffset) *
                   T(x.scalingPivot) * Mat4::Scale(s[0], s[1], s[2]) * N(x.scalingPivot);
        }

        class Reader
        {
        public:
            Reader(Scene& s, String& error) : s_(s), error_(error) {}

            bool Run(Span<const u8> data)
            {
                if (!d_.Read(data, error_))
                    return false;
                if (!d_.binary && d_.version != 0 && d_.version < 7000)
                    return Fail(Format("FBX version {}: files before 7.0 are not read", d_.version));
                if (d_.Find(0, "Objects") == fbx::kNoNode)
                    return Fail("no Objects");
                Settings();
                Objects();
                Connections();
                return Models() && Materials() && Meshes() && Attributes() && Animations();
            }

        private:
            bool Fail(StringView what)
            {
                error_ = String(what);
                return false;
            }

            // ---------------------------------------------------------- properties

            // The P entry of a Properties70 block of an object node (or P of Properties60), kNoNode when missing.
            u32 P(u32 object, StringView name) const
            {
                for (StringView block : {StringView("Properties70"), StringView("Properties60")})
                {
                    u32 b = d_.Find(object, block);
                    if (b == fbx::kNoNode)
                        continue;
                    for (u32 i = 0; i < d_.ChildCount(b); i++)
                    {
                        u32 p = d_.Child(b, i);
                        if (d_.PropertyCount(p) >= 1 && d_.Text(d_.Prop(p, 0)) == name)
                            return p;
                    }
                }
                return fbx::kNoNode;
            }

            // Values start at 4 in Properties70 (name, type, label, flags) and at 3 in Properties60.
            u32 First(u32 p) const
            {
                u32 n = d_.PropertyCount(p);
                for (u32 i = 1; i < n; i++)
                    if (d_.Prop(p, i).IsNumber() || d_.Prop(p, i).IsArray())
                        return i;
                return n;
            }

            double Number(u32 object, StringView name, double fallback) const
            {
                u32 p = P(object, name);
                if (p == fbx::kNoNode)
                    return fallback;
                u32 i = First(p);
                if (i >= d_.PropertyCount(p))
                    return fallback;
                const Property& v = d_.Prop(p, i);
                return v.type == PropertyType::Float || v.type == PropertyType::Double ? v.number : double(v.integer);
            }

            bool Vector(u32 object, StringView name, double* out) const
            {
                u32 p = P(object, name);
                if (p == fbx::kNoNode)
                    return false;
                u32 i = First(p);
                if (i + 3 > d_.PropertyCount(p))
                    return false;
                for (u32 k = 0; k < 3; k++)
                {
                    const Property& v = d_.Prop(p, i + k);
                    out[k] = v.type == PropertyType::Float || v.type == PropertyType::Double ? v.number : double(v.integer);
                }
                return true;
            }

            StringView Str(u32 object, StringView name) const
            {
                u32 p = P(object, name);
                if (p == fbx::kNoNode || d_.PropertyCount(p) < 5)
                    return StringView();
                return d_.Text(d_.Prop(p, 4));
            }

            // An array child ("Vertices: *n {...}" or binary arrays), numbers.
            bool Numbers(u32 node, StringView child, Array<double>& out) const
            {
                out.Clear();
                u32 c = d_.Find(node, child);
                if (c == fbx::kNoNode)
                    return false;
                // ASCII files of older exporters write arrays as plain value lists.
                if (d_.PropertyCount(c) == 1)
                    return d_.Numbers(d_.Prop(c, 0), out);
                for (u32 i = 0; i < d_.PropertyCount(c); i++)
                {
                    const Property& p = d_.Prop(c, i);
                    if (!p.IsNumber())
                        return false;
                    out.Push(p.type == PropertyType::Float || p.type == PropertyType::Double ? p.number : double(p.integer));
                }
                return true;
            }

            bool Integers(u32 node, StringView child, Array<i64>& out) const
            {
                out.Clear();
                u32 c = d_.Find(node, child);
                if (c == fbx::kNoNode)
                    return false;
                if (d_.PropertyCount(c) == 1)
                    return d_.Integers(d_.Prop(c, 0), out);
                for (u32 i = 0; i < d_.PropertyCount(c); i++)
                    out.Push(d_.Prop(c, i).integer);
                return true;
            }

            StringView ChildText(u32 node, StringView child) const
            {
                const Property* p = d_.Value(node, child);
                return p ? d_.Text(*p) : StringView();
            }

            // ---------------------------------------------------------- settings, objects, connections

            void Settings()
            {
                u32 g = d_.Find(0, "GlobalSettings");
                AxisSystem& a = s_.axes;
                auto axis = [&](StringView name, u8& out, i8& sign, StringView signName, u8 fallback) {
                    double v = Number(g, name, fallback);
                    out = v >= 0 && v <= 2 ? u8(v) : fallback;
                    sign = Number(g, signName, 1) < 0 ? i8(-1) : i8(1);
                };
                axis("UpAxis", a.up, a.upSign, "UpAxisSign", 1);
                axis("FrontAxis", a.front, a.frontSign, "FrontAxisSign", 2);
                axis("CoordAxis", a.coord, a.coordSign, "CoordAxisSign", 0);
                if (a.up == a.front || a.up == a.coord || a.front == a.coord)
                {
                    s_.warnings.Push("GlobalSettings: axes that repeat; y up, z front and x right assumed");
                    a = AxisSystem();
                }
                // Centimeters per unit.
                double unit = Number(g, "UnitScaleFactor", 1);
                s_.unitScale = float(unit > 0 && std::isfinite(unit) ? unit / 100 : 0.01);
                static constexpr float kRates[] = {30, 120, 100, 60, 50, 48, 30, 30, 29.97f, 29.97f, 25, 24, 1000, 23.976f, 0, 96, 72, 59.94f};
                int mode = int(Number(g, "TimeMode", 0));
                float rate = mode >= 0 && mode < 18 ? kRates[mode] : 30;
                if (mode == 14)
                {
                    double custom = Number(g, "CustomFrameRate", 30);
                    rate = custom > 0 && custom < 10000 ? float(custom) : 30;
                }
                frameRate_ = rate;
            }

            void Objects()
            {
                u32 objects = d_.Find(0, "Objects");
                for (u32 i = 0; i < d_.ChildCount(objects); i++)
                {
                    u32 n = d_.Child(objects, i);
                    if (d_.PropertyCount(n) < 1 || !d_.Prop(n, 0).IsNumber())
                        continue;
                    Object o;
                    o.id = d_.Prop(n, 0).integer;
                    o.node = n;
                    o.kind = d_.Name(n);
                    o.name = d_.PropertyCount(n) > 1 ? fbx::ObjectName(d_.Text(d_.Prop(n, 1))) : StringView();
                    o.subclass = d_.PropertyCount(n) > 2 ? d_.Text(d_.Prop(n, 2)) : StringView();
                    if (byId_.Find(o.id))
                        continue;
                    byId_.Set(o.id, u32(objects_.Size()));
                    objects_.Push(o);
                }
            }

            void Connections()
            {
                u32 c = d_.Find(0, "Connections");
                for (u32 i = 0; i < d_.ChildCount(c); i++)
                {
                    u32 n = d_.Child(c, i);
                    if (d_.PropertyCount(n) < 3 || !d_.Prop(n, 1).IsNumber() || !d_.Prop(n, 2).IsNumber())
                        continue;
                    Link l;
                    l.child = d_.Prop(n, 1).integer;
                    l.parent = d_.Prop(n, 2).integer;
                    if (d_.Text(d_.Prop(n, 0)) == StringView("OP") && d_.PropertyCount(n) >= 4)
                        l.property = d_.Text(d_.Prop(n, 3));
                    l.order = u32(links_.Size());
                    links_.Push(l);
                }
                byParent_ = links_;
                std::sort(byParent_.begin(), byParent_.end(), [](const Link& a, const Link& b) { return a.parent != b.parent ? a.parent < b.parent : a.order < b.order; });
                byChild_ = links_;
                std::sort(byChild_.begin(), byChild_.end(), [](const Link& a, const Link& b) { return a.child != b.child ? a.child < b.child : a.order < b.order; });
            }

            const Object* Get(i64 id) const
            {
                const u32* i = byId_.Find(id);
                return i ? &objects_[*i] : nullptr;
            }

            // Objects connected under `parent` (sources), of a kind (and subclass when given), in file order.
            void Sources(i64 parent, StringView kind, Array<const Object*>& out, StringView subclass = {}, Array<StringView>* properties = nullptr) const
            {
                out.Clear();
                if (properties)
                    properties->Clear();
                const Link* it = std::lower_bound(byParent_.begin(), byParent_.end(), parent, [](const Link& l, i64 p) { return l.parent < p; });
                for (; it != byParent_.end() && it->parent == parent; ++it)
                {
                    const Object* o = Get(it->child);
                    if (o && o->kind == kind && (subclass.Empty() || o->subclass == subclass))
                    {
                        out.Push(o);
                        if (properties)
                            properties->Push(it->property);
                    }
                }
            }

            // Objects `child` is connected to, of a kind.
            void Targets(i64 child, StringView kind, Array<const Object*>& out, Array<StringView>* properties = nullptr) const
            {
                out.Clear();
                if (properties)
                    properties->Clear();
                const Link* it = std::lower_bound(byChild_.begin(), byChild_.end(), child, [](const Link& l, i64 c) { return l.child < c; });
                for (; it != byChild_.end() && it->child == child; ++it)
                {
                    const Object* o = Get(it->parent);
                    if (o && (kind.Empty() || o->kind == kind))
                    {
                        out.Push(o);
                        if (properties)
                            properties->Push(it->property);
                    }
                }
            }

            // ---------------------------------------------------------- models

            bool Models()
            {
                for (const Object& o : objects_)
                    if (o.kind == StringView("Model"))
                    {
                        modelNode_.Set(o.id, u32(s_.nodes.Size()));
                        Node& n = s_.nodes.Emplace();
                        n.name = o.name;
                        Xform x;
                        Vector(o.node, "Lcl Translation", x.t);
                        Vector(o.node, "Lcl Rotation", x.r);
                        Vector(o.node, "Lcl Scaling", x.s);
                        Vector(o.node, "PreRotation", x.pre);
                        Vector(o.node, "PostRotation", x.post);
                        Vector(o.node, "RotationOffset", x.rotationOffset);
                        Vector(o.node, "RotationPivot", x.rotationPivot);
                        Vector(o.node, "ScalingOffset", x.scalingOffset);
                        Vector(o.node, "ScalingPivot", x.scalingPivot);
                        Vector(o.node, "GeometricTranslation", x.gt);
                        Vector(o.node, "GeometricRotation", x.gr);
                        Vector(o.node, "GeometricScaling", x.gs);
                        double order = Number(o.node, "RotationOrder", 0);
                        x.order = order >= 0 && order <= 5 ? int(order) : 0;
                        x.rotationActive = Number(o.node, "RotationActive", 0) != 0;
                        // Files that set pre and post rotations without RotationActive still mean them (most exporters).
                        if (!x.rotationActive && P(o.node, "RotationActive") == fbx::kNoNode)
                            x.rotationActive = true;
                        xforms_.Push(x);
                        SetTransform(u32(s_.nodes.Size() - 1), Local(x, x.t, x.r, x.s));
                    }
                // Parents: the first model a model is connected to; others hang from the root.
                Array<const Object*> parents;
                Array<Array<u32>> kids;
                kids.Resize(s_.nodes.Size());
                for (const Object& o : objects_)
                {
                    const u32* self = modelNode_.Find(o.id);
                    if (!self)
                        continue;
                    Targets(o.id, "Model", parents);
                    u32 parent = kNone;
                    for (const Object* p : parents)
                        if (const u32* pn = modelNode_.Find(p->id); pn && *pn != *self)
                        {
                            parent = *pn;
                            break;
                        }
                    // A loop of parents is broken where it closes.
                    for (u32 at = parent, steps = 0; at != kNone && steps <= s_.nodes.Size(); at = s_.nodes[at].parent, steps++)
                        if (at == *self)
                        {
                            parent = kNone;
                            s_.warnings.Push(Format("model '{}': a cycle of parents, made a root", o.name));
                            break;
                        }
                    s_.nodes[*self].parent = parent;
                    if (parent == kNone)
                        s_.roots.Push(*self);
                    else
                        kids[parent].Push(*self);
                }
                for (u32 i = 0; i < s_.nodes.Size(); i++)
                {
                    s_.nodes[i].children = {u32(s_.nodeChildren.Size()), u32(kids[i].Size())};
                    s_.nodeChildren.Append(kids[i].Data(), kids[i].Size());
                }
                return true;
            }

            void SetTransform(u32 node, const Mat4& m)
            {
                double t[3], q[4], sc[3];
                detail::Decompose(m, t, q, sc);
                Node& n = s_.nodes[node];
                for (int i = 0; i < 3; i++)
                    n.translation[i] = float(t[i]), n.scale[i] = float(sc[i]);
                for (int i = 0; i < 4; i++)
                    n.rotation[i] = float(q[i]);
            }

            // ---------------------------------------------------------- materials and textures

            u32 Image(const Object& texture)
            {
                StringView file = ChildText(texture.node, "RelativeFilename");
                if (file.Empty())
                    file = ChildText(texture.node, "FileName");
                Array<const Object*> videos;
                Sources(texture.id, "Video", videos);
                for (u32 i = 0; i < s_.images.Size(); i++)
                    if (videos.Empty() && !file.Empty() && s_.images[i].uri == file)
                        return i;
                model::Image& im = s_.images.Emplace();
                im.name = texture.name;
                im.uri = file;
                if (!videos.Empty())
                {
                    const Property* content = d_.Value(videos[0]->node, "Content");
                    if (content && content->type == PropertyType::Raw && content->count > 0)
                    {
                        im.bytes = {u32(s_.bytes.Size()), content->count};
                        s_.bytes.Append(d_.bytes.Data() + content->offset, content->count);
                    }
                    else if (content && content->type == PropertyType::String && content->count > 0)
                    {
                        // ASCII files hold the content as base64.
                        Array<u8> decoded;
                        if (Base64(d_.Text(*content), decoded))
                        {
                            im.bytes = {u32(s_.bytes.Size()), u32(decoded.Size())};
                            s_.bytes.Append(decoded.Data(), decoded.Size());
                        }
                    }
                    if (im.uri.Empty())
                        im.uri = ChildText(videos[0]->node, "RelativeFilename");
                }
                return u32(s_.images.Size() - 1);
            }

            bool Materials()
            {
                Array<const Object*> textures;
                Array<StringView> slots;
                for (const Object& o : objects_)
                {
                    if (o.kind != StringView("Material"))
                        continue;
                    materialIndex_.Set(o.id, u32(s_.materials.Size()));
                    Material& m = s_.materials.Emplace();
                    m.name = o.name;
                    m.metallic = 0;
                    double c[3];
                    if (Vector(o.node, "DiffuseColor", c) || Vector(o.node, "Diffuse", c))
                        m.baseColor[0] = float(c[0]), m.baseColor[1] = float(c[1]), m.baseColor[2] = float(c[2]);
                    if (Vector(o.node, "EmissiveColor", c) || Vector(o.node, "Emissive", c))
                    {
                        double f = Number(o.node, "EmissiveFactor", 1);
                        m.emissive[0] = float(c[0] * f), m.emissive[1] = float(c[1] * f), m.emissive[2] = float(c[2] * f);
                    }
                    if (Vector(o.node, "SpecularColor", c) || Vector(o.node, "Specular", c))
                    {
                        double f = Number(o.node, "SpecularFactor", 1);
                        m.specularColorClassic[0] = float(c[0] * f), m.specularColorClassic[1] = float(c[1] * f), m.specularColorClassic[2] = float(c[2] * f);
                    }
                    double shininess = Number(o.node, "ShininessExponent", Number(o.node, "Shininess", -1));
                    if (shininess >= 0)
                    {
                        m.shininess = float(shininess);
                        m.roughness = float(std::sqrt(2.0 / (shininess + 2)));
                    }
                    double opacity = P(o.node, "Opacity") != fbx::kNoNode ? Number(o.node, "Opacity", 1) : 1 - Number(o.node, "TransparencyFactor", 0);
                    m.baseColor[3] = float(std::clamp(opacity, 0.0, 1.0));
                    if (m.baseColor[3] < 1)
                        m.alphaMode = AlphaMode::Blend;
                    Sources(o.id, "Texture", textures, {}, &slots);
                    for (usize i = 0; i < textures.Size(); i++)
                    {
                        TextureRef* slot = Slot(m, slots[i]);
                        if (!slot)
                        {
                            s_.warnings.Push(Format("material '{}': a texture on '{}' skipped", m.name, slots[i]));
                            continue;
                        }
                        const Object& t = *textures[i];
                        u32 texture;
                        if (const u32* known = textureIndex_.Find(t.id))
                            texture = *known;
                        else
                        {
                            texture = u32(s_.textures.Size());
                            Texture& tx = s_.textures.Emplace();
                            tx.name = t.name;
                            tx.image = Image(t);
                            textureIndex_.Set(t.id, texture);
                        }
                        slot->texture = texture;
                        double v[3];
                        if (Vector(t.node, "Translation", v))
                            slot->offset[0] = float(v[0]), slot->offset[1] = float(v[1]);
                        if (Vector(t.node, "Scaling", v))
                            slot->scale[0] = float(v[0]), slot->scale[1] = float(v[1]);
                        if (slot == &m.bumpTexture || slot == &m.normalTexture)
                            slot->strength = float(Number(o.node, "BumpFactor", 1));
                    }
                }
                return true;
            }

            static TextureRef* Slot(Material& m, StringView p)
            {
                if (p == StringView("DiffuseColor") || p == StringView("Diffuse"))
                    return &m.baseColorTexture;
                if (p == StringView("NormalMap"))
                    return &m.normalTexture;
                if (p == StringView("Bump"))
                    return &m.bumpTexture;
                if (p == StringView("SpecularColor") || p == StringView("SpecularFactor"))
                    return &m.specularColorClassicTexture;
                if (p == StringView("ShininessExponent") || p == StringView("Shininess"))
                    return &m.shininessTexture;
                if (p == StringView("EmissiveColor") || p == StringView("EmissiveFactor"))
                    return &m.emissiveTexture;
                if (p == StringView("TransparentColor") || p == StringView("TransparencyFactor"))
                    return &m.alphaTexture;
                if (p == StringView("DisplacementColor") || p == StringView("DisplacementFactor"))
                    return &m.displacementTexture;
                if (p == StringView("AmbientColor") || p == StringView("ReflectionColor") || p == StringView("ReflectionFactor"))
                    return &scratch_;
                return nullptr;
            }

            // ---------------------------------------------------------- geometry

            // One layer element: where its values are and how a corner finds its element.
            struct Layer
            {
                bool present = false;
                int mapping = 0; // 0 by polygon vertex, 1 by vertex, 2 by polygon, 3 by edge, 4 all same
                bool indexed = false;
                Array<double> values;
                Array<i64> indices;
                u32 components = 3;
                String name;
            };

            bool ReadLayer(u32 element, StringView valuesName, StringView indexName, u32 components, Layer& out, StringView what)
            {
                out.present = true;
                out.components = components;
                StringView mapping = ChildText(element, "MappingInformationType");
                StringView reference = ChildText(element, "ReferenceInformationType");
                if (mapping == StringView("ByPolygonVertex"))
                    out.mapping = 0;
                else if (mapping == StringView("ByVertex") || mapping == StringView("ByVertice") || mapping == StringView("ByControlPoint"))
                    out.mapping = 1;
                else if (mapping == StringView("ByPolygon"))
                    out.mapping = 2;
                else if (mapping == StringView("ByEdge"))
                    out.mapping = 3;
                else if (mapping == StringView("AllSame"))
                    out.mapping = 4;
                else
                    return Fail(Format("{}: mapping '{}'", what, mapping));
                out.indexed = reference == StringView("IndexToDirect") || reference == StringView("Index");
                Numbers(element, valuesName, out.values);
                if (out.indexed)
                    Integers(element, indexName, out.indices);
                out.name = ChildText(element, "Name");
                return true;
            }

            // The element of a corner, kNone when the layer has none for it.
            u32 Element(const Layer& l, u32 corner, u32 polygon, u32 point, u32 edge) const
            {
                u64 i;
                switch (l.mapping)
                {
                case 0: i = corner; break;
                case 1: i = point; break;
                case 2: i = polygon; break;
                case 3: i = edge; break;
                default: i = 0; break;
                }
                if (l.indexed)
                {
                    if (i >= l.indices.Size() || l.indices[usize(i)] < 0)
                        return kNone;
                    i = u64(l.indices[usize(i)]);
                }
                return i < l.values.Size() / l.components ? u32(i) : kNone;
            }

            // A geometry several models use with the same materials and no geometric transform becomes one mesh (and
            // skin) they share, as instances do.
            struct SharedMesh
            {
                i64 geometry;
                Array<u32> materials;
                u32 mesh, skin;
            };

            bool Meshes()
            {
                Array<const Object*> geometries, materials;
                Array<SharedMesh> shared;
                for (const Object& model : objects_)
                {
                    const u32* node = modelNode_.Find(model.id);
                    if (!node)
                        continue;
                    Sources(model.id, "Geometry", geometries, "Mesh");
                    if (geometries.Empty())
                        continue;
                    Sources(model.id, "Material", materials);
                    Array<u32> list;
                    for (const Object* m : materials)
                        list.Push(*materialIndex_.Find(m->id));
                    const Xform& x = xforms_[*node];
                    bool plain = x.gt[0] == 0 && x.gt[1] == 0 && x.gt[2] == 0 && x.gr[0] == 0 && x.gr[1] == 0 && x.gr[2] == 0 && x.gs[0] == 1 && x.gs[1] == 1 &&
                                 x.gs[2] == 1;
                    const SharedMesh* known = nullptr;
                    for (const SharedMesh& m : shared)
                        if (plain && m.geometry == geometries[0]->id && m.materials.Size() == list.Size() &&
                            (list.Empty() || std::memcmp(m.materials.Data(), list.Data(), list.Size() * sizeof(u32)) == 0))
                            known = &m;
                    if (known)
                    {
                        s_.nodes[*node].mesh = known->mesh;
                        s_.nodes[*node].skin = known->skin;
                        continue;
                    }
                    Mat4 geometric = Mat4::Translation(x.gt[0], x.gt[1], x.gt[2]) * Euler(x.gr, 0) * Mat4::Scale(x.gs[0], x.gs[1], x.gs[2]);
                    u32 mesh;
                    if (!Mesh(*geometries[0], model, list, geometric, mesh))
                        return false;
                    s_.nodes[*node].mesh = mesh;
                    if (plain)
                        shared.Push({geometries[0]->id, list, mesh, s_.nodes[*node].skin});
                }
                return true;
            }

            struct Influence
            {
                u32 joint;
                float weight;
            };

            bool Mesh(const Object& g, const Object& model, const Array<u32>& materials, const Mat4& geometric, u32& meshIndex)
            {
                String what = Format("geometry '{}'", g.name);
                Array<double> points;
                Array<i64> pvi, edges;
                if (!Numbers(g.node, "Vertices", points) || points.Size() % 3 != 0)
                    return Fail(Format("{}: no Vertices", what));
                Integers(g.node, "PolygonVertexIndex", pvi);
                Integers(g.node, "Edges", edges);
                u32 pointCount = u32(points.Size() / 3);
                // Corners: control point, polygon, edge.
                u32 corners = u32(pvi.Size());
                Array<u32> cornerPoint, cornerPolygon, polygonStart;
                cornerPoint.Resize(corners);
                cornerPolygon.Resize(corners);
                u32 polygon = 0;
                polygonStart.Push(0);
                for (u32 c = 0; c < corners; c++)
                {
                    i64 v = pvi[c];
                    bool last = v < 0;
                    if (last)
                        v = ~v;
                    if (v >= pointCount)
                        return Fail(Format("{}: control point {} of {}", what, v, pointCount));
                    cornerPoint[c] = u32(v);
                    cornerPolygon[c] = polygon;
                    if (last)
                    {
                        polygon++;
                        polygonStart.Push(c + 1);
                    }
                }
                if (polygonStart.Back() != corners)
                    polygonStart.Push(corners), polygon++;
                Array<u32> cornerEdge;
                cornerEdge.Resize(corners, kNone);
                for (u32 e = 0; e < edges.Size(); e++)
                    if (edges[e] >= 0 && u64(edges[e]) < corners)
                        cornerEdge[usize(edges[e])] = e;

                // Layers: the first of each kind, every UV set.
                Layer normals, tangents, binormals, colors, materialLayer;
                Array<Layer> uvs;
                for (u32 i = 0; i < d_.ChildCount(g.node); i++)
                {
                    u32 e = d_.Child(g.node, i);
                    StringView name = d_.Name(e);
                    bool ok = true;
                    if (name == StringView("LayerElementNormal") && !normals.present)
                        ok = ReadLayer(e, "Normals", "NormalsIndex", 3, normals, what);
                    else if (name == StringView("LayerElementTangent") && !tangents.present)
                        ok = ReadLayer(e, "Tangents", "TangentsIndex", 3, tangents, what);
                    else if (name == StringView("LayerElementBinormal") && !binormals.present)
                        ok = ReadLayer(e, "Binormals", "BinormalsIndex", 3, binormals, what);
                    else if (name == StringView("LayerElementColor") && !colors.present)
                        ok = ReadLayer(e, "Colors", "ColorIndex", 4, colors, what);
                    else if (name == StringView("LayerElementUV") && uvs.Size() < 8)
                        ok = ReadLayer(e, "UV", "UVIndex", 2, uvs.Emplace(), what);
                    else if (name == StringView("LayerElementMaterial") && !materialLayer.present)
                    {
                        materialLayer.present = true;
                        materialLayer.components = 1;
                        StringView mapping = ChildText(e, "MappingInformationType");
                        materialLayer.mapping = mapping == StringView("AllSame") ? 4 : 2;
                        Integers(e, "Materials", materialLayer.indices);
                    }
                    if (!ok)
                        return false;
                }
                // NormalsIndex is also written as NormalIndex.
                if (normals.indexed && normals.indices.Empty())
                    for (u32 i = 0; i < d_.ChildCount(g.node); i++)
                        if (d_.Name(d_.Child(g.node, i)) == StringView("LayerElementNormal"))
                        {
                            Integers(d_.Child(g.node, i), "NormalIndex", normals.indices);
                            break;
                        }

                // Skin influences by control point.
                Array<Array<Influence>> influences;
                u32 skinIndex = kNone;
                Skin(g, pointCount, influences, skinIndex);
                // Blend shapes: per target, deltas by control point.
                struct Target
                {
                    String name;
                    u32 channel;
                    float fullWeight;
                    Array<float> positions, normals;
                    bool hasNormals;
                };
                Array<Target> targets;
                Array<float> channelWeights;
                Array<const Object*> shapes, channels, deformers;
                Sources(g.id, "Deformer", deformers, "BlendShape");
                for (const Object* bs : deformers)
                {
                    Sources(bs->id, "Deformer", channels, "BlendShapeChannel");
                    for (const Object* ch : channels)
                    {
                        u32 channel = u32(channelWeights.Size());
                        double percent = Number(ch->node, "DeformPercent", 0);
                        if (const Property* v = d_.Value(ch->node, "DeformPercent"); v && v->IsNumber())
                            percent = v->type == PropertyType::Double || v->type == PropertyType::Float ? v->number : double(v->integer);
                        channelWeights.Push(float(percent / 100));
                        channelOf_.Set(ch->id, ChannelRef{u32(s_.meshes.Size()), channel, model.id});
                        Array<double> full;
                        Numbers(ch->node, "FullWeights", full);
                        Sources(ch->id, "Geometry", shapes, "Shape");
                        for (usize k = 0; k < shapes.Size(); k++)
                        {
                            Target& t = targets.Emplace();
                            t.name = shapes.Size() > 1 ? String(Format("{}_{}", ch->name, k)) : String(ch->name);
                            t.channel = channel;
                            t.fullWeight = k < full.Size() ? float(full[k]) : 100;
                            t.positions.Resize(usize(pointCount) * 3, 0.0f);
                            t.normals.Resize(usize(pointCount) * 3, 0.0f);
                            Array<i64> idx;
                            Array<double> dv, dn;
                            Integers(shapes[k]->node, "Indexes", idx);
                            Numbers(shapes[k]->node, "Vertices", dv);
                            t.hasNormals = Numbers(shapes[k]->node, "Normals", dn) && dn.Size() == idx.Size() * 3;
                            if (dv.Size() != idx.Size() * 3)
                                return Fail(Format("shape '{}': {} deltas for {} indexes", shapes[k]->name, dv.Size() / 3, idx.Size()));
                            for (usize i = 0; i < idx.Size(); i++)
                            {
                                if (idx[i] < 0 || idx[i] >= pointCount)
                                    return Fail(Format("shape '{}': control point {} of {}", shapes[k]->name, idx[i], pointCount));
                                for (int a = 0; a < 3; a++)
                                {
                                    t.positions[usize(idx[i]) * 3 + a] = float(dv[i * 3 + a]);
                                    if (t.hasNormals)
                                        t.normals[usize(idx[i]) * 3 + a] = float(dn[i * 3 + a]);
                                }
                            }
                        }
                    }
                }

                // Corners to vertices, shared where every element is the same, one primitive per material.
                u32 uvCount = u32(uvs.Size());
                u32 keySize = 5 + uvCount;
                struct Part
                {
                    u32 material;
                    Array<u32> keys;
                    Array<u32> indices;
                    HashMap<u64, u32> seen;
                };
                Array<Part> parts;
                Array<u32> key;
                key.Resize(keySize);
                u32 maxInfluences = 0;
                for (const Array<Influence>& in : influences)
                    maxInfluences = std::max(maxInfluences, u32(in.Size()));
                for (u32 p = 0; p + 1 < polygonStart.Size(); p++)
                {
                    u32 begin = polygonStart[p], end = polygonStart[p + 1];
                    if (end - begin < 3)
                        continue;
                    u32 material = kNone;
                    if (materialLayer.present && !materialLayer.indices.Empty())
                    {
                        i64 m = materialLayer.mapping == 4 ? materialLayer.indices[0] : p < materialLayer.indices.Size() ? materialLayer.indices[p] : 0;
                        material = m >= 0 && m < i64(materials.Size()) ? materials[usize(m)] : kNone;
                    }
                    else if (!materials.Empty())
                        material = materials[0];
                    Part* part = nullptr;
                    for (Part& q : parts)
                        if (q.material == material)
                            part = &q;
                    if (!part)
                    {
                        part = &parts.Emplace();
                        part->material = material;
                    }
                    auto vertex = [&](u32 c) {
                        u32 point = cornerPoint[c], edge = cornerEdge[c];
                        key[0] = point;
                        key[1] = normals.present ? Element(normals, c, p, point, edge) : kNone;
                        key[2] = tangents.present ? Element(tangents, c, p, point, edge) : kNone;
                        key[3] = binormals.present ? Element(binormals, c, p, point, edge) : kNone;
                        key[4] = colors.present ? Element(colors, c, p, point, edge) : kNone;
                        for (u32 u = 0; u < uvCount; u++)
                            key[5 + u] = Element(uvs[u], c, p, point, edge);
                        u64 h = 1469598103934665603ull;
                        for (u32 k : key)
                            h = (h ^ k) * 1099511628211ull;
                        // Open addressing over the hash: equal hashes compare the keys.
                        for (u64 probe = h;; probe = probe * 31 + 7)
                        {
                            const u32* known = part->seen.Find(probe);
                            if (!known)
                            {
                                u32 index = u32(part->keys.Size() / keySize);
                                part->seen.Set(probe, index);
                                part->keys.Append(key.Data(), keySize);
                                return index;
                            }
                            if (std::memcmp(part->keys.Data() + usize(*known) * keySize, key.Data(), keySize * 4) == 0)
                                return *known;
                        }
                    };
                    u32 first = vertex(begin);
                    u32 previous = vertex(begin + 1);
                    for (u32 c = begin + 2; c < end; c++)
                    {
                        u32 v = vertex(c);
                        part->indices.Push(first);
                        part->indices.Push(previous);
                        part->indices.Push(v);
                        previous = v;
                    }
                }

                // The normal matrix of the geometric transform.
                Mat4 normalMatrix = detail::Inverse(geometric);
                model::Mesh mesh;
                mesh.name = g.name.Empty() ? model.name : g.name;
                mesh.primitives.begin = u32(s_.primitives.Size());
                for (Part& part : parts)
                {
                    u32 vertices = u32(part.keys.Size() / keySize);
                    Primitive prim;
                    prim.material = part.material;
                    prim.vertexCount = vertices;
                    prim.streams.begin = u32(s_.streams.Size());
                    auto begin = [&](Semantic semantic, u32 set, u32 components) {
                        VertexStream st;
                        st.semantic = semantic;
                        st.set = set;
                        st.components = components;
                        st.offset = u32(s_.floats.Size());
                        s_.streams.Push(st);
                    };
                    const u32* keys = part.keys.Data();
                    begin(Semantic::Position, 0, 3);
                    for (u32 v = 0; v < vertices; v++)
                    {
                        const double* q = points.Data() + usize(keys[v * keySize]) * 3;
                        for (int r = 0; r < 3; r++)
                            s_.floats.Push(float(geometric(r, 0) * q[0] + geometric(r, 1) * q[1] + geometric(r, 2) * q[2] + geometric(r, 3)));
                    }
                    auto direction = [&](const Layer& l, u32 slot, bool normal) {
                        for (u32 v = 0; v < vertices; v++)
                        {
                            u32 e = keys[v * keySize + slot];
                            double n[3] = {0, 0, 0};
                            if (e != kNone)
                                for (int a = 0; a < 3; a++)
                                    n[a] = l.values[usize(e) * l.components + a];
                            double o[3];
                            for (int r = 0; r < 3; r++)
                                o[r] = normal ? normalMatrix(0, r) * n[0] + normalMatrix(1, r) * n[1] + normalMatrix(2, r) * n[2]
                                              : geometric(r, 0) * n[0] + geometric(r, 1) * n[1] + geometric(r, 2) * n[2];
                            double len = std::sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);
                            for (int r = 0; r < 3; r++)
                                s_.floats.Push(float(len > 0 ? o[r] / len : 0));
                            if (!normal)
                            {
                                // The handedness from the binormal: w = sign(dot(cross(n, t), b)).
                                float w = 1;
                                if (normals.present && binormals.present)
                                {
                                    u32 ne = keys[v * keySize + 1], be = keys[v * keySize + 3];
                                    if (ne != kNone && be != kNone)
                                    {
                                        const double* nn = normals.values.Data() + usize(ne) * normals.components;
                                        const double* bb = binormals.values.Data() + usize(be) * binormals.components;
                                        double cx = nn[1] * n[2] - nn[2] * n[1], cy = nn[2] * n[0] - nn[0] * n[2], cz = nn[0] * n[1] - nn[1] * n[0];
                                        w = cx * bb[0] + cy * bb[1] + cz * bb[2] < 0 ? -1.0f : 1.0f;
                                    }
                                }
                                s_.floats.Push(w);
                            }
                        }
                    };
                    if (normals.present)
                    {
                        begin(Semantic::Normal, 0, 3);
                        direction(normals, 1, true);
                    }
                    if (tangents.present)
                    {
                        begin(Semantic::Tangent, 0, 4);
                        direction(tangents, 2, false);
                    }
                    for (u32 u = 0; u < uvCount; u++)
                    {
                        begin(Semantic::TexCoord, u, 2);
                        s_.streams.Back().name = uvs[u].name;
                        for (u32 v = 0; v < vertices; v++)
                        {
                            u32 e = keys[v * keySize + 5 + u];
                            s_.floats.Push(e != kNone ? float(uvs[u].values[usize(e) * 2]) : 0.0f);
                            s_.floats.Push(e != kNone ? float(uvs[u].values[usize(e) * 2 + 1]) : 0.0f);
                        }
                    }
                    if (colors.present)
                    {
                        begin(Semantic::Color, 0, 4);
                        for (u32 v = 0; v < vertices; v++)
                        {
                            u32 e = keys[v * keySize + 4];
                            for (int a = 0; a < 4; a++)
                                s_.floats.Push(e != kNone ? float(colors.values[usize(e) * 4 + a]) : 1.0f);
                        }
                    }
                    // Joints and weights, four influences a set.
                    for (u32 set = 0; set * 4 < maxInfluences; set++)
                    {
                        begin(Semantic::Joints, set, 4);
                        for (u32 v = 0; v < vertices; v++)
                        {
                            const Array<Influence>& in = influences[keys[v * keySize]];
                            for (u32 k = set * 4; k < set * 4 + 4; k++)
                                s_.floats.Push(k < in.Size() ? float(in[k].joint) : 0.0f);
                        }
                        begin(Semantic::Weights, set, 4);
                        for (u32 v = 0; v < vertices; v++)
                        {
                            const Array<Influence>& in = influences[keys[v * keySize]];
                            for (u32 k = set * 4; k < set * 4 + 4; k++)
                                s_.floats.Push(k < in.Size() ? in[k].weight : 0.0f);
                        }
                    }
                    prim.streams.count = u32(s_.streams.Size()) - prim.streams.begin;
                    prim.indices = {u32(s_.indices.Size()), u32(part.indices.Size())};
                    s_.indices.Append(part.indices.Data(), part.indices.Size());
                    prim.targets.begin = u32(s_.morphTargets.Size());
                    for (const Target& t : targets)
                    {
                        MorphTarget mt;
                        mt.name = t.name;
                        mt.channel = t.channel;
                        mt.fullWeight = t.fullWeight;
                        mt.streams.begin = u32(s_.streams.Size());
                        begin(Semantic::Position, 0, 3);
                        for (u32 v = 0; v < vertices; v++)
                        {
                            const float* dp = t.positions.Data() + usize(keys[v * keySize]) * 3;
                            for (int r = 0; r < 3; r++)
                                s_.floats.Push(float(geometric(r, 0) * dp[0] + geometric(r, 1) * dp[1] + geometric(r, 2) * dp[2]));
                        }
                        if (t.hasNormals)
                        {
                            begin(Semantic::Normal, 0, 3);
                            for (u32 v = 0; v < vertices; v++)
                            {
                                const float* dn = t.normals.Data() + usize(keys[v * keySize]) * 3;
                                for (int r = 0; r < 3; r++)
                                    s_.floats.Push(float(normalMatrix(0, r) * dn[0] + normalMatrix(1, r) * dn[1] + normalMatrix(2, r) * dn[2]));
                            }
                        }
                        mt.streams.count = u32(s_.streams.Size()) - mt.streams.begin;
                        s_.morphTargets.Push(mt);
                    }
                    prim.targets.count = u32(s_.morphTargets.Size()) - prim.targets.begin;
                    s_.primitives.Push(prim);
                }
                mesh.primitives.count = u32(s_.primitives.Size()) - mesh.primitives.begin;
                mesh.weights = {u32(s_.floats.Size()), u32(channelWeights.Size())};
                s_.floats.Append(channelWeights.Data(), channelWeights.Size());
                meshIndex = u32(s_.meshes.Size());
                s_.meshes.Push(mesh);
                if (skinIndex != kNone)
                    s_.nodes[*modelNode_.Find(model.id)].skin = skinIndex;
                return true;
            }

            void Skin(const Object& g, u32 points, Array<Array<Influence>>& influences, u32& skinIndex)
            {
                influences.Clear();
                influences.Resize(points);
                Array<const Object*> skins, clusters, bones;
                Sources(g.id, "Deformer", skins, "Skin");
                if (skins.Empty())
                    return;
                Sources(skins[0]->id, "Deformer", clusters, "Cluster");
                model::Skin skin;
                skin.name = skins[0]->name;
                Array<u32> joints;
                Array<float> matrices;
                for (const Object* c : clusters)
                {
                    Sources(c->id, "Model", bones);
                    if (bones.Empty())
                        continue;
                    const u32* bone = modelNode_.Find(bones[0]->id);
                    u32 joint = u32(joints.Size());
                    joints.Push(*bone);
                    Array<double> transform, link;
                    Numbers(c->node, "Transform", transform);
                    Numbers(c->node, "TransformLink", link);
                    Mat4 t, l;
                    if (transform.Size() == 16)
                        for (int i = 0; i < 16; i++)
                            t.m[i] = transform[i];
                    if (link.Size() == 16)
                        for (int i = 0; i < 16; i++)
                            l.m[i] = link[i];
                    Mat4 inverseBind = detail::Inverse(l) * t;
                    for (double v : inverseBind.m)
                        matrices.Push(float(v));
                    Array<i64> idx;
                    Array<double> weights;
                    Integers(c->node, "Indexes", idx);
                    Numbers(c->node, "Weights", weights);
                    for (usize i = 0; i < idx.Size() && i < weights.Size(); i++)
                        if (idx[i] >= 0 && idx[i] < points && weights[i] > 0)
                            influences[usize(idx[i])].Push({joint, float(weights[i])});
                }
                if (joints.Empty())
                    return;
                for (Array<Influence>& in : influences)
                {
                    std::sort(in.begin(), in.end(), [](const Influence& a, const Influence& b) { return a.weight > b.weight; });
                    float sum = 0;
                    for (const Influence& i : in)
                        sum += i.weight;
                    if (sum > 0)
                        for (Influence& i : in)
                            i.weight /= sum;
                }
                skin.joints = {u32(s_.skinJoints.Size()), u32(joints.Size())};
                s_.skinJoints.Append(joints.Data(), joints.Size());
                skin.inverseBindMatrices = {u32(s_.floats.Size()), u32(matrices.Size())};
                s_.floats.Append(matrices.Data(), matrices.Size());
                skinIndex = u32(s_.skins.Size());
                s_.skins.Push(skin);
            }

            // ---------------------------------------------------------- cameras and lights

            bool Attributes()
            {
                Array<const Object*> attributes;
                for (const Object& model : objects_)
                {
                    const u32* node = modelNode_.Find(model.id);
                    if (!node)
                        continue;
                    Sources(model.id, "NodeAttribute", attributes);
                    for (const Object* a : attributes)
                    {
                        if (a->subclass == StringView("Camera"))
                        {
                            Camera c;
                            c.name = model.name;
                            double fov = Number(a->node, "FieldOfViewY", Number(a->node, "FieldOfView", 45));
                            c.yfov = float(fov * 3.14159265358979 / 180);
                            double w = Number(a->node, "AspectWidth", 0), h = Number(a->node, "AspectHeight", 0);
                            c.aspectRatio = h > 0 ? float(w / h) : 0;
                            c.znear = float(Number(a->node, "NearPlane", 10));
                            c.zfar = float(Number(a->node, "FarPlane", 4000));
                            c.orthographic = Number(a->node, "CameraProjectionType", 0) == 1;
                            c.ymag = c.xmag = float(Number(a->node, "OrthoZoom", 1));
                            s_.nodes[*node].camera = u32(s_.cameras.Size());
                            s_.cameras.Push(c);
                        }
                        else if (a->subclass == StringView("Light"))
                        {
                            Light l;
                            l.name = model.name;
                            int type = int(Number(a->node, "LightType", 0));
                            l.type = type == 1 ? LightType::Directional : type == 2 ? LightType::Spot : LightType::Point;
                            double c[3];
                            if (Vector(a->node, "Color", c))
                                l.color[0] = float(c[0]), l.color[1] = float(c[1]), l.color[2] = float(c[2]);
                            l.intensity = float(Number(a->node, "Intensity", 100) / 100);
                            // FBX cone angles are whole cone angles in degrees; the scene's are half angles in radians.
                            l.outerConeAngle = float(Number(a->node, "OuterAngle", 45) * 3.14159265358979 / 360);
                            l.innerConeAngle = float(Number(a->node, "InnerAngle", 0) * 3.14159265358979 / 360);
                            if (Number(a->node, "EnableFarAttenuation", 0) != 0)
                                l.range = float(Number(a->node, "FarAttenuationEnd", 0));
                            s_.nodes[*node].light = u32(s_.lights.Size());
                            s_.lights.Push(l);
                        }
                    }
                }
                return true;
            }

            // ---------------------------------------------------------- animation

            struct Curve
            {
                Array<i64> times;
                Array<float> values;
                Array<i32> flags;
                Array<float> data;
                Array<i32> refs;
                // Per key: interpolation and the attribute index.
                Array<u32> attribute;
            };

            bool ReadCurve(const Object& c, Curve& out)
            {
                Integers(c.node, "KeyTime", out.times);
                Array<double> v;
                Numbers(c.node, "KeyValueFloat", v);
                if (v.Size() != out.times.Size())
                    return Fail(Format("curve {}: {} values for {} keys", c.id, v.Size(), out.times.Size()));
                for (double x : v)
                    out.values.Push(float(x));
                Array<i64> f, r;
                Array<double> data;
                Integers(c.node, "KeyAttrFlags", f);
                Numbers(c.node, "KeyAttrDataFloat", data);
                Integers(c.node, "KeyAttrRefCount", r);
                for (i64 x : f)
                    out.flags.Push(i32(x));
                for (double x : data)
                    out.data.Push(float(x));
                // Attributes are run lengths over the keys.
                out.attribute.Resize(out.times.Size(), 0);
                usize key = 0;
                for (usize a = 0; a < r.Size() && key < out.times.Size(); a++)
                    for (i64 k = 0; k < r[a] && key < out.times.Size(); k++)
                        out.attribute[key++] = u32(a);
                for (usize k = 1; k < out.times.Size(); k++)
                    if (out.times[k] < out.times[k - 1])
                        return Fail(Format("curve {}: key times out of order", c.id));
                return true;
            }

            static float Evaluate(const Curve& c, double seconds)
            {
                usize n = c.times.Size();
                if (n == 0)
                    return 0;
                i64 t = i64(seconds * kTicksPerSecond);
                if (t <= c.times[0])
                    return c.values[0];
                if (t >= c.times[n - 1])
                    return c.values[n - 1];
                usize k = usize(std::upper_bound(c.times.begin(), c.times.end(), t) - c.times.begin()) - 1;
                u32 a = c.attribute[k];
                i32 flags = a < c.flags.Size() ? c.flags[a] : 4;
                double t0 = double(c.times[k]) / kTicksPerSecond, t1 = double(c.times[k + 1]) / kTicksPerSecond;
                double u = t1 > t0 ? (seconds - t0) / (t1 - t0) : 0;
                float v0 = c.values[k], v1 = c.values[k + 1];
                if (flags & 0x02)
                    return v0;
                if ((flags & 0x08) && usize(a) * 4 + 1 < c.data.Size())
                {
                    double dt = t1 - t0;
                    double m0 = c.data[usize(a) * 4] * dt, m1 = c.data[usize(a) * 4 + 1] * dt;
                    double u2 = u * u, u3 = u2 * u;
                    return float((2 * u3 - 3 * u2 + 1) * v0 + (u3 - 2 * u2 + u) * m0 + (-2 * u3 + 3 * u2) * v1 + (u3 - u2) * m1);
                }
                return float(v0 + (v1 - v0) * u);
            }

            bool Animations()
            {
                s_.frameRate = frameRate_;
                Array<const Object*> layers, nodes, curves, targets;
                Array<StringView> props, curveProps;
                for (const Object& stack : objects_)
                {
                    if (stack.kind != StringView("AnimationStack"))
                        continue;
                    Sources(stack.id, "AnimationLayer", layers);
                    if (layers.Empty())
                        continue;
                    if (layers.Size() > 1)
                        s_.warnings.Push(Format("animation '{}': only the first of {} layers is read", stack.name, layers.Size()));
                    // Per model: the curves of translation, rotation and scaling (x, y, z); per channel: DeformPercent.
                    struct Animated
                    {
                        // Curve indices into the pool, kNone where a component has none.
                        u32 c[3][3] = {{kNone, kNone, kNone}, {kNone, kNone, kNone}, {kNone, kNone, kNone}};
                        double defaults[3][3] = {};
                        bool any[3] = {};
                    };
                    HashMap<i64, u32> animatedOf;
                    Array<i64> animatedModels;
                    Array<Animated> animated;
                    Array<Curve> pool;
                    struct Weight
                    {
                        i64 channel;
                        u32 curve;
                        double fallback;
                    };
                    Array<Weight> weights;
                    i64 first = INT64_MAX, last = INT64_MIN;
                    Sources(layers[0]->id, "AnimationCurveNode", nodes);
                    for (const Object* cn : nodes)
                    {
                        Targets(cn->id, {}, targets, &props);
                        Sources(cn->id, "AnimationCurve", curves, {}, &curveProps);
                        for (usize t = 0; t < targets.Size(); t++)
                        {
                            if (props[t].Empty())
                                continue;
                            int which = props[t] == StringView("Lcl Translation") ? 0 : props[t] == StringView("Lcl Rotation") ? 1 : props[t] == StringView("Lcl Scaling") ? 2
                                        : props[t] == StringView("DeformPercent")                                                ? 3
                                                                                                                                  : -1;
                            if (which < 0)
                                continue;
                            for (usize k = 0; k < curves.Size(); k++)
                            {
                                Curve& c = pool.Emplace();
                                if (!ReadCurve(*curves[k], c))
                                    return false;
                                if (!c.times.Empty())
                                    first = std::min(first, c.times[0]), last = std::max(last, c.times.Back());
                            }
                            if (which == 3)
                            {
                                u32 c = curves.Empty() ? kNone : u32(pool.Size() - curves.Size());
                                weights.Push({targets[t]->id, c, Number(cn->node, "d|DeformPercent", 0)});
                                continue;
                            }
                            if (!modelNode_.Find(targets[t]->id))
                                continue;
                            u32 index;
                            if (const u32* known = animatedOf.Find(targets[t]->id))
                                index = *known;
                            else
                            {
                                index = u32(animated.Size());
                                animatedOf.Set(targets[t]->id, index);
                                animatedModels.Push(targets[t]->id);
                                animated.Emplace();
                            }
                            Animated& a = animated[index];
                            a.any[which] = true;
                            const Xform& x = xforms_[*modelNode_.Find(targets[t]->id)];
                            const double* base = which == 0 ? x.t : which == 1 ? x.r : x.s;
                            for (int axis = 0; axis < 3; axis++)
                            {
                                static constexpr StringView kDefaults[] = {"d|X", "d|Y", "d|Z"};
                                a.defaults[which][axis] = Number(cn->node, kDefaults[axis], base[axis]);
                            }
                            for (usize k = 0; k < curves.Size(); k++)
                            {
                                int axis = curveProps[k] == StringView("d|X") ? 0 : curveProps[k] == StringView("d|Y") ? 1 : curveProps[k] == StringView("d|Z") ? 2 : -1;
                                if (axis >= 0)
                                    a.c[which][axis] = u32(pool.Size() - curves.Size() + k);
                            }
                        }
                    }
                    if (first > last)
                        continue;
                    // The stack's span, when it gives one.
                    i64 start = first, stop = last;
                    u32 local = P(stack.node, "LocalStop");
                    if (local != fbx::kNoNode)
                    {
                        start = i64(Number(stack.node, "LocalStart", double(first)));
                        stop = i64(Number(stack.node, "LocalStop", double(last)));
                        if (stop <= start)
                            start = first, stop = last;
                    }
                    double begin = double(start) / kTicksPerSecond, duration = double(stop - start) / kTicksPerSecond;
                    u64 frames = u64(std::floor(duration * frameRate_ + 0.5)) + 1;
                    if (frames > kMaxFrames)
                        return Fail(Format("animation '{}': {} frames, more than {}", stack.name, frames, kMaxFrames));
                    AnimationClip clip;
                    clip.name = stack.name;
                    clip.duration = float(duration);
                    clip.channels.begin = u32(s_.channels.Size());
                    Range times = {u32(s_.floats.Size()), u32(frames)};
                    for (u64 f = 0; f < frames; f++)
                        s_.floats.Push(float(std::min(double(f) / frameRate_, duration)));
                    for (usize m = 0; m < animated.Size(); m++)
                    {
                        u32 node = *modelNode_.Find(animatedModels[m]);
                        const Xform& x = xforms_[node];
                        const Animated& a = animated[m];
                        AnimationChannel ch[3];
                        for (int p = 0; p < 3; p++)
                        {
                            ch[p].node = node;
                            ch[p].path = p == 0 ? AnimationPath::Translation : p == 1 ? AnimationPath::Rotation : AnimationPath::Scale;
                            ch[p].components = p == 1 ? 4 : 3;
                            ch[p].times = times;
                        }
                        Array<float> values[3];
                        double previous[4] = {0, 0, 0, 1};
                        for (u64 f = 0; f < frames; f++)
                        {
                            double at = begin + double(f) / frameRate_;
                            double v[3][3];
                            for (int p = 0; p < 3; p++)
                                for (int axis = 0; axis < 3; axis++)
                                {
                                    const double* base = p == 0 ? x.t : p == 1 ? x.r : x.s;
                                    v[p][axis] = a.c[p][axis] != kNone ? Evaluate(pool[a.c[p][axis]], at) : a.any[p] ? a.defaults[p][axis] : base[axis];
                                }
                            double t[3], q[4], sc[3];
                            detail::Decompose(Local(x, v[0], v[1], v[2]), t, q, sc);
                            // Keep the quaternions on one side, so interpolation takes the short way.
                            if (q[0] * previous[0] + q[1] * previous[1] + q[2] * previous[2] + q[3] * previous[3] < 0)
                                for (double& c : q)
                                    c = -c;
                            for (int i = 0; i < 4; i++)
                                previous[i] = q[i];
                            for (int i = 0; i < 3; i++)
                                values[0].Push(float(t[i])), values[2].Push(float(sc[i]));
                            for (int i = 0; i < 4; i++)
                                values[1].Push(float(q[i]));
                        }
                        for (int p = 0; p < 3; p++)
                        {
                            ch[p].values = {u32(s_.floats.Size()), u32(values[p].Size())};
                            s_.floats.Append(values[p].Data(), values[p].Size());
                            s_.channels.Push(ch[p]);
                        }
                    }
                    // Morph weights: one channel per mesh node, every blend shape channel of the mesh in it.
                    HashMap<i64, u32> done;
                    for (const Weight& w : weights)
                    {
                        const ChannelRef* ref = channelOf_.Find(w.channel);
                        if (!ref || done.Find(ref->model))
                            continue;
                        done.Set(ref->model, 1);
                        const model::Mesh& mesh = s_.meshes[ref->mesh];
                        u32 count = mesh.weights.count;
                        AnimationChannel ch;
                        ch.node = *modelNode_.Find(ref->model);
                        ch.path = AnimationPath::Weights;
                        ch.components = count;
                        ch.times = times;
                        ch.values = {u32(s_.floats.Size()), u32(frames * count)};
                        for (u64 f = 0; f < frames; f++)
                            for (u32 k = 0; k < count; k++)
                            {
                                float value = s_.floats[mesh.weights.begin + k];
                                for (const Weight& other : weights)
                                {
                                    const ChannelRef* r = channelOf_.Find(other.channel);
                                    if (r && r->model == ref->model && r->channel == k)
                                        value = float((other.curve != kNone ? Evaluate(pool[other.curve], begin + double(f) / frameRate_) : other.fallback) / 100);
                                }
                                s_.floats.Push(value);
                            }
                        s_.channels.Push(ch);
                    }
                    clip.channels.count = u32(s_.channels.Size()) - clip.channels.begin;
                    s_.clips.Push(clip);
                }
                return true;
            }

            struct ChannelRef
            {
                u32 mesh;
                u32 channel;
                i64 model;
            };

            Scene& s_;
            String& error_;
            Document d_;
            Array<Object> objects_;
            HashMap<i64, u32> byId_;
            Array<Link> links_, byParent_, byChild_;
            HashMap<i64, u32> modelNode_, materialIndex_, textureIndex_;
            HashMap<i64, ChannelRef> channelOf_;
            Array<Xform> xforms_;
            float frameRate_ = 30;
            static inline TextureRef scratch_;
        };
    }

    bool IsBinaryFbx(Span<const u8> data) { return data.Size() >= 21 && std::memcmp(data.Data(), "Kaydara FBX Binary  ", 20) == 0; }

    bool ReadFbx(Span<const u8> data, Scene& out, String& error, const ReadOptions&)
    {
        out.Clear();
        Reader r(out, error);
        if (r.Run(data))
            return true;
        out.Clear();
        return false;
    }
}
