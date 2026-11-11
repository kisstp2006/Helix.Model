#include <Helix/Model/Gltf/Gltf.h>

#include <Helix/Core/Strings/Format.h>
#include <Helix/Serialization/Json/Parser.h>

#include <cmath>
#include <cstring>
#include <type_traits>

namespace helix::model
{
    namespace
    {
        namespace json = serialization::json;

        constexpr StringView kSupported[] = {
            "KHR_materials_emissive_strength", "KHR_materials_ior",         "KHR_materials_transmission", "KHR_materials_volume",
            "KHR_materials_clearcoat",         "KHR_materials_sheen",       "KHR_materials_specular",     "KHR_materials_iridescence",
            "KHR_materials_anisotropy",        "KHR_materials_dispersion",  "KHR_materials_unlit",        "KHR_materials_variants",
            "KHR_texture_transform",           "KHR_mesh_quantization",     "KHR_lights_punctual",
        };

        // Floats any one file may decode into: a damaged count cannot make the reader allocate without end.
        constexpr u64 kMaxFloats = u64(1) << 28;

        bool Base64(StringView text, Array<u8>& out)
        {
            auto value = [](char c) -> int {
                if (c >= 'A' && c <= 'Z')
                    return c - 'A';
                if (c >= 'a' && c <= 'z')
                    return c - 'a' + 26;
                if (c >= '0' && c <= '9')
                    return c - '0' + 52;
                if (c == '+' || c == '-')
                    return 62;
                if (c == '/' || c == '_')
                    return 63;
                return -1;
            };
            out.Clear();
            out.Reserve(text.Size() / 4 * 3);
            u32 bits = 0;
            int have = 0;
            for (char c : text)
            {
                if (c == '=')
                    break;
                int v = value(c);
                if (v < 0)
                {
                    if (c == '\r' || c == '\n' || c == ' ')
                        continue;
                    return false;
                }
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

        // Percent escapes of a URI.
        String Unescape(StringView uri)
        {
            String s;
            for (usize i = 0; i < uri.Size(); i++)
            {
                auto hex = [](char c) { return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1; };
                if (uri[i] == '%' && i + 2 < uri.Size() && hex(uri[i + 1]) >= 0 && hex(uri[i + 2]) >= 0)
                {
                    s.Append(char(hex(uri[i + 1]) * 16 + hex(uri[i + 2])));
                    i += 2;
                }
                else
                    s.Append(uri[i]);
            }
            return s;
        }

        u32 ComponentSize(u32 type)
        {
            switch (type)
            {
            case 5120:
            case 5121: return 1;
            case 5122:
            case 5123: return 2;
            case 5125:
            case 5126: return 4;
            default: return 0;
            }
        }

        struct View
        {
            const u8* data = nullptr;
            usize size = 0;
            u32 stride = 0;
        };

        class Reader
        {
        public:
            Reader(Scene& scene, String& error, const ReadOptions& options) : s_(scene), error_(error), options_(options) {}

            bool Run(Span<const u8> data)
            {
                StringView text;
                Span<const u8> bin;
                if (IsGlb(data))
                {
                    if (!Glb(data, text, bin))
                        return false;
                }
                else
                    text = StringView(reinterpret_cast<const char*>(data.Data()), data.Size());
                String jsonError;
                if (!json::Parse(text, j_, jsonError))
                    return Fail(Format("not glTF JSON: {}", jsonError));
                root_ = j_.root;
                if (!j_.IsObject(root_))
                    return Fail("the document is not a JSON object");
                bin_ = bin;
                s_.handedness = Scene::Handedness::Right;
                s_.upAxis = 1;
                s_.unitScale = 1;
                return Asset() && Extensions() && Buffers() && BufferViews() && Images() && Samplers() && Textures() && Materials() && Meshes() &&
                       Cameras() && Lights() && Skins() && Nodes() && Scenes() && Animations();
            }

        private:
            bool Fail(StringView what)
            {
                error_ = String(what);
                return false;
            }

            // ---------------------------------------------------------- JSON helpers

            u32 Get(u32 object, StringView key) const { return j_.Find(object, key); }

            // The text of a string node, empty for anything else (a missing member included).
            StringView Text(u32 node) const { return j_.IsString(node) ? j_.Text(node) : StringView(); }

            u32 Array(u32 object, StringView key) const
            {
                u32 a = Get(object, key);
                return j_.IsArray(a) ? a : json::kNoNode;
            }
            u32 Count(u32 array) const { return j_.IsArray(array) ? j_.Count(array) : 0; }

            bool Number(u32 node, double& out) const { return j_.IsNumber(node) && j_.AsDouble(node, out); }

            float Float(u32 object, StringView key, float fallback) const
            {
                double v;
                return Number(Get(object, key), v) ? float(v) : fallback;
            }

            bool Bool(u32 object, StringView key, bool fallback) const
            {
                bool b;
                return j_.AsBool(Get(object, key), b) ? b : fallback;
            }

            String Str(u32 object, StringView key) const
            {
                u32 n = Get(object, key);
                return j_.IsString(n) ? String(Text(n)) : String();
            }

            // A non-negative integer, kNone when missing; false when present and not one.
            bool Index(u32 object, StringView key, u32& out, StringView path)
            {
                out = kNone;
                u32 n = Get(object, key);
                if (n == json::kNoNode)
                    return true;
                u64 v;
                if (!j_.AsUInt(n, v) || v >= kNone)
                    return Fail(Format("{}.{}: not an index", path, key));
                out = u32(v);
                return true;
            }

            // An index that must be present and below `limit`.
            bool Required(u32 object, StringView key, u32 limit, u32& out, StringView path)
            {
                if (!Index(object, key, out, path))
                    return false;
                if (out == kNone)
                    return Fail(Format("{}.{}: missing", path, key));
                if (out >= limit)
                    return Fail(Format("{}.{}: {} of {}", path, key, out, limit));
                return true;
            }

            bool Optional(u32 object, StringView key, u32 limit, u32& out, StringView path)
            {
                if (!Index(object, key, out, path))
                    return false;
                if (out != kNone && out >= limit)
                    return Fail(Format("{}.{}: {} of {}", path, key, out, limit));
                return true;
            }

            void Floats(u32 object, StringView key, float* out, u32 n) const
            {
                u32 a = Array(object, key);
                if (Count(a) != n)
                    return;
                for (u32 i = 0; i < n; i++)
                {
                    double v;
                    if (Number(j_.Item(a, i), v))
                        out[i] = float(v);
                }
            }

            u32 Ext(u32 object, StringView name) const { return Get(Get(object, "extensions"), name); }

            // ---------------------------------------------------------- container

            bool Glb(Span<const u8> d, StringView& text, Span<const u8>& bin)
            {
                auto u32At = [&](usize at) {
                    u32 v;
                    std::memcpy(&v, d.Data() + at, 4);
                    return v;
                };
                if (d.Size() < 20)
                    return Fail("a GLB shorter than its header");
                if (u32At(4) != 2)
                    return Fail(Format("GLB version {}", u32At(4)));
                u32 length = u32At(8);
                if (length > d.Size() || length < 20)
                    return Fail(Format("a GLB of {} bytes in {}", length, d.Size()));
                usize at = 12;
                bool json = false;
                while (at + 8 <= length)
                {
                    u32 size = u32At(at), type = u32At(at + 4);
                    if (size > length - at - 8)
                        return Fail(Format("GLB chunk at {} runs past the end", at));
                    const u8* body = d.Data() + at + 8;
                    if (type == 0x4E4F534A && !json)
                    {
                        text = StringView(reinterpret_cast<const char*>(body), size);
                        json = true;
                    }
                    else if (type == 0x004E4942 && bin.Empty())
                        bin = Span<const u8>(body, size);
                    at += 8 + ((size + 3) & ~usize(3));
                }
                if (!json)
                    return Fail("a GLB without its JSON chunk");
                return true;
            }

            bool Asset()
            {
                u32 asset = Get(root_, "asset");
                if (!j_.IsObject(asset))
                    return Fail("asset: missing");
                String version = Str(asset, "version");
                if (version.Empty() || version[0] != '2')
                    return Fail(Format("asset.version: '{}', this reader reads 2.x", version));
                String min = Str(asset, "minVersion");
                if (!min.Empty() && !(min == StringView("2.0")))
                    return Fail(Format("asset.minVersion: '{}'", min));
                return true;
            }

            bool Extensions()
            {
                u32 required = Array(root_, "extensionsRequired");
                for (u32 i = 0; i < Count(required); i++)
                {
                    StringView name = Text(j_.Item(required, i));
                    bool known = false;
                    for (StringView k : kSupported)
                        known = known || k == name;
                    if (!known)
                        return Fail(Format("extensionsRequired: {} is not supported", name));
                }
                return true;
            }

            bool Buffers()
            {
                u32 a = Array(root_, "buffers");
                buffers_.Resize(Count(a));
                for (u32 i = 0; i < Count(a); i++)
                {
                    u32 b = j_.Item(a, i);
                    String path = Format("buffers[{}]", i);
                    double length;
                    if (!Number(Get(b, "byteLength"), length) || length < 0)
                        return Fail(Format("{}.byteLength: missing", path));
                    String uri = Str(b, "uri");
                    Span<const u8> bytes;
                    if (uri.Empty())
                    {
                        if (i != 0 || bin_.Empty())
                            return Fail(Format("{}: no uri and no GLB binary chunk", path));
                        bytes = bin_;
                    }
                    else if (uri.View().StartsWith("data:"))
                    {
                        usize comma = uri.View().Find(',');
                        if (comma == StringView::npos || uri.View().Substr(0, comma).Find(";base64") == StringView::npos)
                            return Fail(Format("{}.uri: a data URI that is not base64", path));
                        if (!Base64(uri.View().Substr(comma + 1), owned_.Emplace()))
                            return Fail(Format("{}.uri: damaged base64", path));
                        bytes = owned_.Back();
                    }
                    else
                    {
                        if (!options_.loader || !options_.loader(options_.context, Unescape(uri), owned_.Emplace()))
                            return Fail(Format("{}.uri: '{}' could not be loaded", path, uri));
                        bytes = owned_.Back();
                    }
                    if (bytes.Size() < usize(length))
                        return Fail(Format("{}: {} bytes where byteLength says {}", path, bytes.Size(), length));
                    buffers_[i] = Span<const u8>(bytes.Data(), usize(length));
                }
                return true;
            }

            bool BufferViews()
            {
                u32 a = Array(root_, "bufferViews");
                views_.Resize(Count(a));
                for (u32 i = 0; i < Count(a); i++)
                {
                    u32 v = j_.Item(a, i);
                    String path = Format("bufferViews[{}]", i);
                    u32 buffer;
                    if (!Required(v, "buffer", u32(buffers_.Size()), buffer, path))
                        return false;
                    double offset = 0, length = -1, stride = 0;
                    Number(Get(v, "byteOffset"), offset);
                    Number(Get(v, "byteStride"), stride);
                    if (!Number(Get(v, "byteLength"), length) || length < 0 || offset < 0)
                        return Fail(Format("{}.byteLength: missing", path));
                    Span<const u8> b = buffers_[buffer];
                    if (offset + length > double(b.Size()))
                        return Fail(Format("{}: bytes {} to {} of a buffer of {}", path, offset, offset + length, b.Size()));
                    if (stride != 0 && (stride < 4 || stride > 252 || std::fmod(stride, 4) != 0))
                        return Fail(Format("{}.byteStride: {}", path, stride));
                    views_[i] = {b.Data() + usize(offset), usize(length), u32(stride)};
                }
                return true;
            }

            // ---------------------------------------------------------- accessors

            // An accessor as floats (`components` per element, normalized integers to their range, others by value), or as
            // exact unsigned integers for indices.
            template <class T>
            bool Accessor(u32 index, ::helix::Array<T>& out, u32& components, StringView path)
            {
                constexpr bool kFloat = std::is_same_v<T, float>;
                u32 a = Array(root_, "accessors");
                if (index >= Count(a))
                    return Fail(Format("{}: accessor {} of {}", path, index, Count(a)));
                u32 acc = j_.Item(a, index);
                String where = Format("accessors[{}]", index);
                u64 count;
                if (!j_.AsUInt(Get(acc, "count"), count))
                    return Fail(Format("{}.count: missing", where));
                u32 type = 0;
                {
                    u64 t;
                    if (!j_.AsUInt(Get(acc, "componentType"), t))
                        return Fail(Format("{}.componentType: missing", where));
                    type = u32(t);
                }
                u32 size = ComponentSize(type);
                if (size == 0)
                    return Fail(Format("{}.componentType: {}", where, type));
                if constexpr (!kFloat)
                    if (type != 5121 && type != 5123 && type != 5125)
                        return Fail(Format("{}.componentType: {} for indices", where, type));
                StringView kind = Text(Get(acc, "type"));
                u32 rows = 1, cols = 1;
                if (kind == StringView("SCALAR"))
                    rows = 1;
                else if (kind == StringView("VEC2"))
                    rows = 2;
                else if (kind == StringView("VEC3"))
                    rows = 3;
                else if (kind == StringView("VEC4"))
                    rows = 4;
                else if (kind == StringView("MAT2"))
                    rows = cols = 2;
                else if (kind == StringView("MAT3"))
                    rows = cols = 3;
                else if (kind == StringView("MAT4"))
                    rows = cols = 4;
                else
                    return Fail(Format("{}.type: '{}'", where, kind));
                components = rows * cols;
                if (count * components > kMaxFloats || floatsUsed_ + count * components > kMaxFloats)
                    return Fail(Format("{}: {} elements, more than a model holds", where, count));
                floatsUsed_ += count * components;
                bool normalized = Bool(acc, "normalized", false);
                // Matrix columns start at multiples of 4 bytes.
                u32 column = cols > 1 ? (rows * size + 3) & ~3u : rows * size;
                u32 element = column * cols;
                out.Clear();
                out.Resize(usize(count) * components, T(0));

                auto decode = [&](const u8* p, T* dst) {
                    if constexpr (!kFloat)
                    {
                        for (u32 r = 0; r < rows; r++)
                        {
                            const u8* q = p + r * size;
                            dst[r] = size == 1 ? T(*q) : size == 2 ? T(q[0] | q[1] << 8) : T(u32(q[0]) | u32(q[1]) << 8 | u32(q[2]) << 16 | u32(q[3]) << 24);
                        }
                    }
                    else
                    {
                    // Plain floats of vectors and scalars copy as they are.
                    if (type == 5126 && cols == 1)
                    {
                        std::memcpy(dst, p, usize(rows) * 4);
                        return;
                    }
                    for (u32 c = 0; c < cols; c++)
                        for (u32 r = 0; r < rows; r++)
                        {
                            const u8* q = p + c * column + r * size;
                            float v;
                            switch (type)
                            {
                            case 5120: v = normalized ? std::fmax(float(i8(*q)) / 127.0f, -1.0f) : float(i8(*q)); break;
                            case 5121: v = normalized ? float(*q) / 255.0f : float(*q); break;
                            case 5122:
                            {
                                i16 x;
                                std::memcpy(&x, q, 2);
                                v = normalized ? std::fmax(float(x) / 32767.0f, -1.0f) : float(x);
                                break;
                            }
                            case 5123:
                            {
                                u16 x;
                                std::memcpy(&x, q, 2);
                                v = normalized ? float(x) / 65535.0f : float(x);
                                break;
                            }
                            case 5125:
                            {
                                u32 x;
                                std::memcpy(&x, q, 4);
                                v = float(x);
                                break;
                            }
                            default: std::memcpy(&v, q, 4); break;
                            }
                            dst[c * rows + r] = T(v);
                        }
                    }
                };

                u32 viewIndex;
                if (!Optional(acc, "bufferView", u32(views_.Size()), viewIndex, where))
                    return false;
                if (viewIndex != kNone)
                {
                    const View& view = views_[viewIndex];
                    double offset = 0;
                    Number(Get(acc, "byteOffset"), offset);
                    u32 stride = view.stride ? view.stride : element;
                    if (count > 0 && (offset < 0 || double(offset) + double(stride) * double(count - 1) + element > double(view.size)))
                        return Fail(Format("{}: {} elements of {} bytes from {} do not fit bufferView {}", where, count, stride, offset, viewIndex));
                    const u8* base = view.data + usize(offset);
                    for (u64 i = 0; i < count; i++)
                        decode(base + i * stride, out.Data() + i * components);
                }

                u32 sparse = Get(acc, "sparse");
                if (sparse != json::kNoNode)
                {
                    u64 n;
                    if (!j_.AsUInt(Get(sparse, "count"), n) || n > count)
                        return Fail(Format("{}.sparse.count: missing or above count", where));
                    u32 idx = Get(sparse, "indices"), val = Get(sparse, "values");
                    u32 iv = kNone, vv = kNone;
                    u64 itype;
                    if (!Required(idx, "bufferView", u32(views_.Size()), iv, Format("{}.sparse.indices", where)) ||
                        !Required(val, "bufferView", u32(views_.Size()), vv, Format("{}.sparse.values", where)))
                        return false;
                    if (!j_.AsUInt(Get(idx, "componentType"), itype) || (itype != 5121 && itype != 5123 && itype != 5125))
                        return Fail(Format("{}.sparse.indices.componentType: not an unsigned integer type", where));
                    double io = 0, vo = 0;
                    Number(Get(idx, "byteOffset"), io);
                    Number(Get(val, "byteOffset"), vo);
                    u32 isize = ComponentSize(u32(itype));
                    if (io < 0 || vo < 0 || io + double(n) * isize > double(views_[iv].size) || vo + double(n) * element > double(views_[vv].size))
                        return Fail(Format("{}.sparse: indices or values outside their bufferViews", where));
                    for (u64 k = 0; k < n; k++)
                    {
                        const u8* ip = views_[iv].data + usize(io) + k * isize;
                        u32 target = isize == 1 ? *ip : isize == 2 ? u32(ip[0] | ip[1] << 8) : u32(ip[0] | ip[1] << 8 | ip[2] << 16 | u32(ip[3]) << 24);
                        if (target >= count)
                            return Fail(Format("{}.sparse: index {} of {}", where, target, count));
                        decode(views_[vv].data + usize(vo) + k * element, out.Data() + usize(target) * components);
                    }
                }
                return true;
            }

            bool Indices(u32 index, ::helix::Array<u32>& out, u32 vertexCount, StringView path)
            {
                u32 components;
                if (!Accessor(index, out, components, path))
                    return false;
                if (components != 1)
                    return Fail(Format("{}: indices of {} components", path, components));
                for (usize i = 0; i < out.Size(); i++)
                    if (out[i] >= vertexCount)
                        return Fail(Format("{}: index {} of {} vertices", path, out[i], vertexCount));
                return true;
            }

            // ---------------------------------------------------------- images, samplers, textures, materials

            bool Images()
            {
                u32 a = Array(root_, "images");
                for (u32 i = 0; i < Count(a); i++)
                {
                    u32 im = j_.Item(a, i);
                    String path = Format("images[{}]", i);
                    Image& image = s_.images.Emplace();
                    image.name = Str(im, "name");
                    image.mimeType = Str(im, "mimeType");
                    String uri = Str(im, "uri");
                    u32 view;
                    if (!Optional(im, "bufferView", u32(views_.Size()), view, path))
                        return false;
                    if (view != kNone)
                    {
                        image.bytes = {u32(s_.bytes.Size()), u32(views_[view].size)};
                        s_.bytes.Append(views_[view].data, views_[view].size);
                    }
                    else if (uri.View().StartsWith("data:"))
                    {
                        usize comma = uri.View().Find(',');
                        ::helix::Array<u8> bytes;
                        if (comma == StringView::npos || !Base64(uri.View().Substr(comma + 1), bytes))
                            return Fail(Format("{}.uri: a damaged data URI", path));
                        usize semi = uri.View().Find(';');
                        if (image.mimeType.Empty() && semi != StringView::npos && semi > 5)
                            image.mimeType = uri.View().Substr(5, semi - 5);
                        image.bytes = {u32(s_.bytes.Size()), u32(bytes.Size())};
                        s_.bytes.Append(bytes.Data(), bytes.Size());
                    }
                    else if (!uri.Empty())
                        image.uri = Unescape(uri);
                    else
                        return Fail(Format("{}: neither uri nor bufferView", path));
                }
                return true;
            }

            bool Samplers()
            {
                u32 a = Array(root_, "samplers");
                samplers_.Resize(Count(a));
                for (u32 i = 0; i < Count(a); i++)
                {
                    u32 sm = j_.Item(a, i);
                    auto filter = [&](StringView key) {
                        double v;
                        if (!Number(Get(sm, key), v))
                            return Filter::Unset;
                        switch (int(v))
                        {
                        case 9728: return Filter::Nearest;
                        case 9729: return Filter::Linear;
                        case 9984: return Filter::NearestMipmapNearest;
                        case 9985: return Filter::LinearMipmapNearest;
                        case 9986: return Filter::NearestMipmapLinear;
                        case 9987: return Filter::LinearMipmapLinear;
                        default: return Filter::Unset;
                        }
                    };
                    auto wrap = [&](StringView key) {
                        double v;
                        if (!Number(Get(sm, key), v))
                            return Wrap::Repeat;
                        return int(v) == 33071 ? Wrap::ClampToEdge : int(v) == 33648 ? Wrap::MirroredRepeat : Wrap::Repeat;
                    };
                    Texture& t = samplers_[i];
                    t.magFilter = filter("magFilter");
                    t.minFilter = filter("minFilter");
                    t.wrapS = wrap("wrapS");
                    t.wrapT = wrap("wrapT");
                }
                return true;
            }

            bool Textures()
            {
                u32 a = Array(root_, "textures");
                for (u32 i = 0; i < Count(a); i++)
                {
                    u32 tx = j_.Item(a, i);
                    String path = Format("textures[{}]", i);
                    u32 sampler, source;
                    if (!Optional(tx, "sampler", u32(samplers_.Size()), sampler, path) || !Optional(tx, "source", u32(s_.images.Size()), source, path))
                        return false;
                    Texture t = sampler != kNone ? samplers_[sampler] : Texture();
                    t.name = Str(tx, "name");
                    t.image = source;
                    s_.textures.Push(t);
                }
                return true;
            }

            bool TextureInfo(u32 object, StringView key, TextureRef& out, StringView path, StringView strengthKey = {})
            {
                u32 info = Get(object, key);
                if (info == json::kNoNode)
                    return true;
                String where = Format("{}.{}", path, key);
                if (!Required(info, "index", u32(s_.textures.Size()), out.texture, where))
                    return false;
                double v;
                if (Number(Get(info, "texCoord"), v) && v >= 0)
                    out.texCoord = u32(v);
                if (!strengthKey.Empty())
                    out.strength = Float(info, strengthKey, 1);
                u32 transform = Ext(info, "KHR_texture_transform");
                if (transform != json::kNoNode)
                {
                    Floats(transform, "offset", out.offset, 2);
                    Floats(transform, "scale", out.scale, 2);
                    out.rotation = Float(transform, "rotation", 0);
                    if (Number(Get(transform, "texCoord"), v) && v >= 0)
                        out.texCoord = u32(v);
                }
                return true;
            }

            bool Materials()
            {
                u32 a = Array(root_, "materials");
                for (u32 i = 0; i < Count(a); i++)
                {
                    u32 m = j_.Item(a, i);
                    String path = Format("materials[{}]", i);
                    Material& mat = s_.materials.Emplace();
                    mat.name = Str(m, "name");
                    u32 pbr = Get(m, "pbrMetallicRoughness");
                    if (pbr != json::kNoNode)
                    {
                        String p = Format("{}.pbrMetallicRoughness", path);
                        Floats(pbr, "baseColorFactor", mat.baseColor, 4);
                        mat.metallic = Float(pbr, "metallicFactor", 1);
                        mat.roughness = Float(pbr, "roughnessFactor", 1);
                        if (!TextureInfo(pbr, "baseColorTexture", mat.baseColorTexture, p) || !TextureInfo(pbr, "metallicRoughnessTexture", mat.metallicRoughnessTexture, p))
                            return false;
                    }
                    if (!TextureInfo(m, "normalTexture", mat.normalTexture, path, "scale") || !TextureInfo(m, "occlusionTexture", mat.occlusionTexture, path, "strength") ||
                        !TextureInfo(m, "emissiveTexture", mat.emissiveTexture, path))
                        return false;
                    Floats(m, "emissiveFactor", mat.emissive, 3);
                    StringView alpha = Text(Get(m, "alphaMode"));
                    mat.alphaMode = alpha == StringView("MASK") ? AlphaMode::Mask : alpha == StringView("BLEND") ? AlphaMode::Blend : AlphaMode::Opaque;
                    mat.alphaCutoff = Float(m, "alphaCutoff", 0.5f);
                    mat.doubleSided = Bool(m, "doubleSided", false);

                    u32 ext = Get(m, "extensions");
                    String p = Format("{}.extensions", path);
                    mat.unlit = Get(ext, "KHR_materials_unlit") != json::kNoNode;
                    if (u32 e = Get(ext, "KHR_materials_emissive_strength"); e != json::kNoNode)
                        mat.emissiveStrength = Float(e, "emissiveStrength", 1);
                    if (u32 e = Get(ext, "KHR_materials_ior"); e != json::kNoNode)
                        mat.ior = Float(e, "ior", 1.5f);
                    if (u32 e = Get(ext, "KHR_materials_dispersion"); e != json::kNoNode)
                        mat.dispersion = Float(e, "dispersion", 0);
                    if (u32 e = Get(ext, "KHR_materials_transmission"); e != json::kNoNode)
                    {
                        mat.hasTransmission = true;
                        mat.transmission = Float(e, "transmissionFactor", 0);
                        if (!TextureInfo(e, "transmissionTexture", mat.transmissionTexture, p))
                            return false;
                    }
                    if (u32 e = Get(ext, "KHR_materials_volume"); e != json::kNoNode)
                    {
                        mat.hasVolume = true;
                        mat.thickness = Float(e, "thicknessFactor", 0);
                        mat.attenuationDistance = Float(e, "attenuationDistance", 0);
                        Floats(e, "attenuationColor", mat.attenuationColor, 3);
                        if (!TextureInfo(e, "thicknessTexture", mat.thicknessTexture, p))
                            return false;
                    }
                    if (u32 e = Get(ext, "KHR_materials_clearcoat"); e != json::kNoNode)
                    {
                        mat.hasClearcoat = true;
                        mat.clearcoat = Float(e, "clearcoatFactor", 0);
                        mat.clearcoatRoughness = Float(e, "clearcoatRoughnessFactor", 0);
                        if (!TextureInfo(e, "clearcoatTexture", mat.clearcoatTexture, p) || !TextureInfo(e, "clearcoatRoughnessTexture", mat.clearcoatRoughnessTexture, p) ||
                            !TextureInfo(e, "clearcoatNormalTexture", mat.clearcoatNormalTexture, p, "scale"))
                            return false;
                    }
                    if (u32 e = Get(ext, "KHR_materials_sheen"); e != json::kNoNode)
                    {
                        mat.hasSheen = true;
                        Floats(e, "sheenColorFactor", mat.sheenColor, 3);
                        mat.sheenRoughness = Float(e, "sheenRoughnessFactor", 0);
                        if (!TextureInfo(e, "sheenColorTexture", mat.sheenColorTexture, p) || !TextureInfo(e, "sheenRoughnessTexture", mat.sheenRoughnessTexture, p))
                            return false;
                    }
                    if (u32 e = Get(ext, "KHR_materials_specular"); e != json::kNoNode)
                    {
                        mat.hasSpecular = true;
                        mat.specular = Float(e, "specularFactor", 1);
                        Floats(e, "specularColorFactor", mat.specularColor, 3);
                        if (!TextureInfo(e, "specularTexture", mat.specularTexture, p) || !TextureInfo(e, "specularColorTexture", mat.specularColorTexture, p))
                            return false;
                    }
                    if (u32 e = Get(ext, "KHR_materials_iridescence"); e != json::kNoNode)
                    {
                        mat.hasIridescence = true;
                        mat.iridescence = Float(e, "iridescenceFactor", 0);
                        mat.iridescenceIor = Float(e, "iridescenceIor", 1.3f);
                        mat.iridescenceThicknessMin = Float(e, "iridescenceThicknessMinimum", 100);
                        mat.iridescenceThicknessMax = Float(e, "iridescenceThicknessMaximum", 400);
                        if (!TextureInfo(e, "iridescenceTexture", mat.iridescenceTexture, p) ||
                            !TextureInfo(e, "iridescenceThicknessTexture", mat.iridescenceThicknessTexture, p))
                            return false;
                    }
                    if (u32 e = Get(ext, "KHR_materials_anisotropy"); e != json::kNoNode)
                    {
                        mat.hasAnisotropy = true;
                        mat.anisotropyStrength = Float(e, "anisotropyStrength", 0);
                        mat.anisotropyRotation = Float(e, "anisotropyRotation", 0);
                        if (!TextureInfo(e, "anisotropyTexture", mat.anisotropyTexture, p))
                            return false;
                    }
                }
                u32 variants = Array(Ext(root_, "KHR_materials_variants"), "variants");
                for (u32 i = 0; i < Count(variants); i++)
                    s_.variants.Push(Str(j_.Item(variants, i), "name"));
                return true;
            }

            // ---------------------------------------------------------- meshes

            bool Attribute(StringView name, Semantic& semantic, u32& set)
            {
                set = 0;
                auto numbered = [&](StringView prefix, Semantic sm) {
                    if (!name.StartsWith(prefix))
                        return false;
                    StringView n = name.Substr(prefix.Size());
                    if (n.Empty())
                        return false;
                    u32 v = 0;
                    for (char c : n)
                    {
                        if (c < '0' || c > '9' || v > 1000)
                            return false;
                        v = v * 10 + u32(c - '0');
                    }
                    semantic = sm, set = v;
                    return true;
                };
                if (name == StringView("POSITION"))
                    return semantic = Semantic::Position, true;
                if (name == StringView("NORMAL"))
                    return semantic = Semantic::Normal, true;
                if (name == StringView("TANGENT"))
                    return semantic = Semantic::Tangent, true;
                if (numbered("TEXCOORD_", Semantic::TexCoord) || numbered("COLOR_", Semantic::Color) || numbered("JOINTS_", Semantic::Joints) ||
                    numbered("WEIGHTS_", Semantic::Weights))
                    return true;
                semantic = Semantic::Custom;
                return true;
            }

            // The streams of an attributes object, appended to Scene::streams; `vertices` is set from the first.
            bool Streams(u32 attributes, Range& out, u32& vertices, StringView path)
            {
                out.begin = u32(s_.streams.Size());
                if (!j_.IsObject(attributes))
                    return Fail(Format("{}: not an object", path));
                ::helix::Array<float> data;
                for (u32 k = 0; k < j_.Count(attributes); k++)
                {
                    StringView name = Text(j_.Key(attributes, k));
                    u64 accessor;
                    if (!j_.AsUInt(j_.Value(attributes, k), accessor) || accessor >= kNone)
                        return Fail(Format("{}.{}: not an accessor index", path, name));
                    VertexStream st;
                    if (!Attribute(name, st.semantic, st.set))
                        return false;
                    if (st.semantic == Semantic::Custom)
                        st.name = name;
                    if (!Accessor(u32(accessor), data, st.components, Format("{}.{}", path, name)))
                        return false;
                    u32 count = st.components ? u32(data.Size() / st.components) : 0;
                    if (vertices == kNone)
                        vertices = count;
                    else if (count != vertices)
                        return Fail(Format("{}.{}: {} elements where the primitive has {} vertices", path, name, count, vertices));
                    st.offset = u32(s_.floats.Size());
                    s_.floats.Append(data.Data(), data.Size());
                    s_.streams.Push(st);
                }
                out.count = u32(s_.streams.Size()) - out.begin;
                return true;
            }

            bool Meshes()
            {
                u32 a = Array(root_, "meshes");
                for (u32 m = 0; m < Count(a); m++)
                {
                    u32 mesh = j_.Item(a, m);
                    String mpath = Format("meshes[{}]", m);
                    Mesh out;
                    out.name = Str(mesh, "name");
                    u32 prims = Array(mesh, "primitives");
                    if (Count(prims) == 0)
                        return Fail(Format("{}.primitives: missing", mpath));
                    u32 names = Array(Get(mesh, "extras"), "targetNames");
                    out.primitives.begin = u32(s_.primitives.Size());
                    for (u32 p = 0; p < Count(prims); p++)
                    {
                        u32 prim = j_.Item(prims, p);
                        String path = Format("{}.primitives[{}]", mpath, p);
                        Primitive pr;
                        u32 vertices = kNone;
                        if (!Streams(Get(prim, "attributes"), pr.streams, vertices, Format("{}.attributes", path)))
                            return false;
                        pr.vertexCount = vertices == kNone ? 0 : vertices;
                        double mode = 4;
                        Number(Get(prim, "mode"), mode);
                        if (mode < 0 || mode > 6 || mode != std::floor(mode))
                            return Fail(Format("{}.mode: {}", path, mode));
                        pr.topology = Topology(u8(mode));
                        if (!Optional(prim, "material", u32(s_.materials.Size()), pr.material, path))
                            return false;
                        u32 indices;
                        if (!Index(prim, "indices", indices, path))
                            return false;
                        if (indices != kNone)
                        {
                            ::helix::Array<u32> list;
                            if (!Indices(indices, list, pr.vertexCount, Format("{}.indices", path)))
                                return false;
                            pr.indices = {u32(s_.indices.Size()), u32(list.Size())};
                            s_.indices.Append(list.Data(), list.Size());
                        }
                        u32 targets = Array(prim, "targets");
                        pr.targets.begin = u32(s_.morphTargets.Size());
                        for (u32 t = 0; t < Count(targets); t++)
                        {
                            MorphTarget target;
                            if (t < Count(names))
                                target.name = Text(j_.Item(names, t));
                            u32 count = pr.vertexCount;
                            if (!Streams(j_.Item(targets, t), target.streams, count, Format("{}.targets[{}]", path, t)))
                                return false;
                            s_.morphTargets.Push(target);
                        }
                        pr.targets.count = u32(s_.morphTargets.Size()) - pr.targets.begin;
                        u32 mappings = Array(Ext(prim, "KHR_materials_variants"), "mappings");
                        pr.variants.begin = u32(s_.variantMappings.Size() / 2);
                        for (u32 k = 0; k < Count(mappings); k++)
                        {
                            u32 mp = j_.Item(mappings, k);
                            u32 material;
                            String where = Format("{}.extensions.KHR_materials_variants.mappings[{}]", path, k);
                            if (!Required(mp, "material", u32(s_.materials.Size()), material, where))
                                return false;
                            u32 vs = Array(mp, "variants");
                            for (u32 v = 0; v < Count(vs); v++)
                            {
                                u64 variant;
                                if (!j_.AsUInt(j_.Item(vs, v), variant) || variant >= s_.variants.Size())
                                    return Fail(Format("{}.variants[{}]: not a variant", where, v));
                                s_.variantMappings.Push(u32(variant));
                                s_.variantMappings.Push(material);
                            }
                        }
                        pr.variants.count = u32(s_.variantMappings.Size() / 2) - pr.variants.begin;
                        s_.primitives.Push(pr);
                    }
                    out.primitives.count = u32(s_.primitives.Size()) - out.primitives.begin;
                    u32 weights = Array(mesh, "weights");
                    out.weights = {u32(s_.floats.Size()), Count(weights)};
                    for (u32 w = 0; w < Count(weights); w++)
                    {
                        double v = 0;
                        Number(j_.Item(weights, w), v);
                        s_.floats.Push(float(v));
                    }
                    s_.meshes.Push(out);
                }
                return true;
            }

            // ---------------------------------------------------------- cameras, lights, skins

            bool Cameras()
            {
                u32 a = Array(root_, "cameras");
                for (u32 i = 0; i < Count(a); i++)
                {
                    u32 c = j_.Item(a, i);
                    Camera cam;
                    cam.name = Str(c, "name");
                    StringView type = Text(Get(c, "type"));
                    if (type == StringView("orthographic"))
                    {
                        u32 o = Get(c, "orthographic");
                        cam.orthographic = true;
                        cam.xmag = Float(o, "xmag", 1);
                        cam.ymag = Float(o, "ymag", 1);
                        cam.znear = Float(o, "znear", 0.01f);
                        cam.zfar = Float(o, "zfar", 100);
                    }
                    else if (type == StringView("perspective"))
                    {
                        u32 p = Get(c, "perspective");
                        cam.yfov = Float(p, "yfov", 0.8f);
                        cam.aspectRatio = Float(p, "aspectRatio", 0);
                        cam.znear = Float(p, "znear", 0.01f);
                        cam.zfar = Float(p, "zfar", 0);
                    }
                    else
                        return Fail(Format("cameras[{}].type: '{}'", i, type));
                    s_.cameras.Push(cam);
                }
                return true;
            }

            bool Lights()
            {
                u32 a = Array(Ext(root_, "KHR_lights_punctual"), "lights");
                for (u32 i = 0; i < Count(a); i++)
                {
                    u32 l = j_.Item(a, i);
                    Light light;
                    light.name = Str(l, "name");
                    StringView type = Text(Get(l, "type"));
                    if (type == StringView("directional"))
                        light.type = LightType::Directional;
                    else if (type == StringView("point"))
                        light.type = LightType::Point;
                    else if (type == StringView("spot"))
                        light.type = LightType::Spot;
                    else
                        return Fail(Format("extensions.KHR_lights_punctual.lights[{}].type: '{}'", i, type));
                    Floats(l, "color", light.color, 3);
                    light.intensity = Float(l, "intensity", 1);
                    light.range = Float(l, "range", 0);
                    u32 spot = Get(l, "spot");
                    light.innerConeAngle = Float(spot, "innerConeAngle", 0);
                    light.outerConeAngle = Float(spot, "outerConeAngle", 0.7853982f);
                    s_.lights.Push(light);
                }
                return true;
            }

            bool Skins()
            {
                u32 a = Array(root_, "skins");
                u32 nodes = Count(Array(root_, "nodes"));
                for (u32 i = 0; i < Count(a); i++)
                {
                    u32 sk = j_.Item(a, i);
                    String path = Format("skins[{}]", i);
                    Skin skin;
                    skin.name = Str(sk, "name");
                    u32 joints = Array(sk, "joints");
                    if (Count(joints) == 0)
                        return Fail(Format("{}.joints: missing", path));
                    skin.joints = {u32(s_.skinJoints.Size()), Count(joints)};
                    for (u32 k = 0; k < Count(joints); k++)
                    {
                        u64 n;
                        if (!j_.AsUInt(j_.Item(joints, k), n) || n >= nodes)
                            return Fail(Format("{}.joints[{}]: not a node", path, k));
                        s_.skinJoints.Push(u32(n));
                    }
                    if (!Optional(sk, "skeleton", nodes, skin.skeleton, path))
                        return false;
                    u32 ibm;
                    if (!Index(sk, "inverseBindMatrices", ibm, path))
                        return false;
                    if (ibm != kNone)
                    {
                        ::helix::Array<float> m;
                        u32 components;
                        if (!Accessor(ibm, m, components, Format("{}.inverseBindMatrices", path)))
                            return false;
                        if (components != 16 || m.Size() / 16 < Count(joints))
                            return Fail(Format("{}.inverseBindMatrices: {} matrices for {} joints", path, components == 16 ? m.Size() / 16 : 0, Count(joints)));
                        skin.inverseBindMatrices = {u32(s_.floats.Size()), u32(Count(joints) * 16)};
                        s_.floats.Append(m.Data(), usize(Count(joints)) * 16);
                    }
                    s_.skins.Push(skin);
                }
                return true;
            }

            // ---------------------------------------------------------- nodes and scenes

            static void Decompose(const float* m, float* t, float* r, float* s)
            {
                t[0] = m[12], t[1] = m[13], t[2] = m[14];
                for (int c = 0; c < 3; c++)
                    s[c] = std::sqrt(m[c * 4] * m[c * 4] + m[c * 4 + 1] * m[c * 4 + 1] + m[c * 4 + 2] * m[c * 4 + 2]);
                float det = m[0] * (m[5] * m[10] - m[6] * m[9]) - m[4] * (m[1] * m[10] - m[2] * m[9]) + m[8] * (m[1] * m[6] - m[2] * m[5]);
                if (det < 0)
                    s[0] = -s[0];
                float x[9];
                for (int c = 0; c < 3; c++)
                    for (int rr = 0; rr < 3; rr++)
                        x[c * 3 + rr] = s[c] != 0 ? m[c * 4 + rr] / s[c] : 0;
                // Columns of the rotation: x[0..2], x[3..5], x[6..8]; m(row, col) = x[col * 3 + row].
                float m00 = x[0], m11 = x[4], m22 = x[8];
                float trace = m00 + m11 + m22;
                if (trace > 0)
                {
                    float q = std::sqrt(trace + 1) * 2;
                    r[3] = 0.25f * q;
                    r[0] = (x[5] - x[7]) / q;
                    r[1] = (x[6] - x[2]) / q;
                    r[2] = (x[1] - x[3]) / q;
                }
                else if (m00 > m11 && m00 > m22)
                {
                    float q = std::sqrt(1 + m00 - m11 - m22) * 2;
                    r[3] = (x[5] - x[7]) / q;
                    r[0] = 0.25f * q;
                    r[1] = (x[3] + x[1]) / q;
                    r[2] = (x[6] + x[2]) / q;
                }
                else if (m11 > m22)
                {
                    float q = std::sqrt(1 + m11 - m00 - m22) * 2;
                    r[3] = (x[6] - x[2]) / q;
                    r[0] = (x[3] + x[1]) / q;
                    r[1] = 0.25f * q;
                    r[2] = (x[7] + x[5]) / q;
                }
                else
                {
                    float q = std::sqrt(1 + m22 - m00 - m11) * 2;
                    r[3] = (x[1] - x[3]) / q;
                    r[0] = (x[6] + x[2]) / q;
                    r[1] = (x[7] + x[5]) / q;
                    r[2] = 0.25f * q;
                }
            }

            bool Nodes()
            {
                u32 a = Array(root_, "nodes");
                u32 n = Count(a);
                s_.nodes.Resize(n);
                for (u32 i = 0; i < n; i++)
                {
                    u32 nd = j_.Item(a, i);
                    String path = Format("nodes[{}]", i);
                    Node& node = s_.nodes[i];
                    node.name = Str(nd, "name");
                    if (!Optional(nd, "mesh", u32(s_.meshes.Size()), node.mesh, path) || !Optional(nd, "skin", u32(s_.skins.Size()), node.skin, path) ||
                        !Optional(nd, "camera", u32(s_.cameras.Size()), node.camera, path) ||
                        !Optional(Ext(nd, "KHR_lights_punctual"), "light", u32(s_.lights.Size()), node.light, Format("{}.extensions.KHR_lights_punctual", path)))
                        return false;
                    if (Count(Array(nd, "matrix")) == 16)
                    {
                        node.hasMatrix = true;
                        Floats(nd, "matrix", node.matrix, 16);
                        Decompose(node.matrix, node.translation, node.rotation, node.scale);
                    }
                    else
                    {
                        Floats(nd, "translation", node.translation, 3);
                        Floats(nd, "rotation", node.rotation, 4);
                        Floats(nd, "scale", node.scale, 3);
                    }
                    u32 weights = Array(nd, "weights");
                    node.weights = {u32(s_.floats.Size()), Count(weights)};
                    for (u32 w = 0; w < Count(weights); w++)
                    {
                        double v = 0;
                        Number(j_.Item(weights, w), v);
                        s_.floats.Push(float(v));
                    }
                    u32 children = Array(nd, "children");
                    node.children = {u32(s_.nodeChildren.Size()), Count(children)};
                    for (u32 c = 0; c < Count(children); c++)
                    {
                        u64 child;
                        if (!j_.AsUInt(j_.Item(children, c), child) || child >= n || child == i)
                            return Fail(Format("{}.children[{}]: not another node", path, c));
                        s_.nodeChildren.Push(u32(child));
                    }
                }
                // Parents, each node at most one.
                for (u32 i = 0; i < n; i++)
                    for (u32 c = 0; c < s_.nodes[i].children.count; c++)
                    {
                        u32 child = s_.nodeChildren[s_.nodes[i].children.begin + c];
                        if (s_.nodes[child].parent != kNone)
                            return Fail(Format("nodes[{}]: a child of nodes {} and {}", child, s_.nodes[child].parent, i));
                        s_.nodes[child].parent = i;
                    }
                // No cycles: every walk up ends.
                for (u32 i = 0; i < n; i++)
                {
                    u32 at = i, steps = 0;
                    while (at != kNone && steps <= n)
                        at = s_.nodes[at].parent, steps++;
                    if (steps > n)
                        return Fail(Format("nodes[{}]: its parents form a cycle", i));
                }
                return true;
            }

            bool Scenes()
            {
                u32 a = Array(root_, "scenes");
                u32 chosen;
                if (!Optional(root_, "scene", Count(a) ? Count(a) : kNone, chosen, "scene"))
                    return false;
                if (Count(a) == 0)
                {
                    for (u32 i = 0; i < s_.nodes.Size(); i++)
                        if (s_.nodes[i].parent == kNone)
                            s_.roots.Push(i);
                    return true;
                }
                u32 sc = j_.Item(a, chosen == kNone ? 0 : chosen);
                u32 nodes = Array(sc, "nodes");
                for (u32 i = 0; i < Count(nodes); i++)
                {
                    u64 nd;
                    if (!j_.AsUInt(j_.Item(nodes, i), nd) || nd >= s_.nodes.Size() || s_.nodes[usize(nd)].parent != kNone)
                        return Fail(Format("scenes[{}].nodes[{}]: not a root node", chosen == kNone ? 0 : chosen, i));
                    s_.roots.Push(u32(nd));
                }
                return true;
            }

            // ---------------------------------------------------------- animations

            bool Animations()
            {
                u32 a = Array(root_, "animations");
                for (u32 i = 0; i < Count(a); i++)
                {
                    u32 an = j_.Item(a, i);
                    String path = Format("animations[{}]", i);
                    AnimationClip clip;
                    clip.name = Str(an, "name");
                    u32 samplers = Array(an, "samplers");
                    u32 channels = Array(an, "channels");
                    clip.channels.begin = u32(s_.channels.Size());
                    for (u32 c = 0; c < Count(channels); c++)
                    {
                        u32 ch = j_.Item(channels, c);
                        String cpath = Format("{}.channels[{}]", path, c);
                        u32 sampler;
                        if (!Required(ch, "sampler", Count(samplers), sampler, cpath))
                            return false;
                        u32 target = Get(ch, "target");
                        AnimationChannel out;
                        if (!Optional(target, "node", u32(s_.nodes.Size()), out.node, Format("{}.target", cpath)))
                            return false;
                        if (out.node == kNone)
                            continue;
                        StringView p = Text(Get(target, "path"));
                        if (p == StringView("translation"))
                            out.path = AnimationPath::Translation;
                        else if (p == StringView("rotation"))
                            out.path = AnimationPath::Rotation;
                        else if (p == StringView("scale"))
                            out.path = AnimationPath::Scale;
                        else if (p == StringView("weights"))
                            out.path = AnimationPath::Weights;
                        else
                            continue;
                        u32 sm = j_.Item(samplers, sampler);
                        String spath = Format("{}.samplers[{}]", path, sampler);
                        StringView interp = Text(Get(sm, "interpolation"));
                        out.interpolation = interp == StringView("STEP") ? Interpolation::Step : interp == StringView("CUBICSPLINE") ? Interpolation::CubicSpline : Interpolation::Linear;
                        u32 input, output;
                        if (!Required(sm, "input", kNone, input, spath) || !Required(sm, "output", kNone, output, spath))
                            return false;
                        ::helix::Array<float> times, values;
                        u32 tc, vc;
                        if (!Accessor(input, times, tc, Format("{}.input", spath)) || !Accessor(output, values, vc, Format("{}.output", spath)))
                            return false;
                        if (tc != 1)
                            return Fail(Format("{}.input: times of {} components", spath, tc));
                        u32 keys = u32(times.Size());
                        u32 perKey = out.interpolation == Interpolation::CubicSpline ? 3 : 1;
                        if (keys == 0 || values.Size() % (usize(keys) * perKey) != 0)
                            return Fail(Format("{}.output: {} values for {} keys", spath, values.Size(), keys));
                        out.components = u32(values.Size() / (usize(keys) * perKey));
                        for (u32 k = 1; k < keys; k++)
                            if (times[k] < times[k - 1])
                                return Fail(Format("{}.input: times out of order at key {}", spath, k));
                        clip.duration = std::fmax(clip.duration, times[keys - 1]);
                        out.times = {u32(s_.floats.Size()), keys};
                        s_.floats.Append(times.Data(), times.Size());
                        out.values = {u32(s_.floats.Size()), u32(values.Size())};
                        s_.floats.Append(values.Data(), values.Size());
                        s_.channels.Push(out);
                    }
                    clip.channels.count = u32(s_.channels.Size()) - clip.channels.begin;
                    s_.clips.Push(clip);
                }
                return true;
            }

            Scene& s_;
            String& error_;
            const ReadOptions& options_;
            json::Tree j_;
            u32 root_ = json::kNoNode;
            Span<const u8> bin_;
            ::helix::Array<Span<const u8>> buffers_;
            ::helix::Array<::helix::Array<u8>> owned_;
            ::helix::Array<View> views_;
            ::helix::Array<Texture> samplers_;
            u64 floatsUsed_ = 0;
        };
    }

    Span<const StringView> SupportedGltfExtensions() { return Span<const StringView>(kSupported); }

    bool IsGlb(Span<const u8> data) { return data.Size() >= 4 && std::memcmp(data.Data(), "glTF", 4) == 0; }

    bool ReadGltf(Span<const u8> data, Scene& out, String& error, const ReadOptions& options)
    {
        out.Clear();
        Reader r(out, error, options);
        if (r.Run(data))
            return true;
        out.Clear();
        return false;
    }
}
