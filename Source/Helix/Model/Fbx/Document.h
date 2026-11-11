#pragma once

#include <Helix/Core/Basics/Span.h>
#include <Helix/Core/Containers/Array.h>
#include <Helix/Core/Strings/String.h>

// The node tree of an FBX file, binary or ASCII, before any meaning is given to it: nodes with a name, properties and
// children, in flat arrays. Arrays are decoded (zlib arrays inflated) into one pool.
namespace helix::model::fbx
{
    inline constexpr u32 kNoNode = ~0u;

    enum class PropertyType : u8
    {
        Bool,
        Int16,
        Int32,
        Int64,
        Float,
        Double,
        String,
        Raw,
        BoolArray,
        Int32Array,
        Int64Array,
        FloatArray,
        DoubleArray,
    };

    struct Property
    {
        PropertyType type = PropertyType::Int32;
        // Scalars, as both an integer and a number.
        i64 integer = 0;
        double number = 0;
        // Strings and raw bytes (into Document::bytes), arrays (elements of their type, into Document::arrays).
        u32 offset = 0;
        u32 count = 0;

        bool IsArray() const { return type >= PropertyType::BoolArray; }
        bool IsNumber() const { return type <= PropertyType::Double; }
    };

    struct Node
    {
        // Into Document::bytes.
        u32 nameOffset = 0, nameSize = 0;
        u32 firstProperty = 0, propertyCount = 0;
        // Into Document::children.
        u32 firstChild = 0, childCount = 0;
    };

    class Document
    {
    public:
        // Reads a binary or ASCII FBX file. On failure `error` names the byte offset (binary) or line (ASCII).
        bool Read(Span<const u8> data, String& error);

        bool binary = false;
        // 7400 for 7.4; ASCII files tell it in FBXHeaderExtension.
        u32 version = 0;
        // Node 0 is the root; its children are the top level nodes.
        Array<Node> nodes;
        Array<Property> properties;
        Array<u32> children;
        Array<u8> bytes;
        Array<u8> arrays;

        StringView Name(u32 node) const { return StringView(reinterpret_cast<const char*>(bytes.Data()) + nodes[node].nameOffset, nodes[node].nameSize); }
        // Counts of a missing node (kNoNode) are 0.
        u32 PropertyCount(u32 node) const { return node < nodes.Size() ? nodes[node].propertyCount : 0; }
        const Property& Prop(u32 node, u32 i) const { return properties[nodes[node].firstProperty + i]; }
        u32 ChildCount(u32 node) const { return node < nodes.Size() ? nodes[node].childCount : 0; }
        u32 Child(u32 node, u32 i) const { return children[nodes[node].firstChild + i]; }
        // The first child of a name, kNoNode when there is none.
        u32 Find(u32 node, StringView name) const;

        // A property as text (strings), number or integer; arrays as numbers or integers of any element type.
        StringView Text(const Property& p) const;
        double Number(const Property& p) const { return p.number; }
        i64 Integer(const Property& p) const { return p.integer; }
        bool Numbers(const Property& p, Array<double>& out) const;
        bool Integers(const Property& p, Array<i64>& out) const;
        // The first property of the first child of a name: Value(node, "Version").
        const Property* Value(u32 node, StringView child, u32 index = 0) const;

    private:
        friend struct BinaryReader;
        friend struct AsciiReader;
    };

    // An object's name without its class: "Cube" of the binary "Cube\0\1Model" and of the ASCII "Model::Cube".
    StringView ObjectName(StringView full);
}
