#pragma once

#include <Helix/Model/Scene/Scene.h>

// Mesh processing on the primitives of a scene, as the model importer runs it: every function takes the scene and a
// primitive index and keeps every stream and morph target of the primitive in step with the vertices it changes.
// Functions that change vertices append the new stream data to Scene::floats; Compact drops what is no longer used.
namespace helix::model
{
    // ---------------------------------------------------------------------- editing

    // Strips and fans to triangle lists (indices made when the primitive had none); other topologies are left.
    void ToTriangleList(Scene& scene, u32 primitive);
    // Vertices in a new order: new vertex i is old vertex `order[i]` (an old vertex may appear several times or not at
    // all); the indices are not touched.
    void GatherVertices(Scene& scene, u32 primitive, const Array<u32>& order);
    // Vertices equal in every stream and morph target merged, the indices rewritten; returns the vertices left.
    u32 WeldVertices(Scene& scene, u32 primitive);
    // The float pool rebuilt with only the ranges streams, matrices, weights and clips use.
    void Compact(Scene& scene);

    // ---------------------------------------------------------------------- normals and tangents

    enum class NormalWeighting : u8
    {
        AreaAndAngle,
        Area,
        Angle,
        Unweighted,
    };

    // Normals of a triangle list: each corner sums the face normals around its position whose angle to its own face is
    // within `smoothingAngle` degrees (180 smooths everything, 0 gives faceted normals); vertices split where corners
    // differ. Replaces the normal stream.
    void CalculateNormals(Scene& scene, u32 primitive, float smoothingAngle = 60, NormalWeighting weighting = NormalWeighting::AreaAndAngle);

    // MikkTSpace tangents (xyz, and w the sign of the bitangent: bitangent = w * cross(normal, tangent)) from the normals
    // and UV set `uvSet`; vertices split where tangent spaces differ. False when the primitive lacks either stream.
    bool CalculateTangents(Scene& scene, u32 primitive, u32 uvSet = 0);

    // ---------------------------------------------------------------------- optimization

    struct VertexCacheStats
    {
        // Vertices transformed per triangle, and per vertex of the mesh (1 is the best).
        float acmr = 0, atvr = 0;
    };

    struct OverdrawStats
    {
        // Pixels shaded per pixel covered, from six axis directions with a depth test (1 is no overdraw).
        float overdraw = 0;
    };

    VertexCacheStats AnalyzeVertexCache(const u32* indices, usize indexCount, u32 vertexCount, u32 cacheSize = 16);
    OverdrawStats AnalyzeOverdraw(const float* positions, const u32* indices, usize indexCount);

    // Triangles reordered for a post transform cache (Tom Forsyth, "Linear-Speed Vertex Cache Optimisation").
    void OptimizeVertexCache(Scene& scene, u32 primitive);
    // Triangle clusters reordered outside in to reduce overdraw (Sander, Nehab and Barczak, "Fast Triangle Reordering
    // for Vertex Locality and Reduced Overdraw"), keeping the cache efficiency within `threshold` (1.05: 5 % worse).
    void OptimizeOverdraw(Scene& scene, u32 primitive, float threshold = 1.05f);
    // Vertices in the order the triangles first use them; unused vertices dropped.
    void OptimizeVertexFetch(Scene& scene, u32 primitive);

    // ---------------------------------------------------------------------- simplification

    struct SimplifyResult
    {
        u32 indexCount = 0;
        // The largest quadric error accepted, as a distance relative to the mesh's size.
        float error = 0;
    };

    // Quadric edge collapse (Garland and Heckbert) of a triangle list toward `targetIndexCount`, stopping before the
    // error would pass `targetError` (relative to the mesh's size). Vertices are kept; the triangles move onto them.
    // Borders collapse only along themselves, attribute seams and non-manifold vertices stay. The new indices go to `out`.
    SimplifyResult SimplifyIndices(const float* positions, u32 vertexCount, const u32* indices, usize indexCount, usize targetIndexCount,
                                   float targetError, Array<u32>& out);
    // The same on a primitive, its index range replaced.
    SimplifyResult Simplify(Scene& scene, u32 primitive, float ratio, float targetError = 0.01f);

    // ---------------------------------------------------------------------- meshlets

    struct Meshlet
    {
        // Into Meshlets::vertices (indices of the primitive's vertices) and Meshlets::triangles (3 local indices each).
        u32 vertexOffset = 0, vertexCount = 0;
        u32 triangleOffset = 0, triangleCount = 0;
        float center[3] = {0, 0, 0}, radius = 0;
        // The normal cone: every triangle faces away from a view direction d with dot(d, axis) >= cutoff (above 1 when
        // no direction is, the normals spreading over more than a half space).
        float coneAxis[3] = {0, 0, 1}, coneCutoff = 2;
    };

    struct Meshlets
    {
        Array<Meshlet> meshlets;
        Array<u32> vertices;
        Array<u8> triangles;
    };

    inline constexpr u32 kMeshletVertices = 64, kMeshletTriangles = 124;

    void BuildMeshlets(const float* positions, u32 vertexCount, const u32* indices, usize indexCount, Meshlets& out, u32 maxVertices = kMeshletVertices,
                       u32 maxTriangles = kMeshletTriangles);

    // ---------------------------------------------------------------------- blend shapes

    // A blend shape delta stream kept only where it moves, at 16 bits a component.
    struct CompressedDeltas
    {
        Array<u32> vertices;
        Array<i16> values;
        // Units per step: value * scale is the delta.
        float scale = 0;
        u32 components = 3;
    };

    // Vertices whose delta is below `threshold` in every component are dropped; the rest are quantized.
    void CompressDeltas(const float* deltas, u32 vertexCount, u32 components, float threshold, CompressedDeltas& out);
    void DecompressDeltas(const CompressedDeltas& in, u32 vertexCount, Array<float>& out);
}
