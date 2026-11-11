#pragma once

#include <Support/GltfWriter.h>

// Models the fuzz targets and benchmarks share.
namespace helix::model::testing
{
    // A small glTF with a skinned, morphed, animated, textured triangle and every node feature; `body` gets the JSON
    // members after the buffers.
    inline GltfWriter SampleWriter(String& body)
    {
        GltfWriter w;
        u32 pos = w.Data<float>({0, 0, 0, 1, 0, 0, 0, 1, 0}, 5126, "VEC3", 3);
        u32 nrm = w.Data<i8>({0, 0, 127, 0, 0, 0, 127, 0, 0, 0, 127, 0}, 5120, "VEC3", 4, true);
        u32 uv = w.Data<u16>({0, 0, 65535, 0, 0, 65535}, 5123, "VEC2", 2, true);
        u32 joints = w.Data<u8>({0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0}, 5121, "VEC4", 4);
        u32 weights = w.Data<float>({1, 0, 0, 0, 0.5f, 0.5f, 0, 0, 1, 0, 0, 0}, 5126, "VEC4", 4);
        u32 indices = w.Data<u16>({0, 1, 2}, 5123, "SCALAR", 1);
        u32 ibm = w.Data<float>({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, 5126, "MAT4", 16);
        u32 times = w.Data<float>({0, 1}, 5126, "SCALAR", 1);
        u32 moves = w.Data<float>({0, 0, 0, 1, 0, 0}, 5126, "VEC3", 3);
        u32 si = w.View(std::initializer_list<u8>({1, 0, 0, 0}).begin(), 4);
        u32 sv = w.View(std::initializer_list<float>({0, 0, 2}).begin(), 12);
        u32 sparse = w.Accessor(~0u, 5126, 3, "VEC3", false, 0,
                                Format("\"sparse\":{{\"count\":1,\"indices\":{{\"bufferView\":{},\"componentType\":5121}},\"values\":{{\"bufferView\":{}}}}}", si, sv));
        body = Format(
            "\"extensionsUsed\":[\"KHR_mesh_quantization\",\"KHR_texture_transform\",\"KHR_lights_punctual\"],"
            "\"images\":[{{\"uri\":\"a.png\"}},{{\"uri\":\"data:image/png;base64,SGVs\"}}],\"samplers\":[{{\"magFilter\":9729}}],"
            "\"textures\":[{{\"source\":0,\"sampler\":0}},{{\"source\":1}}],"
            "\"materials\":[{{\"pbrMetallicRoughness\":{{\"baseColorTexture\":{{\"index\":0,\"extensions\":{{\"KHR_texture_transform\":{{\"scale\":[2,2]}}}}}}}},"
            "\"normalTexture\":{{\"index\":1}},\"extensions\":{{\"KHR_materials_clearcoat\":{{\"clearcoatFactor\":1}}}}}}],"
            "\"meshes\":[{{\"primitives\":[{{\"attributes\":{{\"POSITION\":{},\"NORMAL\":{},\"TEXCOORD_0\":{},\"JOINTS_0\":{},\"WEIGHTS_0\":{}}},"
            "\"indices\":{},\"material\":0,\"targets\":[{{\"POSITION\":{}}}]}}],\"weights\":[0.5]}}],"
            "\"skins\":[{{\"joints\":[1,2],\"inverseBindMatrices\":{}}}],\"cameras\":[{{\"type\":\"perspective\",\"perspective\":{{\"yfov\":1,\"znear\":0.1}}}}],"
            "\"extensions\":{{\"KHR_lights_punctual\":{{\"lights\":[{{\"type\":\"point\"}}]}}}},"
            "\"nodes\":[{{\"mesh\":0,\"skin\":0,\"children\":[1]}},{{\"children\":[2],\"matrix\":[1,0,0,0,0,1,0,0,0,0,1,0,1,2,3,1]}},{{\"camera\":0,"
            "\"extensions\":{{\"KHR_lights_punctual\":{{\"light\":0}}}}}}],\"scenes\":[{{\"nodes\":[0]}}],"
            "\"animations\":[{{\"samplers\":[{{\"input\":{},\"output\":{}}}],\"channels\":[{{\"sampler\":0,\"target\":{{\"node\":1,\"path\":\"translation\"}}}}]}}]",
            pos, nrm, uv, joints, weights, indices, sparse, ibm, times, moves);
        return w;
    }

    inline constexpr const char* kSampleObj = "mtllib m.mtl\no A\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0 1 0 0\nvt 0 0\nvt 1 0\nvt 1 1\nvn 0 0 1\n"
                                              "usemtl x\nf 1/1/1 2/2/1 3/3/1 4/1/1\ng B\nl 1 2 3\np 4\nf -1 -2 -3\n";
    inline constexpr const char* kSampleMtl = "newmtl x\nKd 1 0.5 0\nNs 50\nd 0.5\nmap_Kd -s 2 2 -o 0.1 0.1 -clamp on tex.png\nmap_bump -bm 2 b.png\nPr 0.5\nPm 0.1\n";
}
