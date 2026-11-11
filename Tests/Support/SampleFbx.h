#pragma once

#include <Support/FbxWriter.h>

// An FBX scene with one of everything the reader turns into the scene, for the tests, fuzz targets and benchmarks.
//
//   Root (Null) - Bone (LimbNode, pre rotation 90 about z, translation animated) - Tip (LimbNode, rotation 90 about z
//   around the pivot (1, 0, 0)); Cube (Mesh: translation (1, 2, 3), rotation 90 about x, scale 2, geometric translation
//   (0, 0, 1)); Cam (Camera); Lamp (Light).
//   The cube: 6 quads, normals by polygon vertex, UV set "map" indexed, UV set "second" by vertex, colors by polygon,
//   materials Red (faces 0 to 2) and Blue (3 to 5), a skin of two clusters, a blend shape channel of two shapes.
//   Z up, front -y, centimeters, 30 frames a second; one stack of one second.
namespace helix::model::testing
{
    inline constexpr i64 kSecond = 46186158000;

    inline FbxFile SampleFbx()
    {
        FbxFile f;
        FbxNode settings("GlobalSettings");
        FbxNode p70("Properties70");
        p70.Add(PInt("UpAxis", 2)).Add(PInt("UpAxisSign", 1)).Add(PInt("FrontAxis", 1)).Add(PInt("FrontAxisSign", -1)).Add(PInt("CoordAxis", 0));
        p70.Add(PInt("CoordAxisSign", 1)).Add(P("UnitScaleFactor", "double", {1})).Add(PInt("TimeMode", 6));
        settings.Add(Move(p70));
        f.top.Push(Move(settings));

        FbxNode objects("Objects");
        // The cube geometry.
        FbxNode g = Object("Geometry", 100, "Geometry", "CubeMesh", "Mesh");
        g.Numbers("Vertices", {-1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1});
        g.IntArrayChild("PolygonVertexIndex", {0, 3, 2, -2, 4, 5, 6, -8, 0, 1, 5, -5, 3, 7, 6, -3, 0, 4, 7, -4, 1, 2, 6, -6});
        g.IntArrayChild("Edges", {0, 1, 2, 3});
        {
            FbxNode n("LayerElementNormal");
            n.I(0).Int("Version", 101).Text("MappingInformationType", "ByPolygonVertex").Text("ReferenceInformationType", "Direct");
            Array<double> normals;
            const double faces[6][3] = {{0, 0, -1}, {0, 0, 1}, {0, -1, 0}, {0, 1, 0}, {-1, 0, 0}, {1, 0, 0}};
            for (int face = 0; face < 6; face++)
                for (int c = 0; c < 4; c++)
                    for (int a = 0; a < 3; a++)
                        normals.Push(faces[face][a]);
            FbxNode values("Normals");
            values.DoubleArray(normals);
            n.Add(Move(values));
            g.Add(Move(n));
        }
        {
            FbxNode uv("LayerElementUV");
            uv.I(0).Text("Name", "map").Text("MappingInformationType", "ByPolygonVertex").Text("ReferenceInformationType", "IndexToDirect");
            uv.Numbers("UV", {0, 0, 1, 0, 1, 1, 0, 1});
            Array<i64> index;
            for (int i = 0; i < 24; i++)
                index.Push(i % 4);
            FbxNode ix("UVIndex");
            ix.IntArray(index);
            uv.Add(Move(ix));
            g.Add(Move(uv));
        }
        {
            FbxNode uv("LayerElementUV");
            uv.I(1).Text("Name", "second").Text("MappingInformationType", "ByVertice").Text("ReferenceInformationType", "Direct");
            uv.Numbers("UV", {0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1});
            g.Add(Move(uv));
        }
        {
            FbxNode c("LayerElementColor");
            c.I(0).Text("MappingInformationType", "ByPolygon").Text("ReferenceInformationType", "Direct");
            c.Numbers("Colors", {0, 0, 0, 1, 0.2, 0, 0, 1, 0.4, 0, 0, 1, 0.6, 0, 0, 1, 0.8, 0, 0, 1, 1, 0, 0, 1});
            g.Add(Move(c));
        }
        {
            FbxNode m("LayerElementMaterial");
            m.I(0).Text("MappingInformationType", "ByPolygon").Text("ReferenceInformationType", "IndexToDirect");
            m.IntArrayChild("Materials", {0, 0, 0, 1, 1, 1});
            g.Add(Move(m));
        }
        objects.Add(Move(g));

        auto model = [](i64 id, StringView name, StringView kind, std::initializer_list<FbxNode> props) {
            FbxNode m = Object("Model", id, "Model", name, kind);
            FbxNode p("Properties70");
            for (const FbxNode& x : props)
                p.Add(x);
            m.Add(Move(p));
            return m;
        };
        objects.Add(model(200, "Cube", "Mesh",
                          {P("Lcl Translation", "Lcl Translation", {1, 2, 3}), P("Lcl Rotation", "Lcl Rotation", {90, 0, 0}),
                           P("Lcl Scaling", "Lcl Scaling", {2, 2, 2}), P("GeometricTranslation", "Vector3D", {0, 0, 1})}));
        objects.Add(model(201, "Root", "Null", {}));
        objects.Add(model(202, "Bone", "LimbNode", {P("Lcl Translation", "Lcl Translation", {0, 1, 0}), P("PreRotation", "Vector3D", {0, 0, 90}), PInt("RotationActive", 1)}));
        objects.Add(model(203, "Tip", "LimbNode", {P("Lcl Rotation", "Lcl Rotation", {0, 0, 90}), P("RotationPivot", "Vector3D", {1, 0, 0})}));
        objects.Add(model(204, "Cam", "Camera", {}));
        objects.Add(model(205, "Lamp", "Light", {}));

        auto material = [](i64 id, StringView name, std::initializer_list<FbxNode> props) {
            FbxNode m = Object("Material", id, "Material", name, "");
            m.Text("ShadingModel", "phong");
            FbxNode p("Properties70");
            for (const FbxNode& x : props)
                p.Add(x);
            m.Add(Move(p));
            return m;
        };
        objects.Add(material(300, "Red", {P("DiffuseColor", "Color", {1, 0, 0}), P("Opacity", "double", {0.5}), P("ShininessExponent", "double", {20}),
                                          P("EmissiveColor", "Color", {0, 1, 0}), P("EmissiveFactor", "double", {0.5})}));
        objects.Add(material(301, "Blue", {P("DiffuseColor", "Color", {0, 0, 1}), P("TransparencyFactor", "double", {0})}));
        {
            FbxNode t = Object("Texture", 400, "Texture", "RedTex", "");
            t.Text("FileName", "C:/art/tex/red.png").Text("RelativeFilename", "tex/red.png");
            FbxNode p("Properties70");
            p.Add(P("Scaling", "Vector", {2, 3, 1}));
            t.Add(Move(p));
            objects.Add(Move(t));
            FbxNode v = Object("Video", 401, "Video", "RedVideo", "Clip");
            FbxNode content("Content");
            Array<u8> bytes;
            bytes.Append(reinterpret_cast<const u8*>("PNG!"), 4);
            content.R(bytes);
            v.Add(Move(content));
            objects.Add(Move(v));
        }
        {
            objects.Add(Object("Deformer", 500, "Deformer", "Skin", "Skin"));
            FbxNode c1 = Object("Deformer", 501, "SubDeformer", "C1", "Cluster");
            c1.IntArrayChild("Indexes", {0, 1, 2, 3}).Numbers("Weights", {1, 1, 0.5, 0.5});
            c1.Numbers("Transform", {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}).Numbers("TransformLink", {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 5, 1});
            objects.Add(Move(c1));
            FbxNode c2 = Object("Deformer", 502, "SubDeformer", "C2", "Cluster");
            c2.IntArrayChild("Indexes", {2, 3}).Numbers("Weights", {0.5, 0.5});
            c2.Numbers("Transform", {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}).Numbers("TransformLink", {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1});
            objects.Add(Move(c2));
        }
        {
            objects.Add(Object("Deformer", 600, "Deformer", "Shapes", "BlendShape"));
            FbxNode ch = Object("Deformer", 601, "SubDeformer", "Bulge", "BlendShapeChannel");
            ch.Int("Version", 100).Numbers("FullWeights", {50, 100});
            FbxNode p("Properties70");
            p.Add(P("DeformPercent", "Number", {25}));
            ch.Add(Move(p));
            objects.Add(Move(ch));
            FbxNode s1 = Object("Geometry", 602, "Geometry", "Half", "Shape");
            s1.IntArrayChild("Indexes", {0}).Numbers("Vertices", {0, 0, 1}).Numbers("Normals", {0, 0, 0});
            objects.Add(Move(s1));
            FbxNode s2 = Object("Geometry", 603, "Geometry", "Full", "Shape");
            s2.IntArrayChild("Indexes", {0}).Numbers("Vertices", {0, 0, 2});
            objects.Add(Move(s2));
        }
        {
            FbxNode cam = Object("NodeAttribute", 700, "NodeAttribute", "CamAttr", "Camera");
            FbxNode p("Properties70");
            p.Add(P("FieldOfView", "FieldOfView", {60})).Add(P("NearPlane", "double", {1})).Add(P("FarPlane", "double", {1000}));
            p.Add(P("AspectWidth", "double", {1920})).Add(P("AspectHeight", "double", {1080}));
            cam.Add(Move(p));
            objects.Add(Move(cam));
            FbxNode light = Object("NodeAttribute", 701, "NodeAttribute", "LampAttr", "Light");
            FbxNode q("Properties70");
            q.Add(PInt("LightType", 2)).Add(P("Color", "Color", {1, 0.5, 0.25})).Add(P("Intensity", "Number", {200})).Add(P("OuterAngle", "Number", {60}));
            light.Add(Move(q));
            objects.Add(Move(light));
        }
        {
            FbxNode stack = Object("AnimationStack", 800, "AnimStack", "Take", "");
            FbxNode p("Properties70");
            p.Add(PTime("LocalStart", 0)).Add(PTime("LocalStop", kSecond));
            stack.Add(Move(p));
            objects.Add(Move(stack));
            objects.Add(Object("AnimationLayer", 801, "AnimLayer", "Base", ""));
            FbxNode node = Object("AnimationCurveNode", 802, "AnimCurveNode", "T", "");
            FbxNode q("Properties70");
            q.Add(P("d|X", "Number", {0})).Add(P("d|Y", "Number", {1})).Add(P("d|Z", "Number", {0}));
            node.Add(Move(q));
            objects.Add(Move(node));
            auto curve = [](i64 id, double a, double b, i64 flags) {
                FbxNode c = Object("AnimationCurve", id, "AnimCurve", "", "");
                c.Numbers("Default", {a});
                FbxNode times("KeyTime");
                times.Ints({0, kSecond}, 'l');
                c.Add(Move(times));
                FbxNode values("KeyValueFloat");
                values.Doubles({a, b}, 'f');
                c.Add(Move(values));
                FbxNode fl("KeyAttrFlags");
                fl.Ints({flags});
                c.Add(Move(fl));
                FbxNode data("KeyAttrDataFloat");
                data.Doubles({0, 0, 0, 0}, 'f');
                c.Add(Move(data));
                FbxNode refs("KeyAttrRefCount");
                refs.Ints({2});
                c.Add(Move(refs));
                return c;
            };
            objects.Add(curve(803, 0, 10, 4));
            objects.Add(curve(804, 3, 7, 2));
            objects.Add(Object("AnimationCurveNode", 805, "AnimCurveNode", "DeformPercent", ""));
            objects.Add(curve(806, 0, 100, 4));
        }
        f.top.Push(Move(objects));

        FbxNode c("Connections");
        c.Add(Connect(201, 0)).Add(Connect(202, 201)).Add(Connect(203, 202)).Add(Connect(200, 0)).Add(Connect(204, 0)).Add(Connect(205, 0));
        c.Add(Connect(100, 200)).Add(Connect(300, 200)).Add(Connect(301, 200)).Add(Connect(400, 300, "DiffuseColor")).Add(Connect(401, 400));
        c.Add(Connect(500, 100)).Add(Connect(501, 500)).Add(Connect(502, 500)).Add(Connect(202, 501)).Add(Connect(203, 502));
        c.Add(Connect(600, 100)).Add(Connect(601, 600)).Add(Connect(602, 601)).Add(Connect(603, 601));
        c.Add(Connect(700, 204)).Add(Connect(701, 205));
        c.Add(Connect(801, 800)).Add(Connect(802, 801)).Add(Connect(802, 202, "Lcl Translation")).Add(Connect(803, 802, "d|X")).Add(Connect(804, 802, "d|Y"));
        c.Add(Connect(805, 801)).Add(Connect(805, 601, "DeformPercent")).Add(Connect(806, 805, "d|DeformPercent"));
        f.top.Push(Move(c));
        return f;
    }
}
