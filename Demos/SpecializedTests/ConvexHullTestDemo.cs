using System;
using System.Collections.Generic;
using System.Numerics;
using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using static BepuPhysics.Collidables.ConvexHullHelper;
using System.Diagnostics;
using BepuUtilities;
using BepuPhysics.Constraints;
using BepuUtilities.Memory;
using System.Text.Json;
using System.IO;
using DemoRenderer.Constraints;
using DemoUtilities;
using DemoRenderer.UI;

namespace Demos.SpecializedTests;

public class ConvexHullTestDemo : Demo
{
    Buffer<Vector3> CreateRandomConvexHullPoints()
    {
        const int pointCount = 50;
        BufferPool.Take<Vector3>(pointCount, out var points);

        var random = new Random(5);
        for (int i = 0; i < pointCount; ++i)
        {
            points[i] = new Vector3(3 * random.NextSingle(), 1 * random.NextSingle(), 3 * random.NextSingle());
        }

        return points;
    }



    Vector3[] CreateBwaa()
    {
        var points = new Vector3[]
        {
             new(-0.637357891f, 0.347849399f, -0.303436399f),
             new(-0.636290252f, 0.345867455f, -0.301366687f),
             new(-0.992014945f, 0.348357588f, -0.3031407f),
             new(-1.00909662f, 0.386065364f, -0.303337872f),
             new(0.637357891f, 0.347849399f, -0.303436399f),
             new(-0.636290252f, 0.345918268f, 0.701366544f),
             new(-0.636503756f, 0.345918268f, 0.700873733f),
             new(-0.992655516f, 0.346578926f, 0.701070845f),
             new(-0.992655516f, 0.346578926f, -0.301070988f),
             new(0.636290252f, 0.345867455f, -0.301366687f),
             new(-0.995858312f, 0.348510057f, -0.301859498f),
             new(-1.01272643f, 0.385912925f, -0.302056611f),
             new(-1.01037765f, 0.390029252f, -0.302746475f),
             new(-0.637357891f, 0.389521062f, -0.302845061f),
             new(1.00909662f, 0.386065364f, -0.303337872f),
             new(0.992014945f, 0.348357588f, -0.3031407f),
             new(-0.637357891f, 0.347849399f, 0.703436255f),
             new(-0.992014945f, 0.348357588f, 0.703140557f),
             new(0.636290252f, 0.345918268f, 0.701366544f),
             new(-0.995858312f, 0.348510057f, 0.701859355f),
             new(-1.02553761f, 0.351406753f, 0.678599536f),
             new(-1.0251106f, 0.35013628f, 0.675938487f),
             new(-1.0251106f, 0.35013628f, -0.2759386f),
             new(-1.02553761f, 0.351406753f, -0.278599679f),
             new(0.992655516f, 0.346578926f, -0.301070988f),
             new(0.992655516f, 0.346578926f, 0.701070845f),
             new(0.636503756f, 0.345918268f, 0.700873733f),
             new(-1.04432738f, 0.37869662f, -0.274558783f),
             new(-1.01400757f, 0.389673531f, -0.301465213f),
             new(-1.04582202f, 0.382304758f, -0.273770332f),
             new(-1.0582062f, 0.67344743f, -0.220745891f),
             new(-1.0545764f, 0.674260557f, -0.22183004f),
             new(-0.637144327f, 0.674158931f, -0.221928596f),
             new(1.01037765f, 0.390029252f, -0.302746475f),
             new(0.637357891f, 0.389521062f, -0.302845061f),
             new(1.01272643f, 0.385912925f, -0.302056611f),
             new(0.995858312f, 0.348510057f, -0.301859498f),
             new(-1.00909662f, 0.386065364f, 0.703337729f),
             new(0.637357891f, 0.347849399f, 0.703436255f),
             new(0.992014945f, 0.348357588f, 0.703140557f),
             new(-1.01272643f, 0.385912925f, 0.702056468f),
             new(-1.04432738f, 0.37869662f, 0.67455864f),
             new(-1.04582202f, 0.380170345f, 0.671404779f),
             new(-1.04582202f, 0.380170345f, -0.271404922f),
             new(1.02553761f, 0.351406753f, -0.278599679f),
             new(1.0251106f, 0.35013628f, -0.2759386f),
             new(1.0251106f, 0.35013628f, 0.675938487f),
             new(1.02553761f, 0.351406753f, 0.678599536f),
             new(0.995858312f, 0.348510057f, 0.701859355f),
             new(-1.08980727f, 0.656575501f, -0.196303427f),
             new(-1.09023428f, 0.656982064f, -0.193346679f),
             new(-1.0584197f, 0.67675066f, -0.21867618f),
             new(-1.0550034f, 0.677512944f, -0.219858885f),
             new(-1.09023428f, 0.659827888f, -0.194233686f),
             new(0.637144327f, 0.674158931f, -0.221928596f),
             new(1.01400757f, 0.389673531f, -0.301465213f),
             new(1.0545764f, 0.674260557f, -0.22183004f),
             new(1.0582062f, 0.67344743f, -0.220745891f),
             new(1.04582202f, 0.382304758f, -0.273770332f),
             new(1.04432738f, 0.37869662f, -0.274558783f),
             new(-0.637357891f, 0.389521062f, 0.702844918f),
             new(-1.01037765f, 0.390029252f, 0.702746332f),
             new(1.00909662f, 0.386065364f, 0.703337729f),
             new(-1.01400757f, 0.389673531f, 0.70146507f),
             new(-1.04582202f, 0.382304758f, 0.673770189f),
             new(-1.09023428f, 0.656982064f, 0.593346536f),
             new(1.04582202f, 0.380170345f, -0.271404922f),
             new(1.04582202f, 0.380170345f, 0.671404779f),
             new(1.04432738f, 0.37869662f, 0.67455864f),
             new(1.01272643f, 0.385912925f, 0.702056468f),
             new(-1.09066129f, 0.832155526f, 0.199999928f),
             new(-1.0584197f, 0.86234206f, -0.0161386579f),
             new(-1.0550034f, 0.863663316f, -0.0167300105f),
             new(1.0550034f, 0.677512944f, -0.219858885f),
             new(-1.09023428f, 0.833781719f, -0.00135488808f),
             new(1.08980727f, 0.656575501f, -0.196303427f),
             new(1.0584197f, 0.67675066f, -0.21867618f),
             new(1.09023428f, 0.659827888f, -0.194233686f),
             new(1.09023428f, 0.656982064f, -0.193346679f),
             new(0.637357891f, 0.389521062f, 0.702844918f),
             new(1.01037765f, 0.390029252f, 0.702746332f),
             new(-0.637144327f, 0.674158931f, 0.621928453f),
             new(-1.0545764f, 0.674260557f, 0.621829867f),
             new(-1.0582062f, 0.67344743f, 0.620745778f),
             new(-1.08980727f, 0.656575501f, 0.596303284f),
             new(-1.09023428f, 0.659827888f, 0.594233513f),
             new(1.04582202f, 0.382304758f, 0.673770189f),
             new(1.09023428f, 0.656982064f, 0.593346536f),
             new(1.01400757f, 0.389673531f, 0.70146507f),
             new(-1.09044778f, 0.834950566f, 0.199999928f),
             new(-1.09023428f, 0.833781719f, 0.40135473f),
             new(-1.0584197f, 0.863663316f, -0.0124919862f),
             new(-1.0550034f, 0.865035474f, -0.0132804662f),
             new(1.0550034f, 0.863663316f, -0.0167300105f),
             new(-1.09002078f, 0.835204661f, 0.00219321251f),
             new(1.0584197f, 0.86234206f, -0.0161386579f),
             new(1.09023428f, 0.833781719f, -0.00135488808f),
             new(1.09066129f, 0.832155526f, 0.199999928f),
             new(1.0582062f, 0.67344743f, 0.620745778f),
             new(1.0545764f, 0.674260557f, 0.621829867f),
             new(0.637144327f, 0.674158931f, 0.621928453f),
             new(-1.0550034f, 0.677512944f, 0.619858742f),
             new(-1.0584197f, 0.67675066f, 0.618676066f),
             new(-1.0584197f, 0.86234206f, 0.41613853f),
             new(1.08980727f, 0.656575501f, 0.596303284f),
             new(1.09023428f, 0.659827888f, 0.594233513f),
             new(-1.09002078f, 0.835204661f, 0.397806644f),
             new(-1.05863321f, 0.863612533f, 0.199999928f),
             new(-1.0584197f, 0.863663316f, 0.412491858f),
             new(-1.0550034f, 0.865035474f, 0.413280308f),
             new(1.0550034f, 0.865035474f, -0.0132804662f),
             new(1.0584197f, 0.863663316f, -0.0124919862f),
             new(1.09002078f, 0.835204661f, 0.00219321251f),
             new(1.09044778f, 0.834950566f, 0.199999928f),
             new(1.09023428f, 0.833781719f, 0.40135473f),
             new(1.0584197f, 0.67675066f, 0.618676066f),
             new(1.0550034f, 0.677512944f, 0.619858742f),
             new(-1.0550034f, 0.863663316f, 0.416729867f),
             new(1.0584197f, 0.86234206f, 0.41613853f),
             new(1.0550034f, 0.865035474f, 0.413280308f),
             new(1.05863321f, 0.863612533f, 0.199999928f),
             new(1.09002078f, 0.835204661f, 0.397806644f),
             new(1.0584197f, 0.863663316f, 0.412491858f),
             new(1.0550034f, 0.8636633f, 0.41672987f),
        };
        return points;
    }

    Buffer<Vector3> CreatePlaneish()
    {
        var points = new Buffer<Vector3>(12, BufferPool);
        points[0] = new Vector3(-13.82f, 16.79f, 13.83f);
        points[1] = new Vector3(13.82f, -16.79f, -13.83f);
        points[2] = new Vector3(13.82f, 16.79f, -13.83f);
        points[3] = new Vector3(-13.82f, 16.79f, 13.83f);
        points[4] = new Vector3(13.82f, 16.79f, -13.83f);
        points[5] = new Vector3(-13.82f, -16.79f, 13.83f);
        points[6] = new Vector3(13.82f, 16.79f, -13.83f);
        points[7] = new Vector3(13.82f, -16.79f, -13.83f);
        points[8] = new Vector3(-13.82f, -16.79f, 13.83f);
        points[9] = new Vector3(-13.82f, 16.79f, 13.83f);
        points[10] = new Vector3(-13.82f, -16.79f, 13.83f);
        points[11] = new Vector3(13.82f, -16.79f, -13.83f);
        return points;
    }

    Buffer<Vector3> CreateDistantPlane()
    {
        var points = new Buffer<Vector3>(12, BufferPool);
        points[0] = new(-151.0875f, -2.2505488f, 102.17515f);
        points[1] = new(-151.10571f, 2.1121342f, -17.699797f);
        points[2] = new(-151.08746f, -2.2504745f, -17.699797f);
        points[3] = new(-151.10571f, 2.1121342f, -17.699797f);
        points[4] = new(-151.0875f, -2.2505488f, 102.17515f);
        points[5] = new(-151.10574f, 2.1120775f, 102.17517f);
        points[6] = new(-151.10571f, 2.1121342f, -17.699797f);
        points[7] = new(-151.10574f, 2.1120775f, 102.17517f);
        points[8] = new(-151.08746f, -2.2504745f, -17.699797f);
        points[9] = new(-151.08746f, -2.2504745f, -17.699797f);
        points[10] = new(-151.10574f, 2.1120775f, 102.17517f);
        points[11] = new(-151.0875f, -2.2505488f, 102.17515f);
        return points;
    }

    Buffer<Vector3> CreateMeshConvexHull(MeshContent meshContent, Vector3 scale)
    {
        //This is actually a pretty good example of how *not* to make a convex hull shape.
        //Generating it directly from a graphical data source tends to have way more surface complexity than needed,
        //and it tends to have a lot of near-but-not-quite-coplanar surfaces which can make the contact manifold less stable.
        //Prefer a simpler source with more distinct features, possibly created with an automated content-time tool.
        BufferPool.Take<Vector3>(meshContent.Triangles.Length * 3, out var points);
        for (int i = 0; i < meshContent.Triangles.Length; ++i)
        {
            ref var triangle = ref meshContent.Triangles[i];
            //resisting the urge to just reinterpret the memory
            points[i * 3 + 0] = triangle.A * scale;
            points[i * 3 + 1] = triangle.B * scale;
            points[i * 3 + 2] = triangle.C * scale;
        }
        return points;
    }

    Buffer<Vector3> CreateBoxConvexHull(float boxScale)
    {
        BufferPool.Take<Vector3>(8, out var points);
        points[0] = new Vector3(0, 0, 0);
        points[1] = new Vector3(0, 0, boxScale);
        points[2] = new Vector3(0, boxScale, 0);
        points[3] = new Vector3(0, boxScale, boxScale);
        points[4] = new Vector3(boxScale, 0, 0);
        points[5] = new Vector3(boxScale, 0, boxScale);
        points[6] = new Vector3(boxScale, boxScale, 0);
        points[7] = new Vector3(boxScale, boxScale, boxScale);
        return points;
    }

    //A couple of test point sets from PEEL: https://github.com/Pierre-Terdiman/PEEL_PhysX_Edition
    Buffer<Vector3> CreateTestConvexHull()
    {
        BufferPool.Take<Vector3>(50, out var vertices);
        vertices[0] = new Vector3(-0.000000f, -0.297120f, -0.000000f);
        vertices[1] = new Vector3(0.258819f, -0.297120f, 0.965926f);
        vertices[2] = new Vector3(-0.000000f, -0.297120f, 1.000000f);
        vertices[3] = new Vector3(0.500000f, -0.297120f, 0.866026f);
        vertices[4] = new Vector3(0.707107f, -0.297120f, 0.707107f);
        vertices[5] = new Vector3(0.866026f, -0.297120f, 0.500000f);
        vertices[6] = new Vector3(0.965926f, -0.297120f, 0.258819f);
        vertices[7] = new Vector3(1.000000f, -0.297120f, -0.000000f);
        vertices[8] = new Vector3(0.965926f, -0.297120f, -0.258819f);
        vertices[9] = new Vector3(0.866026f, -0.297120f, -0.500000f);
        vertices[10] = new Vector3(0.707107f, -0.297120f, -0.707107f);
        vertices[11] = new Vector3(0.500000f, -0.297120f, -0.866026f);
        vertices[12] = new Vector3(0.258819f, -0.297120f, -0.965926f);
        vertices[13] = new Vector3(-0.000000f, -0.297120f, -1.000000f);
        vertices[14] = new Vector3(-0.258819f, -0.297120f, -0.965926f);
        vertices[15] = new Vector3(-0.500000f, -0.297120f, -0.866025f);
        vertices[16] = new Vector3(-0.707107f, -0.297120f, -0.707107f);
        vertices[17] = new Vector3(-0.866026f, -0.297120f, -0.500000f);
        vertices[18] = new Vector3(-0.965926f, -0.297120f, -0.258819f);
        vertices[19] = new Vector3(-1.000000f, -0.297120f, 0.000000f);
        vertices[20] = new Vector3(-0.965926f, -0.297120f, 0.258819f);
        vertices[21] = new Vector3(-0.866025f, -0.297120f, 0.500000f);
        vertices[22] = new Vector3(-0.707107f, -0.297120f, 0.707107f);
        vertices[23] = new Vector3(-0.500000f, -0.297120f, 0.866026f);
        vertices[24] = new Vector3(-0.258819f, -0.297120f, 0.965926f);
        vertices[25] = new Vector3(-0.000000f, 0.297120f, -0.000000f);
        vertices[26] = new Vector3(-0.000000f, 0.297120f, 0.537813f);
        vertices[27] = new Vector3(0.139196f, 0.297120f, 0.519487f);
        vertices[28] = new Vector3(0.268907f, 0.297120f, 0.465760f);
        vertices[29] = new Vector3(0.380291f, 0.297120f, 0.380291f);
        vertices[30] = new Vector3(0.465760f, 0.297120f, 0.268907f);
        vertices[31] = new Vector3(0.519487f, 0.297120f, 0.139196f);
        vertices[32] = new Vector3(0.537813f, 0.297120f, -0.000000f);
        vertices[33] = new Vector3(0.519487f, 0.297120f, -0.139196f);
        vertices[34] = new Vector3(0.465760f, 0.297120f, -0.268907f);
        vertices[35] = new Vector3(0.380291f, 0.297120f, -0.380291f);
        vertices[36] = new Vector3(0.268907f, 0.297120f, -0.465760f);
        vertices[37] = new Vector3(0.139196f, 0.297120f, -0.519487f);
        vertices[38] = new Vector3(-0.000000f, 0.297120f, -0.537813f);
        vertices[39] = new Vector3(-0.139196f, 0.297120f, -0.519487f);
        vertices[40] = new Vector3(-0.268907f, 0.297120f, -0.465760f);
        vertices[41] = new Vector3(-0.380291f, 0.297120f, -0.380291f);
        vertices[42] = new Vector3(-0.465760f, 0.297120f, -0.268907f);
        vertices[43] = new Vector3(-0.519487f, 0.297120f, -0.139196f);
        vertices[44] = new Vector3(-0.537813f, 0.297120f, 0.000000f);
        vertices[45] = new Vector3(-0.519487f, 0.297120f, 0.139196f);
        vertices[46] = new Vector3(-0.465760f, 0.297120f, 0.268907f);
        vertices[47] = new Vector3(-0.380291f, 0.297120f, 0.380291f);
        vertices[48] = new Vector3(-0.268907f, 0.297120f, 0.465760f);
        vertices[49] = new Vector3(-0.139196f, 0.297120f, 0.519487f);
        return vertices;
    }

    Buffer<Vector3> CreateTestConvexHull2()
    {
        BufferPool.Take<Vector3>(120, out var vertices);
        vertices[0] = new Vector3(0.153478f, 0.993671f, 0.124687f);
        vertices[1] = new Vector3(0.153478f, 0.993671f, -0.117774f);
        vertices[2] = new Vector3(-0.147939f, 0.993671f, -0.117774f);
        vertices[3] = new Vector3(-0.147939f, 0.993671f, 0.124687f);
        vertices[4] = new Vector3(0.137286f, 0.817392f, 0.586192f);
        vertices[5] = new Vector3(0.333441f, 0.696161f, 0.661116f);
        vertices[6] = new Vector3(0.484149f, 0.789305f, 0.417265f);
        vertices[7] = new Vector3(0.287995f, 0.910536f, 0.342339f);
        vertices[8] = new Vector3(0.794945f, 0.410936f, 0.484838f);
        vertices[9] = new Vector3(0.916176f, 0.336012f, 0.288682f);
        vertices[10] = new Vector3(0.823033f, 0.579863f, 0.137973f);
        vertices[11] = new Vector3(0.701803f, 0.654787f, 0.334128f);
        vertices[12] = new Vector3(0.916176f, 0.336012f, -0.281770f);
        vertices[13] = new Vector3(0.794945f, 0.410936f, -0.477925f);
        vertices[14] = new Vector3(0.701803f, 0.654787f, -0.327216f);
        vertices[15] = new Vector3(0.823033f, 0.579863f, -0.131060f);
        vertices[16] = new Vector3(0.333441f, 0.696161f, -0.654204f);
        vertices[17] = new Vector3(0.137286f, 0.817392f, -0.579280f);
        vertices[18] = new Vector3(0.287995f, 0.910536f, -0.335426f);
        vertices[19] = new Vector3(0.484149f, 0.789305f, -0.410352f);
        vertices[20] = new Vector3(-0.131747f, 0.817392f, -0.579280f);
        vertices[21] = new Vector3(-0.327903f, 0.696161f, -0.654204f);
        vertices[22] = new Vector3(-0.478612f, 0.789305f, -0.410352f);
        vertices[23] = new Vector3(-0.282457f, 0.910536f, -0.335426f);
        vertices[24] = new Vector3(-0.789408f, 0.410936f, -0.477925f);
        vertices[25] = new Vector3(-0.910638f, 0.336012f, -0.281770f);
        vertices[26] = new Vector3(-0.817496f, 0.579863f, -0.131060f);
        vertices[27] = new Vector3(-0.696265f, 0.654787f, -0.327216f);
        vertices[28] = new Vector3(-0.910638f, 0.336012f, 0.288682f);
        vertices[29] = new Vector3(-0.789408f, 0.410936f, 0.484838f);
        vertices[30] = new Vector3(-0.696265f, 0.654787f, 0.334128f);
        vertices[31] = new Vector3(-0.817496f, 0.579863f, 0.137973f);
        vertices[32] = new Vector3(-0.327903f, 0.696161f, 0.661116f);
        vertices[33] = new Vector3(-0.131747f, 0.817392f, 0.586192f);
        vertices[34] = new Vector3(-0.282457f, 0.910536f, 0.342339f);
        vertices[35] = new Vector3(-0.478612f, 0.789305f, 0.417265f);
        vertices[36] = new Vector3(0.416578f, 0.478508f, 0.795634f);
        vertices[37] = new Vector3(0.341652f, 0.282353f, 0.916863f);
        vertices[38] = new Vector3(0.585505f, 0.131646f, 0.823721f);
        vertices[39] = new Vector3(0.660429f, 0.327801f, 0.702490f);
        vertices[40] = new Vector3(0.124000f, 0.147837f, 1.000000f);
        vertices[41] = new Vector3(-0.118461f, 0.147837f, 1.000000f);
        vertices[42] = new Vector3(-0.118461f, -0.153580f, 1.000000f);
        vertices[43] = new Vector3(0.124000f, -0.153580f, 1.000000f);
        vertices[44] = new Vector3(-0.336113f, 0.282353f, 0.916863f);
        vertices[45] = new Vector3(-0.411039f, 0.478508f, 0.795634f);
        vertices[46] = new Vector3(-0.654891f, 0.327801f, 0.702490f);
        vertices[47] = new Vector3(-0.579966f, 0.131646f, 0.823721f);
        vertices[48] = new Vector3(-0.993774f, 0.118359f, -0.147252f);
        vertices[49] = new Vector3(-0.993774f, -0.124103f, -0.147252f);
        vertices[50] = new Vector3(-0.993774f, -0.124103f, 0.154165f);
        vertices[51] = new Vector3(-0.993774f, 0.118359f, 0.154165f);
        vertices[52] = new Vector3(-0.817496f, -0.585607f, 0.137973f);
        vertices[53] = new Vector3(-0.696265f, -0.660531f, 0.334128f);
        vertices[54] = new Vector3(-0.789408f, -0.416680f, 0.484838f);
        vertices[55] = new Vector3(-0.910638f, -0.341756f, 0.288682f);
        vertices[56] = new Vector3(-0.411039f, -0.484253f, 0.795634f);
        vertices[57] = new Vector3(-0.336113f, -0.288097f, 0.916863f);
        vertices[58] = new Vector3(-0.579966f, -0.137388f, 0.823721f);
        vertices[59] = new Vector3(-0.654891f, -0.333543f, 0.702490f);
        vertices[60] = new Vector3(0.341652f, -0.288097f, 0.916863f);
        vertices[61] = new Vector3(0.416578f, -0.484253f, 0.795634f);
        vertices[62] = new Vector3(0.660429f, -0.333543f, 0.702490f);
        vertices[63] = new Vector3(0.585505f, -0.137388f, 0.823721f);
        vertices[64] = new Vector3(0.333441f, -0.701905f, 0.661116f);
        vertices[65] = new Vector3(0.137286f, -0.823136f, 0.586192f);
        vertices[66] = new Vector3(0.287995f, -0.916278f, 0.342339f);
        vertices[67] = new Vector3(0.484149f, -0.795049f, 0.417265f);
        vertices[68] = new Vector3(-0.131747f, -0.823136f, 0.586192f);
        vertices[69] = new Vector3(-0.327903f, -0.701905f, 0.661116f);
        vertices[70] = new Vector3(-0.478612f, -0.795049f, 0.417265f);
        vertices[71] = new Vector3(-0.282457f, -0.916278f, 0.342339f);
        vertices[72] = new Vector3(-0.910638f, -0.341756f, -0.281770f);
        vertices[73] = new Vector3(-0.789408f, -0.416680f, -0.477925f);
        vertices[74] = new Vector3(-0.696265f, -0.660531f, -0.327216f);
        vertices[75] = new Vector3(-0.817496f, -0.585607f, -0.131060f);
        vertices[76] = new Vector3(-0.327903f, -0.701905f, -0.654204f);
        vertices[77] = new Vector3(-0.131747f, -0.823136f, -0.579280f);
        vertices[78] = new Vector3(-0.282457f, -0.916278f, -0.335426f);
        vertices[79] = new Vector3(-0.478612f, -0.795049f, -0.410352f);
        vertices[80] = new Vector3(0.153478f, -0.999415f, -0.117774f);
        vertices[81] = new Vector3(0.153478f, -0.999415f, 0.124687f);
        vertices[82] = new Vector3(-0.147939f, -0.999415f, 0.124687f);
        vertices[83] = new Vector3(-0.147939f, -0.999415f, -0.117774f);
        vertices[84] = new Vector3(0.701803f, -0.660531f, 0.334128f);
        vertices[85] = new Vector3(0.823033f, -0.585607f, 0.137973f);
        vertices[86] = new Vector3(0.916176f, -0.341756f, 0.288682f);
        vertices[87] = new Vector3(0.794945f, -0.416680f, 0.484838f);
        vertices[88] = new Vector3(0.823033f, -0.585607f, -0.131060f);
        vertices[89] = new Vector3(0.701803f, -0.660531f, -0.327216f);
        vertices[90] = new Vector3(0.794945f, -0.416680f, -0.477925f);
        vertices[91] = new Vector3(0.916176f, -0.341756f, -0.281770f);
        vertices[92] = new Vector3(0.484149f, -0.795049f, -0.410352f);
        vertices[93] = new Vector3(0.287995f, -0.916278f, -0.335426f);
        vertices[94] = new Vector3(0.137286f, -0.823136f, -0.579280f);
        vertices[95] = new Vector3(0.333441f, -0.701905f, -0.654204f);
        vertices[96] = new Vector3(-0.654891f, -0.333543f, -0.695578f);
        vertices[97] = new Vector3(-0.579966f, -0.137388f, -0.816807f);
        vertices[98] = new Vector3(-0.336113f, -0.288097f, -0.909951f);
        vertices[99] = new Vector3(-0.411039f, -0.484253f, -0.788719f);
        vertices[100] = new Vector3(-0.118461f, 0.147837f, -0.993087f);
        vertices[101] = new Vector3(0.124000f, 0.147837f, -0.993087f);
        vertices[102] = new Vector3(0.124000f, -0.153580f, -0.993087f);
        vertices[103] = new Vector3(-0.118461f, -0.153580f, -0.993087f);
        vertices[104] = new Vector3(0.585505f, -0.137388f, -0.816807f);
        vertices[105] = new Vector3(0.660429f, -0.333543f, -0.695578f);
        vertices[106] = new Vector3(0.416578f, -0.484253f, -0.788719f);
        vertices[107] = new Vector3(0.341652f, -0.288097f, -0.909951f);
        vertices[108] = new Vector3(0.999313f, -0.124103f, -0.147252f);
        vertices[109] = new Vector3(0.999313f, 0.118359f, -0.147252f);
        vertices[110] = new Vector3(0.999313f, 0.118359f, 0.154165f);
        vertices[111] = new Vector3(0.999313f, -0.124103f, 0.154165f);
        vertices[112] = new Vector3(0.660429f, 0.327801f, -0.695578f);
        vertices[113] = new Vector3(0.585505f, 0.131646f, -0.816807f);
        vertices[114] = new Vector3(0.341652f, 0.282353f, -0.909951f);
        vertices[115] = new Vector3(0.416578f, 0.478508f, -0.788719f);
        vertices[116] = new Vector3(-0.579966f, 0.131646f, -0.816807f);
        vertices[117] = new Vector3(-0.654891f, 0.327801f, -0.695578f);
        vertices[118] = new Vector3(-0.411039f, 0.478508f, -0.788719f);
        vertices[119] = new Vector3(-0.336113f, 0.282353f, -0.909951f);
        return vertices;
    }


    Buffer<Vector3> CreateTestConvexHull3()
    {
        BufferPool.Take<Vector3>(22, out var vertices);
        vertices[0] = new Vector3(-0.103558f, 1.000000f, -0.490575f);
        vertices[1] = new Vector3(0.266493f, 0.659794f, -0.363751f);
        vertices[2] = new Vector3(-0.245774f, 0.762636f, -0.615304f);
        vertices[3] = new Vector3(0.164688f, -0.777634f, -0.365919f);
        vertices[4] = new Vector3(0.503268f, -0.846406f, -0.131286f);
        vertices[5] = new Vector3(0.171066f, -0.931723f, -0.140738f);
        vertices[6] = new Vector3(-0.247963f, -0.738059f, -0.413146f);
        vertices[7] = new Vector3(-0.319203f, -0.260078f, -0.609331f);
        vertices[8] = new Vector3(0.469624f, -0.747848f, -0.286486f);
        vertices[9] = new Vector3(0.398526f, -0.238233f, -0.435281f);
        vertices[10] = new Vector3(0.448274f, 0.295416f, -0.246327f);
        vertices[11] = new Vector3(-0.245774f, 0.762636f, 0.596521f);
        vertices[12] = new Vector3(0.266493f, 0.659794f, 0.344974f);
        vertices[13] = new Vector3(-0.103558f, 1.000000f, 0.471792f);
        vertices[14] = new Vector3(0.171066f, -0.931723f, 0.121961f);
        vertices[15] = new Vector3(0.503268f, -0.846406f, 0.112509f);
        vertices[16] = new Vector3(0.164688f, -0.777634f, 0.347137f);
        vertices[17] = new Vector3(-0.319203f, -0.260078f, 0.590548f);
        vertices[18] = new Vector3(-0.247963f, -0.738059f, 0.394364f);
        vertices[19] = new Vector3(0.469624f, -0.747848f, 0.267709f);
        vertices[20] = new Vector3(0.398526f, -0.238233f, 0.416498f);
        vertices[21] = new Vector3(0.448274f, 0.295411f, 0.227550f);
        return vertices;
    }

    Buffer<Vector3> CreateJSONSourcedConvexHull(string filePath)
    {
        //ChatGPT wrote this, of course.
        List<Vector3> points = new List<Vector3>();
        if (File.Exists(filePath))
        {
            string jsonContent = File.ReadAllText(filePath);
            List<List<double>> rawPoints = JsonSerializer.Deserialize<List<List<double>>>(jsonContent);
            foreach (List<double> point in rawPoints)
            {
                if (point.Count == 3)
                {
                    Vector3 vector3Point = new Vector3((float)point[0], (float)point[1], (float)point[2]);
                    points.Add(vector3Point);
                }
            }
        }
        else
        {
            Console.Error.WriteLine("File not found: " + filePath);
        }

        BufferPool.Take(points.Count, out Buffer<Vector3> buffer);
        for (int i = 0; i < buffer.Length; ++i)
        {
            buffer[i] = points[i];
        }
        return buffer;
    }

    void CreateHellCubeFace(int widthInPoints, float step, Buffer<Vector3> facePoints, Matrix3x3 transform, float localFaceOffset)
    {
        var offset = step * (widthInPoints - 1) * 0.5f;
        for (int i = 0; i < widthInPoints; ++i)
        {
            var x = i * step - offset;
            for (int j = 0; j < widthInPoints; ++j)
            {
                var y = j * step - offset;
                var localOffset = new Vector3(x, y, localFaceOffset);
                Matrix3x3.Transform(localOffset, transform, out var worldOffset);
                facePoints[i * widthInPoints + j] = worldOffset;
            }
        }
    }
    Buffer<Vector3> CreateHellCube(int widthInPoints)
    {
        var facePointCount = widthInPoints * widthInPoints;
        BufferPool.Take(facePointCount * 6, out Buffer<Vector3> buffer);
        var size = 8f;
        var halfSize = size / 2;
        var step = size / (widthInPoints - 1);
        Matrix3x3.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1f, 1, 1f)), float.Pi / 2, out var cubeTransform);
        Matrix3x3.CreateFromAxisAngle(new Vector3(0, 1, 0), float.Pi / 2, out var zFace);
        zFace *= cubeTransform;
        Matrix3x3.CreateFromAxisAngle(new Vector3(0, 1, 0), float.Pi, out var localFace2);
        Matrix3x3.CreateFromAxisAngle(new Vector3(1, 0, 0), -float.Pi / 2, out var yFace);
        yFace *= cubeTransform;
        CreateHellCubeFace(widthInPoints, step, buffer.Slice(facePointCount * 0, facePointCount), cubeTransform, halfSize);
        CreateHellCubeFace(widthInPoints, step, buffer.Slice(facePointCount * 1, facePointCount), zFace, halfSize);
        CreateHellCubeFace(widthInPoints, step, buffer.Slice(facePointCount * 2, facePointCount), cubeTransform, -halfSize);
        CreateHellCubeFace(widthInPoints, step, buffer.Slice(facePointCount * 3, facePointCount), zFace, -halfSize);
        CreateHellCubeFace(widthInPoints, step, buffer.Slice(facePointCount * 4, facePointCount), yFace, halfSize);
        CreateHellCubeFace(widthInPoints, step, buffer.Slice(facePointCount * 5, facePointCount), yFace, -halfSize);
        return buffer;
    }

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, -2.5f, 10);
        camera.Yaw = 0;
        camera.Pitch = 0;

        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -0, 0)), new SolveDescription(8, 1));
        //var hullPoints = CreateJSONSourcedConvexHull(@"Content/testHull.json");
        //for (int i = 0; i < hullPoints.Length; ++i)
        //{
        //    hullPoints[i] *= 0.03f;
        //}
        var hullPoints = CreateRandomConvexHullPoints();
        //var hullPoints = CreateMeshConvexHull(content.Load<MeshContent>(@"Content\newt.obj"), new Vector3(1, 1.5f, 1f));
        //var hullPoints = CreateHellCube(200);
        //var hullPoints = CreateBwaa();
        //var hullPoints = CreatePlaneish();
        //var hullPoints = CreateDistantPlane();
        //var hullPoints = CreateTestConvexHull();
        //var hullPoints = CreateTestConvexHull2();
        //var hullPoints = CreateTestConvexHull3();
        //var hullPoints = CreateBoxConvexHull(2);
        var hullShape = new ConvexHull(hullPoints, BufferPool, out _);
        //float largestError = 0;
        //for (int i = 0; i < hullShape.FaceToVertexIndicesStart.Length; ++i)
        //{
        //    hullShape.GetVertexIndicesForFace(i, out var faceVertices);
        //    BundleIndexing.GetBundleIndices(i, out var normalBundleIndex, out var normalIndexInBundle);
        //    Vector3Wide.ReadSlot(ref hullShape.BoundingPlanes[normalBundleIndex].Normal, normalIndexInBundle, out var faceNormal);
        //    var offset = hullShape.BoundingPlanes[normalBundleIndex].Offset[normalIndexInBundle];
        //    Console.WriteLine($"Face {i} errors:");
        //    for (int j = 0; j < faceVertices.Length; ++j)
        //    {
        //        hullShape.GetPoint(faceVertices[j], out var point);
        //        var error = Vector3.Dot(point, faceNormal) - offset;
        //        Console.WriteLine($"v{j}: {error}");
        //        largestError = MathF.Max(MathF.Abs(error), largestError);
        //    }
        //}
        //Console.WriteLine($"Largest error: {largestError}");

        ConvexHullHelper.ComputeHull(hullPoints, BufferPool, out var hullData, out debugSteps);
        this.points = new Vector3[hullPoints.Length];
        hullPoints.CopyTo(0, this.points, 0, this.points.Length);
        //this.points = hullPoints;

        var boxHullPoints = CreateBoxConvexHull(2);
        var boxHullShape = new ConvexHull(boxHullPoints, BufferPool, out _);

        //Matrix3x3.CreateScale(new Vector3(5, 0.5f, 3), out var scale);
        //var transform = Matrix3x3.CreateFromAxisAngle(Vector3.Normalize(new Vector3(3, 2, 1)), 1207) * scale;
        //const int transformCount = 10000;
        //var transformStart = Stopwatch.GetTimestamp();
        //for (int i = 0; i < transformCount; ++i)
        //{
        //    CreateTransformedCopy(hullShape, transform, BufferPool, out var transformedHullShape);
        //    transformedHullShape.Dispose(BufferPool);
        //}
        //var transformEnd = Stopwatch.GetTimestamp();
        //Console.WriteLine($"Transform hull computation time (us): {(transformEnd - transformStart) * 1e6 / (transformCount * Stopwatch.Frequency)}");

        //hullShape.RayTest(RigidPose.Identity, new Vector3(0, 1, 0), -Vector3.UnitY, out var t, out var normal);

        //const int rayIterationCount = 10000;
        //var rayPose = RigidPose.Identity;
        //var rayOrigin = new Vector3(0, 2, 0);
        //var rayDirection = new Vector3(0, -1, 0);

        //int hitCounter = 0;
        //var start = Stopwatch.GetTimestamp();
        //for (int i = 0; i < rayIterationCount; ++i)
        //{
        //    if (hullShape.RayTest(rayPose, rayOrigin, rayDirection, out _, out _))
        //    {
        //        ++hitCounter;
        //    }
        //}
        //var end = Stopwatch.GetTimestamp();
        //Console.WriteLine($"Hit counter: {hitCounter}, computation time (us): {(end - start) * 1e6 / (rayIterationCount * Stopwatch.Frequency)}");

        //const int iterationCount = 100;
        //start = Stopwatch.GetTimestamp();
        //for (int i = 0; i < iterationCount; ++i)
        //{
        //    CreateShape(hullPoints, BufferPool, out _, out var perfTestShape);
        //    perfTestShape.Dispose(BufferPool);
        //}
        //end = Stopwatch.GetTimestamp();
        //Console.WriteLine($"Hull computation time (us): {(end - start) * 1e6 / (iterationCount * Stopwatch.Frequency)}");

        var hullShapeIndex = Simulation.Shapes.Add(hullShape);
        var boxHullShapeIndex = Simulation.Shapes.Add(boxHullShape);
        var inertia = hullShape.ComputeInertia(1);
        Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(0, 0, 5), inertia, new(hullShapeIndex, 20, 20), -0.01f));
        Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(0, 0, 3), boxHullShape.ComputeInertia(1), new(boxHullShapeIndex, 20, 20), -0.01f));

        Simulation.Statics.Add(new StaticDescription(new Vector3(-25, -5, 0), Simulation.Shapes.Add(new Sphere(2))));
        Simulation.Statics.Add(new StaticDescription(new Vector3(-20, -5, 0), Simulation.Shapes.Add(new Capsule(0.5f, 2))));
        Simulation.Statics.Add(new StaticDescription(new Vector3(-15, -5, 0), Simulation.Shapes.Add(new Box(2f, 2f, 2f))));
        Simulation.Statics.Add(new StaticDescription(new Vector3(-10, -5, 5), Simulation.Shapes.Add(new Triangle { A = new Vector3(0, 0, -10), B = new Vector3(5, 0, -10), C = new Vector3(0, 0, -5) })));
        Simulation.Statics.Add(new StaticDescription(new Vector3(-5, -5, 0), Simulation.Shapes.Add(new Cylinder(1, 1))));
        Simulation.Statics.Add(new StaticDescription(new Vector3(-5, -5, 5), Simulation.Shapes.Add(new Cylinder(1, 1))));
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, 0), hullShapeIndex));
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, 5), Simulation.Shapes.Add(boxHullShape)));

        var spacing = new Vector3(3f, 3f, 3);
        int width = 16;
        int height = 16;
        int length = 0;
        var origin = -0.5f * spacing * new Vector3(width, 0, length) + new Vector3(40, 0.2f, -40);
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                for (int k = 0; k < length; ++k)
                {
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(
                        (origin + spacing * new Vector3(i, j, k), QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathHelper.Pi * 0.05f)),
                        inertia, hullShapeIndex, 0.01f));
                }
            }
        }
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -10, 0), Simulation.Shapes.Add(new Box(1000, 1, 1000))));

        Random random = new Random(5);
        var mesh = DemoMeshHelper.CreateDeformedPlane(64, 64, (x, y) => new Vector3(
            x + 8,
            2f * MathF.Sin(x * 0.125f) * MathF.Sin(y * 0.125f) + 0.1f * random.NextSingle() - 3,
            y - 8), new Vector3(1, 1, 1), BufferPool);
        Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(mesh)));
    }

    void TestConvexHullCreation()
    {
        var random = new Random(5);
        for (int iterationIndex = 0; iterationIndex < 100000; ++iterationIndex)
        {
            const int pointCount = 32;
            var points = new QuickList<Vector3>(pointCount, BufferPool);
            for (int i = 0; i < pointCount; ++i)
            {
                points.AllocateUnsafely() = new Vector3(1 * random.NextSingle(), 2 * random.NextSingle(), 3 * random.NextSingle());
            }

            var pointsBuffer = points.Span.Slice(points.Count);
            CreateShape(pointsBuffer, BufferPool, out _, out var hullShape);

            hullShape.Dispose(BufferPool);
        }
    }

    //Buffer<Vector3> points;
    Vector3[] points;
    List<DebugStep> debugSteps;

    int stepIndex = 0;

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        if (input.TypedCharacters.Contains('x'))
        {
            stepIndex = Math.Max(stepIndex - 1, 0);
        }
        if (input.TypedCharacters.Contains('c'))
        {
            stepIndex = Math.Min(stepIndex + 1, debugSteps.Count - 1);
        }
        if (input.WasPushed(OpenTK.Input.Key.P))
        {
            showWireframe = !showWireframe;
        }
        if (input.WasPushed(OpenTK.Input.Key.U))
        {
            showDeleted = !showDeleted;
        }
        if (input.WasPushed(OpenTK.Input.Key.Y))
        {
            showVertexIndices = !showVertexIndices;
        }
        if (input.WasPushed(OpenTK.Input.Key.H))
        {
            showFaceVertexStatuses = !showFaceVertexStatuses;
        }
        base.Update(window, camera, input, dt);
    }

    bool showWireframe;
    bool showDeleted;
    bool showVertexIndices;
    bool showFaceVertexStatuses = true;
    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        var step = debugSteps[stepIndex];
        var scale = 15f;
        var renderOffset = new Vector3(-15, 25, 0);

        void DrawVertexIndex(int i, Vector3 color, Vector2 offset = default)
        {
            if (DemoRenderer.Helpers.GetScreenLocation(points[i] * scale + renderOffset, camera.ViewProjection, renderer.Surface.Resolution, out var location))
            {
                float fontSize = 10;
                float spacing = 12;
                renderer.TextBatcher.Write(text.Clear().Append(i), location + offset * spacing, fontSize, color, font);
            }
        }

        for (int i = 0; i < points.Length; ++i)
        {
            var pose = new RigidPose(renderOffset + points[i] * scale);
            renderer.Shapes.AddShape(new Box(0.1f, 0.1f, 0.1f), Simulation.Shapes, pose, new Vector3(0.5f, 0.5f, 0.5f));
            if (!step.AllowVertex[i] && showFaceVertexStatuses)
            {
                var color = new Vector3(1, 0, 0);
                renderer.Shapes.AddShape(new Box(0.6f, 0.25f, 0.25f), Simulation.Shapes, pose, color);
                if (showVertexIndices)
                    DrawVertexIndex(i, color, new Vector2(0, 1));
            }
        }
        if (showFaceVertexStatuses)
        {
            for (int i = 0; i < step.Raw.Length; ++i)
            {
                var color = new Vector3(0.3f, 0.3f, 1);
                var pose = new RigidPose(renderOffset + points[step.Raw[i]] * scale);
                renderer.Shapes.AddShape(new Box(0.25f, 0.6f, 0.25f), Simulation.Shapes, pose, color);
                if (showVertexIndices)
                    DrawVertexIndex(step.Raw[i], color, new Vector2(0, 2));
            }
            for (int i = 0; i < step.Reduced.Length; ++i)
            {
                var color = new Vector3(0, 1, 0);
                var pose = new RigidPose(renderOffset + points[step.Reduced[i]] * scale);
                renderer.Shapes.AddShape(new Box(0.25f, 0.25f, 0.6f), Simulation.Shapes, pose, color);
                if (showVertexIndices)
                    DrawVertexIndex(step.Reduced[i], color, new Vector2(0, 3));
            }
        }

        {
            var pose = new RigidPose(renderOffset);
            void DrawFace(DebugStep step, int[] reduced, bool deleted, int i)
            {
                var color = deleted ? new Vector3(0.25f, 0.25f, 0.25f) : stepIndex == i ? new Vector3(1, 0, 0.5f) : new Vector3(1, 0, 1);
                var deletionInducedScale = deleted ? new Vector3(1.1f) : new Vector3(1f);

                var offset = deleted ? step.FaceNormal * 0.25f : new Vector3();
                if (showWireframe)
                {
                    var previousIndex = reduced.Length - 1;
                    for (int q = 0; q < reduced.Length; ++q)
                    {
                        var a = points[reduced[q]] * scale + pose.Position + offset;
                        var b = points[reduced[previousIndex]] * scale + pose.Position + offset;
                        previousIndex = q;
                        renderer.Lines.Allocate() = new LineInstance(a, b, color, Vector3.Zero);
                    }
                }
                else
                {
                    for (int k = 2; k < reduced.Length; ++k)
                    {
                        renderer.Shapes.AddShape(new Triangle
                        {
                            A = points[reduced[0]] * scale + offset,
                            B = points[reduced[k]] * scale + offset,
                            C = points[reduced[k - 1]] * scale + offset
                        }, Simulation.Shapes, pose, color);
                    }
                }
            }
            for (int i = 0; i <= stepIndex; ++i)
            {
                var localStep = debugSteps[i];
                DrawFace(localStep, localStep.Reduced, false, i);
                if (localStep.OverwrittenOriginal != null)
                {
                    DrawFace(localStep, localStep.OverwrittenOriginal, true, i);
                }

            }
        }
        Console.WriteLine($"face count: {step.FaceStarts.Count}");

        if (showVertexIndices)
        {
            for (int i = 0; i < points.Length; ++i)
            {
                DrawVertexIndex(i, Vector3.One);
            }
        }

        var edgeMidpoint = renderOffset + (points[step.SourceEdge.A] + points[step.SourceEdge.B]) * scale * 0.5f;
        renderer.Lines.Allocate() = new LineInstance(edgeMidpoint, edgeMidpoint + step.BasisX * scale * 0.5f, new Vector3(1, 1, 0), new Vector3());
        renderer.Lines.Allocate() = new LineInstance(edgeMidpoint, edgeMidpoint + step.BasisY * scale * 0.5f, new Vector3(0, 1, 0), new Vector3());
        renderer.TextBatcher.Write(
            text.Clear().Append($"Enumerate step with X and C. Current step: ").Append(stepIndex + 1).Append(" out of ").Append(debugSteps.Count),
            new Vector2(32, renderer.Surface.Resolution.Y - 140), 20, new Vector3(1), font);
        renderer.TextBatcher.Write(text.Clear().Append("Show wireframe: P ").Append(showWireframe ? "(on)" : "(off)"), new Vector2(32, renderer.Surface.Resolution.Y - 120), 20, new Vector3(1), font);
        renderer.TextBatcher.Write(text.Clear().Append("Show deleted: U ").Append(showDeleted ? "(on)" : "(off)"), new Vector2(32, renderer.Surface.Resolution.Y - 100), 20, new Vector3(1), font);
        renderer.TextBatcher.Write(text.Clear().Append("Show vertex indices: Y ").Append(showVertexIndices ? "(on)" : "(off)"), new Vector2(32, renderer.Surface.Resolution.Y - 80), 20, new Vector3(1), font);
        renderer.TextBatcher.Write(text.Clear().Append("Show face vertex statuses: H ").Append(showFaceVertexStatuses ? "(on)" : "(off)"), new Vector2(32, renderer.Surface.Resolution.Y - 60), 20, new Vector3(1), font);
        renderer.TextBatcher.Write(text.Clear().Append("Face count: ").Append(step.FaceStarts.Count), new Vector2(32, renderer.Surface.Resolution.Y - 20), 20, new Vector3(1), font);


        base.Render(renderer, camera, input, text, font);
    }
}
