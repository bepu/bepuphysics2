// Enabling DEBUG_STEPS on this test requires the same define within ConvexHullHelper.cs.
#define DEBUG_STEPS
using System;
using System.Collections.Generic;
using System.Numerics;
using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using static BepuPhysics.Collidables.ConvexHullHelper;
using BepuUtilities;
using BepuPhysics.Constraints;
using BepuUtilities.Memory;
using System.Text.Json;
using System.IO;
using DemoRenderer.Constraints;
using DemoUtilities;
using DemoRenderer.UI;
using System.Diagnostics;



namespace Demos.SpecializedTests;

public class ConvexHullTestDemo : Demo
{
    Vector3[] CreateRandomConvexHullPoints()
    {
        var points = new Vector3[50];
        var random = new Random(5);
        for (int i = 0; i < points.Length; ++i)
        {
            points[i] = new(3 * random.NextSingle(), 1 * random.NextSingle(), 3 * random.NextSingle());
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

    Vector3[] CreatePlaneish()
    {
        var points = new Vector3[]
        {
            new(-13.82f, 16.79f, 13.83f),
            new(13.82f, -16.79f, -13.83f),
            new(13.82f, 16.79f, -13.83f),
            new(-13.82f, 16.79f, 13.83f),
            new(13.82f, 16.79f, -13.83f),
            new(-13.82f, -16.79f, 13.83f),
            new(13.82f, 16.79f, -13.83f),
            new(13.82f, -16.79f, -13.83f),
            new(-13.82f, -16.79f, 13.83f),
            new(-13.82f, 16.79f, 13.83f),
            new(-13.82f, -16.79f, 13.83f),
            new(13.82f, -16.79f, -13.83f),
        };
        return points;
    }

    Vector3[] CreateDistantPlane()
    {
        var points = new Vector3[]
        {
            new(-151.0875f, -2.2505488f, 102.17515f),
            new(-151.10571f, 2.1121342f, -17.699797f),
            new(-151.08746f, -2.2504745f, -17.699797f),
            new(-151.10571f, 2.1121342f, -17.699797f),
            new(-151.0875f, -2.2505488f, 102.17515f),
            new(-151.10574f, 2.1120775f, 102.17517f),
            new(-151.10571f, 2.1121342f, -17.699797f),
            new(-151.10574f, 2.1120775f, 102.17517f),
            new(-151.08746f, -2.2504745f, -17.699797f),
            new(-151.08746f, -2.2504745f, -17.699797f),
            new(-151.10574f, 2.1120775f, 102.17517f),
            new(-151.0875f, -2.2505488f, 102.17515f),
        };
        return points;
    }

    Vector3[] CreateMeshConvexHull(MeshContent meshContent, Vector3 scale)
    {
        //This is actually a pretty good example of how *not* to make a convex hull shape.
        //Generating it directly from a graphical data source tends to have way more surface complexity than needed,
        //and it tends to have a lot of near-but-not-quite-coplanar surfaces which can make the contact manifold less stable.
        //Prefer a simpler source with more distinct features, possibly created with an automated content-time tool.
        var points = new Vector3[meshContent.Triangles.Length * 3];
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

    Vector3[] CreateBoxConvexHull(float boxScale)
    {
        var points = new Vector3[]
        {
            new(0, 0, 0),
            new(0, 0, boxScale),
            new(0, boxScale, 0),
            new(0, boxScale, boxScale),
            new(boxScale, 0, 0),
            new(boxScale, 0, boxScale),
            new(boxScale, boxScale, 0),
            new(boxScale, boxScale, boxScale),
        };
        return points;
    }

    //A couple of test point sets from PEEL: https://github.com/Pierre-Terdiman/PEEL_PhysX_Edition
    Vector3[] CreateTestConvexHull()
    {
        var vertices = new Vector3[]
        {
            new(-0.000000f, -0.297120f, -0.000000f),
            new(0.258819f, -0.297120f, 0.965926f),
            new(-0.000000f, -0.297120f, 1.000000f),
            new(0.500000f, -0.297120f, 0.866026f),
            new(0.707107f, -0.297120f, 0.707107f),
            new(0.866026f, -0.297120f, 0.500000f),
            new(0.965926f, -0.297120f, 0.258819f),
            new(1.000000f, -0.297120f, -0.000000f),
            new(0.965926f, -0.297120f, -0.258819f),
            new(0.866026f, -0.297120f, -0.500000f),
            new(0.707107f, -0.297120f, -0.707107f),
            new(0.500000f, -0.297120f, -0.866026f),
            new(0.258819f, -0.297120f, -0.965926f),
            new(-0.000000f, -0.297120f, -1.000000f),
            new(-0.258819f, -0.297120f, -0.965926f),
            new(-0.500000f, -0.297120f, -0.866025f),
            new(-0.707107f, -0.297120f, -0.707107f),
            new(-0.866026f, -0.297120f, -0.500000f),
            new(-0.965926f, -0.297120f, -0.258819f),
            new(-1.000000f, -0.297120f, 0.000000f),
            new(-0.965926f, -0.297120f, 0.258819f),
            new(-0.866025f, -0.297120f, 0.500000f),
            new(-0.707107f, -0.297120f, 0.707107f),
            new(-0.500000f, -0.297120f, 0.866026f),
            new(-0.258819f, -0.297120f, 0.965926f),
            new(-0.000000f, 0.297120f, -0.000000f),
            new(-0.000000f, 0.297120f, 0.537813f),
            new(0.139196f, 0.297120f, 0.519487f),
            new(0.268907f, 0.297120f, 0.465760f),
            new(0.380291f, 0.297120f, 0.380291f),
            new(0.465760f, 0.297120f, 0.268907f),
            new(0.519487f, 0.297120f, 0.139196f),
            new(0.537813f, 0.297120f, -0.000000f),
            new(0.519487f, 0.297120f, -0.139196f),
            new(0.465760f, 0.297120f, -0.268907f),
            new(0.380291f, 0.297120f, -0.380291f),
            new(0.268907f, 0.297120f, -0.465760f),
            new(0.139196f, 0.297120f, -0.519487f),
            new(-0.000000f, 0.297120f, -0.537813f),
            new(-0.139196f, 0.297120f, -0.519487f),
            new(-0.268907f, 0.297120f, -0.465760f),
            new(-0.380291f, 0.297120f, -0.380291f),
            new(-0.465760f, 0.297120f, -0.268907f),
            new(-0.519487f, 0.297120f, -0.139196f),
            new(-0.537813f, 0.297120f, 0.000000f),
            new(-0.519487f, 0.297120f, 0.139196f),
            new(-0.465760f, 0.297120f, 0.268907f),
            new(-0.380291f, 0.297120f, 0.380291f),
            new(-0.268907f, 0.297120f, 0.465760f),
            new(-0.139196f, 0.297120f, 0.519487f),
        };
        return vertices;
    }

    Vector3[] CreateTestConvexHull2()
    {
        var vertices = new Vector3[]
        {
            new(0.153478f, 0.993671f, 0.124687f),
            new(0.153478f, 0.993671f, -0.117774f),
            new(-0.147939f, 0.993671f, -0.117774f),
            new(-0.147939f, 0.993671f, 0.124687f),
            new(0.137286f, 0.817392f, 0.586192f),
            new(0.333441f, 0.696161f, 0.661116f),
            new(0.484149f, 0.789305f, 0.417265f),
            new(0.287995f, 0.910536f, 0.342339f),
            new(0.794945f, 0.410936f, 0.484838f),
            new(0.916176f, 0.336012f, 0.288682f),
            new(0.823033f, 0.579863f, 0.137973f),
            new(0.701803f, 0.654787f, 0.334128f),
            new(0.916176f, 0.336012f, -0.281770f),
            new(0.794945f, 0.410936f, -0.477925f),
            new(0.701803f, 0.654787f, -0.327216f),
            new(0.823033f, 0.579863f, -0.131060f),
            new(0.333441f, 0.696161f, -0.654204f),
            new(0.137286f, 0.817392f, -0.579280f),
            new(0.287995f, 0.910536f, -0.335426f),
            new(0.484149f, 0.789305f, -0.410352f),
            new(-0.131747f, 0.817392f, -0.579280f),
            new(-0.327903f, 0.696161f, -0.654204f),
            new(-0.478612f, 0.789305f, -0.410352f),
            new(-0.282457f, 0.910536f, -0.335426f),
            new(-0.789408f, 0.410936f, -0.477925f),
            new(-0.910638f, 0.336012f, -0.281770f),
            new(-0.817496f, 0.579863f, -0.131060f),
            new(-0.696265f, 0.654787f, -0.327216f),
            new(-0.910638f, 0.336012f, 0.288682f),
            new(-0.789408f, 0.410936f, 0.484838f),
            new(-0.696265f, 0.654787f, 0.334128f),
            new(-0.817496f, 0.579863f, 0.137973f),
            new(-0.327903f, 0.696161f, 0.661116f),
            new(-0.131747f, 0.817392f, 0.586192f),
            new(-0.282457f, 0.910536f, 0.342339f),
            new(-0.478612f, 0.789305f, 0.417265f),
            new(0.416578f, 0.478508f, 0.795634f),
            new(0.341652f, 0.282353f, 0.916863f),
            new(0.585505f, 0.131646f, 0.823721f),
            new(0.660429f, 0.327801f, 0.702490f),
            new(0.124000f, 0.147837f, 1.000000f),
            new(-0.118461f, 0.147837f, 1.000000f),
            new(-0.118461f, -0.153580f, 1.000000f),
            new(0.124000f, -0.153580f, 1.000000f),
            new(-0.336113f, 0.282353f, 0.916863f),
            new(-0.411039f, 0.478508f, 0.795634f),
            new(-0.654891f, 0.327801f, 0.702490f),
            new(-0.579966f, 0.131646f, 0.823721f),
            new(-0.993774f, 0.118359f, -0.147252f),
            new(-0.993774f, -0.124103f, -0.147252f),
            new(-0.993774f, -0.124103f, 0.154165f),
            new(-0.993774f, 0.118359f, 0.154165f),
            new(-0.817496f, -0.585607f, 0.137973f),
            new(-0.696265f, -0.660531f, 0.334128f),
            new(-0.789408f, -0.416680f, 0.484838f),
            new(-0.910638f, -0.341756f, 0.288682f),
            new(-0.411039f, -0.484253f, 0.795634f),
            new(-0.336113f, -0.288097f, 0.916863f),
            new(-0.579966f, -0.137388f, 0.823721f),
            new(-0.654891f, -0.333543f, 0.702490f),
            new(0.341652f, -0.288097f, 0.916863f),
            new(0.416578f, -0.484253f, 0.795634f),
            new(0.660429f, -0.333543f, 0.702490f),
            new(0.585505f, -0.137388f, 0.823721f),
            new(0.333441f, -0.701905f, 0.661116f),
            new(0.137286f, -0.823136f, 0.586192f),
            new(0.287995f, -0.916278f, 0.342339f),
            new(0.484149f, -0.795049f, 0.417265f),
            new(-0.131747f, -0.823136f, 0.586192f),
            new(-0.327903f, -0.701905f, 0.661116f),
            new(-0.478612f, -0.795049f, 0.417265f),
            new(-0.282457f, -0.916278f, 0.342339f),
            new(-0.910638f, -0.341756f, -0.281770f),
            new(-0.789408f, -0.416680f, -0.477925f),
            new(-0.696265f, -0.660531f, -0.327216f),
            new(-0.817496f, -0.585607f, -0.131060f),
            new(-0.327903f, -0.701905f, -0.654204f),
            new(-0.131747f, -0.823136f, -0.579280f),
            new(-0.282457f, -0.916278f, -0.335426f),
            new(-0.478612f, -0.795049f, -0.410352f),
            new(0.153478f, -0.999415f, -0.117774f),
            new(0.153478f, -0.999415f, 0.124687f),
            new(-0.147939f, -0.999415f, 0.124687f),
            new(-0.147939f, -0.999415f, -0.117774f),
            new(0.701803f, -0.660531f, 0.334128f),
            new(0.823033f, -0.585607f, 0.137973f),
            new(0.916176f, -0.341756f, 0.288682f),
            new(0.794945f, -0.416680f, 0.484838f),
            new(0.823033f, -0.585607f, -0.131060f),
            new(0.701803f, -0.660531f, -0.327216f),
            new(0.794945f, -0.416680f, -0.477925f),
            new(0.916176f, -0.341756f, -0.281770f),
            new(0.484149f, -0.795049f, -0.410352f),
            new(0.287995f, -0.916278f, -0.335426f),
            new(0.137286f, -0.823136f, -0.579280f),
            new(0.333441f, -0.701905f, -0.654204f),
            new(-0.654891f, -0.333543f, -0.695578f),
            new(-0.579966f, -0.137388f, -0.816807f),
            new(-0.336113f, -0.288097f, -0.909951f),
            new(-0.411039f, -0.484253f, -0.788719f),
            new(-0.118461f, 0.147837f, -0.993087f),
            new(0.124000f, 0.147837f, -0.993087f),
            new(0.124000f, -0.153580f, -0.993087f),
            new(-0.118461f, -0.153580f, -0.993087f),
            new(0.585505f, -0.137388f, -0.816807f),
            new(0.660429f, -0.333543f, -0.695578f),
            new(0.416578f, -0.484253f, -0.788719f),
            new(0.341652f, -0.288097f, -0.909951f),
            new(0.999313f, -0.124103f, -0.147252f),
            new(0.999313f, 0.118359f, -0.147252f),
            new(0.999313f, 0.118359f, 0.154165f),
            new(0.999313f, -0.124103f, 0.154165f),
            new(0.660429f, 0.327801f, -0.695578f),
            new(0.585505f, 0.131646f, -0.816807f),
            new(0.341652f, 0.282353f, -0.909951f),
            new(0.416578f, 0.478508f, -0.788719f),
            new(-0.579966f, 0.131646f, -0.816807f),
            new(-0.654891f, 0.327801f, -0.695578f),
            new(-0.411039f, 0.478508f, -0.788719f),
            new(-0.336113f, 0.282353f, -0.909951f),
        };
        return vertices;
    }


    Vector3[] CreateTestConvexHull3()
    {
        var vertices = new Vector3[]
        {
            new(-0.103558f, 1.000000f, -0.490575f),
            new(0.266493f, 0.659794f, -0.363751f),
            new(-0.245774f, 0.762636f, -0.615304f),
            new(0.164688f, -0.777634f, -0.365919f),
            new(0.503268f, -0.846406f, -0.131286f),
            new(0.171066f, -0.931723f, -0.140738f),
            new(-0.247963f, -0.738059f, -0.413146f),
            new(-0.319203f, -0.260078f, -0.609331f),
            new(0.469624f, -0.747848f, -0.286486f),
            new(0.398526f, -0.238233f, -0.435281f),
            new(0.448274f, 0.295416f, -0.246327f),
            new(-0.245774f, 0.762636f, 0.596521f),
            new(0.266493f, 0.659794f, 0.344974f),
            new(-0.103558f, 1.000000f, 0.471792f),
            new(0.171066f, -0.931723f, 0.121961f),
            new(0.503268f, -0.846406f, 0.112509f),
            new(0.164688f, -0.777634f, 0.347137f),
            new(-0.319203f, -0.260078f, 0.590548f),
            new(-0.247963f, -0.738059f, 0.394364f),
            new(0.469624f, -0.747848f, 0.267709f),
            new(0.398526f, -0.238233f, 0.416498f),
            new(0.448274f, 0.295411f, 0.227550f),
        };
        return vertices;
    }

    Vector3[] CreateJSONSourcedConvexHull(string filePath)
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
        return points.ToArray();
    }

    void CreateHellCubeFace(int widthInPoints, float step, Span<Vector3> facePoints, Matrix3x3 transform, float localFaceOffset)
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
    Vector3[] CreateHellCube(int widthInPoints)
    {
        var facePointCount = widthInPoints * widthInPoints;
        var buffer = new Vector3[facePointCount * 6];
        var size = 8f;
        var halfSize = size / 2;
        var step = size / (widthInPoints - 1);
        Matrix3x3.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1f, 1, 1f)), float.Pi / 2, out var cubeTransform);
        Matrix3x3.CreateFromAxisAngle(new Vector3(0, 1, 0), float.Pi / 2, out var zFace);
        zFace *= cubeTransform;
        Matrix3x3.CreateFromAxisAngle(new Vector3(0, 1, 0), float.Pi, out var localFace2);
        Matrix3x3.CreateFromAxisAngle(new Vector3(1, 0, 0), -float.Pi / 2, out var yFace);
        yFace *= cubeTransform;
        CreateHellCubeFace(widthInPoints, step, buffer.AsSpan(facePointCount * 0, facePointCount), cubeTransform, halfSize);
        CreateHellCubeFace(widthInPoints, step, buffer.AsSpan(facePointCount * 1, facePointCount), zFace, halfSize);
        CreateHellCubeFace(widthInPoints, step, buffer.AsSpan(facePointCount * 2, facePointCount), cubeTransform, -halfSize);
        CreateHellCubeFace(widthInPoints, step, buffer.AsSpan(facePointCount * 3, facePointCount), zFace, -halfSize);
        CreateHellCubeFace(widthInPoints, step, buffer.AsSpan(facePointCount * 4, facePointCount), yFace, halfSize);
        CreateHellCubeFace(widthInPoints, step, buffer.AsSpan(facePointCount * 5, facePointCount), yFace, -halfSize);
        return buffer;
    }

    struct HullTestData
    {
        public Vector3[] Points;
        public HullData HullData;
#if DEBUG_STEPS
        public List<DebugStep> DebugSteps;
#endif
        public ConvexHull Hull;
        public TypedIndex ShapeIndex;
    }

    HullTestData[] hullTests;

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, -2.5f, 10);
        camera.Yaw = 0;
        camera.Pitch = 0;

        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

        var hullPointSets = new Vector3[][]
        {
            CreateRandomConvexHullPoints(),
            CreateMeshConvexHull(content.Load<MeshContent>(@"Content\newt.obj"), new Vector3(1, 1.5f, 1f)),
            CreateHellCube(200),
            CreateBwaa(),
            CreateTestConvexHull(),
            CreateTestConvexHull2(),
            CreateTestConvexHull3(),
            CreateBoxConvexHull(2),
            //CreateJSONSourcedConvexHull(@"Content/testHull.json"),
            //CreateDistantPlane(),
            //CreatePlaneish(),
        };

        hullTests = new HullTestData[hullPointSets.Length];
        for (int i = 0; i < hullPointSets.Length; ++i)
        {
            ref var test = ref hullTests[i];
            test.Points = hullPointSets[i];
#if DEBUG_STEPS
            ComputeHull(hullPointSets[i], BufferPool, out test.HullData, out test.DebugSteps);
#else
            ComputeHull(hullPointSets[i], BufferPool, out test.HullData);
#endif
            CreateShape(hullPointSets[i], test.HullData, BufferPool, out _, out test.Hull);
            test.ShapeIndex = Simulation.Shapes.Add(test.Hull);

            //// Check divergence between face planes and vertices.
            //float largestError = 0;
            //for (int j = 0; j < test.Hull.FaceToVertexIndicesStart.Length; ++j)
            //{
            //    test.Hull.GetVertexIndicesForFace(j, out var faceVertices);
            //    BundleIndexing.GetBundleIndices(j, out var normalBundleIndex, out var normalIndexInBundle);
            //    Vector3Wide.ReadSlot(ref test.Hull.BoundingPlanes[normalBundleIndex].Normal, normalIndexInBundle, out var faceNormal);
            //    var offset = test.Hull.BoundingPlanes[normalBundleIndex].Offset[normalIndexInBundle];
            //    Console.WriteLine($"Face {j} errors:");
            //    for (int k = 0; k < faceVertices.Length; ++k)
            //    {
            //        test.Hull.GetPoint(faceVertices[k], out var point);
            //        var error = Vector3.Dot(point, faceNormal) - offset;
            //        Console.WriteLine($"v{k}: {error}");
            //        largestError = MathF.Max(MathF.Abs(error), largestError);
            //    }
            //}
            //Console.WriteLine($"Largest error: {largestError}");


            //Matrix3x3.CreateScale(new Vector3(5, 0.5f, 3), out var scale);
            //var transform = Matrix3x3.CreateFromAxisAngle(Vector3.Normalize(new Vector3(3, 2, 1)), 1207) * scale;
            //const int transformCount = 10000;
            //var transformStart = Stopwatch.GetTimestamp();
            //for (int j = 0; j < transformCount; ++j)
            //{
            //    CreateTransformedCopy(test.Hull, transform, BufferPool, out var transformedHullShape);
            //    transformedHullShape.Dispose(BufferPool);
            //}
            //var transformEnd = Stopwatch.GetTimestamp();
            //Console.WriteLine($"Transform hull computation time (us): {(transformEnd - transformStart) * 1e6 / (transformCount * Stopwatch.Frequency)}");

            //test.Hull.RayTest(RigidPose.Identity, new Vector3(0, 1, 0), -Vector3.UnitY, out var t, out var normal);
            //const int rayIterationCount = 10000;
            //var rayPose = RigidPose.Identity;
            //var rayOrigin = new Vector3(0, 2, 0);
            //var rayDirection = new Vector3(0, -1, 0);

            //int hitCounter = 0;
            //var start = Stopwatch.GetTimestamp();
            //for (int j = 0; j < rayIterationCount; ++j)
            //{
            //    if (test.Hull.RayTest(rayPose, rayOrigin, rayDirection, out _, out _))
            //    {
            //        ++hitCounter;
            //    }
            //}
            //var end = Stopwatch.GetTimestamp();
            //Console.WriteLine($"Hit counter: {hitCounter}, computation time (us): {(end - start) * 1e6 / (rayIterationCount * Stopwatch.Frequency)}");

            //const int iterationCount = 100;
            //start = Stopwatch.GetTimestamp();
            //for (int j = 0; j < iterationCount; ++j)
            //{
            //    CreateShape(test.Points, BufferPool, out _, out var perfTestShape);
            //    perfTestShape.Dispose(BufferPool);
            //}
            //end = Stopwatch.GetTimestamp();
            //Console.WriteLine($"Hull computation time (us): {(end - start) * 1e6 / (iterationCount * Stopwatch.Frequency)}");
        }

        var boxHullShape = new ConvexHull(CreateBoxConvexHull(2), BufferPool, out _);




        TypedIndex[] otherShapes =
        [
            Simulation.Shapes.Add(new Sphere(2)),
            Simulation.Shapes.Add(new Capsule(0.5f, 2)),
            Simulation.Shapes.Add(new Box(2f, 2f, 2f)),
            Simulation.Shapes.Add(new Triangle { A = new Vector3(0, 0, -10), B = new Vector3(5, 0, -10), C = new Vector3(0, 0, -5) }),
            Simulation.Shapes.Add(new Cylinder(1, 1)),
            Simulation.Shapes.Add(boxHullShape),
        ];
        float spacing = 2.5f;
        float z = 0;
        for (int otherShapeIndex = 0; otherShapeIndex < otherShapes.Length; ++otherShapeIndex)
        {
            Simulation.Shapes.UpdateBounds(RigidPose.Identity, otherShapes[otherShapeIndex], out var bounds);
            var otherShapeSpan = bounds.Max - bounds.Min;
            var staticOffset = (bounds.Max + bounds.Min) * -0.5f + new Vector3(0, -5f, 0);
            var staticTop = bounds.Max.Y + staticOffset.Y;
            float x = 0;
            float effectiveZSpan = otherShapeSpan.Z;
            for (int hullIndex = 0; hullIndex < hullTests.Length; ++hullIndex)
            {
                ref var test = ref hullTests[hullIndex];
                test.Hull.ComputeBounds(Quaternion.Identity, out var min, out var max);
                var span = max - min;
                effectiveZSpan = MathF.Max(span.Z, effectiveZSpan);

                var spanX = MathF.Max(otherShapeSpan.X, span.X);
                var shapeX = x + spanX * 0.5f;
                var shapeY = staticTop + span.Y * 0.5f;
                Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(shapeX, shapeY, z), test.Hull.ComputeInertia(1), new(test.ShapeIndex, 20, 20), -0.01f));
                Simulation.Statics.Add(new StaticDescription(new Vector3(shapeX, 0, z) + staticOffset, otherShapes[otherShapeIndex]));
                x += spanX + spacing;
            }
            z += effectiveZSpan + spacing;
        }

        var pileSpacing = new Vector3(3f, 3f, 3);
        int width = 16;
        int height = 16;
        int length = 0;
        var origin = -0.5f * spacing * new Vector3(width, 0, length) + new Vector3(40, 0.2f, -40);
        var pileInertia = hullTests[0].Hull.ComputeInertia(1);
        var pileShape = hullTests[0].ShapeIndex;
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                for (int k = 0; k < length; ++k)
                {
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(
                        (origin + pileSpacing * new Vector3(i, j, k), QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathHelper.Pi * 0.05f)),
                        pileInertia, pileShape, 0.01f));
                }
            }
        }
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -10, 0), Simulation.Shapes.Add(new Box(1000, 1, 1000))));

        Random random = new Random(5);
        var mesh = DemoMeshHelper.CreateDeformedPlane(64, 64, (x, y) => new Vector3(
            x + 8,
            2f * MathF.Sin(x * 0.125f) * MathF.Sin(y * 0.125f) + 0.1f * random.NextSingle() - 3,
            y - 8), new Vector3(1, 1, 1), BufferPool);
        Simulation.Statics.Add(new StaticDescription(new Vector3(64, 0, 0), Simulation.Shapes.Add(mesh)));

#if DEBUG_STEPS
        stepIndices = new int[hullTests.Length];
#endif
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

#if DEBUG_STEPS
    int testIndex;
    int[] stepIndices;

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        ref var stepIndex = ref stepIndices[testIndex];
        if (input.TypedCharacters.Contains('x'))
        {
            stepIndex = Math.Max(stepIndex - 1, 0);
        }
        if (input.TypedCharacters.Contains('c'))
        {
            stepIndex = Math.Min(stepIndex + 1, hullTests[testIndex].DebugSteps.Count - 1);
        }
        if (input.TypedCharacters.Contains('n'))
        {
            testIndex = Math.Max(testIndex - 1, 0);
        }
        if (input.TypedCharacters.Contains('m'))
        {
            testIndex = Math.Min(testIndex + 1, hullTests.Length - 1);
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
        var hullTest = hullTests[testIndex];
        var points = hullTest.Points;
        var debugSteps = hullTest.DebugSteps;
        var stepIndex = stepIndices[testIndex];

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

        void DrawFace(DebugStep step, int[] reduced, Vector3 color, float offsetScale)
        {
            var offset = step.FaceNormal * offsetScale;
            if (showWireframe)
            {
                var previousIndex = reduced.Length - 1;
                for (int q = 0; q < reduced.Length; ++q)
                {
                    var a = points[reduced[q]] * scale + renderOffset + offset;
                    var b = points[reduced[previousIndex]] * scale + renderOffset + offset;
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
                    }, Simulation.Shapes, renderOffset, color);
                }
            }
        }

        if (showDeleted && step.OverwrittenOriginal != null)
        {
            DrawFace(step, step.OverwrittenOriginal, new Vector3(0.5f, 0.1f, 0.1f), 0.25f);
            for (int j = 0; j < step.DeletedFaces.Count; ++j)
                DrawFace(step, step.DeletedFaces[j], new Vector3(0.1f, 0.1f, 0.1f), 0.25f);
        }

        // Render all current faces in the step
        for (int faceIndex = 0; faceIndex < step.FaceStarts.Count; ++faceIndex)
        {
            var startIndex = step.FaceStarts[faceIndex];
            var endIndex = faceIndex < step.FaceStarts.Count - 1 ? step.FaceStarts[faceIndex + 1] : step.FaceIndices.Count;
            var faceVertexCount = endIndex - startIndex;
            var faceNormal = step.FaceNormals[faceIndex];

            var color = step.FaceIndex == faceIndex ? new Vector3(1, 0, 0.5f) : new Vector3(1, 0, 1);

            if (showWireframe)
            {
                // Draw wireframe edges for the face
                var previousIndex = endIndex - 1;
                for (int vertexIndex = startIndex; vertexIndex < endIndex; ++vertexIndex)
                {
                    var a = points[step.FaceIndices[vertexIndex]] * scale + renderOffset;
                    var b = points[step.FaceIndices[previousIndex]] * scale + renderOffset;
                    previousIndex = vertexIndex;
                    renderer.Lines.Allocate() = new LineInstance(a, b, color, Vector3.Zero);
                }
            }
            else
            {
                // Draw filled triangles for the face
                var baseIndex = step.FaceIndices[startIndex];
                var basePoint = points[baseIndex] * scale;
                for (int vertexIndex = startIndex + 2; vertexIndex < endIndex; ++vertexIndex)
                {
                    renderer.Shapes.AddShape(new Triangle
                    {
                        A = basePoint,
                        B = points[step.FaceIndices[vertexIndex]] * scale,
                        C = points[step.FaceIndices[vertexIndex - 1]] * scale
                    }, Simulation.Shapes, renderOffset, color);
                }
            }
        }

        //Console.WriteLine($"face count: {step.FaceStarts.Count}");

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
            text.Clear().Append("Step: ").Append(stepIndex + 1).Append(" out of ").Append(debugSteps.Count).Append(" for test ").Append(testIndex).Append("."),
            new Vector2(32, renderer.Surface.Resolution.Y - 170), 20, new Vector3(1), font);
       
        renderer.TextBatcher.Write(
            text.Clear().Append($"Enumerate step with X and C, change test with N and M."),
            new Vector2(32, renderer.Surface.Resolution.Y - 140), 20, new Vector3(1), font);
        renderer.TextBatcher.Write(text.Clear().Append("Show wireframe: P ").Append(showWireframe ? "(on)" : "(off)"), new Vector2(32, renderer.Surface.Resolution.Y - 120), 20, new Vector3(1), font);
        renderer.TextBatcher.Write(text.Clear().Append("Show deleted: U ").Append(showDeleted ? "(on)" : "(off)"), new Vector2(32, renderer.Surface.Resolution.Y - 100), 20, new Vector3(1), font);
        renderer.TextBatcher.Write(text.Clear().Append("Show vertex indices: Y ").Append(showVertexIndices ? "(on)" : "(off)"), new Vector2(32, renderer.Surface.Resolution.Y - 80), 20, new Vector3(1), font);
        renderer.TextBatcher.Write(text.Clear().Append("Show face vertex statuses: H ").Append(showFaceVertexStatuses ? "(on)" : "(off)"), new Vector2(32, renderer.Surface.Resolution.Y - 60), 20, new Vector3(1), font);
        renderer.TextBatcher.Write(text.Clear().Append("Face count: ").Append(step.FaceStarts.Count), new Vector2(32, renderer.Surface.Resolution.Y - 20), 20, new Vector3(1), font);


        base.Render(renderer, camera, input, text, font);
    }
#endif
}
