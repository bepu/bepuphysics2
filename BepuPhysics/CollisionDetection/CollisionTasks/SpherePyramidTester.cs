using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct SpherePyramidTester : IPairTester<SphereWide, PyramidWide, Convex1ContactManifoldWide>
    {
        public static int BatchSize => 32;

        public static void Test(ref SphereWide a, ref PyramidWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            Test(ref a, ref b, ref speculativeMargin, ref offsetB, ref orientationB, pairCount, out manifold);
        }

        public static void Test(ref SphereWide a, ref PyramidWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public static void Test(ref SphereWide a, ref PyramidWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotSupportedException();
        }
    }
}
