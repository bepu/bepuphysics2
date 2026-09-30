using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct RectangleConvexHullTester : IPairTester<RectangleWide, ConvexHullWide, Convex4ContactManifoldWide>
    {
        public static int BatchSize => 16;

        public static void Test(ref RectangleWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public static void Test(ref RectangleWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotSupportedException();
        }

        public static void Test(ref RectangleWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotSupportedException();
        }
    }
}
