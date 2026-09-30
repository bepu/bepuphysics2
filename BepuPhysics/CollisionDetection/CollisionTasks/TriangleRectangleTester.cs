using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct TriangleRectangleTester : IPairTester<TriangleWide, RectangleWide, Convex4ContactManifoldWide>
    {
        public static int BatchSize => 32;

        public static void Test(ref TriangleWide a, ref RectangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public static void Test(ref TriangleWide a, ref RectangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotSupportedException();
        }

        public static void Test(ref TriangleWide a, ref RectangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotSupportedException();
        }
    }
}
