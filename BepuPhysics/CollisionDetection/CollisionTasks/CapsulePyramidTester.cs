using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CapsulePyramidTester : IPairTester<CapsuleWide, PyramidWide, Convex2ContactManifoldWide>
    {
        public static int BatchSize => 32;

        public static unsafe void Test(ref CapsuleWide a, ref PyramidWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            // Convert pyramid to a convex hull and use the capsule-convex tester. The pyramid is a convex shape, so this is valid, albeit (extremely) inefficient.
            // Todo: implement a proper capsule-pyramid tester.
            ConvexHullWide hull = default;
            {
                var data = stackalloc byte[hull.InternalAllocationSize];
                hull.Initialize(new(data, hull.InternalAllocationSize));
            }

            using BufferPool pool = new();
            for (int i = 0; i < pairCount; ++i)
            {
                b.ReadSlot(i, out Pyramid pyramid);
                ConvexHull hullShape = pyramid.ToConvexHull(pool);
                hull.WriteSlot(i, in hullShape);
            }

            CapsuleConvexHullTester.Test(ref a, ref hull, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out manifold);
        }

        public static void Test(ref CapsuleWide a, ref PyramidWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            throw new NotSupportedException();
        }

        public static void Test(ref CapsuleWide a, ref PyramidWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            throw new NotSupportedException();
        }
    }
}
