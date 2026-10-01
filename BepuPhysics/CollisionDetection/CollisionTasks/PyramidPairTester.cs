using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct PyramidPairTester : IPairTester<PyramidWide, PyramidWide, Convex4ContactManifoldWide>
    {
        public static int BatchSize => 32;

        public static unsafe void Test(ref PyramidWide a, ref PyramidWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            // Convert pyramid B to a convex hull and use the pyramid-convex tester. The pyramid is a convex shape, so this is valid, albeit (extremely) inefficient.
            // Todo: implement a proper pyramid-pyramid tester.
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

            PyramidConvexHullTester.Test(ref a, ref hull, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out manifold);
        }

        public static void Test(ref PyramidWide a, ref PyramidWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotSupportedException();
        }

        public static void Test(ref PyramidWide a, ref PyramidWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotSupportedException();
        }
    }
}
