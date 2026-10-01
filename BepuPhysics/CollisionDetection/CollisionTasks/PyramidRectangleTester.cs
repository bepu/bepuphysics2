using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct PyramidRectangleTester : IPairTester<PyramidWide, RectangleWide, Convex4ContactManifoldWide>
    {
        public static int BatchSize => 32;

        public static unsafe void Test(ref PyramidWide a, ref RectangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            // Convert pyramid to a convex hull and use the rectangle-convex tester. The pyramid is a convex shape, so this is valid, albeit (extremely) inefficient.
            // Todo: implement a proper pyramid-rectangle tester.
            ConvexHullWide hull = default;
            {
                var data = stackalloc byte[hull.InternalAllocationSize];
                hull.Initialize(new(data, hull.InternalAllocationSize));
            }

            using BufferPool pool = new();
            for (int i = 0; i < pairCount; ++i)
            {
                a.ReadSlot(i, out Pyramid pyramid);
                ConvexHull hullShape = pyramid.ToConvexHull(pool);
                hull.WriteSlot(i, in hullShape);
            }

            // We need to flip the offset and orientations because the rectangle is now the first shape in the pair.
            Vector3Wide.Negate(offsetB, out var reversedOffsetB);
            RectangleConvexHullTester.Test(ref b, ref hull, ref speculativeMargin, ref reversedOffsetB, ref orientationB, ref orientationA, pairCount, out manifold);

            // The manifold is generated in the rectangle-convex order, but we need it in the pyramid-rectangle order. Flip the manifold to match.
            var flipAll = new Vector<int>(-1);
            manifold.ApplyFlipMask(ref reversedOffsetB, in flipAll);
        }

        public static void Test(ref PyramidWide a, ref RectangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotSupportedException();
        }

        public static void Test(ref PyramidWide a, ref RectangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotSupportedException();
        }
    }
}
