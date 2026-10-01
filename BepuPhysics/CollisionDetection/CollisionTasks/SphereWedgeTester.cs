using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct SphereWedgeTester : IPairTester<SphereWide, WedgeWide, Convex1ContactManifoldWide>
    {
        public static int BatchSize => 32;

        public static void Test(ref SphereWide a, ref WedgeWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            Test(ref a, ref b, ref speculativeMargin, ref offsetB, ref orientationB, pairCount, out manifold);
        }

        public static unsafe void Test(ref SphereWide a, ref WedgeWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            // Convert wedge to a convex hull and use the sphere-convex tester. The wedge is a convex shape, so this is valid, albeit (extremely) inefficient.
            // Todo: implement a proper sphere-wedge tester.
            ConvexHullWide hull = default;
            {
                var data = stackalloc byte[hull.InternalAllocationSize];
                hull.Initialize(new(data, hull.InternalAllocationSize));
            }

            using BufferPool pool = new();
            for (int i = 0; i < pairCount; ++i)
            {
                b.ReadSlot(i, out Wedge wedge);
                ConvexHull hullShape = wedge.ToConvexHull(pool);
                hull.WriteSlot(i, in hullShape);
            }

            SphereConvexHullTester.Test(ref a, ref hull, ref speculativeMargin, ref offsetB, ref orientationB, pairCount, out manifold);
        }

        public static void Test(ref SphereWide a, ref WedgeWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotSupportedException();
        }
    }
}
