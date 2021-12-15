using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct SphereConvexHullTester : IPairTester<SphereWide, ConvexHullWide, Convex1ContactManifoldWide>
    {
        public int BatchSize => 16;

        public void Test(ref SphereWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref SphereWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var hullOrientation);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, hullOrientation, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);
            Matrix3x3Wide.CreateIdentity(out var identity);
            Vector3Wide.Length(localOffsetA, out var centerDistance);
            Vector3Wide.Scale(localOffsetA, Vector<float>.One / centerDistance, out var initialNormal);
            var useInitialFallback = Vector.LessThan(centerDistance, new Vector<float>(1e-8f));
            initialNormal.X = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.X);
            initialNormal.Y = Vector.ConditionalSelect(useInitialFallback, Vector<float>.One, initialNormal.Y);
            initialNormal.Z = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.Z);
            var hullSupportFinder = default(ConvexHullSupportFinder);
            var sphereSupportFinder = default(SphereSupportFinder);
            var inactiveLanes = BundleIndexing.CreateTrailingMaskForCountInBundle(pairCount);
            b.EstimateEpsilonScale(inactiveLanes, out var hullEpsilonScale);
            var epsilonScale = Vector.Min(a.Radius, hullEpsilonScale);
            DepthRefiner<ConvexHull, ConvexHullWide, ConvexHullSupportFinder, Sphere, SphereWide, SphereSupportFinder>.FindMinimumDepth(
                b, a, localOffsetA, identity, ref hullSupportFinder, ref sphereSupportFinder, initialNormal, inactiveLanes, 1e-5f * epsilonScale, -speculativeMargin, 
                out var depth, out var localNormal, out var closestOnHull);

            Matrix3x3Wide.TransformWithoutOverlap(closestOnHull, hullOrientation, out var hullToContact);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, hullOrientation, out manifold.Normal);
            Vector3Wide.Add(hullToContact, offsetB, out manifold.OffsetA);
                      
            manifold.FeatureId = Vector<int>.Zero;
            manifold.Depth = depth;
            manifold.ContactExists = Vector.GreaterThanOrEqual(manifold.Depth, -speculativeMargin);
        }

        public void Test(ref SphereWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
