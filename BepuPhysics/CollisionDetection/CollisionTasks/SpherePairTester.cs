using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    //Individual pair testers are designed to be used outside of the narrow phase. They need to be usable for queries and such, so all necessary data must be gathered externally.
    public struct SpherePairTester : IPairTester<SphereWide, SphereWide, Convex1ContactManifoldWide>
    {
        public int BatchSize => 32;

        public void Test(ref SphereWide a, ref SphereWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref SphereWide a, ref SphereWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(ref SphereWide a, ref SphereWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            Vector3Wide.Length(offsetB, out var centerDistance);
            //Note the negative 1. By convention, the normal points from B to A.
            var inverseDistance = new Vector<float>(-1f) / centerDistance;
            Vector3Wide.Scale(offsetB, inverseDistance, out manifold.Normal);
            var normalIsValid = Vector.GreaterThan(centerDistance, Vector<float>.Zero);
            //Arbitrarily choose the (0,1,0) if the two spheres are in the same position. Any unit length vector is equally valid.
            manifold.Normal.X = Vector.ConditionalSelect(normalIsValid, manifold.Normal.X, Vector<float>.Zero);
            manifold.Normal.Y = Vector.ConditionalSelect(normalIsValid, manifold.Normal.Y, Vector<float>.One);
            manifold.Normal.Z = Vector.ConditionalSelect(normalIsValid, manifold.Normal.Z, Vector<float>.Zero);
            manifold.Depth = a.Radius + b.Radius - centerDistance;

            //The contact position relative to object A is computed as the average of the extreme point along the normal toward the opposing sphere on each sphere, averaged.
            var negativeOffsetFromA = manifold.Depth * 0.5f - a.Radius;
            Vector3Wide.Scale(manifold.Normal, negativeOffsetFromA, out manifold.OffsetA);
            manifold.ContactExists = Vector.GreaterThan(manifold.Depth, -speculativeMargin);
            manifold.FeatureId = default;
        }
    }
}
