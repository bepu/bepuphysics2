using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    //Individual pair testers are designed to be used outside of the narrow phase. They need to be usable for queries and such, so all necessary data must be gathered externally.
    public struct SphereBoxTester : IPairTester<SphereWide, BoxWide, Convex1ContactManifoldWide>
    {
        public int BatchSize => 32;

        public void Test(ref SphereWide a, ref BoxWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(ref SphereWide a, ref BoxWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            //Clamp the position of the sphere to the box.
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var orientationMatrixB);
            //Note that we're working with localOffsetB, which is the offset from A to B, even though conceptually we want to be operating on the offset from B to A.
            //Those offsets differ only by their sign, so are equivalent due to the symmetry of the box. The negation is left implicit.
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, orientationMatrixB, out var localOffsetB);
            Vector3Wide clampedLocalOffsetB;
            clampedLocalOffsetB.X = Vector.Min(Vector.Max(localOffsetB.X, -b.HalfWidth), b.HalfWidth);
            clampedLocalOffsetB.Y = Vector.Min(Vector.Max(localOffsetB.Y, -b.HalfHeight), b.HalfHeight);
            clampedLocalOffsetB.Z = Vector.Min(Vector.Max(localOffsetB.Z, -b.HalfLength), b.HalfLength);
            //Implicit negation to make the normal point from B to A, following convention.
            Vector3Wide.Subtract(clampedLocalOffsetB, localOffsetB, out var outsideNormal);
            Vector3Wide.Length(outsideNormal, out var distance);
            var inverseDistance = Vector<float>.One / distance;
            Vector3Wide.Scale(outsideNormal, inverseDistance, out outsideNormal);
            var outsideDepth = a.Radius - distance;

            //If the sphere center is inside the box, then the shortest local axis to exit must be chosen.
            var depthX = b.HalfWidth - Vector.Abs(localOffsetB.X);
            var depthY = b.HalfHeight - Vector.Abs(localOffsetB.Y);
            var depthZ = b.HalfLength - Vector.Abs(localOffsetB.Z);
            var insideDepth = Vector.Min(depthX, Vector.Min(depthY, depthZ));
            //Only one axis may have a nonzero component.
            var useX = Vector.Equals(insideDepth, depthX);
            var useY = Vector.AndNot(Vector.Equals(insideDepth, depthY), useX);
            var useZ = Vector.OnesComplement(Vector.BitwiseOr(useX, useY));
            Vector3Wide insideNormal;
            //A faster sign test would be nice.
            insideNormal.X = Vector.ConditionalSelect(useX, Vector.ConditionalSelect(Vector.LessThan(localOffsetB.X, Vector<float>.Zero), new Vector<float>(1f), new Vector<float>(-1f)), Vector<float>.Zero);
            insideNormal.Y = Vector.ConditionalSelect(useY, Vector.ConditionalSelect(Vector.LessThan(localOffsetB.Y, Vector<float>.Zero), new Vector<float>(1f), new Vector<float>(-1f)), Vector<float>.Zero);
            insideNormal.Z = Vector.ConditionalSelect(useZ, Vector.ConditionalSelect(Vector.LessThan(localOffsetB.Z, Vector<float>.Zero), new Vector<float>(1f), new Vector<float>(-1f)), Vector<float>.Zero);

            insideDepth += a.Radius;
            var useInside = Vector.Equals(distance, Vector<float>.Zero);
            Vector3Wide.ConditionalSelect(useInside, insideNormal, outsideNormal, out var localNormal);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, orientationMatrixB, out manifold.Normal);
            manifold.Depth = Vector.ConditionalSelect(useInside, insideDepth, outsideDepth);
            manifold.FeatureId = Vector<int>.Zero;

            //The contact position relative to object A (the sphere) is computed as the average of the extreme point along the normal toward the opposing shape on each shape, averaged.
            //For capsule-sphere, this can be computed from the normal and depth.
            var negativeOffsetFromSphere = manifold.Depth * 0.5f - a.Radius;
            Vector3Wide.Scale(manifold.Normal, negativeOffsetFromSphere, out manifold.OffsetA);
            manifold.ContactExists = Vector.GreaterThan(manifold.Depth, -speculativeMargin);
        }

        public void Test(ref SphereWide a, ref BoxWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
