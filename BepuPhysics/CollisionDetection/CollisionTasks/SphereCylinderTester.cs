using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct SphereCylinderTester : IPairTester<SphereWide, CylinderWide, Convex1ContactManifoldWide>
    {
        public int BatchSize => 32;

        public void Test(ref SphereWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(ref SphereWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            //Clamp the sphere position to the cylinder's volume.
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var orientationMatrixB);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, orientationMatrixB, out var cylinderLocalOffsetB);
            Vector3Wide.Negate(cylinderLocalOffsetB, out var cylinderLocalOffsetA);
            var horizontalOffsetLength = Vector.SquareRoot(cylinderLocalOffsetA.X * cylinderLocalOffsetA.X + cylinderLocalOffsetA.Z * cylinderLocalOffsetA.Z);
            var inverseHorizontalOffsetLength = Vector<float>.One / horizontalOffsetLength;
            var horizontalClampMultiplier = b.Radius * inverseHorizontalOffsetLength;
            var horizontalClampRequired = Vector.GreaterThan(horizontalOffsetLength, b.Radius);
            Vector3Wide clampedSpherePositionLocalB;
            clampedSpherePositionLocalB.X = Vector.ConditionalSelect(horizontalClampRequired, cylinderLocalOffsetA.X * horizontalClampMultiplier, cylinderLocalOffsetA.X);
            clampedSpherePositionLocalB.Y = Vector.Min(b.HalfLength, Vector.Max(-b.HalfLength, cylinderLocalOffsetA.Y));
            clampedSpherePositionLocalB.Z = Vector.ConditionalSelect(horizontalClampRequired, cylinderLocalOffsetA.Z * horizontalClampMultiplier, cylinderLocalOffsetA.Z);

            Vector3Wide.Add(clampedSpherePositionLocalB, cylinderLocalOffsetB, out var sphereToContactLocalB);
            Matrix3x3Wide.TransformWithoutOverlap(sphereToContactLocalB, orientationMatrixB, out manifold.OffsetA);

            //If the sphere center is inside the cylinder, then we must compute the fastest way out of the cylinder.
            var absY = Vector.Abs(cylinderLocalOffsetB.Y);
            var useInternal = Vector.AndNot(Vector.LessThanOrEqual(absY, b.HalfLength), horizontalClampRequired);
            var depthY = b.HalfLength - absY;
            var horizontalDepth = b.Radius - horizontalOffsetLength;
            var useDepthY = Vector.LessThanOrEqual(depthY, horizontalDepth);
            var useTopCapNormal = Vector.GreaterThan(cylinderLocalOffsetA.Y, Vector<float>.Zero);
            Vector3Wide localInternalNormal;
            localInternalNormal.X = Vector.ConditionalSelect(useDepthY, Vector<float>.Zero, cylinderLocalOffsetA.X * inverseHorizontalOffsetLength);
            localInternalNormal.Y = Vector.ConditionalSelect(useDepthY, Vector.ConditionalSelect(useTopCapNormal, Vector<float>.One, new Vector<float>(-1)), Vector<float>.Zero);
            localInternalNormal.Z = Vector.ConditionalSelect(useDepthY, Vector<float>.Zero, cylinderLocalOffsetA.Z * inverseHorizontalOffsetLength);

            Vector3Wide.Length(sphereToContactLocalB, out var contactDistanceFromSphereCenter);
            //Note negation; normal points from B to A by convention.
            Vector3Wide.Scale(sphereToContactLocalB, new Vector<float>(-1) / contactDistanceFromSphereCenter, out var localExternalNormal);

            Vector3Wide.ConditionalSelect(useInternal, localInternalNormal, localExternalNormal, out var localNormal);

            Matrix3x3Wide.TransformWithoutOverlap(localNormal, orientationMatrixB, out manifold.Normal);

            manifold.FeatureId = Vector<int>.Zero;
            manifold.Depth = Vector.ConditionalSelect(useInternal, Vector.ConditionalSelect(useDepthY, depthY, horizontalDepth), -contactDistanceFromSphereCenter) + a.Radius;
            manifold.ContactExists = Vector.GreaterThanOrEqual(manifold.Depth, -speculativeMargin);
        }

        public void Test(ref SphereWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
