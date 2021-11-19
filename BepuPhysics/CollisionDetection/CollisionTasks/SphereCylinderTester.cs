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
        public static void ComputeSphereToClosest(in CylinderWide b, in Vector3Wide offsetB, in Matrix3x3Wide orientationMatrixB,
            out Vector3Wide cylinderLocalOffsetA, out Vector<float> horizontalOffsetLength, out Vector<float> inverseHorizontalOffsetLength, 
            out Vector3Wide sphereToClosestLocalB, out Vector3Wide sphereToClosest)
        {
            //Clamp the sphere position to the cylinder's volume.
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, orientationMatrixB, out var cylinderLocalOffsetB);
            Vector3Wide.Negate(cylinderLocalOffsetB, out cylinderLocalOffsetA);
            horizontalOffsetLength = Vector.SquareRoot(cylinderLocalOffsetA.X * cylinderLocalOffsetA.X + cylinderLocalOffsetA.Z * cylinderLocalOffsetA.Z);
            inverseHorizontalOffsetLength = Vector<float>.One / horizontalOffsetLength;
            var horizontalClampMultiplier = b.Radius * inverseHorizontalOffsetLength;
            var horizontalClampRequired = Vector.GreaterThan(horizontalOffsetLength, b.Radius);
            Vector3Wide clampedSpherePositionLocalB;
            clampedSpherePositionLocalB.X = Vector.ConditionalSelect(horizontalClampRequired, cylinderLocalOffsetA.X * horizontalClampMultiplier, cylinderLocalOffsetA.X);
            clampedSpherePositionLocalB.Y = Vector.Min(b.HalfLength, Vector.Max(-b.HalfLength, cylinderLocalOffsetA.Y));
            clampedSpherePositionLocalB.Z = Vector.ConditionalSelect(horizontalClampRequired, cylinderLocalOffsetA.Z * horizontalClampMultiplier, cylinderLocalOffsetA.Z);

            Vector3Wide.Add(clampedSpherePositionLocalB, cylinderLocalOffsetB, out sphereToClosestLocalB);
            Matrix3x3Wide.TransformWithoutOverlap(sphereToClosestLocalB, orientationMatrixB, out sphereToClosest);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(ref SphereWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var orientationMatrixB);
            ComputeSphereToClosest(b, offsetB, orientationMatrixB, 
                out var cylinderLocalOffsetA, out var horizontalOffsetLength, out var inverseHorizontalOffsetLength, 
                out var sphereToContactLocalB, out manifold.OffsetA);

            //If the sphere center is inside the cylinder, then we must compute the fastest way out of the cylinder.
            var absY = Vector.Abs(cylinderLocalOffsetA.Y);
            var depthY = b.HalfLength - absY;
            var horizontalDepth = b.Radius - horizontalOffsetLength;
            var useDepthY = Vector.LessThanOrEqual(depthY, horizontalDepth);
            var useTopCapNormal = Vector.GreaterThan(cylinderLocalOffsetA.Y, Vector<float>.Zero);
            Vector3Wide localInternalNormal;

            var useHorizontalFallback = Vector.LessThanOrEqual(horizontalOffsetLength, b.Radius * new Vector<float>(1e-5f));
            localInternalNormal.X = Vector.ConditionalSelect(useDepthY, Vector<float>.Zero, Vector.ConditionalSelect(useHorizontalFallback, Vector<float>.One, cylinderLocalOffsetA.X * inverseHorizontalOffsetLength));
            localInternalNormal.Y = Vector.ConditionalSelect(useDepthY, Vector.ConditionalSelect(useTopCapNormal, Vector<float>.One, new Vector<float>(-1)), Vector<float>.Zero);
            localInternalNormal.Z = Vector.ConditionalSelect(useDepthY, Vector<float>.Zero, Vector.ConditionalSelect(useHorizontalFallback, Vector<float>.Zero, cylinderLocalOffsetA.Z * inverseHorizontalOffsetLength));

            Vector3Wide.Length(sphereToContactLocalB, out var contactDistanceFromSphereCenter);
            //Note negation; normal points from B to A by convention.
            Vector3Wide.Scale(sphereToContactLocalB, new Vector<float>(-1) / contactDistanceFromSphereCenter, out var localExternalNormal);

            //Can't rely on the external normal if the sphere is so close to the surface that the normal isn't numerically computable.
            var useInternal = Vector.LessThan(contactDistanceFromSphereCenter, new Vector<float>(1e-7f));
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
