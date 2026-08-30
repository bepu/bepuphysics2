using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct SphereBoxDistanceTester : IPairDistanceTester<SphereWide, BoxWide>
    {
        public void Test(in SphereWide a, in BoxWide b, in Vector3Wide offsetB, in QuaternionWide orientationA, in QuaternionWide orientationB, in Vector<int> inactiveLanes,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
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
            Vector3Wide.Subtract(clampedLocalOffsetB, localOffsetB, out var localNormal);
            Vector3Wide.Length(localNormal, out var innerDistance);
            var inverseDistance = Vector<float>.One / innerDistance;
            Vector3Wide.Scale(localNormal, inverseDistance, out localNormal);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, orientationMatrixB, out normal);
            var negativeRadius = -a.Radius;
            Vector3Wide.Scale(normal, negativeRadius, out closestA);
            distance = innerDistance - a.Radius;
            intersected = Vector.LessThanOrEqual(distance, Vector<float>.Zero);
        }
    }


}
