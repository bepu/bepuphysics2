using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct SphereBoxDistanceTester : IPairDistanceTester<SphereWide, BoxWide>
    {
        public void Test(ref SphereWide a, ref BoxWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            //Clamp the position of the sphere to the box.
            Matrix3x3Wide.CreateFromQuaternion(ref orientationB, out var orientationMatrixB);
            //Note that we're working with localOffsetB, which is the offset from A to B, even though conceptually we want to be operating on the offset from B to A.
            //Those offsets differ only by their sign, so are equivalent due to the symmetry of the box. The negation is left implicit.
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref offsetB, ref orientationMatrixB, out var localOffsetB);
            Vector3Wide clampedLocalOffsetB;
            clampedLocalOffsetB.X = Vector.Min(Vector.Max(localOffsetB.X, -b.HalfWidth), b.HalfWidth);
            clampedLocalOffsetB.Y = Vector.Min(Vector.Max(localOffsetB.Y, -b.HalfHeight), b.HalfHeight);
            clampedLocalOffsetB.Z = Vector.Min(Vector.Max(localOffsetB.Z, -b.HalfLength), b.HalfLength);
            //Implicit negation to make the normal point from B to A, following convention.
            Vector3Wide.Subtract(ref clampedLocalOffsetB, ref localOffsetB, out var localNormal);
            Vector3Wide.Length(ref localNormal, out var innerDistance);
            var inverseDistance = Vector<float>.One / innerDistance;
            Vector3Wide.Scale(ref localNormal, ref inverseDistance, out localNormal);
            Matrix3x3Wide.TransformWithoutOverlap(ref localNormal, ref orientationMatrixB, out normal);
            var negativeRadius = -a.Radius;
            Vector3Wide.Scale(ref normal, ref negativeRadius, out closestA);
            distance = innerDistance - a.Radius;
            intersected = Vector.LessThanOrEqual(distance, Vector<float>.Zero);
        }
    }
    

}
