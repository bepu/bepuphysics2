using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct SphereCapsuleDistanceTester : IPairDistanceTester<SphereWide, CapsuleWide>
    {
        public void Test(ref SphereWide a, ref CapsuleWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            //The contact for a sphere-capsule pair is based on the closest point of the sphere center to the capsule internal line segment.
            QuaternionWide.TransformUnitXY(ref orientationB, out var x, out var y);
            Vector3Wide.Dot(ref y, ref offsetB, out var t);
            t = Vector.Min(b.HalfLength, Vector.Max(-b.HalfLength, -t));
            Vector3Wide.Scale(ref y, ref t, out var capsuleLocalClosestPointOnLineSegment);

            Vector3Wide.Add(ref offsetB, ref capsuleLocalClosestPointOnLineSegment, out var sphereToInternalSegment);
            Vector3Wide.Length(ref sphereToInternalSegment, out var internalDistance);
            //Note that the normal points from B to A by convention. Here, the sphere is A, the capsule is B, so the normalization requires a negation.
            var inverseDistance = new Vector<float>(-1f) / internalDistance;
            Vector3Wide.Scale(ref sphereToInternalSegment, ref inverseDistance, out normal);
            var surfaceOffset = -a.Radius;
            Vector3Wide.Scale(ref normal, ref surfaceOffset, out closestA);
            distance = internalDistance - a.Radius - b.Radius;
            intersected = Vector.LessThanOrEqual(distance, Vector<float>.Zero);
        }
    }

    //TODO: Sphere-capsule is simple enough that it *might* be worth an analytic special case for sweep.

}
