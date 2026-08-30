using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    //This is actually a little *slower* than using GJK, which is interesting. Playing with termination epsilons might change things. Keeping it here for example purposes.
    //Using GJK in the contact generating tester is an option, but it's a bit awkward and the performance difference doesn't seem large enough to worry about in context.
    internal struct CapsuleCylinderDistanceTester : IPairDistanceTester<CapsuleWide, CylinderWide>
    {
        public void Test(in CapsuleWide a, in CylinderWide b, in Vector3Wide offsetB, in QuaternionWide orientationA, in QuaternionWide orientationB, in Vector<int> inactiveLanes,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            QuaternionWide.Conjugate(orientationB, out var inverseOrientationB);
            QuaternionWide.ConcatenateWithoutOverlap(orientationA, inverseOrientationB, out var localOrientationA);
            var capsuleAxis = QuaternionWide.TransformUnitY(localOrientationA);
            QuaternionWide.TransformWithoutOverlap(offsetB, inverseOrientationB, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);

            CapsuleCylinderTester.GetClosestPointBetweenLineSegmentAndCylinder(localOffsetA, capsuleAxis, a.HalfLength, b, Vector<int>.Zero, out var t, out var offsetFromCylinderToLineSegment);

            Vector3Wide.Length(offsetFromCylinderToLineSegment, out distance);
            Vector3Wide.Scale(offsetFromCylinderToLineSegment, Vector<float>.One / distance, out var localNormal);
            distance -= a.Radius;
            intersected = Vector.LessThanOrEqual(distance, Vector<float>.Zero);
            QuaternionWide.TransformWithoutOverlap(localNormal, orientationB, out normal);
            Vector3Wide.Scale(capsuleAxis, t, out var localClosestA);
            Vector3Wide.Scale(localNormal, a.Radius, out var normalOffset);
            Vector3Wide.Subtract(localClosestA, normalOffset, out localClosestA);
            Vector3Wide.Add(localClosestA, localOffsetA, out localClosestA);
            QuaternionWide.TransformWithoutOverlap(localClosestA, orientationB, out closestA);
            Vector3Wide.Add(closestA, offsetB, out closestA);
        }
    }

}
