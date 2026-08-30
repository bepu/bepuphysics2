using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct SpherePairDistanceTester : IPairDistanceTester<SphereWide, SphereWide>
    {
        public void Test(in SphereWide a, in SphereWide b, in Vector3Wide offsetB, in QuaternionWide orientationA, in QuaternionWide orientationB, in Vector<int> inactiveLanes,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            Vector3Wide.Length(offsetB, out var centerDistance);
            //Note the negative 1. By convention, the normal points from B to A.
            var inverseDistance = new Vector<float>(-1f) / centerDistance;
            Vector3Wide.Scale(offsetB, inverseDistance, out normal);
            distance = centerDistance - a.Radius - b.Radius;

            var negativeRadiusA = -a.Radius;
            Vector3Wide.Scale(normal, negativeRadiusA, out closestA);
            intersected = Vector.LessThanOrEqual(distance, Vector<float>.Zero);
        }
    }

    //TODO: Could just use an analytic time of impact for two spheres if there's ever a performance issue.
}
