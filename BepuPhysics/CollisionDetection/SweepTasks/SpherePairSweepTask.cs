using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    //Note that sphere-sphere sweeps use a direct analytic implementation. The distance tester just exists because it's trivial and to maintain API consistency.
    public struct SpherePairDistanceTester : IPairDistanceTester<SphereWide, SphereWide>
    {
        public void Test(ref SphereWide a, ref SphereWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            Vector3Wide.Length(ref offsetB, out var centerDistance);
            //Note the negative 1. By convention, the normal points from B to A.
            var inverseDistance = new Vector<float>(-1f) / centerDistance;
            Vector3Wide.Scale(ref offsetB, ref inverseDistance, out normal);
            distance = centerDistance - a.Radius - b.Radius;

            var negativeRadiusA = -a.Radius;
            Vector3Wide.Scale(ref normal, ref negativeRadiusA, out closestA);
            intersected = Vector.LessThanOrEqual(distance, Vector<float>.Zero);
        }
    }

    public class SpherePairSweepTask : SweepTask
    {
        public SpherePairSweepTask()
        {
            ShapeTypeIndexA = default(Sphere).TypeId;
            ShapeTypeIndexB = default(Sphere).TypeId;
        }
        
        public override unsafe SweepResult Sweep<TSweepFilter>(
            void* shapeDataA, int shapeTypeA, in Quaternion orientationA, in BodyVelocity velocityA, 
            void* shapeDataB, int shapeTypeB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            ref TSweepFilter filter, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            return ConvexSweepTaskCommon.Sweep<Sphere, SphereWide, Sphere, SphereWide, SpherePairDistanceTester>(
                shapeDataA, shapeTypeA, orientationA, velocityA,
                shapeDataB, shapeTypeB, offsetB, orientationB, velocityB,
                maximumT, minimumProgression, convergenceThreshold, maximumIterationCount,
                out t0, out t1, out hitLocation, out hitNormal);
        }
    }
}
