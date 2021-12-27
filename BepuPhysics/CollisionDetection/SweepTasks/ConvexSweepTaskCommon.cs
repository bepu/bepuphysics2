using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public interface IPairDistanceTester<TShapeWideA, TShapeWideB>
    {
        void Test(in TShapeWideA a, in TShapeWideB b, in Vector3Wide offsetB, in QuaternionWide orientationA, in QuaternionWide orientationB, in Vector<int> inactiveLanes,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal);
    }

    public class ConvexPairSweepTask<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester> : SweepTask
            where TShapeA : unmanaged, IConvexShape
            where TShapeB : unmanaged, IConvexShape
            where TShapeWideA : unmanaged, IShapeWide<TShapeA>
            where TShapeWideB : unmanaged, IShapeWide<TShapeB>
            where TPairDistanceTester : struct, IPairDistanceTester<TShapeWideA, TShapeWideB>
    {
        public ConvexPairSweepTask()
        {
            ShapeTypeIndexA = default(TShapeA).TypeId;
            ShapeTypeIndexB = default(TShapeB).TypeId;
        }
        protected override unsafe bool PreorderedTypeSweep(
            void* shapeDataA, in RigidPose localPoseA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, in RigidPose localPoseB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            OffsetSweep sweepModifier = default;
            sweepModifier.LocalPoseA = localPoseA;
            sweepModifier.LocalPoseB = localPoseB;
            return Sweep(
                shapeDataA, orientationA, velocityA,
                shapeDataB, offsetB, orientationB, velocityB,
                maximumT, minimumProgression, convergenceThreshold, maximumIterationCount, ref sweepModifier,
                out t0, out t1, out hitLocation, out hitNormal);
        }

        protected override unsafe bool PreorderedTypeSweep<TSweepFilter>(
            void* shapeDataA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            bool requiresFlip, ref TSweepFilter filter, Shapes shapes, SweepTaskRegistry sweepTasks, BufferPool pool, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            UnoffsetSweep sweepModifier = default;
            return Sweep(
                shapeDataA, orientationA, velocityA,
                shapeDataB, offsetB, orientationB, velocityB,
                maximumT, minimumProgression, convergenceThreshold, maximumIterationCount, ref sweepModifier,
                out t0, out t1, out hitLocation, out hitNormal);
        }

        static bool GetSphereCastInterval(in Vector3 origin, in Vector3 direction, float radius, out float t0, out float t1)
        {
            //Normalize the direction. Sqrts aren't *that* bad, and it both simplifies things and helps avoid numerical problems.
            var dLength = direction.Length();
            if (dLength == 0)
            {
                //Zero length direction => miss or infinite length intersection.
                t0 = 0;
                t1 = float.MaxValue;
                return origin.LengthSquared() <= radius * radius;
            }
            var inverseDLength = 1f / dLength;
            var d = direction * inverseDLength;

            //Move the origin up to the earliest possible impact time. This isn't necessary for math reasons, but it does help avoid some numerical problems.
            var tOffset = -Vector3.Dot(origin, d) - radius;
            if (tOffset < 0)
                tOffset = 0;
            var o = origin + d * tOffset;
            var b = Vector3.Dot(o, d);
            var c = Vector3.Dot(o, o) - radius * radius;

            if (b > 0 && c > 0)
            {
                //Ray is outside and pointing away, no hit.
                t0 = t1 = 0;
                return false;
            }

            var discriminant = b * b - c;
            if (discriminant < 0)
            {
                //Ray misses, no hit.
                t0 = t1 = 0;
                return false;
            }
            var intervalRadius = (float)Math.Sqrt(discriminant);
            t0 = (tOffset - intervalRadius - b) * inverseDLength;
            t1 = (tOffset + intervalRadius - b) * inverseDLength;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetSampleTimes(float t0, float t1, ref Vector<float> samples)
        {
            ref var sampleBase = ref Unsafe.As<Vector<float>, float>(ref samples);
            var sampleSpacing = (t1 - t0) * (1f / (Vector<float>.Count - 1));
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                Unsafe.Add(ref sampleBase, i) = t0 + i * sampleSpacing;
            }
        }

        interface ISweepModifier
        {
            bool GetSphereCastInterval(
                in Vector3 offsetB, in Vector3 linearVelocityB, float maximumT, float maximumRadiusA, float maximumRadiusB,
                in Quaternion orientationA, in Vector3 angularVelocityA, float angularSpeedA,
                in Quaternion orientationB, in Vector3 angularVelocityB, float angularSpeedB, out float t0, out float t1, out Vector3 hitNormal, out Vector3 hitLocation);
            void ConstructSamples(float t0, float t1, ref Vector3Wide linearB, ref Vector3Wide angularA, ref Vector3Wide angularB,
                ref Vector3Wide initialOffsetB, ref QuaternionWide initialOrientationA, ref QuaternionWide initialOrientationB,
                ref Vector<float> samples, ref Vector3Wide sampleOffsetB, ref QuaternionWide sampleOrientationA, ref QuaternionWide sampleOrientationB);
            void GetNonlinearVelocityContribution(ref Vector3Wide normal,
                out Vector<float> velocityContributionA, out Vector<float> maximumDisplacementA,
                out Vector<float> velocityContributionB, out Vector<float> maximumDisplacementB);
            void AdjustHitLocation(in Quaternion initialOrientationA, in BodyVelocity velocityA, float t0, ref Vector3 hitLocation);
        }

        struct UnoffsetSweep : ISweepModifier
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void AdjustHitLocation(in Quaternion initialOrientationA, in BodyVelocity velocityA, float t0, ref Vector3 hitLocation)
            {
                hitLocation += t0 * velocityA.Linear;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void ConstructSamples(float t0, float t1, ref Vector3Wide linearB, ref Vector3Wide angularA, ref Vector3Wide angularB,
                ref Vector3Wide initialOffsetB, ref QuaternionWide initialOrientationA, ref QuaternionWide initialOrientationB,
                ref Vector<float> samples, ref Vector3Wide sampleOffsetB, ref QuaternionWide sampleOrientationA, ref QuaternionWide sampleOrientationB)
            {
                GetSampleTimes(t0, t1, ref samples);
                //Integrate offsetB to sample locations.
                Vector3Wide.Scale(linearB, samples, out var displacement);
                Vector3Wide.Add(initialOffsetB, displacement, out sampleOffsetB);

                //Integrate orientations to sample locations.
                var halfSamples = samples * 0.5f;
                PoseIntegration.Integrate(initialOrientationA, angularA, halfSamples, out sampleOrientationA);
                PoseIntegration.Integrate(initialOrientationB, angularB, halfSamples, out sampleOrientationB);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool GetSphereCastInterval(
                in Vector3 offsetB, in Vector3 linearVelocityB, float maximumT, float maximumRadiusA, float maximumRadiusB,
                in Quaternion orientationA, in Vector3 angularVelocityA, float angularSpeedA,
                in Quaternion orientationB, in Vector3 angularVelocityB, float angularSpeedB, out float t0, out float t1, out Vector3 hitNormal, out Vector3 hitLocation)
            {
                var hit = ConvexPairSweepTask<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester>.GetSphereCastInterval(offsetB, linearVelocityB, maximumRadiusA + maximumRadiusB, out t0, out t1);
                hitLocation = offsetB + linearVelocityB * t0;
                hitNormal = Vector3.Normalize(-hitLocation); //Normals are calibrated to point from B to A.
                hitLocation += hitNormal * maximumRadiusB;
                return hit;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void GetNonlinearVelocityContribution(ref Vector3Wide normal,
                out Vector<float> velocityContributionA, out Vector<float> maximumDisplacementA,
                out Vector<float> velocityContributionB, out Vector<float> maximumDisplacementB)
            {
                velocityContributionA = Vector<float>.Zero;
                maximumDisplacementA = Vector<float>.Zero;
                velocityContributionB = Vector<float>.Zero;
                maximumDisplacementB = Vector<float>.Zero;
            }
        }
        struct OffsetSweep : ISweepModifier
        {
            public RigidPose LocalPoseA;
            public RigidPose LocalPoseB;
            public float TangentSpeedA;
            public float TangentSpeedB;
            public float TwiceRadiusA;
            public float TwiceRadiusB;
            public Vector3 AngularVelocityDirectionA;
            public Vector3 AngularVelocityDirectionB;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void AdjustHitLocation(in Quaternion initialOrientationA, in BodyVelocity velocityA, float t0, ref Vector3 hitLocation)
            {
                PoseIntegration.Integrate(new RigidPose { Orientation = initialOrientationA }, velocityA, t0, out var integratedPose);
                QuaternionEx.Transform(LocalPoseA.Position, integratedPose.Orientation, out var childOffset);
                hitLocation = hitLocation + integratedPose.Position + childOffset;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void ConstructSamples(float t0, float t1, ref Vector3Wide linearB, ref Vector3Wide angularA, ref Vector3Wide angularB,
                ref Vector3Wide initialOffsetB, ref QuaternionWide initialOrientationA, ref QuaternionWide initialOrientationB,
                ref Vector<float> samples, ref Vector3Wide sampleOffsetB, ref QuaternionWide sampleOrientationA, ref QuaternionWide sampleOrientationB)
            {
                GetSampleTimes(t0, t1, ref samples);
                //Integrate offsetB to sample locations.
                Vector3Wide.Scale(linearB, samples, out var displacement);
                Vector3Wide.Add(initialOffsetB, displacement, out sampleOffsetB);

                //Note that the initial orientations are properties of the owning body, not of the child.
                //The orientation of the child itself is the product of localOrientation * bodyOrientation.
                var halfSamples = samples * 0.5f;
                RigidPoseWide.Broadcast(LocalPoseA, out var localPosesA);
                PoseIntegration.Integrate(initialOrientationA, angularA, halfSamples, out var integratedOrientationA);
                Compound.GetRotatedChildPose(localPosesA, integratedOrientationA, out var childPositionA, out sampleOrientationA);

                RigidPoseWide.Broadcast(LocalPoseB, out var localPosesB);
                PoseIntegration.Integrate(initialOrientationB, angularB, halfSamples, out var integratedOrientationB);
                Compound.GetRotatedChildPose(localPosesB, integratedOrientationB, out var childPositionB, out sampleOrientationB);

                Vector3Wide.Subtract(childPositionB, childPositionA, out var netOffsetB);
                Vector3Wide.Add(sampleOffsetB, netOffsetB, out sampleOffsetB);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool GetSphereCastInterval(
                in Vector3 offsetB, in Vector3 linearVelocityB, float maximumT, float maximumRadiusA, float maximumRadiusB,
                in Quaternion orientationA, in Vector3 angularVelocityA, float angularSpeedA,
                in Quaternion orientationB, in Vector3 angularVelocityB, float angularSpeedB, out float t0, out float t1, out Vector3 hitNormal, out Vector3 hitLocation)
            {
                //The tangent velocity magnitude doesn't change over the course of the sweep. Compute and cache it as an upper bound on the contribution from the offset.
                QuaternionEx.TransformWithoutOverlap(LocalPoseA.Position, orientationA, out var rA);
                var tangentA = Vector3.Cross(rA, angularVelocityA);
                TangentSpeedA = tangentA.Length();
                QuaternionEx.TransformWithoutOverlap(LocalPoseB.Position, orientationB, out var rB);
                var tangentB = Vector3.Cross(rB, angularVelocityB);
                TangentSpeedB = tangentB.Length();
                TwiceRadiusA = 2 * LocalPoseA.Position.Length();
                TwiceRadiusB = 2 * LocalPoseB.Position.Length();
                AngularVelocityDirectionA = angularSpeedA > 1e-8f ? angularVelocityA / angularSpeedA : new Vector3();
                AngularVelocityDirectionB = angularSpeedB > 1e-8f ? angularVelocityB / angularSpeedB : new Vector3();
                //The maximum translation due to angular velocity is at 180 degrees, so the maximum translation induced by angular motion is 2 * radius.
                //If the sweep covers a short enough duration that the maximum is not hit, we'll use a (loose) estimate based on extrapolating the tangent speed.
                var nonlinearExpansion = Math.Min(maximumT * (TangentSpeedA + TangentSpeedB), TwiceRadiusA + TwiceRadiusB);
                var offsetBIncludingChildPoses = offsetB + rB - rA;
                var hit = ConvexPairSweepTask<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester>.GetSphereCastInterval(offsetBIncludingChildPoses, linearVelocityB, maximumRadiusA + maximumRadiusB + nonlinearExpansion, out t0, out t1);
                hitLocation = offsetBIncludingChildPoses + linearVelocityB * t0;
                hitNormal = Vector3.Normalize(-hitLocation); //Normals are calibrated to point from B to A.
                hitLocation += hitNormal * (maximumRadiusB + nonlinearExpansion);
                return hit;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void GetNonlinearVelocityContribution(ref Vector3Wide normal,
                out Vector<float> velocityContributionA, out Vector<float> maximumDisplacementA,
                out Vector<float> velocityContributionB, out Vector<float> maximumDisplacementB)
            {
                //Constant angular velocity creates circular motion on a plane. That plane's normal is the angular velocity direction.
                //All offset-angular induced motion is constrained to this plane.
                //So, we can bound the tangent velocity contribution along the normal fairly tightly.
                //The same goes for the maximum displacement- 
                //if the angular velocity is perpendicular to the normal, displacement is no more than 2 * radius. As the N * W/||W|| dot goes to 1, the maximum displacement goes to 0.
                //(Note that you could bound this even more tightly by noting that 2 * radius is only hit when R/||R|| * NOnPlane = 1. Just didn't go that far (yet).)
                Vector3Wide.Broadcast(AngularVelocityDirectionA, out var directionA);
                Vector3Wide.Dot(normal, directionA, out var dotA);
                var scaleA = Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One - dotA * dotA));
                velocityContributionA = new Vector<float>(TangentSpeedA) * scaleA;
                maximumDisplacementA = new Vector<float>(TwiceRadiusA) * scaleA;
                Vector3Wide.Broadcast(AngularVelocityDirectionB, out var directionB);
                Vector3Wide.Dot(normal, directionB, out var dotB);
                var scaleB = Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One - dotB * dotB));
                velocityContributionB = new Vector<float>(TangentSpeedB) * scaleB;
                maximumDisplacementB = new Vector<float>(TwiceRadiusB) * scaleB;
            }
        }

        static unsafe bool Sweep<TSweepModifier>(
            void* shapeDataA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB,
            float maximumT, float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            ref TSweepModifier sweepModifier,
            out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
            where TSweepModifier : struct, ISweepModifier
        {
            ref var shapeA = ref Unsafe.AsRef<TShapeA>(shapeDataA);
            ref var shapeB = ref Unsafe.AsRef<TShapeB>(shapeDataB);
            //TODO: Would be nice to get rid of this pointless zero init (if the compiler doesn't eventually get rid of it).
            var wideA = default(TShapeWideA);
            var wideB = default(TShapeWideB);
            //TODO: Check to make sure the JIT omits the branch.
            if (wideA.InternalAllocationSize > 0)
            {
                var memory = stackalloc byte[wideA.InternalAllocationSize];
                wideA.Initialize(new Buffer<byte>(memory, wideA.InternalAllocationSize));
            }
            if (wideB.InternalAllocationSize > 0)
            {
                var memory = stackalloc byte[wideB.InternalAllocationSize];
                wideB.Initialize(new Buffer<byte>(memory, wideB.InternalAllocationSize));
            }
            wideA.Broadcast(shapeA);
            wideB.Broadcast(shapeB);
            var pairTester = default(TPairDistanceTester);
            //Initialize the interval to the tighter of 1) input bounds [0, maximumT] and 2) the swept impact interval of the bounding spheres of the two shapes.
            //Note that the intersection interval of two swept spheres is equivalent to performing a single ray cast against a sphere with a combined radius.
            //Unlike a traditional sphere-ray test, we keep both the near and far intervals.
            //Treat shapeA's position (at zero) as the target sphere with combined radius. The direction is then the net linear motion.
            //Note that angular motion produces nonlinear relative displacement due to the offsets.
            //Under the assumption that angular velocity tends to be relatively low, we expand the test radius by the maximum contribution of angular motion over the interval.
            //Note that the maximum contribution occurs at 180 degrees.
            var linearVelocityB = velocityB.Linear - velocityA.Linear;
            shapeA.ComputeAngularExpansionData(out var maximumRadiusA, out var maximumAngularExpansionA);
            shapeB.ComputeAngularExpansionData(out var maximumRadiusB, out var maximumAngularExpansionB);
            var angularSpeedA = velocityA.Angular.Length();
            var angularSpeedB = velocityB.Angular.Length();
            if (!sweepModifier.GetSphereCastInterval(
                offsetB, linearVelocityB, maximumT, maximumRadiusA, maximumRadiusB,
                orientationA, velocityA.Angular, angularSpeedA,
                orientationB, velocityB.Angular, angularSpeedB, out t0, out t1, out hitNormal, out hitLocation) || t0 > maximumT || t1 < 0)
            {
                //The bounding spheres do not intersect, or the intersection interval is outside of the requested search interval.
                hitLocation = default;
                hitNormal = default;
                //Console.WriteLine("Early out.");
                return false;
            }
            //Clamp the interval to the intended search space.
            if (t0 < 0)
                t0 = 0;
            if (t1 > maximumT)
                t1 = maximumT;

            //We now have a relatively tight bracket (not much larger than the involved shapes, at least).
            //At a high level, the following sweep uses two parts:
            //1) Conservatively advance interval endpoints toward the root by computing the distance and normal at each point and then pulling the interval
            //as much as is permitted by the normal/distance and velocity.
            //2) If any sample finds an intersection, immediately pull t1 all the way up to it.
            //Continue taking samples until t1 < t0 and a miss is guaranteed, or until the process converges to an intersection within the required epsilon.

            //There are some tricky bits that make this faster than a typical pure conservative approach:
            //1) "Speculative" samples are used. Instead of sampling a known safe location, samples are forced out ahead and then validated after the fact.
            //(To understand this validation, consider each sample's distance and normal as defining a 'safe' interval where there is guaranteed to be no intersection.
            //A conservative sample might only be able to advance so far, but a speculative sample further out may discover that its safe interval extends backwards
            //to overlap the conservative sample's safe interval. Since there can be no intersection in a safe interval, you can merge those safe intervals and more tightly bound 
            //potential intersections.)
            //2) Samples are forced to make nonzero progress by the provided root finding epsilon. In other words, the interval narrowing is not strictly conservative.
            //This means intersections below a certain threshold may be missed entirely, but also puts a low upper bound on the number of iterations needed to find an intersection.
            //For CCD or similar purposes, glancing collisions hardly matter, so being able to choose a larger epsilon is nice. 
            //Note that this does NOT imply that the impact time is imprecise- impact precision is controlled by the other provided epsilon, and once you detect any intersection,
            //the interval should narrow extremely rapidly to arbitrary precision.
            //3) Samples are taken in a wide SIMD fashion. Rather than performing only one sample at a time, Vector<float>.Count samples are distributed.
            //(This doesn't correspond to a linear convergence speed up with SIMD width since the SIMD samples must be done in parallel, but it is significant.)

            var tangentSpeedA = new Vector<float>(maximumRadiusA * angularSpeedA);
            var tangentSpeedB = new Vector<float>(maximumRadiusB * angularSpeedB);
            var maxAngularExpansionA = new Vector<float>(maximumAngularExpansionA);
            var maxAngularExpansionB = new Vector<float>(maximumAngularExpansionB);

            Vector3Wide.Broadcast(offsetB, out var initialOffsetB);
            QuaternionWide.Broadcast(orientationA, out var initialOrientationA);
            QuaternionWide.Broadcast(orientationB, out var initialOrientationB);
            Vector3Wide.Broadcast(linearVelocityB, out var wideLinearVelocityB);
            Vector3Wide.Broadcast(velocityA.Angular, out var wideAngularVelocityA);
            Vector3Wide.Broadcast(velocityB.Angular, out var wideAngularVelocityB);

            //The initial samples are distributed evenly over [t0, t1].
            Unsafe.SkipInit(out Vector<float> samples);
            Unsafe.SkipInit(out Vector3Wide sampleOffsetB);
            Unsafe.SkipInit(out QuaternionWide sampleOrientationA);
            Unsafe.SkipInit(out QuaternionWide sampleOrientationB);
            var minimumProgressionWide = new Vector<float>(minimumProgression);

            float next0 = t0;
            float next1 = t1;
            sweepModifier.ConstructSamples(t0, t1,
                ref wideLinearVelocityB, ref wideAngularVelocityA, ref wideAngularVelocityB,
                ref initialOffsetB, ref initialOrientationA, ref initialOrientationB,
                ref samples, ref sampleOffsetB, ref sampleOrientationA, ref sampleOrientationB);

            bool intersectionEncountered = false;
            int iterationIndex = 0;
            while (true)
            {
                pairTester.Test(wideA, wideB, sampleOffsetB, sampleOrientationA, sampleOrientationB, Vector<int>.Zero,
                    out var intersections, out var distances, out var closestA, out var normals);

                Vector3Wide.Dot(normals, wideLinearVelocityB, out var linearVelocityAlongNormal);
                sweepModifier.GetNonlinearVelocityContribution(ref normals,
                    out var nonlinearVelocityContributionA, out var nonlinearMaximumDisplacementA,
                    out var nonlinearVelocityContributionB, out var nonlinearMaximumDisplacementB);
                //Note that, for any given timespan, the maximum displacement of any point on a body is bounded.
                //timeToNext = distances / 
                //  (linearContribution + 
                //   min(tangentSpeedA, maximumAngularExpansionA / timeToNext) + 
                //   min(tangentSpeedB, maximumAngularExpansionB / timeToNext))
                //The right hand side's dependency on timeToNext requires some extra effort to solve.
                //We construct a worst case for shape A, shape B, and then both A and B, and then choose the most aggressive next time.
                //For example, if both shapes were rotating fast enough that they reached maximum angular expansion:
                //timeToNext = distances / (linearContribution + (maximumAngularExpansionA + maximumAngularExpansionB) / timeToNext)
                //timeToNext * linearContribution + timeToNext * (maximumAngularExpansionA + maximumAngularExpansionB) / timeToNext) = distances
                //timeToNext * linearContribution + (maximumAngularExpansionA + maximumAngularExpansionB)) = distances
                //timeToNext = (distances - (maximumAngularExpansionA + maximumAngularExpansionB)) / linearContribution                
                var aWorstCaseDistances = Vector.Max(Vector<float>.Zero, distances - maxAngularExpansionA - nonlinearMaximumDisplacementA);
                var angularDisplacementB = maxAngularExpansionB + nonlinearMaximumDisplacementB;
                var bWorstCaseDistances = Vector.Max(Vector<float>.Zero, distances - angularDisplacementB);
                var bothWorstCaseDistances = Vector.Max(Vector<float>.Zero, aWorstCaseDistances - angularDisplacementB);
                //Divisions by zero velocity should result in a large (but finite) safe interval.
                //Negative (separating) velocity can be treated in the same way.
                var divisionGuard = new Vector<float>(1e-15f);
                var bothWorstCaseNextTime = bothWorstCaseDistances / Vector.Max(divisionGuard, linearVelocityAlongNormal);
                var angularContributionA = nonlinearVelocityContributionA + tangentSpeedA;
                var angularContributionB = nonlinearVelocityContributionB + tangentSpeedB;
                var aWorstCaseNextTime = aWorstCaseDistances / Vector.Max(divisionGuard, (linearVelocityAlongNormal + angularContributionB));
                var bWorstCaseNextTime = bWorstCaseDistances / Vector.Max(divisionGuard, (linearVelocityAlongNormal + angularContributionA));
                var bestCaseNextTime = distances / Vector.Max(divisionGuard, linearVelocityAlongNormal + angularContributionA + angularContributionB);
                var timeToNext = Vector.Max(Vector.Max(bothWorstCaseNextTime, aWorstCaseNextTime), Vector.Max(bWorstCaseNextTime, bestCaseNextTime));
                var aWorstCasePreviousTime = aWorstCaseDistances / Vector.Max(divisionGuard, angularContributionB - linearVelocityAlongNormal);
                var bWorstCasePreviousTime = bWorstCaseDistances / Vector.Max(divisionGuard, angularContributionA - linearVelocityAlongNormal);
                var bestCasePreviousTime = distances / Vector.Max(divisionGuard, angularContributionA + angularContributionB - linearVelocityAlongNormal);
                var timeToPrevious = Vector.Max(Vector.Max(-bothWorstCaseNextTime, aWorstCasePreviousTime), Vector.Max(bWorstCasePreviousTime, bestCasePreviousTime));
                var safeIntervalStart = samples - timeToPrevious;
                var safeIntervalEnd = samples + timeToNext;
                var forcedIntervalEnd = samples + Vector.Max(timeToNext, minimumProgressionWide);
                if (intersections[0] < 0)
                {
                    //First sample was intersected, can't do anything with the rest of the interval shoving on this iteration.
                    next1 = samples[0];
                    intersectionEncountered = true;
                }
                else
                {
                    int firstIntersectingIndex = Vector<float>.Count;
                    for (int i = 0; i < Vector<float>.Count; ++i)
                    {
                        if (intersections[i] < 0)
                        {
                            //An intersection was found. Pull the interval endpoint all the way up. No point in looking at further samples.
                            firstIntersectingIndex = i;
                            Debug.Assert(samples[i] >= t0);
                            next1 = samples[i];
                            intersectionEncountered = true;
                            break;
                        }
                    }
                    int lastSafeIndex = 0;
                    for (int i = 0; i < firstIntersectingIndex; ++i)
                    {
                        lastSafeIndex = i;
                        var nextIndex = i + 1;
                        if (nextIndex < firstIntersectingIndex)
                        {
                            //Note that we use the forced interval for testing safe traversal.
                            //This allows the root finder to skip small intersections for a substantial speedup in pathological cases.
                            if (safeIntervalStart[i + 1] > forcedIntervalEnd[i])
                            {
                                //Can't make it to the next sample. We've pushed t0 as far as it can go.
                                break;
                            }
                        }
                    }
                    //Note that it's sometimes possible for the safe interval to extend past t1 even when intersection has occurred due to numerical issues.
                    //This isn't catastrophic for the sweep, but it does mean we can't validate that condition here.
                    Debug.Assert((safeIntervalEnd[lastSafeIndex] >= t0) || !intersectionEncountered);
                    next0 = safeIntervalEnd[lastSafeIndex];
                    //Copy the best normal into the output variable. We're going to overwrite all the wide normals in the next iteration, but we need to keep the best guess around.
                    hitNormal = new Vector3(normals.X[lastSafeIndex], normals.Y[lastSafeIndex], normals.Z[lastSafeIndex]);
                    hitLocation = new Vector3(closestA.X[lastSafeIndex], closestA.Y[lastSafeIndex], closestA.Z[lastSafeIndex]);

                    if (!intersectionEncountered)
                    {
                        //If no intersection has yet been detected, we can pull t1 forward to narrow the sample range.      
                        for (int i = Vector<float>.Count - 1; i >= 0; --i)
                        {
                            next1 = safeIntervalStart[i];
                            //Note that we use the forced interval for testing safe traversal.
                            //This allows the root finder to skip small intersections for a substantial speedup in pathological cases.
                            if (i > 0 && forcedIntervalEnd[i - 1] < next1)
                            {
                                //Can't make it to the next sample. We've pushed t1 as far as it can go.
                                break;
                            }
                        }
                    }
                }

                //The sampling region is initialized to nonconservative bounds.
                //The true sample interval computation completes later; this is here just so we can set t0 and t1 and potentially early out. 
                var sample0 = t0 + minimumProgression;
                var sample1 = t1 - minimumProgression;
                var previousIntervalSpan = t1 - t0;
                //In pure linear cast cases, advancing all the way to the conservative bound can result in collision. That won't make the detected t value wrong,
                //but it will mean that we can't get a more accurate normal and closest point. By only advancing within an epsilon, another iteration is forced.
                t0 += (next0 - t0) * 0.9999f;
                t1 = next1;

                var intervalSpan = t1 - t0;
                //A few different termination conditions:
                //1) The span has inverted, implying that a safe path has been found through the entire search interval and there is no intersection
                //2) There has been an intersection, and the interval span is small enough to pass the requested epsilon. Note that this means that misses cannot exit on 'convergence'.
                //3) The interval has not shrunk at all. This will happen when numerical precision is exhausted. No point in continuing; the machine can't represent anything better.
                //4) Out of iterations.
                //Console.WriteLine($"Iteration narrowing: {previousIntervalSpan / intervalSpan}");
                if (intervalSpan < 0 ||
                    (intersectionEncountered && intervalSpan < convergenceThreshold) ||
                    intervalSpan >= previousIntervalSpan ||
                    ++iterationIndex >= maximumIterationCount)
                {
                    break;
                }

                //Now we need to clean up the aggressive sampling interval. Get rid of any inversions.
                if (sample0 < t0)
                    sample0 = t0;
                else if (sample0 > t1)
                    sample0 = t1;
                if (sample1 < t0)
                    sample1 = t0;
                else if (sample1 > t1)
                    sample1 = t1;

                var minimumSpan = minimumProgression * (Vector<float>.Count - 1);
                var sampleSpan = sample1 - sample0;
                if (sampleSpan < minimumSpan)
                {
                    //We've reached the point where individual samples are crowded below the minimum progression.
                    //Try to make room by pushing sample 0 back toward the conservative bound.
                    sample0 -= (minimumSpan - sampleSpan);
                    if (sample0 < t0)
                        sample0 = t0;
                    sampleSpan = sample1 - sample0;
                    //Now check if we need to move sample1 toward t1.
                    //Note that we nver push sample1 all the way to t1. t1 is often in intersection, so taking another sample there has no value.
                    //Instead, we only move up to halfway there.
                    if (sampleSpan < minimumSpan)
                        sample1 += Math.Min(minimumSpan - sampleSpan, (t1 - sample1) * 0.5f);
                }

                //The sample bounds are now constrained to be an aggressive subset of the conservative bounds.
                sweepModifier.ConstructSamples(sample0, sample1,
                    ref wideLinearVelocityB, ref wideAngularVelocityA, ref wideAngularVelocityB,
                    ref initialOffsetB, ref initialOrientationA, ref initialOrientationB,
                    ref samples, ref sampleOffsetB, ref sampleOrientationA, ref sampleOrientationB);
            }
            //Console.WriteLine($"iteration count: {iterationIndex}");
            //If there was an intersection, we need to correct the hit location for the sample location.
            sweepModifier.AdjustHitLocation(orientationA, velocityA, t0, ref hitLocation);
            return intersectionEncountered;
        }


    }
}
