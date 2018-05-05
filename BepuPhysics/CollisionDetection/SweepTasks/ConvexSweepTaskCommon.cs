using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public interface IPairDistanceTester<TShapeWideA, TShapeWideB>
    {
        void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide normal);
    }

    class ConvexSweepTaskCommon
    {
        static bool GetSphereCastInterval(ref Vector3 origin, ref Vector3 direction, float radius, out float t0, out float t1)
        {
            //Normalize the direction. Sqrts aren't *that* bad, and it both simplifies things and helps avoid numerical problems.
            var inverseDLength = 1f / direction.Length();
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
        static void ConstructSamples(float t0, float t1,
            ref Vector3Wide linearVelocityB, ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB,
            ref Vector3Wide initialOffsetB, ref QuaternionWide initialOrientationA, ref QuaternionWide initialOrientationB,
            ref Vector<float> samples, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB)
        {
            ref var sampleBase = ref Unsafe.As<Vector<float>, float>(ref samples);
            var sampleSpacing = (t1 - t0) * (1f / (Vector<float>.Count - 1));
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                Unsafe.Add(ref sampleBase, i) = t0 + i * sampleSpacing;
            }
            //Integrate offsetB to sample locations.
            Vector3Wide.Scale(ref linearVelocityB, ref samples, out var displacement);
            Vector3Wide.Add(ref initialOffsetB, ref displacement, out offsetB);

            //Integrate orientations to sample locations.
            var halfSamples = samples * 0.5f;
            QuaternionWide multiplier;
            multiplier.X = angularVelocityA.X * halfSamples;
            multiplier.Y = angularVelocityA.Y * halfSamples;
            multiplier.Z = angularVelocityA.Z * halfSamples;
            multiplier.W = Vector<float>.Zero;
            QuaternionWide.ConcatenateWithoutOverlap(ref initialOrientationA, ref multiplier, out var incrementA);
            QuaternionWide.Add(ref initialOrientationA, ref incrementA, out orientationA);
            QuaternionWide.Normalize(ref orientationA, out orientationA);
            QuaternionWide.ConcatenateWithoutOverlap(ref initialOrientationB, ref multiplier, out var incrementB);
            QuaternionWide.Add(ref initialOrientationB, ref incrementB, out orientationB);
            QuaternionWide.Normalize(ref orientationB, out orientationB);
        }

        public static unsafe bool Sweep<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester>(
            void* shapeDataA, int shapeTypeA, ref BepuUtilities.Quaternion orientationA, ref BodyVelocity velocityA,
            void* shapeDataB, int shapeTypeB, ref Vector3 offsetB, ref BepuUtilities.Quaternion orientationB, ref BodyVelocity velocityB,
            float maximumT, float minimumProgression, float convergenceThreshold, int maximumIterationCount, out float t0, out float t1, out Vector3 normal)
            where TShapeA : struct, IConvexShape
            where TShapeB : struct, IConvexShape
            where TShapeWideA : struct, IShapeWide<TShapeA>
            where TShapeWideB : struct, IShapeWide<TShapeB>
            where TPairDistanceTester : struct, IPairDistanceTester<TShapeWideA, TShapeWideB>
        {
            Debug.Assert(shapeTypeA == default(TShapeA).TypeId && shapeTypeB == default(TShapeB).TypeId, "This sweep test requires input of a specific type.");
            ref var shapeA = ref Unsafe.AsRef<TShapeA>(shapeDataA);
            ref var shapeB = ref Unsafe.AsRef<TShapeB>(shapeDataB);
            //TODO: Would be nice to get rid of this pointless zero init (if the compiler doesn't eventually get rid of it).
            var wideA = default(TShapeWideA);
            var wideB = default(TShapeWideB);
            wideA.Broadcast(ref shapeA);
            wideB.Broadcast(ref shapeB);
            var pairTester = default(TPairDistanceTester);
            normal = default;

            //Initialize the interval to the tighter of 1) input bounds [0, maximumT] and 2) the swept impact interval of the bounding spheres of the two shapes.
            //Note that the intersection interval of two swept spheres is equivalent to performing a single ray cast against a sphere with a combined radius.
            //Unlike a traditional sphere-ray test, we keep both the near and far intervals.
            //Treat shapeA's position (at zero) as the target sphere with combined radius. The direction is then the net linear motion.
            var linearVelocityB = velocityB.Linear - velocityA.Linear;
            shapeA.ComputeAngularExpansionData(out var maximumRadiusA, out var maximumAngularExpansionA);
            shapeB.ComputeAngularExpansionData(out var maximumRadiusB, out var maximumAngularExpansionB);
            if (!GetSphereCastInterval(ref offsetB, ref linearVelocityB, maximumRadiusA + maximumRadiusB, out t0, out t1) || t0 > maximumT || t1 < 0)
            {
                //The bounding spheres do not intersect, or the intersection interval is outside of the requested search interval.
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

            var tangentSpeedA = new Vector<float>(maximumRadiusA * velocityA.Angular.Length());
            var tangentSpeedB = new Vector<float>(maximumRadiusB * velocityB.Angular.Length());
            var maxAngularExpansionA = new Vector<float>(maximumAngularExpansionA);
            var maxAngularExpansionB = new Vector<float>(maximumAngularExpansionB);

            Vector3Wide.Broadcast(offsetB, out var initialOffsetB);
            QuaternionWide.Broadcast(orientationA, out var initialOrientationA);
            QuaternionWide.Broadcast(orientationB, out var initialOrientationB);
            Vector3Wide.Broadcast(linearVelocityB, out var wideLinearVelocityB);
            Vector3Wide.Broadcast(velocityA.Angular, out var wideAngularVelocityA);
            Vector3Wide.Broadcast(velocityB.Angular, out var wideAngularVelocityB);

            //The initial samples are distributed evenly over [t0, t1].
            Vector<float> samples;
            Vector3Wide sampleOffsetB;
            QuaternionWide sampleOrientationA;
            QuaternionWide sampleOrientationB;
            Vector3Wide normals;
            Vector<float> distances;
            Vector<int> intersections;
            var minimumProgressionWide = new Vector<float>(minimumProgression);

            ConstructSamples(t0, t1,
                ref wideLinearVelocityB, ref wideAngularVelocityA, ref wideAngularVelocityB,
                ref initialOffsetB, ref initialOrientationA, ref initialOrientationB,
                ref samples, ref sampleOffsetB, ref sampleOrientationA, ref sampleOrientationB);

            bool intersectionEncountered = false;
            for (int iterationIndex = 0; iterationIndex < maximumIterationCount; ++iterationIndex)
            {
                pairTester.Test(ref wideA, ref wideB, ref sampleOffsetB, ref sampleOrientationA, ref sampleOrientationB, out intersections, out distances, out normals);

                Vector3Wide.Dot(ref normals, ref wideLinearVelocityB, out var linearVelocityAlongNormal);
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
                var aWorstCaseDistances = distances - maxAngularExpansionA;
                var bWorstCaseDistances = distances - maxAngularExpansionB;
                var bothWorstCaseDistances = aWorstCaseDistances - maxAngularExpansionB;
                var bothWorstCaseNextTime = bothWorstCaseDistances / linearVelocityAlongNormal;
                var aWorstCaseNextTime = aWorstCaseDistances / (linearVelocityAlongNormal + tangentSpeedB);
                var bWorstCaseNextTime = bWorstCaseDistances / (linearVelocityAlongNormal + tangentSpeedA);
                var bestCaseNextTime = distances / (linearVelocityAlongNormal + tangentSpeedA + tangentSpeedB);
                var timeToNext = Vector.Max(Vector.Max(bothWorstCaseNextTime, aWorstCaseNextTime), Vector.Max(bWorstCaseNextTime, bestCaseNextTime));

                var aWorstCasePreviousTime = aWorstCaseDistances / (tangentSpeedB - linearVelocityAlongNormal);
                var bWorstCasePreviousTime = bWorstCaseDistances / (tangentSpeedA - linearVelocityAlongNormal);
                var bestCasePreviousTime = distances / (tangentSpeedA + tangentSpeedB - linearVelocityAlongNormal);
                var timeToPrevious = Vector.Max(Vector.Max(-bothWorstCaseNextTime, aWorstCasePreviousTime), Vector.Max(bWorstCasePreviousTime, bestCasePreviousTime));

                var safeIntervalStart = samples - timeToPrevious;
                var safeIntervalEnd = samples + timeToNext;
                var forcedIntervalEnd = samples + Vector.Max(timeToNext, minimumProgressionWide);
                if (intersections[0] < 0)
                {
                    //First sample was intersected, can't do anything with the rest of the interval shoving on this iteration.
                    t1 = samples[0];
                    intersectionEncountered = true;
                }
                else
                {
                    int lastSafeIndex = 0;
                    for (int i = 0; i < Vector<float>.Count; ++i)
                    {
                        lastSafeIndex = i;
                        if (i < Vector<float>.Count - 1)
                        {
                            var nextIndex = i + 1;
                            if (intersections[nextIndex] < 0)
                            {
                                //An intersection was found. Pull the interval endpoint all the way up. No point in looking at further samples.
                                t1 = samples[nextIndex];
                                intersectionEncountered = true;
                                break;
                            }
                            //Note that we use the forced interval for testing safe traversal.
                            //This allows the root finder to skip small intersections for a substantial speedup in pathological cases.
                            if (safeIntervalStart[i + 1] > forcedIntervalEnd[i])
                            {
                                //Can't make it to the next sample. We've pushed t0 as far as it can go.
                                break;
                            }
                        }
                    }
                    t0 = safeIntervalEnd[lastSafeIndex];
                    //Copy the best normal into the output variable. We're going to overwrite all the wide normals in the next iteration, but we need to keep the best guess around.
                    normal.X = normals.X[lastSafeIndex];
                    normal.Y = normals.Y[lastSafeIndex];
                    normal.Z = normals.Z[lastSafeIndex];
                    if (!intersectionEncountered)
                    {
                        //If no intersection has yet been detected, we can pull t1 forward to narrow the sample range.        
                        for (int i = Vector<float>.Count - 1; i >= 0; --i)
                        {
                            t1 = safeIntervalStart[i];
                            //Note that we use the forced interval for testing safe traversal.
                            //This allows the root finder to skip small intersections for a substantial speedup in pathological cases.
                            if (i > 0 && forcedIntervalEnd[i - 1] < t1)
                            {
                                //Can't make it to the next sample. We've pushed t1 as far as it can go.
                                break;
                            }
                        }
                    }
                }

                //If the interval has narrowed below the convergence threshold, we can quit.
                if (t1 - t0 <= convergenceThreshold)
                {
                    break;
                }

                //Not done yet; construct the new samples.
                //TODO: Need nonconservative sample bounds- t0 and t1 only advance up to safe points, but the sampling region will be a subset of that conservative region.
                //In other words, 
                //nextSample0 = t0 + max(safeProgress0, minimumProgress), nextSample1 = t1 - max(safeProgress1, minimumProgress)
                //t0 += safeProgress0, t1 -= safeProgress1
                //Note that if an intersection has been found, then t1 won't progress, but the nextSample1 will still move up by minimumProgress.
                //(If nextSample1 ends up less than nextSample0, arbitrarily stick it at a point in between nextSample0 and the old t1.)
            }
            return intersectionEncountered;
        }
    }
}
