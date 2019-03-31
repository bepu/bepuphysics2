using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    public struct PlaneWalkerStep
    {
        public Vector3 PreviousNormal;
        public Vector3 PreviousSupport;
        public Vector3 Normal;
        public Vector3 Support;
        public Vector3 InterpolatedNormal;
        public Vector3 InterpolatedSupport;
        public float BestDepth;
        public float NewestDepth;
        public bool Improved;
        public Vector3 PointOnOriginLine;
        public Vector3 NextNormal;
        public float Progression;
    }

    public static class PlaneWalker<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
        where TShapeA : IConvexShape
        where TShapeWideA : IShapeWide<TShapeA>
        where TSupportFinderA : ISupportFinder<TShapeA, TShapeWideA>
        where TShapeB : IConvexShape
        where TShapeWideB : IShapeWide<TShapeB>
        where TSupportFinderB : ISupportFinder<TShapeB, TShapeWideB>
    {


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FindSupport(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB, in Vector3Wide direction, 
            in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            //support(N, A) - support(-N, B)
            supportFinderA.ComputeLocalSupport(a, direction, terminatedLanes, out var extremeA);
            Vector3Wide.Negate(direction, out var negatedDirection);
            supportFinderB.ComputeSupport(b, localOrientationB, negatedDirection, terminatedLanes, out var extremeB);
            Vector3Wide.Add(extremeB, localOffsetB, out extremeB);

            Vector3Wide.Subtract(extremeA, extremeB, out support);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            in Vector3Wide initialNormal, in Vector<int> inactiveLanes, in Vector<float> convergenceThreshold, in Vector<float> minimumDepthThreshold, out Vector<float> depth, out Vector3Wide refinedNormal, List<PlaneWalkerStep> steps, int maximumIterations = 50)
        {
#if DEBUG
            Vector3Wide.LengthSquared(initialNormal, out var initialNormalLengthSquared);
            Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThan(Vector.Abs(initialNormalLengthSquared - Vector<float>.One), new Vector<float>(1e-6f))), Vector<int>.Zero));
#endif
            FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, initialNormal, inactiveLanes, out var initialSupport);
            Vector3Wide.Dot(initialSupport, initialNormal, out var initialDepth);
            FindMinimumDepth(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, initialNormal, initialSupport, initialDepth, inactiveLanes, convergenceThreshold, minimumDepthThreshold, out depth, out refinedNormal, steps, maximumIterations);
        }


        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            in Vector3Wide initialNormal, in Vector3Wide initialSupport, in Vector<float> initialDepth,
            in Vector<int> inactiveLanes, in Vector<float> convergenceThreshold, in Vector<float> minimumDepthThreshold, out Vector<float> depth, out Vector3Wide refinedNormal, List<PlaneWalkerStep> steps, int maximumIterations = 30)
        {
            //The idea behind this algorithm is to greedily hillclimb the penetration depth function in minkowski space.
            //Using a strict gradient descent is hard to generalize due to the poorly behaving gradients and the tendency to converge extremely slowly in valleys (which are very common).
            //Other techniques requiring well behaved analytic second derivatives (newton) or well behaved approximations (quasinewton) tend to do poorly.
            //Nelder-Mead actually works pretty well since it treats the depth function as a black box, but it requires quite a few iterations to converge- it tends to be 2-4x times slower than we want.

            //So, what options do we have? The support function provides an easy way to calculate the extreme point and interval overlap (depth).
            //depth(sampleDirection) = dot(sampleDirection / ||sampleDirection||, FindSupport(sampleDirection) - origin)
            //Consider each support point as a bounding plane- if you sample a convex shape's extreme point along a direction, you know that no point is further along that direction.
            //We want to pull the bounding plane closer to the origin. If no incremental change to the sample direction (the bounding plane) allows the depth to improve, then we've found a local minimum.

            //Conceptually, you can think of the support point found by FindSupport(sampleDirection) as a pivot, 
            //and then we try to push the plane (at the point closest on the plane to the origin) toward (or through) the origin.
            //The key bit of extra information that we're now using is the support point and its associated plane to guide us toward a minimum.

            //Note that this process will successfully converge to the global minimum in the separated case since the separated distance function is convex.
            //In the penetrating case, this will converge to a local minimum that may not be the global minimum. In practice, so long as the initial guess is reasonable,
            //the resulting local minimum is generally good enough.

#if DEBUG
            Vector3Wide.LengthSquared(initialNormal, out var initialNormalLengthSquared);
            Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThan(Vector.Abs(initialNormalLengthSquared - Vector<float>.One), new Vector<float>(1e-6f))), Vector<int>.Zero));
#endif
            refinedNormal = initialNormal;
            var previousNormal = initialNormal;
            var previousSupport = initialSupport;
            depth = initialDepth;


            var terminatedLanes = inactiveLanes;
            var progressionScale = new Vector<float>(0.25f);

            Vector3Wide.Scale(previousNormal, depth, out var closestPointOnPlane);
            Vector3Wide.Subtract(closestPointOnPlane, previousSupport, out var supportToClosestPoint);
            Vector3Wide.Length(supportToClosestPoint, out var offsetLength);

            Vector3Wide.Scale(previousNormal, depth - progressionScale * offsetLength, out var pointOnOriginLine);
            Vector3Wide.Subtract(pointOnOriginLine, previousSupport, out var supportToPointOnOriginLine);
            Vector3Wide.CrossWithoutOverlap(previousNormal, supportToPointOnOriginLine, out var n);
            Vector3Wide.CrossWithoutOverlap(supportToPointOnOriginLine, n, out var normal);

            //TODO: This could be replaced by a faster rsqrt with platform intrinsics.
            Vector3Wide.Length(normal, out var newNormalLength);
            Vector3Wide.Scale(normal, Vector<float>.One / newNormalLength, out normal);

            //If the offset length is zero, the normal length will be zero and the latest support point is the closest point to the origin.
            //This is sufficient to guarantee that no further progress is possible and we can early out.
            //(Note that for shapes with flat faces, this termination condition may not be met- the support point is not unique.)
            var invalidNormal = Vector.LessThan(newNormalLength, new Vector<float>(1e-10f));
            var depthBelowThreshold = Vector.LessThan(depth, minimumDepthThreshold);
            terminatedLanes = Vector.BitwiseOr(Vector.BitwiseOr(invalidNormal, depthBelowThreshold), terminatedLanes);

            if (steps != null)
            {
                PlaneWalkerStep step = default;
                Vector3Wide.ReadSlot(ref previousNormal, 0, out step.Normal);
                Vector3Wide.ReadSlot(ref previousNormal, 0, out step.PreviousNormal);
                Vector3Wide.ReadSlot(ref previousNormal, 0, out step.InterpolatedNormal);
                Vector3Wide.ReadSlot(ref previousSupport, 0, out step.Support);
                Vector3Wide.ReadSlot(ref previousSupport, 0, out step.PreviousSupport);
                Vector3Wide.ReadSlot(ref previousSupport, 0, out step.InterpolatedSupport);
                step.BestDepth = depth[0];
                step.NewestDepth = depth[0];
                step.Improved = true;
                Vector3Wide.ReadSlot(ref pointOnOriginLine, 0, out step.PointOnOriginLine);
                Vector3Wide.ReadSlot(ref normal, 0, out step.NextNormal);
                step.NextNormal = Vector3.Normalize(step.NextNormal);
                step.Progression = progressionScale[0];
                steps.Add(step);
            }

            if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
            {
                refinedNormal = initialNormal;
                return;
            }

            for (int i = 0; i < maximumIterations; ++i)
            {
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, normal, terminatedLanes, out var support);
                Vector3Wide.Dot(support, normal, out var newDepth);
                //If the depth has not improved, we should take a step back toward the previous sample direction.
                //Note that we consider equal depth to be an 'improvement'- if we didn't, it's possible for 
                //the search to get stuck if a new sample overshoots, and there exists no better normal in between
                //the new (bad) normal and the previous best normal.
                var depthNotWorse = Vector.LessThanOrEqual(newDepth, depth);
                var depthStrictlyImproved = Vector.LessThan(newDepth, depth);
                
                var useNewResult = Vector.AndNot(depthNotWorse, terminatedLanes);
                Vector3Wide.ConditionalSelect(useNewResult, normal, refinedNormal, out refinedNormal);
                depth = Vector.ConditionalSelect(useNewResult, newDepth, depth);

                //We'll terminate if we failed to make any improvements and the progression scale is below the target threshold.
                //In theory, progression could speed up again, but for reasonable termination thresholds that's not a concern.
                //(Note that the progression parameter is approximately proportional to tilting angle- if it's 1e-7f, the normal is pretty close.)
                var shouldTerminate = Vector.AndNot(Vector.LessThanOrEqual(progressionScale, convergenceThreshold), depthStrictlyImproved);
                terminatedLanes = Vector.BitwiseOr(shouldTerminate, terminatedLanes);
                if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
                    break;

                //It's possible that the new normal/support has rocketed off to the other side of the origin.
                //This becomes very common when small changes in the normal result in large changes in the support location.
                //Pivoting around the latest support point exclusively in such a case would tend to make very little progress-
                //the applied normal rotation would be mostly perpendicular to the desired direction.
                //Reducing tilt angle in response to oscillation can help a lot, but can make progress toward the origin slow.
                //So instead of pivoting on the latest support point, we pick the closest point on the line between the current and previous
                //support to the origin and tilt the interpolated normal toward the origin.

                //dot(origin - previousSupport, (support - previousSupport) / ||support - previousSupport||) * (support - previousSupport) / ||support - previousSupport||
                Vector3Wide.Subtract(support, previousSupport, out var supportOffset);
                Vector3Wide.LengthSquared(supportOffset, out var supportOffsetLengthSquared);
                Vector3Wide.Dot(previousSupport, supportOffset, out var dot);
                //TODO: Approximate rcp would be sufficient here.
                var t = -dot / supportOffsetLengthSquared;
                //Note that, if the depth has failed to improve, we force the interpolated point backwards quite a distance to try to return to a safe state.
                t = Vector.Max(Vector<float>.Zero, Vector.Min(t, Vector.ConditionalSelect(depthNotWorse, Vector<float>.One, new Vector<float>(0.25f))));
                Vector3Wide.Scale(supportOffset, t, out var previousSupportToClosestPointOnLineToOrigin);
                Vector3Wide.Add(previousSupport, previousSupportToClosestPointOnLineToOrigin, out var interpolatedSupport);
                Vector3Wide.Scale(previousNormal, Vector<float>.One - t, out var weightedPreviousNormal);
                Vector3Wide.Scale(normal, t, out var weightedNormal);
                Vector3Wide.Add(weightedPreviousNormal, weightedNormal, out var interpolatedNormal);

                //Instead of working directly with angles, we create a point on the line passing through the origin along the normal
                //according to where the support plane is and how far away the support point is from the closest point to the origin to the plane is.
                //We then compute a new plane normal that intersects both the current support and the point on the origin line.
                //For example, if you push the point on the line toward the origin (or beyond) by a distance equal to the distance
                //along the plane between the support point and the origin, that is equivalent to a 45 degree rotation.

                //Note the use of the non-interpolated normal mixed with interpolated values.
                //We're being a bit loose here because we don't want to normalize the interpolated normal.
                Vector3Wide.Scale(normal, newDepth, out closestPointOnPlane);
                Vector3Wide.Subtract(closestPointOnPlane, interpolatedSupport, out supportToClosestPoint);
                //TODO: Still converges with a LengthSquared instead of Length, but overall impact on performance isn't always positive.
                //Consider approximate alternatives with platform intrinsics.
                Vector3Wide.Length(supportToClosestPoint, out offsetLength);

                Vector3Wide.Scale(normal, newDepth - progressionScale * offsetLength, out pointOnOriginLine);
                Vector3Wide.Subtract(pointOnOriginLine, interpolatedSupport, out supportToPointOnOriginLine);
                Vector3Wide.CrossWithoutOverlap(interpolatedNormal, supportToPointOnOriginLine, out n);
                Vector3Wide.CrossWithoutOverlap(supportToPointOnOriginLine, n, out var tiltedNormal);

                //If depth didn't improve, then use the interpolated normal directly.
                //If the tilted normal has zero length, that implies the interpolated support was aligned with the origin.
                //If it were a real support, we could early out, but since it's not we just avoid a division by zero
                //by falling back to the interpolated value.
                Vector3Wide.LengthSquared(tiltedNormal, out var tiltedNormalLengthSquared);
                var useFallback = Vector.LessThan(tiltedNormalLengthSquared, new Vector<float>(1e-14f));
                Vector3Wide.ConditionalSelect(Vector.AndNot(depthNotWorse, useFallback), tiltedNormal, interpolatedNormal, out var newNormal);

                if (steps != null)
                {
                    PlaneWalkerStep step;
                    Vector3Wide.ReadSlot(ref previousNormal, 0, out step.PreviousNormal);
                    Vector3Wide.ReadSlot(ref normal, 0, out step.Normal);
                    Vector3Wide.ReadSlot(ref interpolatedNormal, 0, out step.InterpolatedNormal);
                    step.InterpolatedNormal = Vector3.Normalize(step.InterpolatedNormal);
                    Vector3Wide.ReadSlot(ref previousSupport, 0, out step.PreviousSupport);
                    Vector3Wide.ReadSlot(ref support, 0, out step.Support);
                    Vector3Wide.ReadSlot(ref interpolatedSupport, 0, out step.InterpolatedSupport);
                    step.BestDepth = depth[0];
                    step.NewestDepth = newDepth[0];
                    step.Improved = depthStrictlyImproved[0] < 0;
                    Vector3Wide.ReadSlot(ref pointOnOriginLine, 0, out step.PointOnOriginLine);
                    Vector3Wide.ReadSlot(ref newNormal, 0, out step.NextNormal);
                    step.NextNormal = Vector3.Normalize(step.NextNormal);
                    step.Progression = progressionScale[0];
                    steps.Add(step);
                }

                Vector3Wide.ConditionalSelect(depthNotWorse, normal, previousNormal, out previousNormal);
                Vector3Wide.ConditionalSelect(depthNotWorse, support, previousSupport, out previousSupport);
                //TODO: This could be replaced by a faster rsqrt with platform intrinsics.
                Vector3Wide.Length(newNormal, out newNormalLength);
                Vector3Wide.Scale(newNormal, Vector<float>.One / newNormalLength, out normal);

                //progressionScale *= Vector.ConditionalSelect(depthNotWorse, Vector.ConditionalSelect(depthStrictlyImproved, new Vector<float>(1.5f), new Vector<float>(0.125f)), new Vector<float>(0.125f));
                progressionScale *= Vector.ConditionalSelect(depthStrictlyImproved, new Vector<float>(1.5f), new Vector<float>(0.125f));
                
                //If all lanes are sufficiently separated, then we can early out.
                depthBelowThreshold = Vector.LessThan(depth, minimumDepthThreshold);
                terminatedLanes = Vector.BitwiseOr(depthBelowThreshold, terminatedLanes);
                if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
                    break;
            }
        }
    }
}
