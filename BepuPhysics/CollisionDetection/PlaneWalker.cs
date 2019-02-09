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
    public struct PlaneWalkerStep1
    {
        public Vector3 PreviousNormal;
        public Vector3 Normal;
        public Vector3 Support;
        public float BestDepth;
        public float NewestDepth;
        public bool Improved;
        public bool Oscillating;
        public Vector3 PointOnOriginLine;
        public Vector3 NextNormal;
        public float Progression;
    }

    public struct PlaneWalkerStep2
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
        static void FindSupport(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB, in Vector3Wide direction, out Vector3Wide support)
        {
            //support(N, A) - support(-N, B)
            supportFinderA.ComputeLocalSupport(a, direction, out var extremeA);
            Vector3Wide.Negate(direction, out var negatedDirection);
            supportFinderB.ComputeSupport(b, localOrientationB, negatedDirection, out var extremeB);
            Vector3Wide.Add(extremeB, localOffsetB, out extremeB);

            Vector3Wide.Subtract(extremeA, extremeB, out support);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TiltNormal(in Vector3Wide normal, in Vector3Wide support, in Vector<float> depth, in Vector<float> progressionScale, ref Vector<int> terminatedLanes, out Vector3Wide nextNormal)
        {
            Vector3Wide.Scale(normal, depth, out var closestPointOnPlane);
            Vector3Wide.Subtract(closestPointOnPlane, support, out var supportToClosestPoint);
            Vector3Wide.Length(supportToClosestPoint, out var offsetLength);

            Vector3Wide.Scale(normal, depth - progressionScale * offsetLength, out var pointOnOriginLine);
            Vector3Wide.Subtract(pointOnOriginLine, support, out var supportToPointOnOriginLine);
            Vector3Wide.CrossWithoutOverlap(normal, supportToPointOnOriginLine, out var n);
            Vector3Wide.CrossWithoutOverlap(supportToPointOnOriginLine, n, out nextNormal);

            //TODO: This could be replaced by a faster rsqrt with platform intrinsics.
            Vector3Wide.Length(nextNormal, out var newNormalLength);
            Vector3Wide.Scale(nextNormal, Vector<float>.One / newNormalLength, out nextNormal);

            //If the offset length is zero, the normal length will be zero and the latest support point is the closest point to the origin.
            //This is sufficient to guarantee that no further progress is possible and we can early out.
            //(Note that for shapes with flat faces, this termination condition may not be met- the support point is not unique.)
            var invalidNormal = Vector.LessThan(newNormalLength, new Vector<float>(1e-10f));
            terminatedLanes = Vector.BitwiseOr(invalidNormal, terminatedLanes);

        }

        public static void FindMinimumDepth2(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            in Vector3Wide initialNormal, in Vector<int> inactiveLanes, out Vector<float> depth, out Vector3Wide refinedNormal, List<PlaneWalkerStep2> steps, int maximumIterations = 30)
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

            //This tilts the sample direction. If sampling depth at the new location is an improvement, we repeat the process for the new support plane.
            //If the new depth is worse, we've pushed too far. We can simply move back toward the old sample direction incrementally and try again.
            //If we end up not being able to make any progress at all, then the minimum depth has already been found.

            //Note that this process will successfully converge to the global minimum in the separated case since the separated distance function is convex.
            //In the penetrating case, this will converge to a local minimum that may not be the global minimum. In practice, so long as the initial guess is reasonable,
            //the resulting local minimum is generally good enough.

#if DEBUG
            Vector3Wide.LengthSquared(initialNormal, out var initialNormalLengthSquared);
            Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThan(Vector.Abs(initialNormalLengthSquared - Vector<float>.One), new Vector<float>(1e-6f))), Vector<int>.Zero));
#endif
            var previousNormal = initialNormal;
            FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, previousNormal, out var previousSupport);
            Vector3Wide.Dot(previousSupport, previousNormal, out depth);

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
            terminatedLanes = Vector.BitwiseOr(invalidNormal, terminatedLanes);

            PlaneWalkerStep2 step = default;
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

            if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
            {
                refinedNormal = initialNormal;
                return;
            }
            
            for (int i = 0; i < maximumIterations; ++i)
            {
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, normal, out var support);
                Vector3Wide.Dot(support, normal, out var newDepth);
                //If the depth has not improved, we should take a step back toward the previous sample direction.
                //Note that we consider equal depth to be an 'improvement'- if we didn't, it's possible for 
                //the search to get stuck if a new sample overshoots, and there exists no better normal in between
                //the new (bad) normal and the previous best normal.
                var depthNotWorse = Vector.LessThanOrEqual(newDepth, depth);
                var depthStrictlyImproved = Vector.LessThan(newDepth, depth);

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
                var t = -dot / supportOffsetLengthSquared;
                //Note that, if the depth has failed to improve, we force the interpolated point backwards quite a distance to try to return to a safe state.
                t = Vector.Max(Vector<float>.Zero, Vector.Min(t, Vector.ConditionalSelect(depthNotWorse, Vector<float>.One, new Vector<float>(0.125f))));
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

                Vector3Wide.ConditionalSelect(depthNotWorse, normal, previousNormal, out previousNormal);
                Vector3Wide.ConditionalSelect(depthNotWorse, support, previousSupport, out previousSupport);
                //TODO: This could be replaced by a faster rsqrt with platform intrinsics.
                Vector3Wide.Length(newNormal, out newNormalLength);
                Vector3Wide.Scale(newNormal, Vector<float>.One / newNormalLength, out normal);

                progressionScale *= Vector.ConditionalSelect(depthNotWorse, Vector.ConditionalSelect(depthStrictlyImproved, new Vector<float>(1.5f), new Vector<float>(0.5f)), new Vector<float>(0.125f));
                //progressionScale *= Vector.ConditionalSelect(depthStrictlyImproved, new Vector<float>(1.5f), new Vector<float>(0.125f));

                //If the offset length is zero, the normal length will be zero and the latest support point is the closest point to the origin.
                //This is sufficient to guarantee that no further progress is possible and we can early out.
                //(Note that for shapes with flat faces, this termination condition may not be met- the support point is not unique.)
                //invalidNormal = Vector.LessThan(newNormalLength, new Vector<float>(1e-10f));
                //terminatedLanes = Vector.BitwiseOr(invalidNormal, terminatedLanes);
                if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
                    break;
                var useNewResult = Vector.AndNot(depthNotWorse, terminatedLanes);
                Vector3Wide.ConditionalSelect(useNewResult, normal, refinedNormal, out refinedNormal);
                depth = Vector.ConditionalSelect(useNewResult, newDepth, depth);
            }
        }


        public static void FindMinimumDepth1(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            in Vector3Wide initialNormal, in Vector<int> inactiveLanes, out Vector<float> depth, out Vector3Wide refinedNormal, List<PlaneWalkerStep1> steps, int maximumIterations = 30)
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

            //This tilts the sample direction. If sampling depth at the new location is an improvement, we repeat the process for the new support plane.
            //If the new depth is worse, we've pushed too far. We can simply move back toward the old sample direction incrementally and try again.
            //If we end up not being able to make any progress at all, then the minimum depth has already been found.

            //Note that this process will successfully converge to the global minimum in the separated case since the separated distance function is convex.
            //In the penetrating case, this will converge to a local minimum that may not be the global minimum. In practice, so long as the initial guess is reasonable,
            //the resulting local minimum is generally good enough.

            depth = new Vector<float>(float.MaxValue);
#if DEBUG
            Vector3Wide.LengthSquared(initialNormal, out var initialNormalLengthSquared);
            Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThan(Vector.Abs(initialNormalLengthSquared - Vector<float>.One), new Vector<float>(1e-6f))), Vector<int>.Zero));
#endif
            refinedNormal = initialNormal;
            var normal = initialNormal;
            //Since the depth is ~infinite, the initial previous normal is never going to be requested.
            var previousDepth = new Vector<float>(float.MaxValue);
            var previousNormal = default(Vector3Wide);

            var terminatedLanes = inactiveLanes;

            var progressionScale = new Vector<float>(0.25f);

            var previousOriginOffset = default(Vector3Wide);

            for (int i = 0; i < maximumIterations; ++i)
            {
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, normal, out var support);
                Vector3Wide.Dot(support, normal, out var newDepth);
                //If the depth has not improved, we should take a step back toward the previous sample direction.
                var depthImproved = Vector.LessThan(newDepth, depth);

                //There are two possible sources of a new sample direction:
                //1) If the depth has improved, then we tilt the plane by pushing on the closest point on the new support plane to the origin.
                //2) If the depth has not improved, then we don't accept this new location. Instead, try again from the previous normal, but with less tilt.

                //One option is to simply rotate the plane by a fixed number of degrees to push the origin out. If it goes too far, the fallback bisection will pull it back.
                //Doing this naively with a large fixed rotation would result in a lot of time spent in bisection iterations, especially as the search nears the minimum.
                //(This is similar to a gradient descent with fixed length step and bisecting line search. It works, but it's not the most efficient thing.)

                //To address the issues of the fixed magnitude rotation, we'll instead start with a reasonable guess and dynamically modify it.
                //Every time a rotation attempt fails to improve the depth, the rotation magnitude for future attempts is halved.
                //If a rotation attempt does improve the depth, future magnitudes are allowed to grow a little bit.
                //(In general, the rotation magnitude will shrink rapidly; the ability for it to grow is relatively unimportant in most cases.)
                Vector3Wide.Scale(normal, newDepth, out var closestPointOnPlane);
                Vector3Wide.Subtract(closestPointOnPlane, support, out var supportToClosestPoint);
                Vector3Wide.Length(supportToClosestPoint, out var offsetLength);

                //When the local surface is fairly flat, it's possible for the search to oscillate around the origin while only fractionally improving the 
                //depth each time. While depth is improving, it can converge extremely slowly. We can detect oscillation by comparing the 
                //offset from the support to the projected origin for this iteration versus the previous iteration.
                //If they point in opposing directions, then we know the search is jumping over the origin.
                Vector3Wide.Dot(supportToClosestPoint, previousOriginOffset, out var oscillationDot);
                previousOriginOffset = supportToClosestPoint;
                var oscillating = Vector.LessThan(oscillationDot, Vector<float>.Zero);

                //Instead of working directly with angles, we create a point on the line passing through the origin along the normal
                //according to where the support plane is and how far away the support point is from the closest point to the origin to the plane is.
                //We then compute a new plane normal that intersects both the current support and the point on the origin line.
                //For example, if you push the point on the line toward the origin (or beyond) by a distance equal to the distance
                //along the plane between the support point and the origin, that is equivalent to a 45 degree rotation.

                Vector3Wide.Scale(normal, newDepth - progressionScale * offsetLength, out var pointOnOriginLine);
                Vector3Wide.Subtract(pointOnOriginLine, support, out var supportToPointOnOriginLine);
                Vector3Wide.CrossWithoutOverlap(normal, supportToPointOnOriginLine, out var n);
                Vector3Wide.CrossWithoutOverlap(supportToPointOnOriginLine, n, out var newNormal);

                PlaneWalkerStep1 step;
                Vector3Wide.ReadSlot(ref previousNormal, 0, out step.PreviousNormal);
                Vector3Wide.ReadSlot(ref normal, 0, out step.Normal);
                Vector3Wide.ReadSlot(ref support, 0, out step.Support);
                step.BestDepth = Vector.Min(depth, newDepth)[0];
                step.NewestDepth = newDepth[0];
                step.Improved = depthImproved[0] < 0;
                step.Oscillating = oscillating[0] < 0;
                Vector3Wide.ReadSlot(ref pointOnOriginLine, 0, out step.PointOnOriginLine);
                Vector3Wide.ReadSlot(ref newNormal, 0, out step.NextNormal);
                step.NextNormal = Vector3.Normalize(step.NextNormal);
                step.Progression = progressionScale[0];
                steps.Add(step);

                //The previous normal doesn't change if the depth hasn't improved- we need to keep that history around so we can continue to recover if necessary.
                Vector3Wide.Scale(normal, new Vector<float>(0.25f), out var newContribution);
                Vector3Wide.Scale(previousNormal, new Vector<float>(0.75f), out var oldContribution);
                Vector3Wide.Add(newContribution, oldContribution, out var fallbackNormal);
                Vector3Wide.ConditionalSelect(depthImproved, normal, previousNormal, out previousNormal);
                Vector3Wide.ConditionalSelect(depthImproved, newNormal, fallbackNormal, out newNormal);
                progressionScale *= Vector.ConditionalSelect(depthImproved, Vector.ConditionalSelect(oscillating, new Vector<float>(0.25f), new Vector<float>(1.5f)), new Vector<float>(0.25f));

                //TODO: This could be replaced by a faster rsqrt with platform intrinsics.
                Vector3Wide.Length(newNormal, out var newNormalLength);
                Vector3Wide.Scale(newNormal, Vector<float>.One / newNormalLength, out normal);

                //If the offset length is zero, the normal length will be zero and the latest support point is the closest point to the origin.
                //This is sufficient to guarantee that no further progress is possible and we can early out.
                //(Note that for shapes with flat faces, this termination condition may not be met- the support point is not unique.)
                var invalidNormal = Vector.LessThan(newNormalLength, new Vector<float>(1e-10f));
                terminatedLanes = Vector.BitwiseOr(invalidNormal, terminatedLanes);
                if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
                    break;
                var useNewResult = Vector.AndNot(depthImproved, terminatedLanes);
                Vector3Wide.ConditionalSelect(useNewResult, normal, refinedNormal, out refinedNormal);
                depth = Vector.ConditionalSelect(useNewResult, newDepth, depth);
            }
        }
    }
}
