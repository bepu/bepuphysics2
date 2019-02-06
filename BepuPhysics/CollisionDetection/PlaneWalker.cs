using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
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

        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB, 
            in Vector3Wide initialNormal, out Vector<float> depth, in Vector<int> inactiveLanes, out Vector3Wide normal, int maximumIterations = 30)
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
            normal = initialNormal;
            //Since the depth is ~infinite, the initial previous normal is never going to be requested.
            var previousDepth = new Vector<float>(float.MaxValue);
            var previousNormal = default(Vector3Wide);

            var terminatedLanes = inactiveLanes;

            var progressionScale = new Vector<float>(0.5f);

            for (int i = 0; i < maximumIterations; ++i)
            {
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, normal, out var newSupport);
                Vector3Wide.Dot(newSupport, normal, out var newDepth);
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
                Vector3Wide.Subtract(closestPointOnPlane, newSupport, out var supportToClosestPoint);
                Vector3Wide.Length(supportToClosestPoint, out var offsetLength);

                //Instead of working directly with angles, we create a point on the line passing through the origin along the normal
                //according to where the support plane is and how far away the support point is from the closest point to the origin to the plane is.
                //We then compute a new plane normal that intersects both the current support and the point on the origin line.
                //For example, if you push the point on the line toward the origin (or beyond) by a distance equal to the distance
                //along the plane between the support point and the origin, that is equivalent to a 45 degree rotation.

                //The previous normal doesn't change if the depth hasn't improved- we need to keep that history around so we can continue to recover if necessary.
                Vector3Wide.ConditionalSelect(depthImproved, normal, previousNormal, out previousNormal);
                progressionScale *= Vector.ConditionalSelect(depthImproved, new Vector<float>(1.1f), new Vector<float>(0.5f));
                Vector3Wide.Scale(previousNormal, newDepth - progressionScale * offsetLength, out var pointOnOriginLine);
                Vector3Wide.Subtract(pointOnOriginLine, newSupport, out var supportToPointOnOriginLine);

                Vector3Wide.CrossWithoutOverlap(previousNormal, supportToPointOnOriginLine, out var n);
                Vector3Wide.CrossWithoutOverlap(supportToPointOnOriginLine, n, out var newNormal);
                                
                //TODO: This could be replaced by a faster rsqrt with platform intrinsics.
                Vector3Wide.Length(newNormal, out var newNormalLength);
                Vector3Wide.Scale(newNormal, Vector<float>.One / newNormalLength, out newNormal);

                //If the offset length is zero, the normal length will be zero and the latest support point is the closest point to the origin.
                //This is sufficient to guarantee that no further progress is possible and we can early out.
                //(Note that for shapes with flat faces, this termination condition may not be met- the support point is not unique.)
                var invalidNormal = Vector.LessThan(newNormalLength, new Vector<float>(1e-9f));
                terminatedLanes = Vector.BitwiseOr(invalidNormal, terminatedLanes);
                if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
                    break;
                var useNewResult = Vector.AndNot(depthImproved, terminatedLanes);
                Vector3Wide.ConditionalSelect(useNewResult, newNormal, normal, out normal);
                depth = Vector.ConditionalSelect(useNewResult, newDepth, depth);
            }
        }
    }
}
