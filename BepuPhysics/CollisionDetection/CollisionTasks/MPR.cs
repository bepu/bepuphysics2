using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public static class MPR<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
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
        [Conditional("DEBUG")]
        static void VerifySimplex(in Vector3Wide v0, in Vector3Wide v1, in Vector3Wide v2, in Vector3Wide v3, in Vector3Wide rayDirection, in Vector<int> inactiveLanes)
        {
            //v1, v0, v3
            Vector3Wide.Subtract(v0, v1, out var v1v0);
            Vector3Wide.Subtract(v3, v1, out var v1v3);
            Vector3Wide.Subtract(v2, v1, out var v1v2);
            Vector3Wide.CrossWithoutOverlap(v1v0, v1v3, out var cross1);
            Vector3Wide.Dot(cross1, rayDirection, out var directionProduct1);
            Vector3Wide.Dot(cross1, v1v2, out var offVertexProduct1);
            //v3, v0, v2
            Vector3Wide.Subtract(v0, v3, out var v3v0);
            Vector3Wide.Subtract(v2, v3, out var v3v2);
            Vector3Wide.Subtract(v1, v3, out var v3v1);
            Vector3Wide.CrossWithoutOverlap(v3v0, v3v2, out var cross2);
            Vector3Wide.Dot(cross2, rayDirection, out var directionProduct2);
            Vector3Wide.Dot(cross2, v3v1, out var offVertexProduct2);
            //v2, v0, v1
            Vector3Wide.Subtract(v0, v2, out var v2v0);
            Vector3Wide.Subtract(v1, v2, out var v2v1);
            Vector3Wide.Subtract(v3, v2, out var v2v3);
            Vector3Wide.CrossWithoutOverlap(v2v0, v2v1, out var cross3);
            Vector3Wide.Dot(cross3, rayDirection, out var directionProduct3);
            Vector3Wide.Dot(cross3, v2v3, out var offVertexProduct3);
            var allDirectionsVerified = Vector.BitwiseOr(inactiveLanes,
                Vector.BitwiseAnd(Vector.BitwiseAnd(Vector.GreaterThanOrEqual(directionProduct1, Vector<float>.Zero), Vector.GreaterThanOrEqual(directionProduct2, Vector<float>.Zero)), Vector.GreaterThanOrEqual(directionProduct3, Vector<float>.Zero)));
            Debug.Assert(Vector.EqualsAll(allDirectionsVerified, new Vector<int>(-1)), "All faces must have consistent winding.");
            var allOffVerticesVerified = Vector.BitwiseOr(inactiveLanes,
                Vector.BitwiseAnd(Vector.BitwiseAnd(Vector.GreaterThanOrEqual(offVertexProduct1, Vector<float>.Zero), Vector.GreaterThanOrEqual(offVertexProduct2, Vector<float>.Zero)), Vector.GreaterThanOrEqual(offVertexProduct3, Vector<float>.Zero)));
            Debug.Assert(Vector.EqualsAll(allOffVerticesVerified, new Vector<int>(-1)), "All faces must have consistent winding.");

        }

        [Conditional("DEBUG")]
        static void VerifySimplex(in Vector3Wide v0, in Vector3Wide v1, in Vector3Wide v2, in Vector3Wide v3, in Vector<int> inactiveLanes)
        {
            Vector3Wide.Negate(v0, out var rayDirection);
            VerifySimplex(v0, v1, v2, v3, rayDirection, inactiveLanes);
        }

        public static void Test(
            in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB,
            ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB, in Vector<float> surfaceEpsilon, in Vector<int> inactiveLanes, out Vector<int> intersecting, out Vector3Wide localNormal, int maximumIterations = 15)
        {
            //We'll be using a minkowski difference support(N, A) - support(-N, B).
            //So, the starting point for the origin ray is simply -localOffsetB.

            localNormal = localOffsetB;
            Vector3Wide.Negate(localOffsetB, out var v0);
            Vector3Wide.Dot(localOffsetB, localOffsetB, out var v0ExitDot);
            var normalLengthEpsilon = new Vector<float>(1e-10f);
            var shouldExitV0 = Vector.LessThan(v0ExitDot, normalLengthEpsilon);
            intersecting = shouldExitV0;
            var laneComplete = Vector.BitwiseOr(inactiveLanes, shouldExitV0);

            //Find an initial portal through which the ray passes.
            FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, localOffsetB, out var v1);

            Vector3Wide.Dot(v1, localOffsetB, out var v1ExitDot);

            var shouldExitV1 = Vector.LessThan(v1ExitDot, Vector<float>.Zero);
            intersecting = Vector.ConditionalSelect(Vector.AndNot(shouldExitV1, laneComplete), Vector<int>.Zero, intersecting);
            laneComplete = Vector.BitwiseOr(laneComplete, shouldExitV1);

            Vector3Wide.CrossWithoutOverlap(v1, v0, out var n);
            //Use fallbacks in the parallel case.
            Vector3Wide.LengthSquared(n, out var nLengthSquared);
            var shouldExitPreV2 = Vector.LessThan(nLengthSquared, normalLengthEpsilon);
            Vector3Wide.Subtract(v1, v0, out var fallbackNormal);
            var laneStateChanging = Vector.AndNot(shouldExitPreV2, laneComplete);
            Vector3Wide.ConditionalSelect(laneStateChanging, fallbackNormal, localNormal, out localNormal);
            intersecting = Vector.ConditionalSelect(laneStateChanging, Vector<int>.Zero, intersecting);
            laneComplete = Vector.BitwiseOr(laneComplete, shouldExitPreV2);


            FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, n, out var v2);
            Vector3Wide.Dot(n, v2, out var v2ExitDot);
            var shouldExitV2 = Vector.LessThan(v2ExitDot, Vector<float>.Zero);
            laneStateChanging = Vector.AndNot(shouldExitV2, laneComplete);
            Vector3Wide.ConditionalSelect(laneStateChanging, n, localNormal, out localNormal);
            intersecting = Vector.ConditionalSelect(laneStateChanging, Vector<int>.Zero, intersecting);
            laneComplete = Vector.BitwiseOr(laneComplete, shouldExitV2);

            if (Vector.LessThanAll(laneComplete, Vector<int>.Zero))
            {
                //Just quit; all active lanes have exited.
                return;
            }

            Vector3Wide v3;
            var preloopCompleted = Vector<int>.Zero;
            while (true)
            {
                //Find a final extreme point using the normal of the plane defined by v0, v1, v2.
                Vector3Wide.Subtract(v1, v0, out var v0v1);
                Vector3Wide.Subtract(v2, v0, out var v0v2);
                Vector3Wide.CrossWithoutOverlap(v0v1, v0v2, out n);

                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, n, out v3);

                //If the origin is outside the plane defined by (v1, v0, v3) then the portal is invalid.
                Vector3Wide.CrossWithoutOverlap(v1, v3, out var v1xv3);
                Vector3Wide.Dot(v1xv3, v0, out var originOffset013);
                //v2 was on the inside of the plane; replace it with the latest extreme point v3.
                var firstPlaneTestFoundInvalid = Vector.LessThan(originOffset013, Vector<float>.Zero);
                Vector3Wide.ConditionalSelect(Vector.AndNot(firstPlaneTestFoundInvalid, preloopCompleted), v3, v2, out v2);

                //If the origin is outside the plane defined by (v3, v0, v2) then the portal is invalid.
                Vector3Wide.CrossWithoutOverlap(v3, v2, out var v3xv2);
                Vector3Wide.Dot(v3xv2, v0, out var originOffset302);
                //v1 was on the inside of the plane; replace it with the latest extreme point v3. Note that if the lane already was handled by the first plane test, this does nothing.
                var secondPlaneTestFoundInvalid = Vector.LessThan(originOffset302, Vector<float>.Zero);
                Vector3Wide.ConditionalSelect(Vector.AndNot(Vector.AndNot(secondPlaneTestFoundInvalid, firstPlaneTestFoundInvalid), preloopCompleted), v3, v1, out v1);

                preloopCompleted = Vector.BitwiseOr(preloopCompleted, Vector.AndNot(Vector.OnesComplement(firstPlaneTestFoundInvalid), secondPlaneTestFoundInvalid));
                if (Vector.LessThanAll(Vector.BitwiseOr(preloopCompleted, laneComplete), Vector<int>.Zero))
                    break;
            }
            VerifySimplex(v0, v1, v2, v3, laneComplete);

            //Initial portal created; refine it.
            var squaredSurfaceEpsilon = surfaceEpsilon * surfaceEpsilon;
            for (int i = 0; i < maximumIterations; ++i)
            {
                Vector3Wide.Subtract(v2, v1, out var v1v2);
                Vector3Wide.Subtract(v3, v1, out var v1v3);
                Vector3Wide.CrossWithoutOverlap(v1v2, v1v3, out n);
                Vector3Wide.Dot(n, v1, out var exitDot);
                var foundIntersection = Vector.GreaterThan(exitDot, Vector<float>.Zero);
                laneStateChanging = Vector.AndNot(foundIntersection, laneComplete);
                Vector3Wide.ConditionalSelect(laneStateChanging, n, localNormal, out localNormal);
                intersecting = Vector.ConditionalSelect(laneStateChanging, foundIntersection, intersecting);
                laneComplete = Vector.BitwiseOr(foundIntersection, laneComplete);
                if (Vector.LessThanAll(laneComplete, Vector<int>.Zero))
                {
                    return;
                }
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, n, out var v4);

                //Compare the latest support sample with the triangle. Are we close to the surface?
                Vector3Wide.Subtract(v4, v1, out var v1v4);
                Vector3Wide.Dot(v1v4, n, out var difference);
                //We want to compare actual depths with the epsilon to have a consistent exit, but we don't want a square root.
                //dot(v4, n / ||n||) - dot(v1, n / ||n||) < epsilon
                //dot(v4, n) - dot(v1, n) < epsilon * ||n||
                //(dot(v4, n) - dot(v1, n))^2 < epsilon^2 * ||n||^2
                var squaredDifference = difference * difference;
                Vector3Wide.LengthSquared(n, out nLengthSquared);
                var foundSurface = Vector.LessThanOrEqual(squaredDifference, squaredSurfaceEpsilon * nLengthSquared);
                laneStateChanging = Vector.AndNot(foundSurface, laneComplete);
                Vector3Wide.ConditionalSelect(laneStateChanging, n, localNormal, out localNormal);
                intersecting = Vector.ConditionalSelect(laneStateChanging, Vector<int>.Zero, intersecting);
                laneComplete = Vector.BitwiseOr(laneComplete, foundSurface);
                if (Vector.LessThanAll(laneComplete, Vector<int>.Zero))
                {
                    return;
                }
                //Note that, if not all lanes are done, we just let all of them keep going.
                //Still haven't exited, so refine the portal.
                //Test origin against the three planes that separate the new portal candidates: (v1,v4,v0) (v2,v4,v0) (v3,v4,v0)
                Vector3Wide.CrossWithoutOverlap(v4, v0, out var v4xv0);
                Vector3Wide.Dot(v1, v4xv0, out var v1PlaneDot);
                Vector3Wide.Dot(v2, v4xv0, out var v2PlaneDot);
                Vector3Wide.Dot(v3, v4xv0, out var v3PlaneDot);
                var insideV1 = Vector.GreaterThanOrEqual(v1PlaneDot, Vector<float>.Zero);
                var insideV2 = Vector.GreaterThanOrEqual(v2PlaneDot, Vector<float>.Zero);
                var insideV3 = Vector.GreaterThanOrEqual(v3PlaneDot, Vector<float>.Zero);
                //Inside v1 && inside v2 ==> eliminate v1
                //Outside v1 && outside v3 ==> eliminate v1
                //Outside v1 && inside v3 ==> eliminate v2
                //Inside v1 && outside v2 ==> eliminate v3
                var eliminateV1 = Vector.BitwiseOr(Vector.BitwiseAnd(insideV1, insideV2), Vector.AndNot(Vector.OnesComplement(insideV1), insideV3));
                var eliminateV2 = Vector.AndNot(insideV3, insideV1);
                var eliminateV3 = Vector.AndNot(insideV1, insideV2);
                Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(laneComplete, Vector.Equals(eliminateV1 + eliminateV2 + eliminateV3, new Vector<int>(-1))), Vector<int>.Zero), "Only one vertex should be eliminated.");
                Vector3Wide.ConditionalSelect(eliminateV1, v4, v1, out v1);
                Vector3Wide.ConditionalSelect(eliminateV2, v4, v2, out v2);
                Vector3Wide.ConditionalSelect(eliminateV3, v4, v3, out v3);
            }
            //Exited due to iteration limit. Use a default for any incomplete lanes.
            Vector3Wide.ConditionalSelect(laneComplete, localNormal, n, out localNormal);
        }

        public static void LocalSurfaceCast(
            in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB,
            ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB, in Vector3Wide direction, in Vector<float> surfaceEpsilon, in Vector<int> inactiveLanes, out Vector<float> t, out Vector3Wide localNormal, int maximumIterations = 15)
        {
            //Local surface cast is very similar to regular MPR.  However, instead of starting at an interior point and targeting the origin,
            //the ray starts at the origin (a point known to be in both shapeA and shapeB during overlap), and just goes towards the direction until the surface
            //is found.  The portal (v1, v2, v3) at termination defines the surface normal, and the distance from the origin to the portal along the direction is used as the 't' result.

            //'v0' is no longer explicitly tracked since it is simply the origin.

            //Now that the origin ray is known, create a portal through which the ray passes.
            //To do this, first guess a portal.
            //This implementation is similar to that of the original XenoCollide.
            //'n' will be the direction used to find supports throughout the algorithm.
            FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, direction, out var v1);

            //Find another extreme point in a direction perpendicular to the previous.
            Vector3Wide.CrossWithoutOverlap(direction, v1, out var n);
            Vector3Wide.LengthSquared(n, out var nLengthSquared);
            var completedLanes = Vector.BitwiseOr(inactiveLanes, Vector.LessThan(nLengthSquared, new Vector<float>(1e-10f)));
            //If direction and v1 point in the same direction, then we've lucked out- the intersection with the surface occurs right on the direction and we can quit.
            //Intersect the ray against the surface plane to get the t parameter.
            Vector3Wide.Dot(direction, direction, out var directionLengthSquared);
            Vector3Wide.Dot(direction, v1, out var directionDotV1);
            t = Vector.ConditionalSelect(Vector.GreaterThan(Vector.Abs(directionDotV1), new Vector<float>(1e-10f)), directionLengthSquared / directionDotV1, Vector<float>.Zero);
            localNormal = n;
            if (Vector.LessThanAll(completedLanes, Vector<int>.Zero))
            {
                return;
            }

            FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, n, out var v2);

            //Set n for the first iteration.
            Vector3Wide.CrossWithoutOverlap(v1, v2, out n);

            //It's possible that v1 and v2 were constructed in such a way that 'n' is not properly calibrated
            //relative to the direction vector. If it's not properly calibrated, flip the winding (and the previously calculated normal).
            Vector3Wide.Dot(n, direction, out var nDotDirection);
            var flipWinding = Vector.GreaterThan(nDotDirection, Vector<float>.Zero);
            Vector3Wide.ConditionallyNegate(flipWinding, ref n);
            var swapTemp = v1;
            Vector3Wide.ConditionalSelect(flipWinding, v2, v1, out swapTemp);
            Vector3Wide.ConditionalSelect(flipWinding, v1, v2, out v2);
            v1 = swapTemp;

            Vector3Wide v3;
            var count = 0;
            var preloopComplete = completedLanes;
            VerifySimplex(default, v1, v2, v3, direction, completedLanes);
            while (true)
            {
                if (count > maximumIterations)
                {
                    //Can't enclose the ray! That's a bit odd; something is wrong. Kill any lane still working.
                    completedLanes = Vector.ConditionalSelect(preloopComplete, completedLanes, new Vector<int>(-1));
                    t = Vector.ConditionalSelect(preloopComplete, t, new Vector<float>(float.MaxValue));
                    break;
                }
                count++;

                //Find a final extreme point using the normal of the plane defined by v0, v1, v2.
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, n, out var newV3);
                Vector3Wide.ConditionalSelect(preloopComplete, v3, newV3, out v3);

                //By now, the simplex is a tetrahedron, but it is not known whether or not the ray actually passes through the portal
                //defined by v1, v2, v3.

                //If the direction is outside the plane defined by v1,v0,v3, then the portal is invalid.
                Vector3Wide.CrossWithoutOverlap(v1, v3, out var v1xv3);
                Vector3Wide.Dot(v1xv3, direction, out var v1xv3DotDirection);
                var shouldReplaceV2 = Vector.AndNot(Vector.LessThan(v1xv3DotDirection, Vector<float>.Zero), preloopComplete);
                //Replace the point that was on the inside of the plane (v2) with the new extreme point.
                Vector3Wide.ConditionalSelect(shouldReplaceV2, v3, v2, out v2);
                Vector3Wide.ConditionalSelect(shouldReplaceV2, v1xv3, n, out n);

                //If the direction is outside the plane defined by v3,v0,v2, then the portal is invalid.
                Vector3Wide.CrossWithoutOverlap(v2, v3, out var v2xv3);
                Vector3Wide.Dot(v2xv3, direction, out var v2xv3DotDirection);
                var shouldReplaceV1 = Vector.AndNot(Vector.AndNot(Vector.GreaterThan(v2xv3DotDirection, Vector<float>.Zero), shouldReplaceV2), preloopComplete);
                //Replace the point that was on the inside of the plane (v1) with the new extreme point.
                Vector3Wide.ConditionalSelect(shouldReplaceV1, v3, v1, out v1);
                Vector3Wide.ConditionalSelect(shouldReplaceV1, v2xv3, n, out n);

                preloopComplete = Vector.BitwiseOr(Vector.AndNot(Vector.OnesComplement(shouldReplaceV1), shouldReplaceV2), preloopComplete);
                if (Vector.LessThanAll(preloopComplete, Vector<int>.Zero))
                {
                    //All lanes have finished simplex finding.
                    break;
                }
            }

            VerifySimplex(default, v1, v2, v3, direction, completedLanes);

            // Refine the portal.
            var countWide = Vector<int>.Zero;
            var surfaceEpsilonSquared = surfaceEpsilon * surfaceEpsilon;
            var maximumIterationsWide = new Vector<int>(maximumIterations);
            while (true)
            {
                //Compute the outward facing normal.
                Vector3Wide.Subtract(v1, v2, out var v2v1);
                Vector3Wide.Subtract(v3, v2, out var v2v3);
                Vector3Wide.CrossWithoutOverlap(v2v1, v2v3, out n);
                
                //Keep working towards the surface.  Find the next extreme point.
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, n, out var v4);

                //If the plane which generated the normal is very close to the extreme point, then we're at the surface.
                Vector3Wide.Dot(n, v1, out var v1DotN);
                Vector3Wide.Dot(v4, n, out var v4DotN);
                //We want to compare actual depths with the epsilon to have a consistent exit, but we don't want a square root.
                //dot(v4, n / ||n||) - dot(v1, n / ||n||) < epsilon
                //dot(v4, n) - dot(v1, n) < epsilon * ||n||
                //(dot(v4, n) - dot(v1, n))^2 < epsilon^2 * ||n||^2
                var difference = v4DotN - v1DotN;
                var differenceSquared = difference * difference;
                Vector3Wide.LengthSquared(n, out nLengthSquared);
                var iterationLimitHit = Vector.GreaterThan(countWide, maximumIterationsWide);
                countWide += Vector<int>.One;
                var surfaceFound = Vector.LessThan(differenceSquared, surfaceEpsilonSquared * nLengthSquared);
                var laneExiting = Vector.AndNot(Vector.BitwiseOr(iterationLimitHit, surfaceFound), completedLanes);
                //Upon exiting, a lane should cache its local normal from N that triggered the exit. It'll be used when the loop finally breaks.
                Vector3Wide.ConditionalSelect(laneExiting, n, localNormal, out localNormal);
                completedLanes = Vector.BitwiseOr(completedLanes, laneExiting);
                if (Vector.LessThanAll(completedLanes, Vector<int>.Zero))
                {
                    Vector3Wide.Dot(localNormal, direction, out nDotDirection);
                    Vector3Wide.Dot(v1, localNormal, out v1DotN);                    
                    t = Vector.ConditionalSelect(Vector.GreaterThan(Vector.Abs(nDotDirection), new Vector<float>(1e-15f)), v1DotN / nDotDirection, Vector<float>.Zero);
                    break;
                }

                //Still haven't exited, so refine the portal.
                //Test direction against the three planes that separate the new portal candidates: (v1,v4,v0) (v2,v4,v0) (v3,v4,v0)
                
                //This may look a little weird at first.
                //'inside' here means 'on the positive side of the plane.'
                //There are three total planes being tested, one for each of v1, v2, and v3.
                //The planes are created from consistently wound vertices, so it's possible to determine
                //where the ray passes through the portal based upon its relationship to two of the three planes.
                //The third vertex which is found to be opposite the face which contains the ray is replaced with the extreme point.

                //This v4 x direction is just a minor reordering of a scalar triple product: (v1 x v4) * direction.
                //It eliminates the need for extra cross products for the inner if.
                Vector3Wide.CrossWithoutOverlap(v4, direction, out var v4xDirection);
                Vector3Wide.Dot(v1, v4xDirection, out var v1PlaneDot);
                Vector3Wide.Dot(v2, v4xDirection, out var v2PlaneDot);
                Vector3Wide.Dot(v3, v4xDirection, out var v3PlaneDot);
                var insideV1 = Vector.GreaterThanOrEqual(v1PlaneDot, Vector<float>.Zero);
                var insideV2 = Vector.GreaterThanOrEqual(v2PlaneDot, Vector<float>.Zero);
                var insideV3 = Vector.GreaterThanOrEqual(v3PlaneDot, Vector<float>.Zero);
                //Inside v1 && inside v2 ==> eliminate v1
                //Outside v1 && outside v3 ==> eliminate v1
                //Outside v1 && inside v3 ==> eliminate v2
                //Inside v1 && outside v2 ==> eliminate v3
                var eliminateV1 = Vector.BitwiseOr(Vector.BitwiseAnd(insideV1, insideV2), Vector.AndNot(Vector.OnesComplement(insideV1), insideV3));
                var eliminateV2 = Vector.AndNot(insideV3, insideV1);
                var eliminateV3 = Vector.AndNot(insideV1, insideV2);
                Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(completedLanes, Vector.Equals(eliminateV1 + eliminateV2 + eliminateV3, new Vector<int>(-1))), Vector<int>.Zero), "Only one vertex should be eliminated.");
                Vector3Wide.ConditionalSelect(eliminateV1, v4, v1, out v1);
                Vector3Wide.ConditionalSelect(eliminateV2, v4, v2, out v2);
                Vector3Wide.ConditionalSelect(eliminateV3, v4, v3, out v3);
                
            }
        }
    }
}
