using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CylinderPairTester : IPairTester<CylinderWide, CylinderWide, Convex4ContactManifoldWide>
    {
        public int BatchSize => 32;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetDepthContributionA(in CylinderWide a, in Vector<float> dotNAY, out Vector<float> contribution)
        {
            contribution = a.HalfLength * Vector.Abs(dotNAY) + a.Radius * Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One - dotNAY * dotNAY));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetDepthContributionB(in CylinderWide b, in Vector3Wide normal, out Vector<float> contribution)
        {
            contribution = b.HalfLength * Vector.Abs(normal.Y) + b.Radius * Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One - normal.Y * normal.Y));
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetDepth(in Vector3Wide localAxisYA, in Vector3Wide localOffsetB, in Vector3Wide localNormal, in CylinderWide a, in CylinderWide b, out Vector<float> depth)
        {
            Vector3Wide.Dot(localAxisYA, localNormal, out var dotNAY);
            Vector3Wide.Dot(localOffsetB, localNormal, out var dotOffsetN);
            GetDepthContributionA(a, dotNAY, out var contributionA);
            GetDepthContributionB(b, localNormal, out var contributionB);
            depth = contributionA + contributionB - Vector.Abs(dotOffsetN);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(
            ref CylinderWide a, ref CylinderWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount,
            out Convex4ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var worldRB);
            //Work in b's local space.
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(worldRA, worldRB, out var rA);
            ref var capsuleAxis = ref rA.Y;
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRB, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);

            //Cylinder-sphere was easy, cylinder-capsule was a bit complicated, and things go downhill from there.
            //The potential normal generators are:
            //capNormalA: trivial
            //capNormalB: trivial
            //sideA vs sideB: capNormalA x capNormalB
            //capEdgeA vs sideB: minimum depth of circle and line, slightly more complex than the distance based implementation in capsule-cylinder because cylinders don't have a spherical margin
            //sideA vs capEdgeB: minimum depth of circle and line again
            //capEdgeA vs capEdgeB: uh oh

            //For reference, the distance between a line and circle (capEdge vs side) involves finding the minimal root of a fourth degree polynomial. 
            //The distance between two circles involves finding the minimal root of an eighth degree polynomial.
            //(We aren't computing *distance* here, but rather minimal depth, but computing minimal depth is, at best, no easier than distance.)

            //We can't actually use the same implementation as capsule-cylinder because it assumed nonnegative distance. In the intersecting case, it uses an approximation-
            //that's fine for a capsule which has a large margin, but it can't work for two cylinders.
            //Cylinder-cylinder has to handle the intersecting case accurately, which means you're stuck (conceptually) enumerating 4 or 8 roots per feature pair.
            //There are some pretty fast solvers around, but it's hard to avoid quite a few square roots given the shapes involved.
            //Plus, it would require a lot of special case mathing.

            //Given that we are going to encounter similar difficulties on cylinder-box, cylinder-triangle, and cylinder-hull, can we use some kind of trick to save time and effort?
            //The common approach would be minkowski space methods like GJK and MPR. v1 used them heavily, but I moved away from them for v2 due to numerical concerns.
            //But many of those numerical concerns had to do with contact generation, not finding normals; we can still use custom analytic contact generators with GJK and friends.

            //A couple of notes on convexity:
            //Separated convex objects have a global minimum (negative) depth; the distance function between convex objects is itself convex. GJK takes advantage of this.
            //Intersecting convex objects, in contrast, may have multiple local minima for depth. MPR doesn't find a global minimum depth- in fact, it might not even be a local minimum.
            //You can still sometimes get away with an suboptimal choice of depth, though. The important thing is that colliding objects are always found to be colliding, and separated objects are always found to be separated.

            //GJK and MPR are not the only possible algorithms in this space. For example, consider gradient descent. Starting from an initial guess normal, you can incrementally walk toward a local minimum 
            //through numerical (or in some cases even analytic) computation of the gradient. Gradient descent isn't always the quickest at converging, but if we have reasonably good starting points it can work.

            //Further, at least at a conceptual level, pure gradient descent is just as powerful as GJK or MPR for the purpose of finding a good normal. Enumerating the possible cases:
            //1) Intersecting; GD finds intersection: this will find, at worst, a local minimum informed by any initial queries.
            //2) Separated; GD finds separation: the separating case is convex, so GD can find the global optimum (ignoring convergence speed)
            //3) Intersecting; GD finds separation: impossible by the separating axis theorem
            //4) Separated; GD finds intersection: impossible with sufficient time- again, due to separation convexity, GD will (eventually) converge to the global separated optimum

            //So we have some options.

            //First, we'll try a few easy known normal candidates.
            //1) Offset from B to A
            Vector3Wide.Length(localOffsetA, out var length);
            Vector3Wide.Scale(localOffsetA, Vector<float>.One / length, out var localNormal);
            GetDepth(rA.Y, localOffsetB, localNormal, a, b, out var depth);
            depth = Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-7f)), new Vector<float>(float.MaxValue), depth);

            //2) Cap normal A
            GetDepthContributionB(b, rA.Y, out var capNormalAContributionB);
            Vector3Wide.Dot(localOffsetA, rA.Y, out var capNormalAOffsetDot);
            var capNormalADepth = a.HalfLength + capNormalAContributionB - Vector.Abs(capNormalAOffsetDot);
            var useCapNormalA = Vector.LessThan(capNormalADepth, depth);
            depth = Vector.ConditionalSelect(useCapNormalA, capNormalADepth, depth);
            Vector3Wide.ConditionalSelect(useCapNormalA, rA.Y, localNormal, out localNormal);

            //3) Cap normal B
            GetDepthContributionA(a, rA.Y.Y, out var capNormalBContributionA);
            var capNormalBDepth = b.HalfLength + capNormalBContributionA - Vector.Abs(localOffsetA.Y);
            var useCapNormalB = Vector.LessThan(capNormalBDepth, depth);
            depth = Vector.ConditionalSelect(useCapNormalB, capNormalBDepth, depth);
            localNormal.X = Vector.ConditionalSelect(useCapNormalB, Vector<float>.Zero, localNormal.X);
            localNormal.Y = Vector.ConditionalSelect(useCapNormalB, Vector<float>.One, localNormal.Y);
            localNormal.Z = Vector.ConditionalSelect(useCapNormalB, Vector<float>.Zero, localNormal.Z);

            //4) Axis A x axis B.
            var axisCrossLengthSquared = rA.Y.X * rA.Y.X + rA.Y.Z * rA.Y.Z;
            var axisCrossFallbackLengthSquared = localOffsetA.X * localOffsetA.X + localOffsetA.Z * localOffsetA.Z;
            //If the axes are parallel, just use the horizontal direction pointing from B to A.
            var useAxisCrossFallback = Vector.LessThan(axisCrossLengthSquared, new Vector<float>(1e-10f));
            var skipAxisCross = Vector.BitwiseAnd(useAxisCrossFallback, Vector.LessThan(axisCrossFallbackLengthSquared, new Vector<float>(1e-10f)));
            var inverseAxisCrossLength = Vector<float>.One / Vector.SquareRoot(Vector.ConditionalSelect(useAxisCrossFallback, axisCrossFallbackLengthSquared, axisCrossLengthSquared));
            Vector3Wide axisAxAxisB;
            axisAxAxisB.X = Vector.ConditionalSelect(useAxisCrossFallback, localOffsetA.X, -rA.Y.Z) * inverseAxisCrossLength;
            axisAxAxisB.Y = Vector<float>.Zero;
            axisAxAxisB.Z = Vector.ConditionalSelect(useAxisCrossFallback, localOffsetA.Z, rA.Y.X) * inverseAxisCrossLength;
            //By virtue of being perpendicular to both cylinder axes, there can be no contribution from the half lengths.
            Vector3Wide.Dot(axisAxAxisB, localOffsetA, out var axisCrossOffsetDot);
            var axisCrossDepth = a.Radius + b.Radius - Vector.Abs(axisCrossOffsetDot);
            var useAxisCross = Vector.AndNot(Vector.LessThan(axisCrossDepth, depth), skipAxisCross);
            depth = Vector.ConditionalSelect(useAxisCross, axisCrossDepth, depth);
            Vector3Wide.ConditionalSelect(useAxisCross, axisAxAxisB, localNormal, out localNormal);

            //Calibrate the normal to point from B to A.
            Vector3Wide.Dot(localNormal, localOffsetA, out var normalDotLocalOffsetA);
            Vector3Wide.ConditionallyNegate(Vector.LessThan(normalDotLocalOffsetA, Vector<float>.Zero), ref localNormal);

            //We now have a decent estimate for the local normal. Refine it to a local minimum.
            CylinderSupportFinder supportFinder = default;
            ManifoldCandidateHelper.CreateInactiveMask(pairCount, out var inactiveLanes);
            GradientDescent<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.Refine(
                b, a, localOffsetA, rA, ref supportFinder, ref supportFinder, localNormal, -speculativeMargin, new Vector<float>(1e-4f), 25, inactiveLanes,
                out localNormal, out var depthBelowThreshold);
            inactiveLanes = Vector.BitwiseOr(depthBelowThreshold, inactiveLanes);

            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //All lanes are either inactive or were found to have a depth lower than the speculative margin, so we can just quit early.
                manifold = default;
                return;
            }

            //We generate contacts according to the dominant features along the collision normal.
            //The possible pairs are:
            //Cap A-Cap B
            //Cap A-Side B
            //Side A-Cap B
            //Side A-Side B
            Vector3Wide.Dot(rA.Y, localNormal, out var nDotAY);
            var capThreshold = new Vector<float>(0.70710678118f);
            var useCapA = Vector.GreaterThan(Vector.Abs(nDotAY), capThreshold);
            var useCapB = Vector.GreaterThan(Vector.Abs(localNormal.Y), capThreshold);
            if (Vector.LessThanAny(Vector.AndNot(Vector.BitwiseAnd(useCapA, useCapB), inactiveLanes), Vector<int>.Zero))
            {
                //At least one active lane needs cap-cap contacts.
                //While the obvious approach is to intersect the cylinder caps, doing so on the contact plane involves intersecting two ellipses.
                //That involves solving a fourth degree polynomial. Doable, but not super duper cheap. Further, when the caps are not parallel, 
                //intersecting the caps doesn't necessarily produce 'better' contacts. It's just a heuristic.

                //Instead, compute the extreme point of both caps along the contact normal. 
                //One or both of the extreme points should lie within the opposing cap when projected along the contact normal if the cylinders aren't parallel.
                //The offset from the center of the source cap to the extreme point on that cap, projected onto the contact plane, gives us a line to test against the opposing cap.
                //Then, take a perpendicular line positioned on the midpoint of first intersection interval and compute another two intersections.

                //In the event that the caps are parallel, we can't rely on the extreme point since there is no unique extreme point. 
                //Instead, pick a point along the offset from cap center A to cap center B that should be contained by both projected cylinders if they are parallel.
                //That's easy enough- just use a weighted point along the offset based on the cylinder radii.

               
                Vector3Wide.Scale(rA.Y, Vector.ConditionalSelect(Vector.LessThan(nDotAY, Vector<float>.Zero), -a.HalfLength, a.HalfLength), out var capCenterA);
                Vector3Wide.Add(capCenterA, localOffsetA, out capCenterA);

                Vector3Wide.Dot(rA.X, localNormal, out var ax);
                Vector3Wide.Dot(rA.Z, localNormal, out var az);
                var horizontalNormalLengthA = Vector.SquareRoot(ax * ax + az * az);
                var normalizeScaleA = a.Radius / horizontalNormalLengthA;
                var horizontalNormalLengthValidA = Vector.GreaterThan(horizontalNormalLengthA, new Vector<float>(1e-7f));
                Vector3Wide.Scale(rA.X, Vector.ConditionalSelect(horizontalNormalLengthValidA, ax * normalizeScaleA, Vector<float>.Zero), out var extremeAX);
                Vector3Wide.Scale(rA.Z, Vector.ConditionalSelect(horizontalNormalLengthValidA, az * normalizeScaleA, Vector<float>.Zero), out var extremeAZ);
                Vector3Wide.Add(extremeAX, capCenterA, out var extremeA);
                Vector3Wide.Add(extremeAZ, extremeA, out extremeA);

                var capCenterBY = Vector.ConditionalSelect(Vector.LessThan(localNormal.Y, Vector<float>.Zero), -b.HalfLength, b.HalfLength);

                var horizontalNormalLengthB = Vector.SquareRoot(localNormal.X * localNormal.X + localNormal.Z * localNormal.Z);
                var normalizeScaleB = b.Radius / horizontalNormalLengthB;
                var horizontalNormalLengthValidB = Vector.GreaterThan(horizontalNormalLengthB, new Vector<float>(1e-7f));
                Vector3Wide extremeB;
                extremeB.X = Vector.ConditionalSelect(horizontalNormalLengthValidA, localNormal.X * normalizeScaleB, Vector<float>.Zero);
                extremeB.Y = capCenterBY;
                extremeB.Z = Vector.ConditionalSelect(horizontalNormalLengthValidA, localNormal.Z * normalizeScaleB, Vector<float>.Zero);

                //Test extremeA projected along the normal against cap B.
                //radiusB^2 >= ||extremeA - localNormal * dot(capNormalB, extremeA - capCenterB) / dot(capNormalB, localNormal) - capCenterB||^2
                var inverseLocalNormalY = Vector<float>.One / localNormal.Y;
                var tAOnB = (extremeA.Y - capCenterBY) * inverseLocalNormalY;
                Vector3Wide.Scale(localNormal, tAOnB, out var projectionOffsetA);
                Vector2Wide projectedExtremeAInLocalB;
                projectedExtremeAInLocalB.X = extremeA.X - localNormal.X * tAOnB;
                projectedExtremeAInLocalB.Y = extremeA.Z - localNormal.Z * tAOnB;
                Vector2Wide.LengthSquared(projectedExtremeAInLocalB, out var projectedDistanceSquaredA);
                var bCapIsParallel = Vector.LessThan(Vector.Abs(localNormal.Y), new Vector<float>(1e-7f));
                var signedRadiusDifferenceSquaredA = Vector.ConditionalSelect(bCapIsParallel, new Vector<float>(float.MaxValue), b.Radius - projectedDistanceSquaredA);

                Vector3Wide capCenterBToCapCenterA;
                capCenterBToCapCenterA.X = capCenterA.X;
                capCenterBToCapCenterA.Y = capCenterA.Y - capCenterBY;
                capCenterBToCapCenterA.Z = capCenterA.Z;
                Vector3Wide.Dot(rA.Y, capCenterBToCapCenterA, out var tCapCenterDistanceA);
                var tACapCenterOnB = tCapCenterDistanceA * inverseLocalNormalY;
                Vector2Wide projectedCapCenterAInLocalB;
                projectedCapCenterAInLocalB.X = capCenterA.X + localNormal.X * tACapCenterOnB;
                projectedCapCenterAInLocalB.Y = capCenterA.Z + localNormal.Z * tACapCenterOnB;

                //Test extremeB projected along the normal against cap A.
                //radiusA^2 >= ||extremeB + localNormal * dot(capNormalA, extremeB - capCenterA) / dot(capNormalA, localNormal) - capCenterA||^2
                Vector3Wide.Subtract(extremeB, capCenterA, out var capCenterAToExtremeB);
                Vector3Wide.Dot(rA.Y, capCenterAToExtremeB, out var tDistanceB);
                var inverseNDotAY = Vector<float>.One / nDotAY;
                var tBOnA = tDistanceB * inverseNDotAY;
                Vector3Wide.Scale(localNormal, tBOnA, out var projectionOffsetB);
                Vector3Wide.Add(extremeB, projectionOffsetB, out var projectedExtremeB);
                Vector3Wide.Subtract(projectedExtremeB, capCenterA, out var capCenterAToProjectedExtremeB);
                Vector2Wide projectedExtremeBInLocalA;
                Vector3Wide.Dot(capCenterAToProjectedExtremeB, rA.X, out projectedExtremeBInLocalA.X);
                Vector3Wide.Dot(capCenterAToProjectedExtremeB, rA.Z, out projectedExtremeBInLocalA.Y);
                Vector2Wide.LengthSquared(projectedExtremeBInLocalA, out var projectedDistanceSquaredB);
                var aCapIsParallel = Vector.LessThan(Vector.Abs(nDotAY), new Vector<float>(1e-7f));
                var signedRadiusDifferenceSquaredB = Vector.ConditionalSelect(aCapIsParallel, new Vector<float>(float.MaxValue), a.Radius - projectedDistanceSquaredB);

                Vector3Wide.Dot(rA.Y, capCenterBToCapCenterA, out var tCapCenterDistanceB);
                var tBCapCenterOnA = tCapCenterDistanceB * inverseNDotAY;
                Vector3Wide.Scale(localNormal, tBCapCenterOnA, out var projectedCapCenterB);
                projectedCapCenterB.Y += capCenterBY;
                Vector3Wide.Subtract(projectedCapCenterB, capCenterA, out var capCenterAToProjectedCapCenterB);
                Vector2Wide projectedCapCenterBInLocalA;
                Vector3Wide.Dot(capCenterAToProjectedCapCenterB, rA.X, out projectedCapCenterBInLocalA.X);
                Vector3Wide.Dot(capCenterAToProjectedCapCenterB, rA.Z, out projectedCapCenterBInLocalA.Y);

                //Choose which interval axis to use and which cap to test it against.
                //If extremeA is the most contained, project extremeA and capCenterA down onto capB's plane along the local normal. The connecting line is the interval axis. 
                //Similar for extremeB.
                var useExtremeA = Vector.LessThan(signedRadiusDifferenceSquaredA, signedRadiusDifferenceSquaredB);

                Vector2Wide.ConditionalSelect(useExtremeA, projectedCapCenterAInLocalB, projectedCapCenterBInLocalA, out var line0Start);
                Vector2Wide.ConditionalSelect(useExtremeA, projectedExtremeAInLocalB, projectedExtremeBInLocalA, out var line0End);
                var radius = Vector.ConditionalSelect(useExtremeA, b.Radius, a.Radius);

                Vector2Wide.Subtract(line0End, line0Start, out var direction0);

                //Note that there is a risk of feature id inconsistency as the caps approach a parallel state. The 'deepest' point will jump violently between frames when near parallel
                //and it's not guaranteed to be contained in the opposing cap.
                //To compensate, we have fallbacks.
                //1) Preferentially use the offset between caps as the first interval direction. If it is zero length, then
                //2) Use the local X axis as the first interval direction.
                //Note that we use blending rather than instantaneous transitions for fallbacks. This is just another attempt to avoid single frame feature id freakouts.
                const float parallelEpsilonMin = 0.99f;
                const float parallelEpsilonMax = 0.999f;
                const float epsilonScale = 1f / (parallelEpsilonMax - parallelEpsilonMin);
                var parallelWeight = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, Vector.Abs(rA.Y.Y) * epsilonScale - new Vector<float>(parallelEpsilonMin * epsilonScale)));
                var horizontalCapDistanceSquared = capCenterBToCapCenterA.X * capCenterBToCapCenterA.X + capCenterBToCapCenterA.Z * capCenterBToCapCenterA.Z;

            }

            manifold = default;

        }

        public void Test(ref CylinderWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref CylinderWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
