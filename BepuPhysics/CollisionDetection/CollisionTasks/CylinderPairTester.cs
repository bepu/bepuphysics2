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
                b, a, localOffsetA, rA, ref supportFinder, ref supportFinder, localNormal, -speculativeMargin, new Vector<float>(1e-4f), 1500, inactiveLanes,
                out localNormal, out var depthBelowThreshold);
            inactiveLanes = Vector.BitwiseOr(depthBelowThreshold, inactiveLanes);

            if(Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //All lanes are either inactive or were found to have a depth lower than the speculative margin, so we can just quit early.
                manifold = default;
                return;
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
