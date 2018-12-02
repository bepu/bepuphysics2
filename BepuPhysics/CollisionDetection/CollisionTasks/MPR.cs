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
            supportFinderB.ComputeSupport(b, localOrientationB, direction, out var extremeB);
            Vector3Wide.Add(extremeB, localOffsetB, out extremeB);
            Vector3Wide.Negate(direction, out var negatedNormal);
            supportFinderA.ComputeLocalSupport(a, negatedNormal, out var extremeA);
            Vector3Wide.Subtract(extremeB, extremeA, out support);
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
            ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB, in Vector<float> surfaceEpsilon, in Vector<int> inactiveLanes, out Vector3Wide localNormal)
        {
            //We'll be using a minkowski difference support(N, B) - support(-N, A).
            //So, the starting point for the origin ray is simply localOffsetB.

            //Find an initial portal through which the ray passes.
            Vector3Wide.Negate(localOffsetB, out var originRayDirection);
            FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, originRayDirection, out var v1);

            Vector3Wide.CrossWithoutOverlap(v1, localOffsetB, out var n);
            //Use fallbacks in the parallel case.
            Vector3Wide.LengthSquared(n, out var nLengthSquared);
            //v1 x X = (0, v1.z, -v1.y)
            //v1 x Z = (v1.y, -v1.x, 0)
            var v1YSquared = v1.Y * v1.Y;
            var v1xXLengthSquared = v1YSquared + v1.Z * v1.Z;
            var v1xZLengthSquared = v1.X * v1.X + v1YSquared;
            var normalLengthEpsilon = new Vector<float>(1e-10f);
            var useFallback = Vector.LessThan(nLengthSquared, normalLengthEpsilon);
            var useFallbackAgain = Vector.LessThan(v1xXLengthSquared, normalLengthEpsilon);
            n.X = Vector.ConditionalSelect(useFallback, Vector.ConditionalSelect(useFallbackAgain, v1.Y, Vector<float>.Zero), n.X);
            n.Y = Vector.ConditionalSelect(useFallback, Vector.ConditionalSelect(useFallbackAgain, -v1.X, v1.Z), n.Y);
            n.Z = Vector.ConditionalSelect(useFallback, Vector.ConditionalSelect(useFallbackAgain, Vector<float>.Zero, -v1.Y), n.Z);

            FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, n, out var v2);

            Vector3Wide v3;
            var preloopCompleted = Vector<int>.Zero;
            while (true)
            {
                //Find a final extreme point using the normal of the plane defined by v0, v1, v2.
                Vector3Wide.Subtract(v1, localOffsetB, out var v0v1);
                Vector3Wide.Subtract(v2, localOffsetB, out var v0v2);
                Vector3Wide.CrossWithoutOverlap(v0v1, v0v2, out n);

                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, n, out v3);

                //If the origin is outside the plane defined by (v1, v0, v3) then the portal is invalid.
                Vector3Wide.CrossWithoutOverlap(v1, v3, out var v1xv3);
                Vector3Wide.Dot(v1xv3, localOffsetB, out var originOffset013);
                //v2 was on the inside of the plane; replace it with the latest extreme point v3.
                var firstPlaneTestFoundInvalid = Vector.LessThan(originOffset013, Vector<float>.Zero);
                Vector3Wide.ConditionalSelect(Vector.AndNot(firstPlaneTestFoundInvalid, preloopCompleted), v3, v2, out v2);

                //If the origin is outside the plane defined by (v3, v0, v2) then the portal is invalid.
                Vector3Wide.CrossWithoutOverlap(v3, v2, out var v3xv2);
                Vector3Wide.Dot(v3xv2, localOffsetB, out var originOffset302);
                //v1 was on the inside of the plane; replace it with the latest extreme point v3. Note that if the lane already was handled by the first plane test, this does nothing.
                var secondPlaneTestFoundInvalid = Vector.LessThan(originOffset302, Vector<float>.Zero);
                Vector3Wide.ConditionalSelect(Vector.AndNot(Vector.AndNot(secondPlaneTestFoundInvalid, firstPlaneTestFoundInvalid), preloopCompleted), v3, v1, out v1);

                preloopCompleted = Vector.BitwiseOr(preloopCompleted, Vector.AndNot(Vector.OnesComplement(firstPlaneTestFoundInvalid), secondPlaneTestFoundInvalid));
                if (Vector.LessThanAll(Vector.BitwiseOr(preloopCompleted, inactiveLanes), Vector<int>.Zero))
                    break;
            }
            VerifySimplex(localOffsetB, v1, v2, v3, inactiveLanes);

            //Initial portal created; refine it.
            var squaredSurfaceEpsilon = surfaceEpsilon * surfaceEpsilon;
            while (true)
            {
                Vector3Wide.Subtract(v3, v2, out var v2v3);
                Vector3Wide.Subtract(v1, v2, out var v2v1);
                Vector3Wide.CrossWithoutOverlap(v2v3, v2v1, out n);
                Vector3Wide.Dot(n, v1, out var exitDot);
                var intersecting = Vector.GreaterThan(exitDot, Vector<float>.Zero);
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, n, out var v4);
                Vector3Wide.Subtract(v4, localOffsetB, out var rayDirectionTest);

                //Compare the latest support sample with the triangle. Are we close to the surface?
                Vector3Wide.Subtract(v4, v1, out var v1v4);
                Vector3Wide.Dot(v1v4, n, out var difference);
                //We want to compare actual depths with the epsilon to have a consistent exit, but we don't want a square root.
                //dot(v4, n / ||n||) - dot(v1, n / ||n||) < epsilon
                //dot(v4, n) - dot(v1, n) < epsilon * ||n||
                //(dot(v4, n) - dot(v1, n))^2 < epsilon^2 * ||n||^2
                var squaredDifference = difference * difference;
                Vector3Wide.LengthSquared(n, out nLengthSquared);
                if (Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThanOrEqual(squaredDifference, squaredSurfaceEpsilon * nLengthSquared)), Vector<int>.Zero))
                {
                    Vector3Wide.LengthSquared(localOffsetB, out var fallbackLengthSquared);
                    var useNormalFallback = Vector.LessThan(nLengthSquared, normalLengthEpsilon);
                    var useNormalFallbackAgain = Vector.BitwiseAnd(useNormalFallback, Vector.LessThan(fallbackLengthSquared, normalLengthEpsilon));
                    var finalNormalLengthSquared = Vector.ConditionalSelect(useNormalFallback, fallbackLengthSquared, nLengthSquared);
                    var inverseLength = Vector<float>.One / Vector.SquareRoot(finalNormalLengthSquared);
                    localNormal.X = Vector.ConditionalSelect(useNormalFallbackAgain, Vector<float>.Zero, Vector.ConditionalSelect(useNormalFallback, -localOffsetB.X, n.X) * inverseLength);
                    localNormal.Y = Vector.ConditionalSelect(useNormalFallbackAgain, Vector<float>.Zero, Vector.ConditionalSelect(useNormalFallback, -localOffsetB.Y, n.Y) * inverseLength);
                    localNormal.Z = Vector.ConditionalSelect(useNormalFallbackAgain, Vector<float>.Zero, Vector.ConditionalSelect(useNormalFallback, -localOffsetB.Z, n.Z) * inverseLength);
                    return;
                }
                //Note that, if not all lanes are done, we just let all of them keep going.
                //Still haven't exited, so refine the portal.
                //Test origin against the three planes that separate the new portal candidates: (v1,v4,v0) (v2,v4,v0) (v3,v4,v0)
                Vector3Wide.CrossWithoutOverlap(v4, localOffsetB, out var v4xv0);
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
                Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.Equals(eliminateV1 + eliminateV2 + eliminateV3, new Vector<int>(-1))), Vector<int>.Zero), "Only one vertex should be eliminated.");
                Vector3Wide.ConditionalSelect(eliminateV1, v4, v1, out v1);
                Vector3Wide.ConditionalSelect(eliminateV2, v4, v2, out v2);
                Vector3Wide.ConditionalSelect(eliminateV3, v4, v3, out v3);
            }
        }
    }
}
