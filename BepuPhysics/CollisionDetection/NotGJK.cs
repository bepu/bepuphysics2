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

    public enum NotGJKSimplexNormalSource
    {
        Edge,
        TriangleNormal
    }

    public struct NotGJKVertex
    {
        public Vector3 Support;
        public Vector3 Normal;
    }

    public struct NotGJKStep
    {
        public NotGJKVertex A;
        public NotGJKVertex B;
        public NotGJKVertex C;
        public bool EdgeCase;
        public Vector3 ClosestPointOnTriangleToOrigin;
        public Vector3 NextNormal;
        public float BestDepth;
        public Vector3 BestNormal;
    }


    public static class NotGJK<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
        where TShapeA : IConvexShape
        where TShapeWideA : IShapeWide<TShapeA>
        where TSupportFinderA : ISupportFinder<TShapeA, TShapeWideA>
        where TShapeB : IConvexShape
        where TShapeWideB : IShapeWide<TShapeB>
        where TSupportFinderB : ISupportFinder<TShapeB, TShapeWideB>
    {
        public struct Vertex
        {
            public Vector3Wide Support;
            public Vector3Wide Normal;
            public Vector<int> Exists;
        }

        public struct Simplex
        {
            public Vertex A;
            public Vertex B;
            public Vertex C;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void FillSlot(in Vector<int> shouldFill, ref Vertex vertex, in Vector3Wide support, in Vector3Wide normal)
        {
            //Note that this always fills empty slots. That's important- we avoid figuring out what subsimplex is active
            //and instead just treat it as a degenerate simplex with some duplicates. (Shares code with the actual degenerate path.)
            Vector3Wide.ConditionalSelect(vertex.Exists, vertex.Support, support, out vertex.Support);
            Vector3Wide.ConditionalSelect(vertex.Exists, vertex.Normal, normal, out vertex.Normal);
            vertex.Exists = Vector.BitwiseOr(vertex.Exists, shouldFill);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ForceFillSlot(in Vector<int> shouldFill, ref Vertex vertex, in Vector3Wide support, in Vector3Wide normal)
        {
            vertex.Exists = Vector.BitwiseOr(vertex.Exists, shouldFill);
            Vector3Wide.ConditionalSelect(shouldFill, support, vertex.Support, out vertex.Support);
            Vector3Wide.ConditionalSelect(shouldFill, normal, vertex.Normal, out vertex.Normal);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Create(in Vector3Wide normal, in Vector3Wide support, out Simplex simplex)
        {
            //While only one slot is actually full, GetNextNormal expects every slot to have some kind of data-
            //for those slots which are not yet filled, it should be duplicates of other data.
            //(The sub-triangle case is treated the same as the degenerate case.)
            simplex.A.Support = support;
            simplex.B.Support = support;
            simplex.C.Support = support;
            simplex.A.Normal = normal;
            simplex.B.Normal = normal;
            simplex.C.Normal = normal;
            simplex.A.Exists = new Vector<int>(-1);
            simplex.B.Exists = Vector<int>.Zero;
            simplex.C.Exists = Vector<int>.Zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FindSupport(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB, in Vector3Wide direction, out Vector3Wide support)
        {
            //support(N, A) - support(-N, B)
            supportFinderA.ComputeLocalSupport(a, direction, out var extremeA);
            Vector3Wide.Negate(direction, out var negatedDirection);
            supportFinderB.ComputeSupport(b, localOrientationB, negatedDirection, out var extremeB);
            Vector3Wide.Add(extremeB, localOffsetB, out extremeB);

            Vector3Wide.Subtract(extremeA, extremeB, out support);
        }

        static void AddNewSample(ref Simplex simplex, in Vector3Wide localOffsetB, in Vector3Wide normal, in Vector3Wide support, in Vector<int> terminatedLanes)
        {
            var simplexFull = Vector.BitwiseAnd(simplex.A.Exists, Vector.BitwiseAnd(simplex.B.Exists, simplex.C.Exists));
            if (Vector.LessThanAny(Vector.AndNot(simplexFull, terminatedLanes), Vector<int>.Zero))
            {
                //At least one lane has a full simplex. With the new support, we have three subtriangles.
                //Choose the subtriangle with minimal plane depth.
                Vector3Wide.Subtract(support, simplex.A.Support, out var ad);
                Vector3Wide.Subtract(support, simplex.B.Support, out var bd);
                Vector3Wide.Subtract(support, simplex.C.Support, out var cd);
                Vector3Wide.Cross(ad, bd, out var adbN);
                Vector3Wide.Cross(bd, cd, out var bdcN);
                Vector3Wide.Cross(cd, ad, out var cdaN);
                //The normals here are not normalized.
                //planeDepth = dot(support, N / ||N||)
                //sign(dot(support, N)) * planeDepth^2 = sign(dot(support, N)) * dot(support, N)^2 / ||N||^2
                //planeDepthADB < planeDepthBDC becomes:
                //sign(dot(support, adbN)) * dot(support, adbN)^2 * ||bdcN||^2 < sign(dot(support, bdcN)) * dot(support, bdcN)^2 * ||adbN||^2
                //No sqrt or division required.
                Vector3Wide.Dot(adbN, support, out var adbDot);
                Vector3Wide.Dot(bdcN, support, out var bdcDot);
                Vector3Wide.Dot(cdaN, support, out var cdaDot);
                Vector3Wide.LengthSquared(adbN, out var adbNLengthSquared);
                Vector3Wide.LengthSquared(bdcN, out var bdcNLengthSquared);
                Vector3Wide.LengthSquared(cdaN, out var cdaNLengthSquared);

                var adbDotSquared = adbDot * adbDot;
                var bdcDotSquared = bdcDot * bdcDot;
                var cdaDotSquared = cdaDot * cdaDot;
                //Note that we computed subtriangle normals with consistent winding, but we haven't maintained the simplex's winding so we calibrate.
                Vector3Wide.Dot(adbN, localOffsetB, out var adbNCalibrationDot);
                var shouldCalibrateNormal = Vector.LessThan(adbNCalibrationDot, Vector<float>.Zero);
                adbDot = Vector.ConditionalSelect(shouldCalibrateNormal, -adbDot, adbDot);
                bdcDot = Vector.ConditionalSelect(shouldCalibrateNormal, -bdcDot, bdcDot);
                cdaDot = Vector.ConditionalSelect(shouldCalibrateNormal, -cdaDot, cdaDot);
                var adbDotSquaredSigned = Vector.ConditionalSelect(Vector.LessThan(adbDot, Vector<float>.Zero), -adbDotSquared, adbDotSquared);
                var bdcDotSquaredSigned = Vector.ConditionalSelect(Vector.LessThan(bdcDot, Vector<float>.Zero), -bdcDotSquared, bdcDotSquared);
                var cdaDotSquaredSigned = Vector.ConditionalSelect(Vector.LessThan(cdaDot, Vector<float>.Zero), -cdaDotSquared, cdaDotSquared);
                var adbLessThanBDC = Vector.LessThan(adbDotSquaredSigned * bdcNLengthSquared, bdcDotSquaredSigned * adbNLengthSquared);
                var adbLessThanCDA = Vector.LessThan(adbDotSquaredSigned * cdaNLengthSquared, cdaDotSquaredSigned * adbNLengthSquared);
                var bdcLessThanCDA = Vector.LessThan(bdcDotSquaredSigned * cdaNLengthSquared, cdaDotSquaredSigned * bdcNLengthSquared);
                var useADB = Vector.BitwiseAnd(adbLessThanBDC, adbLessThanCDA);
                var useBDC = Vector.AndNot(bdcLessThanCDA, useADB);
                var useCDA = Vector.AndNot(Vector.OnesComplement(useADB), useBDC);

                ForceFillSlot(useADB, ref simplex.C, support, normal);
                ForceFillSlot(useBDC, ref simplex.A, support, normal);
                ForceFillSlot(useCDA, ref simplex.B, support, normal);
            }
            //If there is an empty slot, fill it with the new support. Note that this cannot conflict with the 
            //simplexFull case above- that only occurs for lanes which started with a full simplex, and it completes with a full simplex.
            var tryFillSlot = Vector.OnesComplement(simplexFull);
            FillSlot(tryFillSlot, ref simplex.A, support, normal);
            FillSlot(tryFillSlot, ref simplex.B, support, normal);
            FillSlot(tryFillSlot, ref simplex.C, support, normal);
        }
        static void GetNextNormal(ref Simplex simplex, in Vector3Wide localOffsetB, in Vector<int> terminatedLanes, out Vector3Wide nextNormal, out NotGJKStep step)
        {
            //Compute the barycentric coordinates of the origin on the triangle.
            //Note that the above FillSlot calls filled all empty slots with the new support, so
            //we combine the empty simplex case with the degenerate case.
            Vector3Wide.Subtract(simplex.B.Support, simplex.A.Support, out var ab);
            Vector3Wide.Subtract(simplex.A.Support, simplex.C.Support, out var ca);
            Vector3Wide.CrossWithoutOverlap(ab, ca, out var triangleNormal);
            Vector3Wide.LengthSquared(triangleNormal, out var nLengthSquared);

            //TODO: Approximate RCP could be useful here.
            var inverseNLengthSquared = Vector<float>.One / nLengthSquared;
            Vector3Wide.CrossWithoutOverlap(ab, simplex.A.Support, out var axab);
            Vector3Wide.CrossWithoutOverlap(ca, simplex.C.Support, out var cxca);
            Vector3Wide.Dot(axab, triangleNormal, out var cNumerator);
            Vector3Wide.Dot(cxca, triangleNormal, out var bNumerator);
            var cWeight = cNumerator * inverseNLengthSquared;
            var bWeight = bNumerator * inverseNLengthSquared;
            var aWeight = Vector<float>.One - bWeight - cWeight;

            //Note that the simplex is not guaranteed to have consistent winding, so calibrate it for remaining use.
            Vector3Wide.Dot(triangleNormal, localOffsetB, out var calibrationDot);
            var shouldCalibrate = Vector.LessThan(calibrationDot, Vector<float>.Zero);
            Vector3Wide.ConditionallyNegate(Vector.LessThan(calibrationDot, Vector<float>.Zero), ref triangleNormal);

            //We need to fall back to an edge test in two cases:
            //1) The simplex is degenerate, so triangular barycentric coordinates cannot be computed
            //2) The projected origin is outside the triangle, so the closest point is on an edge (or vertex).
            //These two cases are collapsed into one to maximize flow coherence.

            //If the simplex is not degenerate, choose which edge to use according to the barycentric weights.
            var useABBarycentric = Vector.LessThan(cWeight, Vector<float>.Zero);
            var useBCBarycentric = Vector.AndNot(Vector.LessThan(aWeight, Vector<float>.Zero), useABBarycentric);
            var useCABarycentric = Vector.AndNot(Vector.LessThan(bWeight, Vector<float>.Zero), Vector.BitwiseOr(useABBarycentric, useBCBarycentric));
            //If the simplex is degenerate, choose which edge to use based on which is longest.
            Vector3Wide.Subtract(simplex.C.Support, simplex.B.Support, out var bc);
            Vector3Wide.LengthSquared(ab, out var abLengthSquared);
            Vector3Wide.LengthSquared(bc, out var bcLengthSquared);
            Vector3Wide.LengthSquared(ca, out var caLengthSquared);
            var abGreaterThanBC = Vector.GreaterThan(abLengthSquared, bcLengthSquared);
            var abGreaterThanCA = Vector.GreaterThan(abLengthSquared, caLengthSquared);
            var bcGreaterThanCA = Vector.GreaterThan(bcLengthSquared, caLengthSquared);
            var useABDegenerate = Vector.BitwiseAnd(abGreaterThanBC, abGreaterThanCA);
            var useBCDegenerate = Vector.AndNot(bcGreaterThanCA, useABDegenerate);
            var useCADegenerate = Vector.AndNot(Vector.OnesComplement(useABDegenerate), useBCDegenerate);

            var simplexIsDegenerate = Vector.LessThan(nLengthSquared, new Vector<float>(1e-14f));
            var useAB = Vector.ConditionalSelect(simplexIsDegenerate, useABDegenerate, useABBarycentric);
            var useBC = Vector.ConditionalSelect(simplexIsDegenerate, useBCDegenerate, useBCBarycentric);
            var useCA = Vector.ConditionalSelect(simplexIsDegenerate, useCADegenerate, useCABarycentric);
            var useEdgeCase = Vector.AndNot(Vector.BitwiseOr(Vector.BitwiseOr(useAB, useBC), useCA), terminatedLanes);
            {
                //DEBUG STUFF
                step = default;
                Vector3Wide.ReadSlot(ref simplex.A.Support, 0, out step.A.Support);
                Vector3Wide.ReadSlot(ref simplex.A.Normal, 0, out step.A.Normal);
                Vector3Wide.ReadSlot(ref simplex.B.Support, 0, out step.B.Support);
                Vector3Wide.ReadSlot(ref simplex.B.Normal, 0, out step.B.Normal);
                Vector3Wide.ReadSlot(ref simplex.C.Support, 0, out step.C.Support);
                Vector3Wide.ReadSlot(ref simplex.C.Normal, 0, out step.C.Normal);
                step.EdgeCase = useEdgeCase[0] < 0;
                step.ClosestPointOnTriangleToOrigin = step.A.Support * aWeight[0] + step.B.Support * bWeight[0] + step.C.Support * cWeight[0];
            }
            if (Vector.LessThanAny(useEdgeCase, Vector<int>.Zero))
            {
                //At least one lane is using an edge case.
                //Remove unused vertices.
                simplex.A.Exists = Vector.AndNot(simplex.A.Exists, useBC);
                simplex.B.Exists = Vector.AndNot(simplex.B.Exists, useCA);
                simplex.C.Exists = Vector.AndNot(simplex.C.Exists, useAB);

                Vector3Wide edgeOffset, edgeStart;
                edgeOffset.X = Vector.ConditionalSelect(useAB, ab.X, Vector.ConditionalSelect(useBC, bc.X, ca.X));
                edgeOffset.Y = Vector.ConditionalSelect(useAB, ab.Y, Vector.ConditionalSelect(useBC, bc.Y, ca.Y));
                edgeOffset.Z = Vector.ConditionalSelect(useAB, ab.Z, Vector.ConditionalSelect(useBC, bc.Z, ca.Z));
                edgeStart.X = Vector.ConditionalSelect(useAB, simplex.A.Support.X, Vector.ConditionalSelect(useBC, simplex.B.Support.X, simplex.C.Support.X));
                edgeStart.Y = Vector.ConditionalSelect(useAB, simplex.A.Support.Y, Vector.ConditionalSelect(useBC, simplex.B.Support.Y, simplex.C.Support.Y));
                edgeStart.Z = Vector.ConditionalSelect(useAB, simplex.A.Support.Z, Vector.ConditionalSelect(useBC, simplex.B.Support.Z, simplex.C.Support.Z));
                var lengthSquared = Vector.ConditionalSelect(useAB, abLengthSquared, Vector.ConditionalSelect(useBC, bcLengthSquared, caLengthSquared));
                //dot(origin - edgeStart, edgeOffset / ||edgeOffset||) * edgeOffset / ||edgeOffset||
                Vector3Wide.Dot(edgeStart, edgeOffset, out var dot);
                //TODO: Approximate rcp would be sufficient here.
                var t = -dot / lengthSquared;
                t = Vector.Min(Vector<float>.One, Vector.Max(Vector<float>.Zero, t));
                //Protecting against division by zero. TODO: Don't think there's a way to guarantee min/max NaN behavior across hardware to avoid this? Doing RCP first for inf...
                t = Vector.ConditionalSelect(Vector.LessThan(lengthSquared, new Vector<float>(1e-14f)), Vector<float>.Zero, t);
                Vector3Wide.Scale(edgeOffset, t, out var interpolatedOffset);
                Vector3Wide.Add(interpolatedOffset, edgeStart, out var closestPointOnTriangleToOrigin);

                //We don't want to search a normal which points backward into the shape. If the origin is contained within the shape,
                //flip the origin over the plane and target the reflected location.
                //The closest point on the triangle does not change due to this flip, so this is effectively
                //just flipping the resulting normal
                //Note that we don't actually know whether the origin is contained, so we approximate it by comparing the origin's location against
                //the plane of the triangle (or, if the triangle is degenerate, an existing sample's normal).
                //Note that as a part of the 'fill' we did earlier, all slots contain some valid data, so we just pick slot A.
                Vector3Wide.ConditionalSelect(simplexIsDegenerate, simplex.A.Normal, triangleNormal, out var flipPlaneNormal);
                Vector3Wide.Dot(flipPlaneNormal, simplex.A.Support, out var planeDepth);
                var shouldFlipNormal = Vector.GreaterThan(planeDepth, Vector<float>.Zero);
                //If we're using the triangle normal as the bounding plane, we already computed its inverse length squared.
                //If we're using a vertex's bounding plane, the normal is unit length so the inverse is just 1.
                //TODO: If we change the normal to allow non-unit length values, this must be updated.
                var inverseFlipNormalLengthSquared = Vector.ConditionalSelect(simplexIsDegenerate, Vector<float>.One, inverseNLengthSquared);

                //The normal should be from the triangle to the (potentially reflected image of the) origin.
                //To flip over the bounding plane:
                //origin + 2 * dot(N / ||N||, pointOnPlane - origin) * N / ||N|| 
                Vector3Wide.Scale(flipPlaneNormal, 2 * planeDepth * inverseFlipNormalLengthSquared, out var flippedOrigin);
                Vector3Wide edgeNormal;
                edgeNormal.X = Vector.ConditionalSelect(shouldFlipNormal, flippedOrigin.X, Vector<float>.Zero) - closestPointOnTriangleToOrigin.X;
                edgeNormal.Y = Vector.ConditionalSelect(shouldFlipNormal, flippedOrigin.Y, Vector<float>.Zero) - closestPointOnTriangleToOrigin.Y;
                edgeNormal.Z = Vector.ConditionalSelect(shouldFlipNormal, flippedOrigin.Z, Vector<float>.Zero) - closestPointOnTriangleToOrigin.Z;

                //It would be strange for the origin to be right on top of the edge, but it's not numerically impossible.
                //In that case, just fall back to the triangle normal... if it's not degenerate.
                Vector3Wide.LengthSquared(closestPointOnTriangleToOrigin, out var edgeNormalLengthSquared);
                var edgeNormalValid = Vector.GreaterThan(edgeNormalLengthSquared, new Vector<float>(1e-14f));
                Vector3Wide.ConditionalSelect(Vector.BitwiseAnd(useEdgeCase, edgeNormalValid), edgeNormal, nextNormal, out nextNormal);
                var degenerateWithBadEdgeNormal = Vector.AndNot(simplexIsDegenerate, edgeNormalValid);
                if (Vector.LessThanAny(Vector.AndNot(degenerateWithBadEdgeNormal, terminatedLanes), Vector<int>.Zero))
                {
                    //If the origin is right on the edge AND the triangle is degenerate, try interpolating the edge normals.
                    //(This should be hit exceptionally rarely, so the branch is prooobably worth it.)
                    Vector3Wide normalStart, normalEnd;
                    normalStart.X = Vector.ConditionalSelect(useAB, simplex.A.Normal.X, Vector.ConditionalSelect(useBC, simplex.B.Normal.X, simplex.C.Normal.X));
                    normalStart.Y = Vector.ConditionalSelect(useAB, simplex.A.Normal.Y, Vector.ConditionalSelect(useBC, simplex.B.Normal.Y, simplex.C.Normal.Y));
                    normalStart.Z = Vector.ConditionalSelect(useAB, simplex.A.Normal.Z, Vector.ConditionalSelect(useBC, simplex.B.Normal.Z, simplex.C.Normal.Z));
                    normalEnd.X = Vector.ConditionalSelect(useAB, simplex.B.Normal.X, Vector.ConditionalSelect(useBC, simplex.C.Normal.X, simplex.A.Normal.X));
                    normalEnd.Y = Vector.ConditionalSelect(useAB, simplex.B.Normal.Y, Vector.ConditionalSelect(useBC, simplex.C.Normal.Y, simplex.A.Normal.Y));
                    normalEnd.Z = Vector.ConditionalSelect(useAB, simplex.B.Normal.Z, Vector.ConditionalSelect(useBC, simplex.C.Normal.Z, simplex.A.Normal.Z));
                    Vector3Wide.Scale(normalStart, Vector<float>.One - t, out var normalStartContribution);
                    Vector3Wide.Scale(normalEnd, t, out var normalEndContribution);
                    Vector3Wide.Add(normalStartContribution, normalEndContribution, out var interpolatedNormal);
                    Vector3Wide.ConditionalSelect(degenerateWithBadEdgeNormal, interpolatedNormal, nextNormal, out nextNormal);

                    //If the simplex is degenerate, the origin is on the edge, AND the edge spans two identical support points/normals such that no progress will be made,
                    //then the search is complete!
                }

                Vector3Wide.ReadSlot(ref closestPointOnTriangleToOrigin, 0, out step.ClosestPointOnTriangleToOrigin);
            }

            //TODO: We can use the nosqrt/nodiv depth comparison instead of requiring normalized normals.
            Vector3Wide.Length(nextNormal, out var nextNormalLength);
            Vector3Wide.Scale(nextNormal, Vector<float>.One / nextNormalLength, out nextNormal);

            Vector3Wide.ReadSlot(ref nextNormal, 0, out step.NextNormal);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
    in Vector3Wide initialNormal, in Vector<int> inactiveLanes, in Vector<float> convergenceThreshold, in Vector<float> minimumDepthThreshold, out Vector<float> depth, out Vector3Wide refinedNormal, List<NotGJKStep> steps, int maximumIterations = 50)
        {
#if DEBUG
            Vector3Wide.LengthSquared(initialNormal, out var initialNormalLengthSquared);
            Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThan(Vector.Abs(initialNormalLengthSquared - Vector<float>.One), new Vector<float>(1e-6f))), Vector<int>.Zero));
#endif
            FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, initialNormal, out var initialSupport);
            Vector3Wide.Dot(initialSupport, initialNormal, out var initialDepth);
            Create(initialNormal, initialSupport, out var simplex);
            FindMinimumDepth(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, ref simplex, initialDepth, inactiveLanes, convergenceThreshold, minimumDepthThreshold, out depth, out refinedNormal, steps, maximumIterations);
        }

        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            ref Simplex simplex, in Vector<float> initialDepth,
            in Vector<int> inactiveLanes, in Vector<float> surfaceThreshold, in Vector<float> minimumDepthThreshold, out Vector<float> refinedDepth, out Vector3Wide refinedNormal, List<NotGJKStep> steps, int maximumIterations = 50)
        {
#if DEBUG
            Vector3Wide.LengthSquared(simplex.A.Normal, out var initialNormalLengthSquared);
            Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThan(Vector.Abs(initialNormalLengthSquared - Vector<float>.One), new Vector<float>(1e-6f))), Vector<int>.Zero));
#endif            
            var depthBelowThreshold = Vector.LessThan(initialDepth, minimumDepthThreshold);
            var terminatedLanes = Vector.BitwiseOr(depthBelowThreshold, inactiveLanes);

            refinedNormal = simplex.A.Normal;
            refinedDepth = initialDepth;
            if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
            {
                return;
            }

            GetNextNormal(ref simplex, localOffsetB, terminatedLanes, out var normal, out var debugStep);
            debugStep.BestDepth = refinedDepth[0];
            Vector3Wide.ReadSlot(ref refinedNormal, 0, out debugStep.BestNormal);
            steps.Add(debugStep);

            for (int i = 0; i < maximumIterations; ++i)
            {
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, normal, out var support);
                Vector3Wide.Dot(support, normal, out var depth);

                var useNewDepth = Vector.LessThan(depth, refinedDepth);
                refinedDepth = Vector.ConditionalSelect(useNewDepth, depth, refinedDepth);
                Vector3Wide.ConditionalSelect(useNewDepth, normal, refinedNormal, out refinedNormal);

                AddNewSample(ref simplex, localOffsetB, normal, support, terminatedLanes);
                GetNextNormal(ref simplex, localOffsetB, terminatedLanes, out normal, out debugStep);
                debugStep.BestDepth = refinedDepth[0];
                Vector3Wide.ReadSlot(ref refinedNormal, 0, out debugStep.BestNormal);
                steps.Add(debugStep);
            }
        }
    }
}
