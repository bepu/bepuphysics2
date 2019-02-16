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
        static void FillSlot(ref Vertex vertex, in Vector3Wide support, in Vector3Wide normal)
        {
            //Note that this always fills empty slots. That's important- we avoid figuring out what subsimplex is active
            //and instead just treat it as a degenerate simplex with some duplicates. (Shares code with the actual degenerate path.)
            Vector3Wide.ConditionalSelect(vertex.Exists, vertex.Support, support, out vertex.Support);
            Vector3Wide.ConditionalSelect(vertex.Exists, vertex.Normal, normal, out vertex.Normal);
            vertex.Exists = new Vector<int>(-1);
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ComputeEdgePlaneTests(in Vector3Wide a, in Vector3Wide b, in Vector3Wide c, out Vector<float> abPlaneTest, out Vector<float> bcPlaneTest, out Vector<float> caPlaneTest, out Vector3Wide triangleNormal, out Vector<float> normalLengthSquared)
        {
            //If you fully explored the identity expansions, there is probably a good amount of potential shared computation between adjacent triangles but... that's a lot of complexity.
            Vector3Wide.Subtract(b, a, out var ab);
            Vector3Wide.Subtract(a, c, out var ca);
            Vector3Wide.CrossWithoutOverlap(ab, ca, out triangleNormal);
            Vector3Wide.LengthSquared(triangleNormal, out normalLengthSquared);
            //Compute the plane sign tests. Note that these are barycentric weights that have not been scaled by the inverse triangle normal length squared;
            //we do not have to compute the correct magnitude to know the sign, and the sign is all we care about.
            Vector3Wide.CrossWithoutOverlap(ab, a, out var abxa);
            Vector3Wide.CrossWithoutOverlap(ca, c, out var caxc);
            Vector3Wide.Dot(abxa, triangleNormal, out abPlaneTest);
            Vector3Wide.Dot(caxc, triangleNormal, out caPlaneTest);
            bcPlaneTest = normalLengthSquared - caPlaneTest - abPlaneTest;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetNormalForEdge(in Vector3Wide edgeStart, in Vector3Wide edgeOffset, out Vector3Wide normal, out Vector<float> distanceSquared)
        {
            Vector3Wide.LengthSquared(edgeOffset, out var edgeOffsetLengthSquared);
            //dot(origin - edgeStart, edgeOffset / ||edgeOffset||) * edgeOffset / ||edgeOffset||
            Vector3Wide.Dot(edgeStart, edgeOffset, out var dot);
            //TODO: Approximate rcp would be sufficient here.
            var negatedT = dot / edgeOffsetLengthSquared;
            negatedT = Vector.Min(Vector<float>.Zero, Vector.Max(new Vector<float>(-1f), negatedT));
            //Protecting against division by zero. TODO: Don't think there's a way to guarantee min/max NaN behavior across hardware to avoid this? Doing RCP first for inf...
            negatedT = Vector.ConditionalSelect(Vector.LessThan(edgeOffsetLengthSquared, new Vector<float>(1e-14f)), Vector<float>.Zero, negatedT);
            Vector3Wide.Scale(edgeOffset, negatedT, out var negativeInterpolatedOffset);
            Vector3Wide.Subtract(negativeInterpolatedOffset, edgeStart, out normal);
            Vector3Wide.LengthSquared(normal, out distanceSquared);
        }

        struct HasNewSupport { }
        struct HasNoNewSupport { }
        static void GetNextNormal<T>(ref Simplex simplex, in Vector3Wide localOffsetB, in Vector3Wide support, in Vector3Wide normal, in Vector<int> terminatedLanes, out Vector3Wide nextNormal, out NotGJKStep step)
        { 
            Vector3Wide triangleNormal;
            Vector<float> abPlaneTest, bcPlaneTest, caPlaneTest, triangleNormalLengthSquared;
            if (typeof(T) == typeof(HasNewSupport))
            {
                //If the simplex is full and there's a new support, then we must choose the best resulting subtriangle (ABD, BCD, or CAD).
                //This shares a codepath with lanes which do not have a full simplex.
                //The first triangle we examine is either ABC, in the event that the simplex had an empty slot, or ABD, if the simplex was full.
                var simplexFull = Vector.BitwiseAnd(simplex.A.Exists, Vector.BitwiseAnd(simplex.B.Exists, simplex.C.Exists));
                //Fill any empty slots with the new support. Combines partial simplex case with degenerate simplex case.
                FillSlot(ref simplex.A, support, normal);
                FillSlot(ref simplex.B, support, normal);
                FillSlot(ref simplex.C, support, normal);
                Vector3Wide.ConditionalSelect(simplexFull, support, simplex.C.Support, out var thirdVertex);

                ComputeEdgePlaneTests(simplex.A.Support, simplex.B.Support, thirdVertex, out abPlaneTest, out bcPlaneTest, out caPlaneTest, out triangleNormal, out triangleNormalLengthSquared);

                var activeFullSimplex = Vector.AndNot(simplexFull, terminatedLanes);
                if (Vector.LessThanAny(activeFullSimplex, Vector<int>.Zero))
                {
                    //At least one lane has a full simplex. Compute the other two triangle weights.
                    ComputeEdgePlaneTests(simplex.B.Support, simplex.C.Support, support, out var bcdBC, out var bcdCD, out var bcdDB, out var bcdN, out var bcdNormalLengthSquared);
                    ComputeEdgePlaneTests(simplex.C.Support, simplex.A.Support, support, out var cadCA, out var cadAD, out var cadDC, out var cadN, out var cadNormalLengthSquared);

                    //For lanes with full simplexes, we have a set of plane tests for all three subtriangles.
                    //We can now choose which triangle is the closest using these.
                    //There are two cases to consider:
                    //1) the origin is outside the planes of at least one subtriangles, so we can pick the subtriangle based on edge plane tests alone.
                    var useABD = Vector.BitwiseAnd(Vector.GreaterThanOrEqual(abPlaneTest, Vector<float>.Zero), Vector.LessThan(bcdDB, Vector<float>.Zero));
                    var useBCD = Vector.BitwiseAnd(Vector.GreaterThanOrEqual(bcdDB, Vector<float>.Zero), Vector.LessThan(cadDC, Vector<float>.Zero));
                    var useCAD = Vector.AndNot(Vector.OnesComplement(useABD), useBCD);
                    //2) the origin is inside all of the subtriangle planes. It's either closest to the face of one of the subtriangles or one of the edges AB, BC, or CA.
                    //Note that we only need to test an edge if the associated subtriangle edge plane (abdAB, bcdBC, cadCA) is violated.
                    //Determining 'insideness' is actually a little tricky because the ABC's winding is not known.
                    //We'll compute ABC's normal and compare it to the localOffsetB to calibrate.
                    //It's not going to be degenerate- in order for the simplex to be full, ABC's face must have contained the origin.
                    //If the numerical situation is good enough to determine containment, it's good enough for calibration.
                    Vector3Wide.Subtract(simplex.B.Support, simplex.A.Support, out var ab);
                    Vector3Wide.Subtract(simplex.A.Support, simplex.C.Support, out var ca);
                    Vector3Wide.CrossWithoutOverlap(ab, ca, out var abcN);
                    Vector3Wide.LengthSquared(abcN, out var abcNLengthSquared);
                    Vector3Wide.Dot(localOffsetB, abcN, out var abcNCalibrationDot);
                    var shouldFlip = Vector.LessThan(abcNCalibrationDot, Vector<float>.Zero);
                    Vector3Wide.Dot(triangleNormal, simplex.A.Support, out var abdSign);
                    Vector3Wide.Dot(bcdN, simplex.B.Support, out var bcdSign);
                    Vector3Wide.Dot(cadN, simplex.C.Support, out var cadSign);
                    var insideABD = Vector.LessThan(Vector.ConditionalSelect(shouldFlip, -abdSign, abdSign), Vector<float>.Zero);
                    var insideBCD = Vector.LessThan(Vector.ConditionalSelect(shouldFlip, -bcdSign, bcdSign), Vector<float>.Zero);
                    var insideCAD = Vector.LessThan(Vector.ConditionalSelect(shouldFlip, -cadSign, cadSign), Vector<float>.Zero);
                    var useInside = Vector.BitwiseAnd(insideABD, Vector.BitwiseAnd(insideBCD, insideCAD));
                    var activeOriginInside = Vector.BitwiseAnd(useInside, activeFullSimplex);
                    if (Vector.LessThanAny(activeOriginInside, Vector<int>.Zero))
                    {
                        //At least one lane has an origin fully contained by the three subtriangle planes.
                        //Check the distance to each of the three planes.
                        //(O - pointOnPlane) * (planeNormal / ||planeNormal||)
                        Vector3Wide.Dot(simplex.A.Support, triangleNormal, out var abdDot);
                        var bestNumerator = abdDot * abdDot;
                        var bestDenominator = triangleNormalLengthSquared;
                        var bestSubtriangle = Vector<int>.Zero;

                        Vector3Wide.Dot(simplex.B.Support, bcdN, out var bcdDot);
                        var bcdNumerator = bcdDot * bcdDot;
                        var bcdBetter = Vector.LessThan(bcdNumerator * bestDenominator, bestNumerator * bcdNormalLengthSquared);
                        bestNumerator = Vector.ConditionalSelect(bcdBetter, bcdNumerator, bestNumerator);
                        bestDenominator = Vector.ConditionalSelect(bcdBetter, bcdNormalLengthSquared, bestDenominator);
                        bestSubtriangle = Vector.ConditionalSelect(bcdBetter, Vector<int>.One, bestSubtriangle);

                        Vector3Wide.Dot(simplex.C.Support, cadN, out var cadDot);
                        var cadNumerator = cadDot * cadDot;
                        var cadBetter = Vector.LessThan(cadNumerator * bestDenominator, bestNumerator * cadNormalLengthSquared);
                        bestNumerator = Vector.ConditionalSelect(cadBetter, cadNumerator, bestNumerator);
                        bestDenominator = Vector.ConditionalSelect(cadBetter, cadNormalLengthSquared, bestDenominator);
                        bestSubtriangle = Vector.ConditionalSelect(cadBetter, new Vector<int>(2), bestSubtriangle);

                        //Note that we throw away the actual results of the edge test. This duplicates work with the later edge test, but keeping the intermediate
                        //results around would be more complicated than just redoing it. Further, these tests should be very rare in practice.
                        //(In other words, this whole thing is an optimization that focuses on the barely intersecting or separating case.)
                        if (Vector.LessThanAny(Vector.BitwiseAnd(activeOriginInside, Vector.LessThan(abPlaneTest, Vector<float>.Zero)), Vector<int>.Zero))
                        {
                            GetNormalForEdge(simplex.A.Support, ab, out _, out var edgeDistanceSquared);
                            var edgeBetter = Vector.LessThan(edgeDistanceSquared * bestDenominator, bestNumerator);
                            bestNumerator = Vector.ConditionalSelect(edgeBetter, edgeDistanceSquared, bestNumerator);
                            bestDenominator = Vector.ConditionalSelect(edgeBetter, Vector<float>.One, bestDenominator);
                            bestSubtriangle = Vector.ConditionalSelect(edgeBetter, Vector<int>.Zero, bestSubtriangle);
                        }
                        if (Vector.LessThanAny(Vector.BitwiseAnd(activeOriginInside, Vector.LessThan(bcdBC, Vector<float>.Zero)), Vector<int>.Zero))
                        {
                            Vector3Wide.Subtract(simplex.C.Support, simplex.B.Support, out var bc);
                            GetNormalForEdge(simplex.B.Support, bc, out _, out var edgeDistanceSquared);
                            var edgeBetter = Vector.LessThan(edgeDistanceSquared * bestDenominator, bestNumerator);
                            bestNumerator = Vector.ConditionalSelect(edgeBetter, edgeDistanceSquared, bestNumerator);
                            bestDenominator = Vector.ConditionalSelect(edgeBetter, Vector<float>.One, bestDenominator);
                            bestSubtriangle = Vector.ConditionalSelect(edgeBetter, Vector<int>.One, bestSubtriangle);
                        }
                        if (Vector.LessThanAny(Vector.BitwiseAnd(activeOriginInside, Vector.LessThan(cadCA, Vector<float>.Zero)), Vector<int>.Zero))
                        {
                            GetNormalForEdge(simplex.C.Support, ca, out _, out var edgeDistanceSquared);
                            var edgeBetter = Vector.LessThan(edgeDistanceSquared * bestDenominator, bestNumerator);
                            bestNumerator = Vector.ConditionalSelect(edgeBetter, edgeDistanceSquared, bestNumerator);
                            bestDenominator = Vector.ConditionalSelect(edgeBetter, Vector<float>.One, bestDenominator);
                            bestSubtriangle = Vector.ConditionalSelect(edgeBetter, new Vector<int>(2), bestSubtriangle);
                        }

                        useABD = Vector.ConditionalSelect(activeOriginInside, Vector.Equals(bestSubtriangle, Vector<int>.Zero), useABD);
                        useBCD = Vector.ConditionalSelect(activeOriginInside, Vector.Equals(bestSubtriangle, Vector<int>.One), useBCD);
                        useCAD = Vector.ConditionalSelect(activeOriginInside, Vector.Equals(bestSubtriangle, new Vector<int>(2)), useCAD);
                    }

                    //Now we know which triangle we want to go forward with. Replace A, B, or C with the new support.
                    useABD = Vector.BitwiseAnd(simplexFull, useABD);
                    useBCD = Vector.BitwiseAnd(simplexFull, useBCD);
                    useCAD = Vector.BitwiseAnd(simplexFull, useCAD);
                    ForceFillSlot(useABD, ref simplex.C, support, normal);
                    ForceFillSlot(useBCD, ref simplex.A, support, normal);
                    ForceFillSlot(useCAD, ref simplex.B, support, normal);

                    //Don't need to swap in ABD's plane tests and normal- we initialized the variables to ABD's values if simplexFull already.
                    //Careful of vertex ordering when choosing these plane tests: ABD, DBC, ADC.
                    abPlaneTest = Vector.ConditionalSelect(useBCD, bcdDB, Vector.ConditionalSelect(useCAD, cadAD, abPlaneTest));
                    bcPlaneTest = Vector.ConditionalSelect(useBCD, bcdBC, Vector.ConditionalSelect(useCAD, cadDC, bcPlaneTest));
                    caPlaneTest = Vector.ConditionalSelect(useBCD, bcdCD, Vector.ConditionalSelect(useCAD, cadCA, caPlaneTest));
                    Vector3Wide.ConditionalSelect(useBCD, bcdN, triangleNormal, out triangleNormal);
                    Vector3Wide.ConditionalSelect(useCAD, cadN, triangleNormal, out triangleNormal);
                }

            }
            else
            {
                //Fill any empty slots to combine degenerate and empty cases.
                FillSlot(ref simplex.A, simplex.A.Support, simplex.A.Normal);
                FillSlot(ref simplex.B, simplex.A.Support, simplex.A.Normal);
                FillSlot(ref simplex.C, simplex.A.Support, simplex.A.Normal);
                ComputeEdgePlaneTests(simplex.A.Support, simplex.B.Support, simplex.C.Support, out abPlaneTest, out bcPlaneTest, out caPlaneTest, out triangleNormal, out triangleNormalLengthSquared);
            }

            //Note that the simplex is not guaranteed to have consistent winding, so calibrate it for remaining use.
            Vector3Wide.Dot(triangleNormal, localOffsetB, out var calibrationDot);
            var shouldCalibrate = Vector.LessThan(calibrationDot, Vector<float>.Zero);
            Vector3Wide.ConditionallyNegate(Vector.LessThan(calibrationDot, Vector<float>.Zero), ref triangleNormal);
            nextNormal = triangleNormal;

            //We need to fall back to an edge test in two cases:
            //1) The simplex is degenerate, so triangular barycentric coordinates cannot be computed
            //2) The projected origin is outside the triangle, so the closest point is on an edge (or vertex).
            //These two cases are collapsed into one to maximize flow coherence.

            //If the simplex is not degenerate, choose which edge to use according to the barycentric weights.
            var useAB = Vector.LessThan(abPlaneTest, Vector<float>.Zero);
            var useBC = Vector.AndNot(Vector.LessThan(bcPlaneTest, Vector<float>.Zero), useAB);
            var useCA = Vector.AndNot(Vector.LessThan(caPlaneTest, Vector<float>.Zero), Vector.BitwiseOr(useAB, useBC));
            var simplexIsDegenerate = Vector.LessThan(triangleNormalLengthSquared, new Vector<float>(1e-14f));
            var useEdgeCase = Vector.AndNot(Vector.BitwiseOr(Vector.BitwiseOr(useAB, useBC), Vector.BitwiseOr(useCA, simplexIsDegenerate)), terminatedLanes);

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
                var inverse = 1f / triangleNormalLengthSquared[0];
                step.ClosestPointOnTriangleToOrigin = step.A.Support * bcPlaneTest[0] * inverse + step.B.Support * caPlaneTest[0] * inverse + step.C.Support * abPlaneTest[0] * inverse;
            }         


            if (Vector.LessThanAny(useEdgeCase, Vector<int>.Zero))
            {
                //If the simplex is degenerate, choose which edge to use based on which is longest.
                Vector3Wide.Subtract(simplex.B.Support, simplex.A.Support, out var ab);
                Vector3Wide.Subtract(simplex.C.Support, simplex.B.Support, out var bc);
                Vector3Wide.Subtract(simplex.A.Support, simplex.C.Support, out var ca);
                Vector3Wide.LengthSquared(ab, out var abLengthSquared);
                Vector3Wide.LengthSquared(bc, out var bcLengthSquared);
                Vector3Wide.LengthSquared(ca, out var caLengthSquared);
                var abGreaterThanBC = Vector.GreaterThan(abLengthSquared, bcLengthSquared);
                var abGreaterThanCA = Vector.GreaterThan(abLengthSquared, caLengthSquared);
                var bcGreaterThanCA = Vector.GreaterThan(bcLengthSquared, caLengthSquared);
                var useABDegenerate = Vector.BitwiseAnd(abGreaterThanBC, abGreaterThanCA);
                var useBCDegenerate = Vector.AndNot(bcGreaterThanCA, useABDegenerate);
                var useCADegenerate = Vector.AndNot(Vector.OnesComplement(useABDegenerate), useBCDegenerate);

                useAB = Vector.ConditionalSelect(simplexIsDegenerate, useABDegenerate, useAB);
                useBC = Vector.ConditionalSelect(simplexIsDegenerate, useBCDegenerate, useBC);
                useCA = Vector.ConditionalSelect(simplexIsDegenerate, useCADegenerate, useCA);
           
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
                //TODO: If we change the normal to allow non-unit length values, this must be updated.
                var inverseFlipNormalLengthSquared = Vector.ConditionalSelect(simplexIsDegenerate, Vector<float>.One, Vector<float>.One / triangleNormalLengthSquared);

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

                if (useEdgeCase[0] < 0)
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

            GetNextNormal<HasNoNewSupport>(ref simplex, localOffsetB, default, default, terminatedLanes, out var normal, out var debugStep);
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
                
                GetNextNormal<HasNewSupport>(ref simplex, localOffsetB, support, normal, terminatedLanes, out normal, out debugStep);
                debugStep.BestDepth = refinedDepth[0];
                Vector3Wide.ReadSlot(ref refinedNormal, 0, out debugStep.BestNormal);
                steps.Add(debugStep);
            }
        }
    }
}
