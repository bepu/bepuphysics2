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
        public NotGJKVertex D;
        public bool EdgeCase;
        public Vector3 ClosestPointOnTriangleToOrigin;
        public Vector3 EdgeOffset;
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetPlaneSquaredDistanceNumerator(in Vector3Wide pointOnPlane, in Vector3Wide planeNormal, in Vector<int> useFallback, out Vector<float> numerator)
        {
            Vector3Wide.Dot(planeNormal, pointOnPlane, out var dot);
            numerator = Vector.ConditionalSelect(useFallback, new Vector<float>(float.MaxValue), dot * dot);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ConditionalSelectEdge(
            in Vector3Wide normal, in Vector3Wide support, in Vector3Wide supportNormal, in Vector3Wide supportNormalFallback, in Vector<int> useFallback, in Vector<int> features, in Vector<float> distanceSquared,
            ref Vector3Wide bestNormal, ref Vector3Wide bestSupport, ref Vector3Wide bestSupportNormal, ref Vector<int> bestFeatures, ref Vector<float> bestDistanceNumerator, ref Vector<float> bestDistanceDenominator)
        {
            var useCandidate = Vector.LessThan(distanceSquared * bestDistanceDenominator, bestDistanceNumerator);
            Vector3Wide.ConditionalSelect(useCandidate, normal, bestNormal, out bestNormal);
            Vector3Wide.ConditionalSelect(useCandidate, support, bestSupport, out bestSupport);
            Vector3Wide.ConditionalSelect(useFallback, supportNormalFallback, supportNormal, out var supportNormalCandidate);
            Vector3Wide.ConditionalSelect(useCandidate, supportNormalCandidate, bestSupportNormal, out bestSupportNormal);
            bestDistanceNumerator = Vector.ConditionalSelect(useCandidate, distanceSquared, bestDistanceNumerator);
            bestDistanceDenominator = Vector.ConditionalSelect(useCandidate, Vector<float>.One, bestDistanceDenominator);
            bestFeatures = Vector.ConditionalSelect(useCandidate, features, bestFeatures);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ConditionalSelectFace(
          in Vector3Wide normal, in Vector3Wide support, in Vector<int> features, in Vector<float> distanceNumerator, in Vector<float> distanceDenominator,
          in Vector3Wide localOffsetB, in Vector<int> allow,
          ref Vector3Wide bestNormal, ref Vector3Wide bestSupport, ref Vector3Wide bestSupportNormal, ref Vector<int> bestFeatures, ref Vector<float> bestDistanceNumerator, ref Vector<float> bestDistanceDenominator)
        {
            var useCandidate = Vector.BitwiseAnd(allow, Vector.LessThan(distanceNumerator * bestDistanceDenominator, bestDistanceNumerator * distanceDenominator));
            Vector3Wide.ConditionalSelect(useCandidate, normal, bestNormal, out bestNormal);
            Vector3Wide.ConditionalSelect(useCandidate, support, bestSupport, out bestSupport);
            Vector3Wide.ConditionalSelect(useCandidate, normal, bestSupportNormal, out bestSupportNormal);
            bestDistanceNumerator = Vector.ConditionalSelect(useCandidate, distanceNumerator, bestDistanceNumerator);
            bestDistanceDenominator = Vector.ConditionalSelect(useCandidate, distanceDenominator, bestDistanceDenominator);
            bestFeatures = Vector.ConditionalSelect(useCandidate, features, bestFeatures);
        }

        struct HasNewSupport { }
        struct HasNoNewSupport { }
        static void GetNextNormal<T>(ref Simplex simplex, in Vector3Wide localOffsetB, in Vector3Wide support, in Vector3Wide normal, in Vector<int> terminatedLanes, out Vector3Wide nextNormal, out NotGJKStep step)
        {
            Vector3Wide triangleNormal;
            Vector<float> abPlaneTest, bcPlaneTest, caPlaneTest, triangleNormalLengthSquared;
            //If the simplex is full and there's a new support, then we must choose the best resulting subtriangle (ABD, BCD, or CAD).
            //This shares a codepath with lanes which do not have a full simplex.
            //The first triangle we examine is either ABC, in the event that the simplex had an empty slot, or ABD, if the simplex was full.
            var simplexFull = Vector.BitwiseAnd(simplex.A.Exists, Vector.BitwiseAnd(simplex.B.Exists, simplex.C.Exists));
            //Fill any empty slots with the new support. Combines partial simplex case with degenerate simplex case.
            if (typeof(T) == typeof(HasNewSupport))
            {
                FillSlot(ref simplex.A, support, normal);
                FillSlot(ref simplex.B, support, normal);
                FillSlot(ref simplex.C, support, normal);
                Vector3Wide.ConditionalSelect(simplexFull, support, simplex.C.Support, out var thirdVertex);
                ComputeEdgePlaneTests(simplex.A.Support, simplex.B.Support, thirdVertex, out abPlaneTest, out bcPlaneTest, out caPlaneTest, out triangleNormal, out triangleNormalLengthSquared);
            }
            else
            {
                FillSlot(ref simplex.A, simplex.A.Support, simplex.A.Normal);
                FillSlot(ref simplex.B, simplex.A.Support, simplex.A.Normal);
                FillSlot(ref simplex.C, simplex.A.Support, simplex.A.Normal);
                ComputeEdgePlaneTests(simplex.A.Support, simplex.B.Support, simplex.C.Support, out abPlaneTest, out bcPlaneTest, out caPlaneTest, out triangleNormal, out triangleNormalLengthSquared);
            }

            var degeneracyThreshold = new Vector<float>(1e-14f);
            var triangleDegenerate = Vector.LessThan(triangleNormalLengthSquared, degeneracyThreshold);
            //An edge should be tested if the origin either 1) violates the associated edge's edge plane, or 2) an associated triangle is degenerate.
            //(If the triangle is degenerate, the edge plane signs are meaningless, so we just act as if a violation has occurred.)
            //This first triangle is either ABC (in the simplex not full case) or ABD (in the simplex full case).
            //Outside of the simplexFull branch, we'll act as if we're working on ABC.
            //If it turns out it's actually ABD, we'll replace the results for BC and CA accordingly.
            //The actual edge tests are deferred until after.
            var testAB = Vector.BitwiseOr(Vector.LessThan(abPlaneTest, Vector<float>.Zero), triangleDegenerate);
            var testBC = Vector.BitwiseOr(Vector.AndNot(Vector.LessThan(bcPlaneTest, Vector<float>.Zero), testAB), triangleDegenerate);
            var testCA = Vector.BitwiseOr(Vector.AndNot(Vector.LessThan(caPlaneTest, Vector<float>.Zero), Vector.BitwiseOr(testAB, testBC)), triangleDegenerate);

            Vector3Wide.Dot(triangleNormal, localOffsetB, out var triangleNormalCalibrationDot);
            Vector3Wide.ConditionallyNegate(Vector.LessThan(triangleNormalCalibrationDot, Vector<float>.Zero), ref triangleNormal);
            //Initialize the best choice to the first triangle (ABC or ABD).
            var originOutsideTriangle = Vector.BitwiseOr(Vector.BitwiseOr(testAB, triangleDegenerate), Vector.BitwiseOr(testBC, testCA));
            var bestNormal = triangleNormal;
            var bestSupportNormal = bestNormal;
            var bestSupport = simplex.A.Support;
            GetPlaneSquaredDistanceNumerator(simplex.A.Support, triangleNormal, originOutsideTriangle, out var bestDistanceNumerator);
            var bestDistanceDenominator = triangleNormalLengthSquared;
            var bestFeatures = Vector.ConditionalSelect(simplexFull, new Vector<int>(1 + 2 + 8), new Vector<int>(1 + 2 + 4));

            if (typeof(T) == typeof(HasNewSupport))
            {
                var activeFullSimplex = Vector.AndNot(simplexFull, terminatedLanes);
                if (Vector.LessThanAny(activeFullSimplex, Vector<int>.Zero))
                {
                    //At least one lane has a full simplex.
                    //In this case, the first tested triangle was ABD, and we also have BCD and CAD, where D is the new support point.
                    ComputeEdgePlaneTests(simplex.B.Support, simplex.C.Support, support, out var bcdBC, out var bcdCD, out var bcdDB, out var bcdN, out var bcdNormalLengthSquared);
                    ComputeEdgePlaneTests(simplex.C.Support, simplex.A.Support, support, out var cadCA, out var cadAD, out var cadDC, out var cadN, out var cadNormalLengthSquared);
                    var bcdDegenerate = Vector.LessThan(bcdNormalLengthSquared, degeneracyThreshold);
                    var cadDegenerate = Vector.LessThan(cadNormalLengthSquared, degeneracyThreshold);

                    Vector3Wide.Dot(localOffsetB, bcdN, out var bcdCalibrationDot);
                    Vector3Wide.ConditionallyNegate(Vector.LessThan(bcdCalibrationDot, Vector<float>.Zero), ref bcdN);
                    Vector3Wide.Dot(localOffsetB, cadN, out var cadCalibrationDot);
                    Vector3Wide.ConditionallyNegate(Vector.LessThan(cadCalibrationDot, Vector<float>.Zero), ref cadN);

                    var originOnBCD = Vector.AndNot(Vector.AndNot(Vector.BitwiseAnd(
                        Vector.GreaterThanOrEqual(bcdBC, Vector<float>.Zero), Vector.BitwiseAnd(
                            Vector.GreaterThanOrEqual(bcdCD, Vector<float>.Zero),
                            Vector.GreaterThanOrEqual(bcdDB, Vector<float>.Zero))), bcdDegenerate), terminatedLanes);
                    var originOnCAD = Vector.AndNot(Vector.AndNot(Vector.BitwiseAnd(
                        Vector.GreaterThanOrEqual(cadCA, Vector<float>.Zero), Vector.BitwiseAnd(
                            Vector.GreaterThanOrEqual(cadAD, Vector<float>.Zero),
                            Vector.GreaterThanOrEqual(cadDC, Vector<float>.Zero))), cadDegenerate), terminatedLanes);
                    if (Vector.LessThanAny(originOnBCD, Vector<int>.Zero))
                    {
                        GetPlaneSquaredDistanceNumerator(simplex.B.Support, bcdN, bcdDegenerate, out var bcdNumerator);
                        ConditionalSelectFace(bcdN, simplex.B.Support, new Vector<int>(2 + 4 + 8), bcdNumerator, bcdNormalLengthSquared, localOffsetB, originOnBCD,
                            ref bestNormal, ref bestSupport, ref bestSupportNormal, ref bestFeatures, ref bestDistanceNumerator, ref bestDistanceDenominator);
                    }
                    if (Vector.LessThanAny(originOnCAD, Vector<int>.Zero))
                    {
                        GetPlaneSquaredDistanceNumerator(simplex.C.Support, cadN, cadDegenerate, out var cadNumerator);
                        ConditionalSelectFace(bcdN, simplex.B.Support, new Vector<int>(4 + 1 + 8), cadNumerator, cadNormalLengthSquared, localOffsetB, originOnCAD,
                            ref bestNormal, ref bestSupport, ref bestSupportNormal, ref bestFeatures, ref bestDistanceNumerator, ref bestDistanceDenominator);
                    }

                    var testAD = Vector.BitwiseAnd(Vector.BitwiseOr(Vector.LessThan(caPlaneTest, Vector<float>.Zero), triangleDegenerate), Vector.BitwiseOr(Vector.LessThan(cadAD, Vector<float>.Zero), cadDegenerate));
                    var testBD = Vector.BitwiseAnd(Vector.BitwiseOr(Vector.LessThan(bcPlaneTest, Vector<float>.Zero), triangleDegenerate), Vector.BitwiseOr(Vector.LessThan(bcdDB, Vector<float>.Zero), bcdDegenerate));
                    var testCD = Vector.BitwiseAnd(Vector.BitwiseOr(Vector.LessThan(bcdCD, Vector<float>.Zero), bcdDegenerate), Vector.BitwiseOr(Vector.LessThan(cadDC, Vector<float>.Zero), cadDegenerate));

                    if (Vector.LessThanAny(Vector.AndNot(testAD, terminatedLanes), Vector<int>.Zero))
                    {
                        Vector3Wide.Subtract(support, simplex.A.Support, out var ad);
                        GetNormalForEdge(simplex.A.Support, ad, out var adNormal, out var adDistanceSquared);
                        ConditionalSelectEdge(
                            adNormal, simplex.A.Support, triangleNormal, simplex.A.Normal, triangleDegenerate, new Vector<int>(1 + 8), adDistanceSquared,
                            ref bestNormal, ref bestSupport, ref bestSupportNormal, ref bestFeatures, ref bestDistanceNumerator, ref bestDistanceDenominator);
                    }
                    if (Vector.LessThanAny(Vector.AndNot(testBD, terminatedLanes), Vector<int>.Zero))
                    {
                        Vector3Wide.Subtract(support, simplex.B.Support, out var bd);
                        GetNormalForEdge(simplex.B.Support, bd, out var bdNormal, out var bdDistanceSquared);
                        ConditionalSelectEdge(
                            bdNormal, simplex.B.Support, bcdN, simplex.B.Normal, triangleDegenerate, new Vector<int>(2 + 8), bdDistanceSquared,
                            ref bestNormal, ref bestSupport, ref bestSupportNormal, ref bestFeatures, ref bestDistanceNumerator, ref bestDistanceDenominator);
                    }
                    if (Vector.LessThanAny(Vector.AndNot(testCD, terminatedLanes), Vector<int>.Zero))
                    {
                        Vector3Wide.Subtract(support, simplex.C.Support, out var cd);
                        GetNormalForEdge(simplex.C.Support, cd, out var cdNormal, out var cdDistanceSquared);
                        ConditionalSelectEdge(
                            cdNormal, simplex.C.Support, cadN, simplex.C.Normal, triangleDegenerate, new Vector<int>(4 + 8), cdDistanceSquared,
                            ref bestNormal, ref bestSupport, ref bestSupportNormal, ref bestFeatures, ref bestDistanceNumerator, ref bestDistanceDenominator);
                    }

                    //In the simplex full case, we should overwrite the ABC-case testAB, testBC, and testCA values.
                    testBC = Vector.ConditionalSelect(simplexFull, Vector.BitwiseOr(Vector.AndNot(Vector.LessThan(bcdBC, Vector<float>.Zero), testAB), bcdDegenerate), testBC);
                    testCA = Vector.ConditionalSelect(simplexFull, Vector.BitwiseOr(Vector.AndNot(Vector.LessThan(cadCA, Vector<float>.Zero), Vector.BitwiseOr(testAB, testBC)), cadDegenerate), testCA);
                }
            }
            if (Vector.LessThanAny(Vector.AndNot(testAB, terminatedLanes), Vector<int>.Zero))
            {
                Vector3Wide.Subtract(simplex.B.Support, simplex.A.Support, out var ab);
                GetNormalForEdge(simplex.A.Support, ab, out var abNormal, out var abDistanceSquared);
                ConditionalSelectEdge(
                    abNormal, simplex.A.Support, triangleNormal, simplex.A.Normal, triangleDegenerate, new Vector<int>(1 + 2), abDistanceSquared,
                    ref bestNormal, ref bestSupport, ref bestSupportNormal, ref bestFeatures, ref bestDistanceNumerator, ref bestDistanceDenominator);
            }
            //TODO: This is broken. The triangle normal should be bcdN or cadN for simplexfull case, and the degeneracy test should match.
            if (Vector.LessThanAny(Vector.AndNot(testBC, terminatedLanes), Vector<int>.Zero))
            {
                Vector3Wide.Subtract(simplex.C.Support, simplex.B.Support, out var bc);
                GetNormalForEdge(simplex.B.Support, bc, out var bcNormal, out var bcDistanceSquared);
                ConditionalSelectEdge(
                    bcNormal, simplex.B.Support, triangleNormal, simplex.B.Normal, triangleDegenerate, new Vector<int>(2 + 4), bcDistanceSquared,
                    ref bestNormal, ref bestSupport, ref bestSupportNormal, ref bestFeatures, ref bestDistanceNumerator, ref bestDistanceDenominator);
            }
            if (Vector.LessThanAny(Vector.AndNot(testCA, terminatedLanes), Vector<int>.Zero))
            {
                Vector3Wide.Subtract(simplex.A.Support, simplex.C.Support, out var ca);
                GetNormalForEdge(simplex.C.Support, ca, out var caNormal, out var caDistanceSquared);
                ConditionalSelectEdge(
                    caNormal, simplex.C.Support, triangleNormal, simplex.C.Normal, triangleDegenerate, new Vector<int>(4 + 1), caDistanceSquared,
                    ref bestNormal, ref bestSupport, ref bestSupportNormal, ref bestFeatures, ref bestDistanceNumerator, ref bestDistanceDenominator);
            }

            //Now that we have the best normal and the associated bounding plane, we may need to reflect over the bounding plane in the event that the origin is 'inside'.
            //TODO: Only necessary for edge cases. This implementation is pretty crappy.
            Vector3Wide.Dot(bestSupportNormal, bestSupport, out var planeDepth);
            var vertexCount =
                Vector.BitwiseAnd(bestFeatures, Vector<int>.One) +
                Vector.ConditionalSelect(Vector.GreaterThan(Vector.BitwiseAnd(bestFeatures, new Vector<int>(2)), Vector<int>.Zero), Vector<int>.One, Vector<int>.Zero) +
                Vector.ConditionalSelect(Vector.GreaterThan(Vector.BitwiseAnd(bestFeatures, new Vector<int>(4)), Vector<int>.Zero), Vector<int>.One, Vector<int>.Zero) +
                Vector.ConditionalSelect(Vector.GreaterThan(Vector.BitwiseAnd(bestFeatures, new Vector<int>(8)), Vector<int>.Zero), Vector<int>.One, Vector<int>.Zero);
            var shouldReflect = Vector.BitwiseAnd(Vector.Equals(vertexCount, new Vector<int>(2)), Vector.GreaterThan(planeDepth, Vector<float>.Zero));
            Vector3Wide.LengthSquared(bestSupportNormal, out var supportNormalLengthSquared);
            //bestNormal - 2 * dot(N / ||N||, bestNormal) * N / ||N|| 
            Vector3Wide.Dot(bestNormal, bestSupportNormal, out var reflectionDot);
            Vector3Wide.Scale(bestSupportNormal, 2 * reflectionDot / supportNormalLengthSquared, out var flipOffset);
            Vector3Wide.Subtract(bestNormal, flipOffset, out var flippedNextNormal);
            Vector3Wide.ConditionalSelect(shouldReflect, flippedNextNormal, bestNormal, out nextNormal);
            Vector3Wide.Length(nextNormal, out var normalLength);
            Vector3Wide.Scale(nextNormal, Vector<float>.One / normalLength, out nextNormal);
            
            {
                //DEBUG STUFF
                step = default;
                Vector3Wide.ReadFirst(simplex.A.Support, out step.A.Support);
                Vector3Wide.ReadFirst(simplex.A.Normal, out step.A.Normal);
                Vector3Wide.ReadFirst(simplex.B.Support, out step.B.Support);
                Vector3Wide.ReadFirst(simplex.B.Normal, out step.B.Normal);
                Vector3Wide.ReadFirst(simplex.C.Support, out step.C.Support);
                Vector3Wide.ReadFirst(simplex.C.Normal, out step.C.Normal);
                Vector3Wide.ReadFirst(support, out step.D.Support);
                Vector3Wide.ReadFirst(normal, out step.D.Normal);
                Vector3Wide.ReadFirst(bestNormal, out step.EdgeOffset);
                step.EdgeCase = Vector.Equals(vertexCount, new Vector<int>(2))[0] < 0;
                step.EdgeOffset = step.EdgeCase ? -step.EdgeOffset : default;
                Vector3Wide.ReadFirst(nextNormal, out step.NextNormal);
            }

            //Trim the simplex down to the involved vertices.
            simplex.A.Exists = Vector.GreaterThan(Vector.BitwiseAnd(bestFeatures, new Vector<int>(1)), Vector<int>.Zero);
            simplex.B.Exists = Vector.GreaterThan(Vector.BitwiseAnd(bestFeatures, new Vector<int>(2)), Vector<int>.Zero);
            simplex.C.Exists = Vector.GreaterThan(Vector.BitwiseAnd(bestFeatures, new Vector<int>(4)), Vector<int>.Zero);

            var dExists = Vector.GreaterThan(Vector.BitwiseAnd(bestFeatures, new Vector<int>(8)), Vector<int>.Zero);
            if (Vector.LessThanAny(Vector.AndNot(dExists, terminatedLanes), Vector<int>.Zero))
            {
                var fillA = Vector.AndNot(dExists, simplex.A.Exists);
                var fillB = Vector.AndNot(Vector.AndNot(dExists, simplex.B.Exists), fillA);
                var fillC = Vector.AndNot(Vector.AndNot(dExists, simplex.C.Exists), Vector.BitwiseOr(fillA, fillB));
                ForceFillSlot(fillA, ref simplex.A, support, normal);
                ForceFillSlot(fillB, ref simplex.B, support, normal);
                ForceFillSlot(fillC, ref simplex.C, support, normal);
            }

     
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
    in Vector3Wide initialNormal, in Vector<int> inactiveLanes, in Vector<float> convergenceThreshold, in Vector<float> minimumDepthThreshold, out Vector<float> depth, out Vector3Wide refinedNormal, List<NotGJKStep> steps, int maximumIterations = 50)
        {
#if DEBUG
            Vector3Wide.LengthSquared(initialNormal, out var initialNormalLengthSquared);
            Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThan(Vector.Abs(initialNormalLengthSquared - Vector<float>.One), new Vector<float>(1e-6f))), Vector<int>.Zero));
#endif
            FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, initialNormal, inactiveLanes, out var initialSupport);
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
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, normal, terminatedLanes, out var support);
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
