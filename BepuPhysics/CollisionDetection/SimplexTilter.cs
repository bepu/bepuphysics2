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

    public struct SimplexTilterVertex
    {
        public Vector3 Support;
        public Vector3 Normal;
        public float Depth;
        public bool Exists;
    }

    public enum SimplexTilterNormalSource
    {
        Initialization = -1,
        TriangleFace = 0,
        Edge = 1,
        Vertex = 2,
        GJKEdgeStyle = 3,
        //EdgeReflect = 2,
        //Contraction = 3,
    }

    public struct SimplexTilterStep
    {
        public SimplexTilterVertex A;
        public SimplexTilterVertex B;
        public SimplexTilterVertex C;
        public SimplexTilterVertex D;
        public float ProgressionScale;
        public SimplexTilterNormalSource NextNormalSource;
        public Vector3 ClosestPointOnTriangle;
        public Vector3 TiltStart;
        public Vector3 TiltTargetPoint;
        public Vector3 TiltOffset;
        public Vector3 NextNormal;

        public float BestDepth;
        public Vector3 BestNormal;
    }


    public static class SimplexTilter<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
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
            public Vector<float> Depth;
            public Vector<int> Exists;
        }

        public struct Simplex
        {
            public Vertex A;
            public Vertex B;
            public Vertex C;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void FillSlot(ref Vertex vertex, in Vector3Wide support, in Vector3Wide normal, in Vector<float> depth)
        {
            //Note that this always fills empty slots. That's important- we avoid figuring out what subsimplex is active
            //and instead just treat it as a degenerate simplex with some duplicates. (Shares code with the actual degenerate path.)
            Vector3Wide.ConditionalSelect(vertex.Exists, vertex.Support, support, out vertex.Support);
            Vector3Wide.ConditionalSelect(vertex.Exists, vertex.Normal, normal, out vertex.Normal);
            vertex.Depth = Vector.ConditionalSelect(vertex.Exists, vertex.Depth, depth);
            vertex.Exists = new Vector<int>(-1);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ForceFillSlot(in Vector<int> shouldFill, ref Vertex vertex, in Vector3Wide support, in Vector3Wide normal, in Vector<float> depth)
        {
            vertex.Exists = Vector.BitwiseOr(vertex.Exists, shouldFill);
            Vector3Wide.ConditionalSelect(shouldFill, support, vertex.Support, out vertex.Support);
            Vector3Wide.ConditionalSelect(shouldFill, normal, vertex.Normal, out vertex.Normal);
            vertex.Depth = Vector.ConditionalSelect(vertex.Exists, vertex.Depth, depth);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Swap(in Vector<int> shouldSwap, ref Vertex a, ref Vertex b)
        {
            var temp = a;
            Vector3Wide.ConditionalSelect(shouldSwap, b.Support, a.Support, out a.Support);
            Vector3Wide.ConditionalSelect(shouldSwap, b.Normal, a.Normal, out a.Normal);
            a.Depth = Vector.ConditionalSelect(shouldSwap, b.Depth, a.Depth);
            Vector3Wide.ConditionalSelect(shouldSwap, temp.Support, b.Support, out b.Support);
            Vector3Wide.ConditionalSelect(shouldSwap, temp.Normal, b.Normal, out b.Normal);
            b.Depth = Vector.ConditionalSelect(shouldSwap, temp.Depth, b.Depth);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Create(in Vector3Wide normal, in Vector3Wide support, in Vector<float> depth, out Simplex simplex)
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
            simplex.A.Depth = depth;
            simplex.B.Depth = depth;
            simplex.C.Depth = depth;
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

        struct HasNewSupport { }
        struct HasNoNewSupport { }
        static void GetNextNormal<T>(ref Simplex simplex, in Vector3Wide localOffsetB, in Vector3Wide support, in Vector3Wide normal, in Vector<float> depth, ref Vector<float> progressionScale, in Vector<int> terminatedLanes,
            in Vector3Wide bestNormal, in Vector<float> bestDepth,
            out Vector3Wide nextNormal, out SimplexTilterStep step)
        {
            {
                //DEBUG STUFF
                step = default;
                Vector3Wide.ReadFirst(simplex.A.Support, out step.A.Support);
                Vector3Wide.ReadFirst(simplex.B.Support, out step.B.Support);
                Vector3Wide.ReadFirst(simplex.C.Support, out step.C.Support);
                Vector3Wide.ReadFirst(support, out step.D.Support);
                Vector3Wide.ReadFirst(simplex.A.Normal, out step.A.Normal);
                Vector3Wide.ReadFirst(simplex.B.Normal, out step.B.Normal);
                Vector3Wide.ReadFirst(simplex.C.Normal, out step.C.Normal);
                Vector3Wide.ReadFirst(normal, out step.D.Normal);
                step.A.Depth = simplex.A.Depth[0];
                step.B.Depth = simplex.B.Depth[0];
                step.C.Depth = simplex.C.Depth[0];
                step.D.Depth = depth[0];
                step.A.Exists = simplex.A.Exists[0] < 0;
                step.B.Exists = simplex.B.Exists[0] < 0;
                step.C.Exists = simplex.C.Exists[0] < 0;
                step.D.Exists = typeof(T) == typeof(HasNewSupport);
            }

            if (typeof(T) == typeof(HasNewSupport))
            {
                var simplexFull = Vector.BitwiseAnd(simplex.A.Exists, Vector.BitwiseAnd(simplex.B.Exists, simplex.C.Exists));
                //Fill any empty slots with the new support. Combines partial simplex case with degenerate simplex case.
                FillSlot(ref simplex.A, support, normal, depth);
                FillSlot(ref simplex.B, support, normal, depth);
                FillSlot(ref simplex.C, support, normal, depth);



                var activeFullSimplex = Vector.AndNot(simplexFull, terminatedLanes);
                if (Vector.LessThanAny(activeFullSimplex, Vector<int>.Zero))
                {
                    //At least one active lane has a full simplex and an incoming new sample.
                    //Choose one of the resulting triangles based on which triangle contains the current best normal.
                    //The three edges we need to test are: ADO, BDO, CDO.
                    Vector3Wide.CrossWithoutOverlap(simplex.A.Support, support, out var axd);
                    Vector3Wide.CrossWithoutOverlap(simplex.B.Support, support, out var bxd);
                    Vector3Wide.CrossWithoutOverlap(simplex.C.Support, support, out var cxd);
                    Vector3Wide.Dot(axd, bestNormal, out var adPlaneTest);
                    Vector3Wide.Dot(bxd, bestNormal, out var bdPlaneTest);
                    Vector3Wide.Dot(cxd, bestNormal, out var cdPlaneTest);

                    var useABD = Vector.BitwiseAnd(Vector.GreaterThanOrEqual(adPlaneTest, Vector<float>.Zero), Vector.LessThan(bdPlaneTest, Vector<float>.Zero));
                    var useBCD = Vector.BitwiseAnd(Vector.GreaterThanOrEqual(bdPlaneTest, Vector<float>.Zero), Vector.LessThan(cdPlaneTest, Vector<float>.Zero));
                    var useCAD = Vector.BitwiseAnd(Vector.GreaterThanOrEqual(cdPlaneTest, Vector<float>.Zero), Vector.LessThan(adPlaneTest, Vector<float>.Zero));

                    //Because the best normal may have changed due to the latest sample, ABC's portal may not contain the best normal anymore, which may mean
                    //that none of the subtriangles do either. This is fairly rare and the fallback heuristic doesn't matter much- this won't cause cycles because
                    //it can only occur on iterations where depth improvement has been made (and thus the best normal has changed).
                    //So we'll do something stupid and cheap!
                    useABD = Vector.ConditionalSelect(Vector.OnesComplement(Vector.BitwiseOr(Vector.BitwiseOr(useABD, useBCD), useCAD)), new Vector<int>(-1), useABD);

                    ForceFillSlot(Vector.BitwiseAnd(useBCD, simplexFull), ref simplex.A, support, normal, depth);
                    ForceFillSlot(Vector.BitwiseAnd(useCAD, simplexFull), ref simplex.B, support, normal, depth);
                    ForceFillSlot(Vector.BitwiseAnd(useABD, simplexFull), ref simplex.C, support, normal, depth);
                }
            }
            else
            {
                FillSlot(ref simplex.A, simplex.A.Support, simplex.A.Normal, simplex.A.Depth);
                FillSlot(ref simplex.B, simplex.A.Support, simplex.A.Normal, simplex.A.Depth);
                FillSlot(ref simplex.C, simplex.A.Support, simplex.A.Normal, simplex.A.Depth);
            }
            Vector3Wide.Subtract(simplex.B.Support, simplex.A.Support, out var ab);
            Vector3Wide.Subtract(simplex.A.Support, simplex.C.Support, out var ca);
            Vector3Wide.Subtract(simplex.C.Support, simplex.B.Support, out var bc);
            Vector3Wide.CrossWithoutOverlap(ab, ca, out var triangleNormal);
            Vector3Wide.LengthSquared(triangleNormal, out var triangleNormalLengthSquared);

            Vector3Wide.LengthSquared(ab, out var abLengthSquared);
            Vector3Wide.LengthSquared(bc, out var bcLengthSquared);
            Vector3Wide.LengthSquared(ca, out var caLengthSquared);
            var longestEdgeLengthSquared = Vector.Max(Vector.Max(abLengthSquared, bcLengthSquared), caLengthSquared);
            var simplexDegenerate = Vector.LessThanOrEqual(triangleNormalLengthSquared, longestEdgeLengthSquared * 1e-10f);
            var degeneracyEpsilon = new Vector<float>(1e-14f);
            var simplexIsAVertex = Vector.LessThan(longestEdgeLengthSquared, degeneracyEpsilon);
            var simplexIsAnEdge = Vector.AndNot(simplexDegenerate, simplexIsAVertex);

            Vector3Wide.Dot(triangleNormal, localOffsetB, out var calibrationDot);
            var shouldCalibrate = Vector.LessThan(calibrationDot, Vector<float>.Zero);
            Vector3Wide.ConditionallyNegate(Vector.LessThan(calibrationDot, Vector<float>.Zero), ref triangleNormal);
            Swap(shouldCalibrate, ref simplex.A, ref simplex.B);

            //Just assume the triangle normal will be used for now.
            nextNormal = triangleNormal;

            Vector3Wide.CrossWithoutOverlap(simplex.B.Support, simplex.A.Support, out var bxa);
            Vector3Wide.CrossWithoutOverlap(simplex.C.Support, simplex.B.Support, out var cxb);
            Vector3Wide.CrossWithoutOverlap(simplex.A.Support, simplex.C.Support, out var axc);
            Vector3Wide.Dot(bxa, bestNormal, out var abPlaneTest);
            Vector3Wide.Dot(cxb, bestNormal, out var bcPlaneTest);
            Vector3Wide.Dot(axc, bestNormal, out var caPlaneTest);

            //Note that these tests will mark degenerate edges as containing the normal, since a degenerate plane test will be 0.
            var outsideAB = Vector.LessThan(abPlaneTest, Vector<float>.Zero);
            var outsideBC = Vector.LessThan(bcPlaneTest, Vector<float>.Zero);
            var outsideCA = Vector.LessThan(caPlaneTest, Vector<float>.Zero);

            //If the origin is outside the edge planes and the objects are separated, use a GJK-style search.
            var outsideSum = outsideAB + outsideBC + outsideCA;
            var bestNormalContained = Vector.AndNot(Vector.Equals(outsideSum, Vector<int>.Zero), simplexDegenerate);
            var relevantFeatures = new Vector<int>(1 + 2 + 4);
            var useGJKStyleEdge = Vector.AndNot(Vector.AndNot(Vector.LessThan(bestDepth, Vector<float>.Zero), bestNormalContained), terminatedLanes);
            if (Vector.LessThanAny(useGJKStyleEdge, Vector<int>.Zero))
            {
                var useAB = Vector.BitwiseOr(Vector.AndNot(outsideAB, simplexDegenerate), Vector.BitwiseAnd(Vector.Equals(longestEdgeLengthSquared, abLengthSquared), simplexDegenerate));
                var useBC = Vector.BitwiseOr(Vector.AndNot(outsideBC, simplexDegenerate), Vector.BitwiseAnd(Vector.Equals(longestEdgeLengthSquared, bcLengthSquared), simplexDegenerate));
                var useCA = Vector.BitwiseOr(Vector.AndNot(outsideCA, simplexDegenerate), Vector.BitwiseAnd(Vector.Equals(longestEdgeLengthSquared, caLengthSquared), simplexDegenerate));
                Vector3Wide.ConditionalSelect(useCA, ca, bc, out var edgeOffset);
                Vector3Wide.ConditionalSelect(useAB, ab, edgeOffset, out edgeOffset);
                var edgeLengthSquared = Vector.ConditionalSelect(useCA, caLengthSquared, Vector.ConditionalSelect(useAB, abLengthSquared, bcLengthSquared));
                var inverseEdgeLengthSquared = Vector<float>.One / edgeLengthSquared;
                Vector3Wide.ConditionalSelect(useCA, simplex.C.Support, simplex.B.Support, out var edgeStart);
                Vector3Wide.ConditionalSelect(useAB, simplex.A.Support, edgeStart, out edgeStart);
                relevantFeatures = Vector.ConditionalSelect(useGJKStyleEdge, Vector.ConditionalSelect(outsideAB, new Vector<int>(1 + 2), Vector.ConditionalSelect(outsideCA, new Vector<int>(1 + 4), new Vector<int>(2 + 4))), relevantFeatures);
                Vector3Wide.Dot(edgeStart, edgeOffset, out var startDot);
                var negatedT = Vector.ConditionalSelect(Vector.GreaterThan(edgeLengthSquared, Vector<float>.Zero), Vector.Max(new Vector<float>(-1f), Vector.Min(Vector<float>.Zero, startDot * inverseEdgeLengthSquared)), Vector<float>.Zero);
                Vector3Wide.Scale(edgeOffset, negatedT, out var negatedScaledOffset);
                Vector3Wide.Subtract(negatedScaledOffset, edgeStart, out var nextNormalCandidate);
                Vector3Wide.LengthSquared(nextNormalCandidate, out var normalCandidateLengthSquared);
                var originIsOnEdge = Vector.LessThan(normalCandidateLengthSquared, degeneracyEpsilon);
                Vector3Wide.ConditionalSelect(Vector.AndNot(useGJKStyleEdge, originIsOnEdge), nextNormalCandidate, nextNormal, out nextNormal);

            }
            //If the best normal is not contained, then the simplex needs to search it out. We already know which edge(s) see the origin as being outside.
            var useVertex = Vector.AndNot(simplexIsAVertex, useGJKStyleEdge);
            var pushScale = Vector<float>.One;// Vector.ConditionalSelect(flipBestNormal, new Vector<float>(-1f), Vector<float>.One);
            if (Vector.LessThanAny(Vector.AndNot(useVertex, terminatedLanes), Vector<int>.Zero))
            {
                //At least one lane has a vertex normal offset source.
                var abDegenerate = Vector.LessThan(abLengthSquared, degeneracyEpsilon);
                var bcDegenerate = Vector.LessThan(bcLengthSquared, degeneracyEpsilon);
                var caDegenerate = Vector.LessThan(caLengthSquared, degeneracyEpsilon);
                var outsideABOrDegenerate = Vector.BitwiseOr(outsideAB, abDegenerate);
                var outsideBCOrDegenerate = Vector.BitwiseOr(outsideBC, bcDegenerate);
                var outsideCAOrDegenerate = Vector.BitwiseOr(outsideCA, caDegenerate);
                var useA = Vector.BitwiseAnd(outsideCAOrDegenerate, outsideABOrDegenerate);
                var useB = Vector.BitwiseAnd(outsideABOrDegenerate, outsideBCOrDegenerate);
                var useC = Vector.BitwiseAnd(outsideBCOrDegenerate, outsideCAOrDegenerate);
                Vector3Wide.ConditionalSelect(useC, simplex.C.Support, simplex.B.Support, out var vertex);
                Vector3Wide.ConditionalSelect(useA, simplex.A.Support, vertex, out vertex);
                Vector3Wide.Dot(vertex, bestNormal, out var vertexDot);
                Vector3Wide.LengthSquared(vertex, out var vertexLengthSquared);
                //TODO: Vertex can be on top of the origin and it's not a termination condition.
                Vector3Wide.Scale(vertex, vertexDot / vertexLengthSquared, out var pointOnVertexLine);
                Vector3Wide.Subtract(bestNormal, pointOnVertexLine, out var offsetToBestNormal);
                Vector3Wide.Scale(offsetToBestNormal, pushScale, out var normalPushOffset);
                //TODO: Push offset can be zero length.
                Vector3Wide.Add(normalPushOffset, bestNormal, out var nextNormalCandidate);
                Vector3Wide.ConditionalSelect(simplexIsAVertex, nextNormalCandidate, nextNormal, out nextNormal);
                relevantFeatures = Vector.ConditionalSelect(simplexIsAVertex, Vector.ConditionalSelect(useA, Vector<int>.One, Vector.ConditionalSelect(useB, new Vector<int>(2), new Vector<int>(4))), relevantFeatures);
            }
            var useEdge = Vector.AndNot(Vector.AndNot(Vector.OnesComplement(bestNormalContained), useVertex), useGJKStyleEdge);
            if (Vector.LessThanAny(Vector.AndNot(useEdge, terminatedLanes), Vector<int>.Zero))
            {
                //At least one lane has an edge normal offset source.
                Vector3Wide.ConditionalSelect(outsideCA, axc, cxb, out var edgePlaneNormal);
                Vector3Wide.ConditionalSelect(outsideAB, bxa, edgePlaneNormal, out edgePlaneNormal);
                Vector3Wide.Dot(edgePlaneNormal, bestNormal, out var edgeDot);
                Vector3Wide.LengthSquared(edgePlaneNormal, out var edgePlaneNormalLengthSquared);
                //The edge plane normal can't be zero length. If it was, the edge would be considered to containing the best normal.
                Vector3Wide.Scale(edgePlaneNormal, pushScale * edgeDot / edgePlaneNormalLengthSquared, out var normalPushOffset);
                //TODO: normal push offset can be zero length.
                Vector3Wide.Add(normalPushOffset, bestNormal, out var nextNormalCandidate);
                Vector3Wide.ConditionalSelect(useEdge, nextNormalCandidate, nextNormal, out nextNormal);
                relevantFeatures = Vector.ConditionalSelect(useEdge, Vector.ConditionalSelect(outsideAB, new Vector<int>(1 + 2), Vector.ConditionalSelect(outsideCA, new Vector<int>(1 + 4), new Vector<int>(2 + 4))), relevantFeatures);
            }

            simplex.A.Exists = Vector.GreaterThan(Vector.BitwiseAnd(relevantFeatures, Vector<int>.One), Vector<int>.Zero);
            simplex.B.Exists = Vector.GreaterThan(Vector.BitwiseAnd(relevantFeatures, new Vector<int>(2)), Vector<int>.Zero);
            simplex.C.Exists = Vector.GreaterThan(Vector.BitwiseAnd(relevantFeatures, new Vector<int>(4)), Vector<int>.Zero);
            Vector3Wide.Length(nextNormal, out var nextNormalLength);
            Vector3Wide.Scale(nextNormal, Vector<float>.One / nextNormalLength, out nextNormal);
            {
                //DEBUG STUFF
                step.NextNormalSource = bestNormalContained[0] < 0 ? SimplexTilterNormalSource.TriangleFace : useVertex[0] < 0 ? SimplexTilterNormalSource.Vertex : useEdge[0] < 0 ? SimplexTilterNormalSource.Edge : SimplexTilterNormalSource.GJKEdgeStyle;

                Vector3Wide.ReadFirst(nextNormal, out step.NextNormal);
                step.ProgressionScale = progressionScale[0];
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            in Vector3Wide initialNormal, in Vector<int> inactiveLanes, in Vector<float> convergenceThreshold, in Vector<float> minimumDepthThreshold, out Vector<float> depth, out Vector3Wide refinedNormal, List<SimplexTilterStep> steps, int maximumIterations = 50)
        {
#if DEBUG
            Vector3Wide.LengthSquared(initialNormal, out var initialNormalLengthSquared);
            Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThan(Vector.Abs(initialNormalLengthSquared - Vector<float>.One), new Vector<float>(1e-6f))), Vector<int>.Zero));
#endif
            FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, initialNormal, out var initialSupport);
            Vector3Wide.Dot(initialSupport, initialNormal, out var initialDepth);
            Create(initialNormal, initialSupport, initialDepth, out var simplex);
            FindMinimumDepth(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, ref simplex, initialDepth, inactiveLanes, convergenceThreshold, minimumDepthThreshold, out depth, out refinedNormal, steps, maximumIterations);
        }

        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            ref Simplex simplex, in Vector<float> initialDepth,
            in Vector<int> inactiveLanes, in Vector<float> surfaceThreshold, in Vector<float> minimumDepthThreshold, out Vector<float> refinedDepth, out Vector3Wide refinedNormal, List<SimplexTilterStep> steps, int maximumIterations = 50)
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

            var progressionScale = new Vector<float>(0.5f);
            GetNextNormal<HasNoNewSupport>(ref simplex, localOffsetB, default, default, default, ref progressionScale, terminatedLanes, refinedNormal, refinedDepth, out var normal, out var debugStep);
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
                //progressionScale = Vector.ConditionalSelect(useNewDepth, progressionScale, progressionScale * 0.85f);

                GetNextNormal<HasNewSupport>(ref simplex, localOffsetB, support, normal, depth, ref progressionScale, terminatedLanes, refinedNormal, refinedDepth, out normal, out debugStep);

                debugStep.BestDepth = refinedDepth[0];
                Vector3Wide.ReadSlot(ref refinedNormal, 0, out debugStep.BestNormal);

                steps.Add(debugStep);
            }
        }
    }
}
