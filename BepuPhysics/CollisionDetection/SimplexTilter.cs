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
        Initialization,
        TriangleFace = 3,
        Edge = 2,
        Vertex = 1,
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
        static void GetNextNormal<T>(ref Simplex simplex, in Vector3Wide localOffsetB, in Vector3Wide support, in Vector3Wide normal, in Vector<float> depth, ref Vector<int> terminatedLanes,
            in Vector3Wide bestNormal, in Vector<float> bestDepth, ref Vector<int> previousIterationWasGJKStyle,
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

                    Vector3Wide.Subtract(simplex.B.Support, simplex.A.Support, out var abEarly);
                    Vector3Wide.Subtract(simplex.A.Support, simplex.C.Support, out var caEarly);
                    Vector3Wide.Subtract(support, simplex.A.Support, out var ad);
                    Vector3Wide.Subtract(support, simplex.B.Support, out var bd);
                    Vector3Wide.Subtract(support, simplex.C.Support, out var cd);
                    Vector3Wide.CrossWithoutOverlap(abEarly, caEarly, out var triangleNormalEarly);
                    //(ad x n) * d = (n x d) * ad
                    Vector3Wide.CrossWithoutOverlap(triangleNormalEarly, support, out var dxn);
                    Vector3Wide.Dot(dxn, ad, out var adPlaneTest);
                    Vector3Wide.Dot(dxn, bd, out var bdPlaneTest);
                    Vector3Wide.Dot(dxn, cd, out var cdPlaneTest);

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

            //Compute the plane sign tests. Note that these are barycentric weights that have not been scaled by the inverse triangle normal length squared;
            //we do not have to compute the correct magnitude to know the sign, and the sign is all we care about.
            Vector3Wide.CrossWithoutOverlap(ab, simplex.A.Support, out var abxa);
            Vector3Wide.CrossWithoutOverlap(ca, simplex.C.Support, out var caxc);
            Vector3Wide.Dot(abxa, triangleNormal, out var abPlaneTest);
            Vector3Wide.Dot(caxc, triangleNormal, out var caPlaneTest);
            var bcPlaneTest = triangleNormalLengthSquared - caPlaneTest - abPlaneTest;
            var outsideAB = Vector.LessThan(abPlaneTest, Vector<float>.Zero);
            var outsideBC = Vector.LessThan(bcPlaneTest, Vector<float>.Zero);
            var outsideCA = Vector.LessThan(caPlaneTest, Vector<float>.Zero);
            
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

            //If the simplex is degenerate and has length, use the longest edge. Otherwise, just use an edge whose plane is violated.
            //Note that if there are two edges with violated edge planes, simply picking one does not guarantee that the resulting closest point
            //will be the globally closest point on the triangle. It *usually* works out, though, and when it fails, the cost
            //is generally just an extra iteration or two- not a huge deal, and we get to avoid doing more than one edge test every iteration.
            var useAB = Vector.BitwiseOr(Vector.AndNot(outsideAB, simplexDegenerate), Vector.BitwiseAnd(Vector.Equals(longestEdgeLengthSquared, abLengthSquared), simplexDegenerate));
            var useBC = Vector.BitwiseOr(Vector.AndNot(outsideBC, simplexDegenerate), Vector.BitwiseAnd(Vector.Equals(longestEdgeLengthSquared, bcLengthSquared), simplexDegenerate));
            var originOutsideTriangleEdges = Vector.BitwiseOr(outsideAB, Vector.BitwiseOr(outsideBC, outsideCA));

            //If the origin is inside the bounding plane, then we use the simplex portal walls to guide the search.
            //If the origin is outside the bounding plane, we use a GJK-ish closest point->origin search.
            Vector3Wide.Dot(triangleNormal, simplex.A.Support, out var aDot);
            var originOutsideBoundingPlane = Vector.BitwiseOr(
                    Vector.BitwiseAnd(simplexDegenerate, Vector.LessThan(bestDepth, Vector<float>.Zero)),
                    Vector.AndNot(Vector.LessThan(aDot, Vector<float>.Zero), simplexDegenerate));

            if (Vector.EqualsAny(Vector.BitwiseOr(originOutsideBoundingPlane, terminatedLanes), Vector<int>.Zero))
            {
                //At least one lane wants to use portal based search.
                Vector3Wide.CrossWithoutOverlap(bc, simplex.B.Support, out var bcxb);
                Vector3Wide.Dot(abxa, bestNormal, out var abPortalDot);
                Vector3Wide.Dot(caxc, bestNormal, out var caPortalDot);
                Vector3Wide.Dot(bcxb, bestNormal, out var bcPortalDot);

                var outsideABPortal = Vector.ConditionalSelect(shouldCalibrate, Vector.GreaterThan(abPortalDot, Vector<float>.Zero), Vector.LessThan(abPortalDot, Vector<float>.Zero));
                var outsideBCPortal = Vector.ConditionalSelect(shouldCalibrate, Vector.GreaterThan(bcPortalDot, Vector<float>.Zero), Vector.LessThan(bcPortalDot, Vector<float>.Zero));
                var outsideCAPortal = Vector.ConditionalSelect(shouldCalibrate, Vector.GreaterThan(caPortalDot, Vector<float>.Zero), Vector.LessThan(caPortalDot, Vector<float>.Zero));

                originOutsideTriangleEdges = Vector.ConditionalSelect(originOutsideBoundingPlane, originOutsideTriangleEdges, Vector.BitwiseOr(outsideABPortal, Vector.BitwiseOr(outsideBCPortal, outsideCAPortal)));
                useAB = Vector.ConditionalSelect(originOutsideBoundingPlane, useAB, outsideABPortal);
                useBC = Vector.ConditionalSelect(originOutsideBoundingPlane, useBC, outsideBCPortal);
            }

            //Compute the direction from the origin to the closest point on the triangle.
            //If the simplex is degenerate and just a vertex, pick the first simplex entry as representative.
            var originToTriangle = simplex.A.Support;

            var relevantFeatures = Vector<int>.One;
            var alignedBestNormal = bestNormal;

            var useEdge = Vector.BitwiseOr(originOutsideTriangleEdges, simplexIsAnEdge);
            //If this is a vertex case and the sample is right on top of the origin, immediately quit.
            Vector3Wide.LengthSquared(simplex.A.Support, out var vertexLengthSquared);
            terminatedLanes = Vector.BitwiseOr(terminatedLanes, Vector.BitwiseAnd(simplexIsAVertex, Vector.LessThan(vertexLengthSquared, degeneracyEpsilon)));

            if (Vector.LessThanAny(Vector.AndNot(useEdge, terminatedLanes), Vector<int>.Zero))
            {
                var edgeLengthSquared = Vector.ConditionalSelect(useAB, abLengthSquared, Vector.ConditionalSelect(useBC, bcLengthSquared, caLengthSquared));
                var inverseEdgeLengthSquared = Vector<float>.One / edgeLengthSquared;
                Vector3Wide.ConditionalSelect(useAB, ab, ca, out var edgeOffset);
                Vector3Wide.ConditionalSelect(useAB, simplex.A.Support, simplex.C.Support, out var edgeStart);
                Vector3Wide.ConditionalSelect(useBC, bc, edgeOffset, out edgeOffset);
                Vector3Wide.ConditionalSelect(useBC, simplex.B.Support, edgeStart, out edgeStart);

                Vector3Wide.Dot(edgeStart, edgeOffset, out var startDot);
                var t = Vector.ConditionalSelect(Vector.GreaterThan(edgeLengthSquared, Vector<float>.Zero), Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, -startDot * inverseEdgeLengthSquared)), Vector<float>.Zero);
                Vector3Wide.Scale(edgeOffset, t, out var negatedScaledOffset);
                Vector3Wide.Add(negatedScaledOffset, edgeStart, out var originToTriangleCandidate);
                Vector3Wide.LengthSquared(originToTriangleCandidate, out var normalCandidateLengthSquared);
                var originIsOnEdge = Vector.LessThan(normalCandidateLengthSquared, degeneracyEpsilon);
                //If the origin is on either vertex of the edge, the search can terminate; whatever normal found the vertex will work.
                var originNearestStart = Vector.Equals(t, Vector<float>.Zero);
                var originNearestEnd = Vector.Equals(t, Vector<float>.One);
                var originIsOnVertex = Vector.BitwiseAnd(originIsOnEdge, Vector.BitwiseOr(originNearestStart, originNearestEnd));
                terminatedLanes = Vector.BitwiseOr(terminatedLanes, Vector.BitwiseAnd(useEdge, originIsOnVertex));
                var featureForAB = Vector.ConditionalSelect(originNearestStart, Vector<int>.One, Vector.ConditionalSelect(originNearestEnd, new Vector<int>(2), new Vector<int>(1 + 2)));
                var featureForBC = Vector.ConditionalSelect(originNearestStart, new Vector<int>(2), Vector.ConditionalSelect(originNearestEnd, new Vector<int>(4), new Vector<int>(2 + 4)));
                var featureForCA = Vector.ConditionalSelect(originNearestStart, new Vector<int>(4), Vector.ConditionalSelect(originNearestEnd, Vector<int>.One, new Vector<int>(4 + 1)));
                relevantFeatures = Vector.ConditionalSelect(useEdge, Vector.ConditionalSelect(useAB, featureForAB, Vector.ConditionalSelect(useBC, featureForBC, featureForCA)), relevantFeatures);
                //If the origin is on the interior of the edge, it may not yet be at the surface.
                var useOnEdgeFallback = Vector.AndNot(originIsOnEdge, terminatedLanes);
                if (Vector.LessThanAny(useOnEdgeFallback, Vector<int>.Zero))
                {
                    Vector3Wide.CrossWithoutOverlap(bestNormal, edgeOffset, out var fallbackNormal);
                    Vector3Wide.ConditionalSelect(useOnEdgeFallback, fallbackNormal, originToTriangleCandidate, out originToTriangleCandidate);
                }
                Vector3Wide.ConditionalSelect(useEdge, originToTriangleCandidate, originToTriangle, out originToTriangle);
                //if (Vector.EqualsAny(Vector.BitwiseOr(Vector.BitwiseAnd(useEdge, closestToAnEndpoint), terminatedLanes), Vector<int>.Zero))
                {
                    //If the origin is closest to the interior of the edge, remove from the best normal any component along the edge offset.
                    //This ensures the later 'push' phase will push out along the edge-origin plane normal, rather than pushing along the edge offset direction.
                    //Mildly improves convergence.
                    Vector3Wide.Dot(edgeOffset, alignedBestNormal, out var alignmentNormalDot);
                    Vector3Wide.Scale(edgeOffset, alignmentNormalDot * inverseEdgeLengthSquared, out var normalAligningOffset);
                    Vector3Wide.Subtract(bestNormal, normalAligningOffset, out var alignedNormalCandidate);
                    Vector3Wide.ConditionalSelect(useEdge, alignedNormalCandidate, alignedBestNormal, out alignedBestNormal);
                }
            }

            //A few cases:
            //1) The origin is 'outside' the triangle plane or the shapes are known to be separated (negative best depth). Next normal should point from the triangle to origin.
            //2) The origin is inside the triangle plane and the best normal was outside the portal edge planes. Push the (aligned) normal away from the closest point offset.
            //3) The origin is inside the triangle plane is within the portal edge planes. Just use the triangle normal.
            var originContainedInEdgePlanes = Vector.AndNot(Vector.OnesComplement(originOutsideTriangleEdges), simplexDegenerate);
            relevantFeatures = Vector.ConditionalSelect(originContainedInEdgePlanes, new Vector<int>(1 + 2 + 4), relevantFeatures);
            //Start with an assumption that the origin is contained.
            nextNormal = triangleNormal;
            var terminatedOrOriginInEdgePlanes = Vector.BitwiseOr(originContainedInEdgePlanes, terminatedLanes);
            if (Vector.EqualsAny(terminatedOrOriginInEdgePlanes, Vector<int>.Zero))
            {
                //At least one lane needs the non-origin-contained case.

                if (Vector.EqualsAny(Vector.BitwiseOr(originOutsideBoundingPlane, terminatedOrOriginInEdgePlanes), Vector<int>.Zero))
                {
                    //At least one lane is inside the bounding plane and outside the triangle edge planes; push.
                    Vector3Wide.LengthSquared(originToTriangle, out var originDistanceSquared);
                    var inverseDistanceSquared = Vector<float>.One / originDistanceSquared;
                    Vector3Wide.Dot(alignedBestNormal, originToTriangle, out var normalDot);
                    Vector3Wide.Scale(originToTriangle, normalDot * inverseDistanceSquared, out var normalProjectedOnOffset);
                    Vector3Wide.Subtract(alignedBestNormal, normalProjectedOnOffset, out var normalPushOffset);
                    Vector3Wide.Scale(normalPushOffset, new Vector<float>(8), out normalPushOffset);
                    Vector3Wide.Add(alignedBestNormal, normalPushOffset, out var nextNormalCandidate);

                    Vector3Wide.ConditionalSelect(originContainedInEdgePlanes, nextNormal, nextNormalCandidate, out nextNormal);
                }
                //Finally, use the triangle->origin direction directly if it's outside and not within the triangle edge planes. GJK-ish.
                Vector3Wide.Negate(originToTriangle, out var outsideNormalCandidate);
                Vector3Wide.ConditionalSelect(Vector.AndNot(originOutsideBoundingPlane, originContainedInEdgePlanes), outsideNormalCandidate, nextNormal, out nextNormal);
            }

            {
                //DEBUG STUFF
                var features = relevantFeatures[0];
                var vertexCount = (features & 1) + ((features & 2) >> 1) + ((features & 4) >> 2);
                step.NextNormalSource = (SimplexTilterNormalSource)vertexCount;
            }

            simplex.A.Exists = Vector.GreaterThan(Vector.BitwiseAnd(relevantFeatures, Vector<int>.One), Vector<int>.Zero);
            simplex.B.Exists = Vector.GreaterThan(Vector.BitwiseAnd(relevantFeatures, new Vector<int>(2)), Vector<int>.Zero);
            simplex.C.Exists = Vector.GreaterThan(Vector.BitwiseAnd(relevantFeatures, new Vector<int>(4)), Vector<int>.Zero);
            Vector3Wide.Length(nextNormal, out var nextNormalLength);
            Vector3Wide.Scale(nextNormal, Vector<float>.One / nextNormalLength, out nextNormal);
            {
                //DEBUG STUFF
                Vector3Wide.ReadFirst(nextNormal, out step.NextNormal);
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

            var previousIterationWasGJKStyle = Vector<int>.Zero;
            GetNextNormal<HasNoNewSupport>(ref simplex, localOffsetB, default, default, default, ref terminatedLanes, refinedNormal, refinedDepth, ref previousIterationWasGJKStyle, out var normal, out var debugStep);
            debugStep.BestDepth = refinedDepth[0];
            Vector3Wide.ReadSlot(ref refinedNormal, 0, out debugStep.BestNormal);
            steps?.Add(debugStep);

            for (int i = 0; i < maximumIterations; ++i)
            {
                if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
                    break;
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, normal, out var support);
                Vector3Wide.Dot(support, normal, out var depth);

                var useNewDepth = Vector.LessThan(depth, refinedDepth);
                refinedDepth = Vector.ConditionalSelect(useNewDepth, depth, refinedDepth);
                Vector3Wide.ConditionalSelect(useNewDepth, normal, refinedNormal, out refinedNormal);

                GetNextNormal<HasNewSupport>(ref simplex, localOffsetB, support, normal, depth, ref terminatedLanes, refinedNormal, refinedDepth, ref previousIterationWasGJKStyle, out normal, out debugStep);

                debugStep.BestDepth = refinedDepth[0];
                Vector3Wide.ReadSlot(ref refinedNormal, 0, out debugStep.BestNormal);

                steps?.Add(debugStep);
            }
        }
    }
}
