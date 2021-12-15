using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    using DepthRefiner = DepthRefiner<ConvexHull, ConvexHullWide, ConvexHullSupportFinder, Triangle, TriangleWide, PretransformedTriangleSupportFinder>;
    public struct TriangleConvexHullTester : IPairTester<TriangleWide, ConvexHullWide, Convex4ContactManifoldWide>
    {
        public int BatchSize => 16;

        public unsafe void Test(ref TriangleWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            Unsafe.SkipInit(out manifold);
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var triangleOrientation);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var hullOrientation);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(triangleOrientation, hullOrientation, out var hullLocalTriangleOrientation);

            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, hullOrientation, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);

            TriangleWide triangle;
            Matrix3x3Wide.TransformWithoutOverlap(a.A, hullLocalTriangleOrientation, out triangle.A);
            Matrix3x3Wide.TransformWithoutOverlap(a.B, hullLocalTriangleOrientation, out triangle.B);
            Matrix3x3Wide.TransformWithoutOverlap(a.C, hullLocalTriangleOrientation, out triangle.C);
            Vector3Wide.Add(triangle.A, triangle.B, out var centroid);
            Vector3Wide.Add(triangle.C, centroid, out centroid);
            Vector3Wide.Scale(centroid, new Vector<float>(1f / 3f), out centroid);
            Vector3Wide.Subtract(triangle.A, centroid, out triangle.A);
            Vector3Wide.Subtract(triangle.B, centroid, out triangle.B);
            Vector3Wide.Subtract(triangle.C, centroid, out triangle.C);
            Vector3Wide.Subtract(centroid, localOffsetB, out var localTriangleCenter);
            Vector3Wide.Subtract(triangle.B, triangle.A, out var triangleAB);
            Vector3Wide.Subtract(triangle.C, triangle.B, out var triangleBC);
            Vector3Wide.Subtract(triangle.A, triangle.C, out var triangleCA);
            //We'll be using B-local triangle vertices quite a bit, so cache them.
            Vector3Wide.Add(triangle.A, localTriangleCenter, out var triangleA);
            Vector3Wide.Add(triangle.B, localTriangleCenter, out var triangleB);
            Vector3Wide.Add(triangle.C, localTriangleCenter, out var triangleC);
            Vector3Wide.CrossWithoutOverlap(triangleAB, triangleCA, out var triangleNormal);
            Vector3Wide.Length(triangleNormal, out var triangleNormalLength);
            Vector3Wide.Scale(triangleNormal, Vector<float>.One / triangleNormalLength, out triangleNormal);

            //Check if the hull's position is within the triangle and below the triangle plane. If so, we can ignore it.
            Vector3Wide.Dot(triangleNormal, localTriangleCenter, out var hullToTriangleCenterDot);
            var hullBelowPlane = Vector.GreaterThanOrEqual(hullToTriangleCenterDot, Vector<float>.Zero);
            Vector3Wide.CrossWithoutOverlap(triangleAB, triangleNormal, out var edgePlaneAB);
            Vector3Wide.CrossWithoutOverlap(triangleBC, triangleNormal, out var edgePlaneBC);
            Vector3Wide.CrossWithoutOverlap(triangleCA, triangleNormal, out var edgePlaneCA);
            Vector3Wide.Dot(edgePlaneAB, triangleA, out var abPlaneTest);
            Vector3Wide.Dot(edgePlaneBC, triangleB, out var bcPlaneTest);
            Vector3Wide.Dot(edgePlaneCA, triangleC, out var caPlaneTest);
            var hullInsideTriangleEdgePlanes =
                Vector.BitwiseAnd(Vector.LessThanOrEqual(abPlaneTest, Vector<float>.Zero),
                    Vector.BitwiseAnd(Vector.LessThanOrEqual(bcPlaneTest, Vector<float>.Zero), Vector.LessThanOrEqual(caPlaneTest, Vector<float>.Zero)));
            var hullInsideAndBelowTriangle = Vector.BitwiseAnd(hullBelowPlane, hullInsideTriangleEdgePlanes);

            var inactiveLanes = BundleIndexing.CreateTrailingMaskForCountInBundle(pairCount);
            TriangleWide.ComputeNondegenerateTriangleMask(triangleAB, triangleCA, triangleNormalLength, out var triangleEpsilonScale, out var nondegenerateMask);
            b.EstimateEpsilonScale(inactiveLanes, out var hullEpsilonScale);
            var epsilonScale = Vector.Min(triangleEpsilonScale, hullEpsilonScale);
            //Note that degenerate triangles will not contribute contacts. They don't have a well defined normal.
            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.OnesComplement(nondegenerateMask));
            inactiveLanes = Vector.BitwiseOr(inactiveLanes, hullInsideAndBelowTriangle);
            //Not every lane will generate contacts. Rather than requiring every lane to carefully clear all contactExists states, just clear them up front.
            manifold.Contact0Exists = default;
            manifold.Contact1Exists = default;
            manifold.Contact2Exists = default;
            manifold.Contact3Exists = default;
            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //No contacts generated.
                return;
            }

            //Note the use of the triangle center as the initial normal rather than the localOffsetA. 
            //Triangles are not guaranteed to be centered on their center of mass, and the DepthRefiner
            //will converge to a depth which does not oppose the so-far best normal- which, on the early iterations,
            //could be the initial normal.
            Vector3Wide.Length(localTriangleCenter, out var centerDistance);
            Vector3Wide.Scale(localTriangleCenter, Vector<float>.One / centerDistance, out var initialNormal);
            var useInitialFallback = Vector.LessThan(centerDistance, new Vector<float>(1e-10f));
            initialNormal.X = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.X);
            initialNormal.Y = Vector.ConditionalSelect(useInitialFallback, Vector<float>.One, initialNormal.Y);
            initialNormal.Z = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.Z);

            //Check if the extreme point of the hull toward the triangle along its face normal lies inside the triangle.
            //If it is, then there's no need for depth refinement.
            var hullSupportFinder = default(ConvexHullSupportFinder);
            var triangleSupportFinder = default(PretransformedTriangleSupportFinder);
            //Sample the hull's extreme point along the triangle face normal- if it's contained within the triangle edge planes, we can avoid doing more expensive refinement.
            Vector3Wide.Negate(triangleNormal, out var negatedTriangleNormal);
            hullSupportFinder.ComputeLocalSupport(b, negatedTriangleNormal, inactiveLanes, out var hullSupportAlongNegatedTriangleNormal);
            Vector3Wide.Subtract(hullSupportAlongNegatedTriangleNormal, localTriangleCenter, out var supportAlongNegatedTriangleNormal);
            Vector3Wide.Dot(supportAlongNegatedTriangleNormal, negatedTriangleNormal, out var triangleFaceDepth);
            Vector3Wide.Subtract(triangleA, hullSupportAlongNegatedTriangleNormal, out var closestToA);
            Vector3Wide.Subtract(triangleB, hullSupportAlongNegatedTriangleNormal, out var closestToB);
            Vector3Wide.Subtract(triangleC, hullSupportAlongNegatedTriangleNormal, out var closestToC);
            Vector3Wide.Dot(edgePlaneAB, closestToA, out var extremeABPlaneTest);
            Vector3Wide.Dot(edgePlaneBC, closestToB, out var extremeBCPlaneTest);
            Vector3Wide.Dot(edgePlaneCA, closestToC, out var extremeCAPlaneTest);
            //Note that the triangle face extreme point can only be trusted if the hull's center is above the triangle's surface *AND* contained within the edge normals.
            //Merely being above the surface is insufficient- imagine a hull off to the side of the triangle, wedged beneath it.
            var triangleNormalIsMinimal = Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    Vector.AndNot(hullInsideTriangleEdgePlanes, hullBelowPlane),
                    Vector.LessThanOrEqual(extremeABPlaneTest, Vector<float>.Zero)),
                Vector.BitwiseAnd(
                    Vector.LessThanOrEqual(extremeBCPlaneTest, Vector<float>.Zero),
                    Vector.LessThanOrEqual(extremeCAPlaneTest, Vector<float>.Zero)));

            var depthThreshold = -speculativeMargin;
            var skipDepthRefine = Vector.BitwiseOr(triangleNormalIsMinimal, inactiveLanes);
            Vector3Wide localNormal, closestOnHull;
            Vector<float> depth;
            if (Vector.EqualsAny(skipDepthRefine, Vector<int>.Zero))
            {
                DepthRefiner.FindMinimumDepth(
                    b, triangle, localTriangleCenter, hullLocalTriangleOrientation, ref hullSupportFinder, ref triangleSupportFinder, initialNormal, skipDepthRefine, 1e-4f * epsilonScale, depthThreshold,
                    out var refinedDepth, out var refinedNormal, out var refinedClosestOnHull);
                Vector3Wide.ConditionalSelect(skipDepthRefine, hullSupportAlongNegatedTriangleNormal, refinedClosestOnHull, out closestOnHull);
                Vector3Wide.ConditionalSelect(skipDepthRefine, negatedTriangleNormal, refinedNormal, out localNormal);
                depth = Vector.ConditionalSelect(skipDepthRefine, triangleFaceDepth, refinedDepth);

            }
            else
            {
                //No depth refine ran; the extreme point prepass did everything we needed. Just use the initial normal.
                localNormal = negatedTriangleNormal;
                closestOnHull = hullSupportAlongNegatedTriangleNormal;
                depth = triangleFaceDepth;
            }


            Vector3Wide.Dot(triangleNormal, localNormal, out var triangleNormalDotLocalNormal);
            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.BitwiseOr(Vector.GreaterThan(triangleNormalDotLocalNormal, new Vector<float>(-TriangleWide.BackfaceNormalDotRejectionThreshold)), Vector.LessThan(depth, depthThreshold)));
            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //No contacts generated.
                return;
            }

            Helpers.FillVectorWithLaneIndices(out var slotOffsetIndices);
            var boundingPlaneEpsilon = 1e-3f * epsilonScale;
            Vector3* slotHullFaceNormals = stackalloc Vector3[Vector<float>.Count];
            Buffer<HullVertexIndex>* hullVertexIndices = stackalloc Buffer<HullVertexIndex>[Vector<float>.Count];
            Unsafe.SkipInit(out Vector3Wide hullFaceNormal);
            int maximumFaceVertexCount = 0;
            for (int slotIndex = 0; slotIndex < pairCount; ++slotIndex)
            {
                if (inactiveLanes[slotIndex] < 0)
                    continue;
                ref var hull = ref b.Hulls[slotIndex];

                ConvexHullTestHelper.PickRepresentativeFace(ref hull, slotIndex, ref localNormal, closestOnHull, slotOffsetIndices, ref boundingPlaneEpsilon, out slotHullFaceNormals[slotIndex], out _, out var bestFaceIndex);
                Vector3Wide.WriteSlot(slotHullFaceNormals[slotIndex], slotIndex, ref hullFaceNormal);
                hull.GetVertexIndicesForFace(bestFaceIndex, out hullVertexIndices[slotIndex]);
                var verticesInFace = hullVertexIndices[slotIndex].Length;
                if (verticesInFace > maximumFaceVertexCount)
                    maximumFaceVertexCount = verticesInFace;
            }

            Vector3Wide.Subtract(triangleA, closestOnHull, out var hullToA);
            Vector3Wide.Subtract(triangleB, closestOnHull, out var hullToB);
            Vector3Wide.Subtract(triangleC, closestOnHull, out var hullToC);
            Vector3Wide.Dot(hullToA, hullFaceNormal, out var numeratorAToHullFace);
            Vector3Wide.Dot(hullToB, hullFaceNormal, out var numeratorBToHullFace);
            Vector3Wide.Dot(hullToC, hullFaceNormal, out var numeratorCToHullFace);
            Vector3Wide.Dot(localNormal, hullFaceNormal, out var denominatorToHullFace);
            var inverseDenominatorToHullFace = Vector<float>.One / denominatorToHullFace;
            var tAToHullFace = numeratorAToHullFace * inverseDenominatorToHullFace;
            var tBToHullFace = numeratorBToHullFace * inverseDenominatorToHullFace;
            var tCToHullFace = numeratorCToHullFace * inverseDenominatorToHullFace;
            Vector3Wide aOnHull, bOnHull, cOnHull;
            aOnHull.X = triangleA.X - localNormal.X * tAToHullFace;
            aOnHull.Y = triangleA.Y - localNormal.Y * tAToHullFace;
            aOnHull.Z = triangleA.Z - localNormal.Z * tAToHullFace;
            bOnHull.X = triangleB.X - localNormal.X * tBToHullFace;
            bOnHull.Y = triangleB.Y - localNormal.Y * tBToHullFace;
            bOnHull.Z = triangleB.Z - localNormal.Z * tBToHullFace;
            cOnHull.X = triangleC.X - localNormal.X * tCToHullFace;
            cOnHull.Y = triangleC.Y - localNormal.Y * tCToHullFace;
            cOnHull.Z = triangleC.Z - localNormal.Z * tCToHullFace;

            Vector3Wide.Subtract(bOnHull, aOnHull, out var abOnHull);
            Vector3Wide.Subtract(cOnHull, bOnHull, out var bcOnHull);
            Vector3Wide.Subtract(aOnHull, cOnHull, out var caOnHull);

            //We do not generate contacts for degenerate triangles; they would have been marked as inactive in the inactiveLanes mask.
            //So we're safe to use a triangle edge as a surface basis.
            Vector3Wide.Normalize(triangleAB, out var triangleTangentX);
            Vector3Wide.CrossWithoutOverlap(triangleTangentX, triangleNormal, out var triangleTangentY);

            Vector3Wide.CrossWithoutOverlap(abOnHull, hullFaceNormal, out var abEdgePlaneOnHull);
            Vector3Wide.CrossWithoutOverlap(bcOnHull, hullFaceNormal, out var bcEdgePlaneOnHull);
            Vector3Wide.CrossWithoutOverlap(caOnHull, hullFaceNormal, out var caEdgePlaneOnHull);

            var inverseTriangleNormalDotLocalNormal = Vector<float>.One / triangleNormalDotLocalNormal;

            int maximumContactCount = Math.Max(6, maximumFaceVertexCount);
            var candidates = stackalloc ManifoldCandidateScalar[maximumContactCount];
            //To find the contact manifold, we'll clip the triangle edges against the hull face as usual, but we're dealing with potentially
            //distinct convex hulls. Rather than vectorizing over the different hulls, we vectorize within each hull.
            for (int slotIndex = 0; slotIndex < pairCount; ++slotIndex)
            {
                if (inactiveLanes[slotIndex] < 0)
                    continue;
                ref var hull = ref b.Hulls[slotIndex];
                var slotFaceNormal = slotHullFaceNormals[slotIndex];
                Vector3Wide.ReadSlot(ref localNormal, slotIndex, out var slotLocalNormal);
                var faceVertexIndices = hullVertexIndices[slotIndex];

                //Test each triangle against the hull face by projecting the triangle onto the hull face and then intersecting the triangle edges against the hull edge planes.
                //While iterating over hull edge planes, also test hull vertices for containment in the *triangle* edge planes to catch triangle vertex contacts.
                //TODO: Could pull a lot of this out into a wide prepass. Would cut down division counts.
                Vector3Wide.ReadSlot(ref triangleA, slotIndex, out var slotTriangleA);
                Vector3Wide.ReadSlot(ref triangleB, slotIndex, out var slotTriangleB);
                Vector3Wide.ReadSlot(ref triangleC, slotIndex, out var slotTriangleC);
                Vector3Wide.ReadSlot(ref triangleNormal, slotIndex, out var slotTriangleNormal);
                var slotInverseTriangleNormalDotLocalNormal = inverseTriangleNormalDotLocalNormal[slotIndex];
                Vector3Wide.ReadSlot(ref aOnHull, slotIndex, out var slotAOnHull);
                Vector3Wide.ReadSlot(ref bOnHull, slotIndex, out var slotBOnHull);
                Vector3Wide.ReadSlot(ref cOnHull, slotIndex, out var slotCOnHull);
                Vector3Wide.ReadSlot(ref triangleTangentX, slotIndex, out var slotTriangleTangentX);
                Vector3Wide.ReadSlot(ref triangleTangentY, slotIndex, out var slotTriangleTangentY);
                Vector3Wide.ReadSlot(ref abEdgePlaneOnHull, slotIndex, out var slotABEdgePlaneOnHull);
                Vector3Wide.ReadSlot(ref bcEdgePlaneOnHull, slotIndex, out var slotBCEdgePlaneOnHull);
                Vector3Wide.ReadSlot(ref caEdgePlaneOnHull, slotIndex, out var slotCAEdgePlaneOnHull);

                var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
                Vector3Wide.ReadSlot(ref hull.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var hullFaceOrigin);
                var previousVertex = hullFaceOrigin;
                var candidateCount = 0;

                float latestEntryAB = float.MinValue, earliestExitAB = float.MaxValue;
                float latestEntryBC = float.MinValue, earliestExitBC = float.MaxValue;
                float latestEntryCA = float.MinValue, earliestExitCA = float.MaxValue;

                var slotABOnHull = slotBOnHull - slotAOnHull;
                var slotBCOnHull = slotCOnHull - slotBOnHull;
                var slotCAOnHull = slotAOnHull - slotCOnHull;
                var slotTriangleAB = slotTriangleB - slotTriangleA;
                var slotTriangleBC = slotTriangleC - slotTriangleB;
                var slotTriangleCA = slotTriangleA - slotTriangleC;

                for (int i = 0; i < faceVertexIndices.Length; ++i)
                {
                    var index = faceVertexIndices[i];
                    Vector3Wide.ReadSlot(ref hull.Points[index.BundleIndex], index.InnerIndex, out var vertex);

                    var hullEdgeOffset = vertex - previousVertex;
                    previousIndex = index;
                    previousVertex = vertex;
                    var ap = vertex - slotAOnHull;
                    var bp = vertex - slotBOnHull;
                    //Note that the edge planes could be zero if the projected edge has zero length. In that case, containment is impossible, because the projected triangle is degenerate.
                    //So, use strict inequality.
                    var vertexContained = Vector3.Dot(ap, slotABEdgePlaneOnHull) < 0 && Vector3.Dot(bp, slotBCEdgePlaneOnHull) < 0 && Vector3.Dot(ap, slotCAEdgePlaneOnHull) < 0;
                    if (vertexContained && candidateCount < maximumContactCount)
                    {
                        //Project the hull vertex down to the triangle's surface. The fact that we determined the vertex was contained means the local normal isn't dangerously perpendicular.
                        var projectionT = Vector3.Dot(vertex - slotTriangleA, slotTriangleNormal) * slotInverseTriangleNormalDotLocalNormal;
                        var projectedVertex = vertex - slotLocalNormal * projectionT;
                        var newContactIndex = candidateCount++;
                        ref var candidate = ref candidates[newContactIndex];
                        //Use triangle.A as the surface basis origin.
                        var toVertex = projectedVertex - slotTriangleA;
                        candidate.X = Vector3.Dot(toVertex, slotTriangleTangentX);
                        candidate.Y = Vector3.Dot(toVertex, slotTriangleTangentY);
                        //Vertex contacts occupy the feature indices after the edge slots.
                        candidate.FeatureId = 6 + i;
                    }

                    //Intersect the three triangle edges against the hull edge.
                    //Use the sign of the denominator to determine if a triangle edge is entering or exiting a given hull edge.
                    var hullEdgePlaneNormal = Vector3.Cross(hullEdgeOffset, slotLocalNormal);
                    var abNumerator = Vector3.Dot(ap, hullEdgePlaneNormal);
                    var abDenominator = Vector3.Dot(hullEdgePlaneNormal, slotABOnHull);
                    if (abDenominator < 0)
                    {
                        if (latestEntryAB * abDenominator > abNumerator) //Note sign flip for comparison.
                            latestEntryAB = abNumerator / abDenominator;
                    }
                    else if (abDenominator > 0)
                    {
                        if (earliestExitAB * abDenominator > abNumerator)
                            earliestExitAB = abNumerator / abDenominator;
                    }
                    else if (abDenominator == 0)
                    {
                        if (abNumerator < 0)
                        {
                            //Parallel and outside the hull face; hull face intersection interval does not exist.
                            earliestExitAB = float.MinValue;
                            latestEntryAB = float.MaxValue;
                        }
                    }
                    var bcNumerator = Vector3.Dot(bp, hullEdgePlaneNormal);
                    var bcDenominator = Vector3.Dot(hullEdgePlaneNormal, slotBCOnHull);
                    if (bcDenominator < 0)
                    {
                        if (latestEntryBC * bcDenominator > bcNumerator) //Note sign flip for comparison.
                            latestEntryBC = bcNumerator / bcDenominator;
                    }
                    else if (bcDenominator > 0)
                    {
                        if (earliestExitBC * bcDenominator > bcNumerator)
                            earliestExitBC = bcNumerator / bcDenominator;
                    }
                    else if (bcDenominator == 0)
                    {
                        if (bcNumerator < 0)
                        {
                            //Parallel and outside the hull face; hull face intersection interval does not exist.
                            earliestExitBC = float.MinValue;
                            latestEntryBC = float.MaxValue;
                        }
                    }
                    var caNumerator = Vector3.Dot(vertex - slotCOnHull, hullEdgePlaneNormal);
                    var caDenominator = Vector3.Dot(hullEdgePlaneNormal, slotCAOnHull);
                    if (caDenominator < 0)
                    {
                        if (latestEntryCA * caDenominator > caNumerator) //Note sign flip for comparison.
                            latestEntryCA = caNumerator / caDenominator;
                    }
                    else if (caDenominator > 0)
                    {
                        if (earliestExitCA * caDenominator > caNumerator)
                            earliestExitCA = caNumerator / caDenominator;
                    }
                    else if (caDenominator == 0)
                    {
                        if (caNumerator < 0)
                        {
                            //Parallel and outside the hull face; hull face intersection interval does not exist.
                            earliestExitCA = float.MinValue;
                            latestEntryCA = float.MaxValue;
                        }
                    }
                }

                //We now have triangle edge intervals. Add contacts for them.
                latestEntryAB = MathF.Max(latestEntryAB, 0);
                latestEntryBC = MathF.Max(latestEntryBC, 0);
                latestEntryCA = MathF.Max(latestEntryCA, 0);
                earliestExitAB = MathF.Min(earliestExitAB, 1);
                earliestExitBC = MathF.Min(earliestExitBC, 1);
                earliestExitCA = MathF.Min(earliestExitCA, 1);
                //Create max contact if max >= min.
                //Create min if min < max and min > 0.  
                if (earliestExitAB >= latestEntryAB && candidateCount < maximumContactCount)
                {
                    //Create max contact.
                    var point = slotTriangleAB * earliestExitAB; //Note triangle A is origin for surface basis.
                    var newContactIndex = candidateCount++;
                    ref var candidate = ref candidates[newContactIndex];
                    candidate.X = Vector3.Dot(point, slotTriangleTangentX);
                    candidate.Y = Vector3.Dot(point, slotTriangleTangentY);
                    candidate.FeatureId = 0;

                }
                if (latestEntryAB < earliestExitAB && latestEntryAB > 0 && candidateCount < 6)
                {
                    //Create min contact.
                    var point = slotTriangleAB * latestEntryAB; //Note triangle A is origin for surface basis.
                    var newContactIndex = candidateCount++;
                    ref var candidate = ref candidates[newContactIndex];
                    candidate.X = Vector3.Dot(point, slotTriangleTangentX);
                    candidate.Y = Vector3.Dot(point, slotTriangleTangentY);
                    candidate.FeatureId = 1;
                }
                if (earliestExitBC >= latestEntryBC && candidateCount < maximumContactCount)
                {
                    //Create max contact.
                    var point = slotTriangleBC * earliestExitBC + slotTriangleAB;
                    var newContactIndex = candidateCount++;
                    ref var candidate = ref candidates[newContactIndex];
                    candidate.X = Vector3.Dot(point, slotTriangleTangentX);
                    candidate.Y = Vector3.Dot(point, slotTriangleTangentY);
                    candidate.FeatureId = 2;

                }
                if (latestEntryBC < earliestExitBC && latestEntryBC > 0 && candidateCount < 6)
                {
                    //Create min contact.
                    var point = slotTriangleBC * latestEntryBC + slotTriangleAB;
                    var newContactIndex = candidateCount++;
                    ref var candidate = ref candidates[newContactIndex];
                    candidate.X = Vector3.Dot(point, slotTriangleTangentX);
                    candidate.Y = Vector3.Dot(point, slotTriangleTangentY);
                    candidate.FeatureId = 3;
                }
                if (earliestExitCA >= latestEntryCA && candidateCount < maximumContactCount)
                {
                    //Create max contact.
                    var point = slotTriangleCA * earliestExitCA - slotTriangleCA;
                    var newContactIndex = candidateCount++;
                    ref var candidate = ref candidates[newContactIndex];
                    candidate.X = Vector3.Dot(point, slotTriangleTangentX);
                    candidate.Y = Vector3.Dot(point, slotTriangleTangentY);
                    candidate.FeatureId = 4;

                }
                if (latestEntryCA < earliestExitCA && latestEntryCA > 0 && candidateCount < 6)
                {
                    //Create min contact.
                    var point = slotTriangleCA * latestEntryCA - slotTriangleCA;
                    var newContactIndex = candidateCount++;
                    ref var candidate = ref candidates[newContactIndex];
                    candidate.X = Vector3.Dot(point, slotTriangleTangentX);
                    candidate.Y = Vector3.Dot(point, slotTriangleTangentY);
                    candidate.FeatureId = 5;
                }

                //We have found all contacts for this hull slot. There may be more contacts than we want (4), so perform a reduction.
                //Note the potential use of an effective normal means the triangle face representative is chosen as the closest point.
                Vector3Wide.ReadSlot(ref offsetB, slotIndex, out var slotOffsetB);
                Matrix3x3Wide.ReadSlot(ref hullOrientation, slotIndex, out var slotHullOrientation);
                ManifoldCandidateHelper.Reduce(candidates, candidateCount, slotFaceNormal, -1f / Vector3.Dot(slotFaceNormal, slotLocalNormal), previousVertex, slotTriangleA, slotTriangleTangentX, slotTriangleTangentY, epsilonScale[slotIndex], depthThreshold[slotIndex],
                   slotHullOrientation, slotOffsetB, slotIndex, ref manifold);
            }

            //The reduction does not assign the normal. Fill it in.
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, hullOrientation, out manifold.Normal);
            //Mesh reductions also make use of a face contact flag in the feature id.
            var faceCollisionFlag = Vector.ConditionalSelect(
                Vector.LessThan(triangleNormalDotLocalNormal, new Vector<float>(-MeshReduction.MinimumDotForFaceCollision)), new Vector<int>(MeshReduction.FaceCollisionFlag), Vector<int>.Zero);
            manifold.FeatureId0 += faceCollisionFlag;
        }

        public void Test(ref TriangleWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref TriangleWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
