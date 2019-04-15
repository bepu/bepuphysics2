using BepuPhysics.Collidables;
using BepuUtilities;
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
            Vector<int> hullInsideAndBelowTriangle;
            Vector3Wide.CrossWithoutOverlap(triangleAB, triangleNormal, out var edgePlaneAB);
            Vector3Wide.CrossWithoutOverlap(triangleBC, triangleNormal, out var edgePlaneBC);
            Vector3Wide.CrossWithoutOverlap(triangleCA, triangleNormal, out var edgePlaneCA);
            if (Vector.LessThanAny(hullBelowPlane, Vector<int>.Zero))
            {
                //Is the hull position within the triangle bounds?
                Vector3Wide.Dot(edgePlaneAB, triangleA, out var abPlaneTest);
                Vector3Wide.Dot(edgePlaneBC, triangleB, out var bcPlaneTest);
                Vector3Wide.Dot(edgePlaneCA, triangleC, out var caPlaneTest);
                hullInsideAndBelowTriangle = Vector.BitwiseAnd(
                    Vector.BitwiseAnd(hullBelowPlane, Vector.LessThanOrEqual(abPlaneTest, Vector<float>.Zero)),
                    Vector.BitwiseAnd(Vector.LessThanOrEqual(bcPlaneTest, Vector<float>.Zero), Vector.LessThanOrEqual(caPlaneTest, Vector<float>.Zero)));
            }
            else
            {
                hullInsideAndBelowTriangle = Vector<int>.Zero;
            }

            ManifoldCandidateHelper.CreateInactiveMask(pairCount, out var inactiveLanes);
            a.EstimateEpsilonScale(out var triangleEpsilonScale);
            b.EstimateEpsilonScale(inactiveLanes, out var hullEpsilonScale);
            var epsilonScale = Vector.Min(triangleEpsilonScale, hullEpsilonScale);
            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.LessThan(triangleNormalLength, epsilonScale * 1e-6f));
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
            Vector<int> triangleNormalIsMinimal;
            var hullSupportFinder = default(ConvexHullSupportFinder);
            DepthRefiner.SimplexWithWitness simplex;
            var triangleSupportFinder = default(PretransformedTriangleSupportFinder);
            //Create a simplex entry for the direction from the hull center to triangle center.
            DepthRefiner.FindSupport(b, triangle, localTriangleCenter, hullLocalTriangleOrientation, ref hullSupportFinder, ref triangleSupportFinder, initialNormal, inactiveLanes, out simplex.A.Support, out simplex.A.SupportOnA);
            Vector3Wide.Dot(simplex.A.Support, initialNormal, out var depth);
            simplex.A.Exists = Vector.OnesComplement(inactiveLanes);
            //Create a simplex entry for the triangle face normal.
            Vector3Wide.Negate(triangleNormal, out var negatedTriangleNormal);
            hullSupportFinder.ComputeLocalSupport(b, negatedTriangleNormal, inactiveLanes, out var hullSupportAlongTriangleNormal);
            simplex.B.SupportOnA = hullSupportAlongTriangleNormal;
            Vector3Wide.Subtract(simplex.B.SupportOnA, localTriangleCenter, out simplex.B.Support);
            Vector3Wide.Dot(simplex.B.Support, negatedTriangleNormal, out var triangleFaceDepth);
            var useTriangleFace = Vector.LessThan(triangleFaceDepth, depth);
            Vector3Wide.ConditionalSelect(useTriangleFace, negatedTriangleNormal, initialNormal, out initialNormal);
            depth = Vector.ConditionalSelect(useTriangleFace, triangleFaceDepth, depth);
            simplex.B.Exists = simplex.A.Exists;
            simplex.C.Exists = default;

            //Check if the extreme point on the hull is contained within the bounds of the triangle face. If it is, there is no need for a full depth refinement.
            Vector3Wide.Subtract(triangleA, hullSupportAlongTriangleNormal, out var closestToA);
            Vector3Wide.Subtract(triangleB, hullSupportAlongTriangleNormal, out var closestToB);
            Vector3Wide.Subtract(triangleC, hullSupportAlongTriangleNormal, out var closestToC);
            Vector3Wide.Dot(edgePlaneAB, closestToA, out var extremeABPlaneTest);
            Vector3Wide.Dot(edgePlaneBC, closestToB, out var extremeBCPlaneTest);
            Vector3Wide.Dot(edgePlaneCA, closestToC, out var extremeCAPlaneTest);
            triangleNormalIsMinimal = Vector.BitwiseAnd(Vector.LessThanOrEqual(extremeABPlaneTest, Vector<float>.Zero), Vector.BitwiseAnd(Vector.LessThanOrEqual(extremeBCPlaneTest, Vector<float>.Zero), Vector.LessThanOrEqual(extremeCAPlaneTest, Vector<float>.Zero)));

            var depthThreshold = -speculativeMargin;
            var skipDepthRefine = Vector.BitwiseOr(triangleNormalIsMinimal, inactiveLanes);
            Vector3Wide localNormal, closestOnHull;
            if (Vector.EqualsAny(skipDepthRefine, Vector<int>.Zero))
            {
                DepthRefiner.FindMinimumDepth(
                    b, triangle, localTriangleCenter, hullLocalTriangleOrientation, ref hullSupportFinder, ref triangleSupportFinder, ref simplex, initialNormal, depth, skipDepthRefine, 1e-5f * epsilonScale, depthThreshold,
                    out var refinedDepth, out var refinedNormal, out var refinedClosestOnHull);
                Vector3Wide.ConditionalSelect(skipDepthRefine, hullSupportAlongTriangleNormal, refinedClosestOnHull, out closestOnHull);
                Vector3Wide.ConditionalSelect(skipDepthRefine, initialNormal, refinedNormal, out localNormal);
                depth = Vector.ConditionalSelect(skipDepthRefine, depth, refinedDepth);
            }
            else
            {
                //No depth refine ran; the extreme point prepass did everything we needed. Just use the initial normal.
                localNormal = initialNormal;
                closestOnHull = hullSupportAlongTriangleNormal;
            }

            Vector3Wide.Dot(triangleNormal, localNormal, out var triangleNormalDotLocalNormal);
            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.BitwiseOr(Vector.GreaterThanOrEqual(triangleNormalDotLocalNormal, Vector<float>.Zero), Vector.LessThan(depth, depthThreshold)));
            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //No contacts generated.
                return;
            }

            //To find the contact manifold, we'll clip the triangle edges against the hull face as usual, but we're dealing with potentially
            //distinct convex hulls. Rather than vectorizing over the different hulls, we vectorize within each hull.
            Helpers.FillVectorWithLaneIndices(out var slotOffsetIndices);
            var boundingPlaneEpsilon = 1e-3f * epsilonScale;
            //There can be no more than 6 contacts (provided there are no numerical errors); 2 per triangle edge.
            var candidates = stackalloc ManifoldCandidateScalar[6];
            for (int slotIndex = 0; slotIndex < pairCount; ++slotIndex)
            {
                if (inactiveLanes[slotIndex] < 0)
                    continue;
                ref var hull = ref b.Hulls[slotIndex];
                ConvexHullTestHelper.PickRepresentativeFace(ref hull, slotIndex, ref localNormal, closestOnHull, slotOffsetIndices, ref boundingPlaneEpsilon, out var slotFaceNormal, out var slotLocalNormal, out var bestFaceIndex);

                //Test each triangle edge against the hull face.
                //Note that we do not use the faceNormal x edgeOffset edge plane, but rather edgeOffset x localNormal.
                //The faces are wound counterclockwise.
                //Note that the triangle edges are packed into a Vector4. Historically, there were some minor codegen issues with Vector3.
                //May not matter anymore, but it costs ~nothing to use a dead slot.
                ref var aSlot = ref GatherScatter.GetOffsetInstance(ref triangleA, slotIndex);
                ref var bSlot = ref GatherScatter.GetOffsetInstance(ref triangleB, slotIndex);
                ref var cSlot = ref GatherScatter.GetOffsetInstance(ref triangleC, slotIndex);
                ref var abSlot = ref GatherScatter.GetOffsetInstance(ref triangleAB, slotIndex);
                ref var bcSlot = ref GatherScatter.GetOffsetInstance(ref triangleBC, slotIndex);
                ref var caSlot = ref GatherScatter.GetOffsetInstance(ref triangleCA, slotIndex);
                var triangleEdgeStartX = new Vector4(aSlot.X[0], bSlot.X[0], cSlot.X[0], 0);
                var triangleEdgeStartY = new Vector4(aSlot.Y[0], bSlot.Y[0], cSlot.Y[0], 0);
                var triangleEdgeStartZ = new Vector4(aSlot.Z[0], bSlot.Z[0], cSlot.Z[0], 0);
                var edgeDirectionX = new Vector4(abSlot.X[0], bcSlot.X[0], caSlot.X[0], 0);
                var edgeDirectionY = new Vector4(abSlot.Y[0], bcSlot.Y[0], caSlot.Y[0], 0);
                var edgeDirectionZ = new Vector4(abSlot.Z[0], bcSlot.Z[0], caSlot.Z[0], 0);

                var slotLocalNormalX = new Vector4(slotLocalNormal.X);
                var slotLocalNormalY = new Vector4(slotLocalNormal.Y);
                var slotLocalNormalZ = new Vector4(slotLocalNormal.Z);

                //edgePlaneNormal = edgeDirection x localNormal
                var triangleEdgePlaneNormalX = edgeDirectionY * slotLocalNormalZ - edgeDirectionZ * slotLocalNormalY;
                var triangleEdgePlaneNormalY = edgeDirectionZ * slotLocalNormalX - edgeDirectionX * slotLocalNormalZ;
                var triangleEdgePlaneNormalZ = edgeDirectionX * slotLocalNormalY - edgeDirectionY * slotLocalNormalX;

                hull.GetVertexIndicesForFace(bestFaceIndex, out var faceVertexIndices);
                var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
                Vector3Wide.ReadSlot(ref hull.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var hullFaceOrigin);
                var previousVertex = hullFaceOrigin;
                var candidateCount = 0;
                Helpers.BuildOrthnormalBasis(slotFaceNormal, out var hullFaceX, out var hullFaceY);
                Vector4 maximumVertexContainmentDots = Vector4.Zero;
                for (int i = 0; i < faceVertexIndices.Length; ++i)
                {
                    var index = faceVertexIndices[i];
                    Vector3Wide.ReadSlot(ref hull.Points[index.BundleIndex], index.InnerIndex, out var vertex);

                    var hullEdgeOffset = vertex - previousVertex;

                    var hullEdgeStartX = new Vector4(previousVertex.X);
                    var hullEdgeStartY = new Vector4(previousVertex.Y);
                    var hullEdgeStartZ = new Vector4(previousVertex.Z);
                    var hullEdgeOffsetX = new Vector4(hullEdgeOffset.X);
                    var hullEdgeOffsetY = new Vector4(hullEdgeOffset.Y);
                    var hullEdgeOffsetZ = new Vector4(hullEdgeOffset.Z);
                    //Containment of a triangle vertex is tested by checking the sign of the triangle vertex against the hull's edge plane normal.
                    //Hull edges wound counterclockwise; edge plane normal points outward.
                    //vertexOutsideEdgePlane = dot(hullEdgeOffset x slotLocalNormal, triangleVertex - hullEdgeStart) > 0
                    var hullEdgePlaneNormal = Vector3.Cross(hullEdgeOffset, slotLocalNormal);
                    var hullEdgePlaneNormalX = new Vector4(hullEdgePlaneNormal.X);
                    var hullEdgePlaneNormalY = new Vector4(hullEdgePlaneNormal.Y);
                    var hullEdgePlaneNormalZ = new Vector4(hullEdgePlaneNormal.Z);
                    var hullEdgeStartToTriangleEdgeX = triangleEdgeStartX - hullEdgeStartX;
                    var hullEdgeStartToTriangleEdgeY = triangleEdgeStartY - hullEdgeStartY;
                    var hullEdgeStartToTriangleEdgeZ = triangleEdgeStartZ - hullEdgeStartZ;
                    var triangleVertexContainmentDots = hullEdgePlaneNormalX * hullEdgeStartToTriangleEdgeX + hullEdgePlaneNormalY * hullEdgeStartToTriangleEdgeY + hullEdgePlaneNormalZ * hullEdgeStartToTriangleEdgeZ;
                    maximumVertexContainmentDots = Vector4.Max(maximumVertexContainmentDots, triangleVertexContainmentDots);
                    //t = dot(pointOnTriangleEdge - hullEdgeStart, edgePlaneNormal) / dot(edgePlaneNormal, hullEdgeOffset)
                    var numerator = hullEdgeStartToTriangleEdgeX * triangleEdgePlaneNormalX + hullEdgeStartToTriangleEdgeY * triangleEdgePlaneNormalY + hullEdgeStartToTriangleEdgeZ * triangleEdgePlaneNormalZ;
                    //Since we're sensitive to the sign of the denominator, the winding of the triangle edges matters.
                    var denominator = triangleEdgePlaneNormalX * hullEdgeOffsetX + triangleEdgePlaneNormalY * hullEdgeOffsetY + triangleEdgePlaneNormalZ * hullEdgeOffsetZ;
                    var edgeIntersections = numerator / denominator;


                    //A plane is being 'entered' if the ray direction opposes the face normal.
                    //Entry denominators are always negative, exit denominators are always positive. Don't have to worry about comparison sign flips.
                    float latestEntry, earliestExit;
                    if (denominator.X < 0)
                    {
                        latestEntry = edgeIntersections.X;
                        earliestExit = float.MaxValue;
                    }
                    else if (denominator.X > 0)
                    {
                        latestEntry = float.MinValue;
                        earliestExit = edgeIntersections.X;
                    }
                    else
                    {
                        latestEntry = float.MinValue;
                        earliestExit = float.MaxValue;
                    }
                    if (denominator.Y < 0)
                    {
                        if (edgeIntersections.Y > latestEntry)
                            latestEntry = edgeIntersections.Y;
                    }
                    else if (denominator.Y > 0)
                    {
                        if (edgeIntersections.Y < earliestExit)
                            earliestExit = edgeIntersections.Y;
                    }
                    if (denominator.Z < 0)
                    {
                        if (edgeIntersections.Z > latestEntry)
                            latestEntry = edgeIntersections.Z;
                    }
                    else if (denominator.Z > 0)
                    {
                        if (edgeIntersections.Z < earliestExit)
                            earliestExit = edgeIntersections.Z;
                    }

                    //We now have a convex hull edge interval. Add contacts for it.
                    latestEntry = latestEntry < 0 ? 0 : latestEntry;
                    earliestExit = earliestExit > 1 ? 1 : earliestExit;
                    //Create max contact if max >= min.
                    //Create min if min < max and min > 0.  
                    var startId = (previousIndex.BundleIndex << BundleIndexing.VectorShift) + previousIndex.InnerIndex;
                    var endId = (index.BundleIndex << BundleIndexing.VectorShift) + index.InnerIndex;
                    var baseFeatureId = (startId ^ endId) << 8;
                    if (earliestExit >= latestEntry && candidateCount < 6)
                    {
                        //Create max contact.
                        var point = hullEdgeOffset * earliestExit + previousVertex - hullFaceOrigin;
                        var newContactIndex = candidateCount++;
                        ref var candidate = ref candidates[newContactIndex];
                        candidate.X = Vector3.Dot(point, hullFaceX);
                        candidate.Y = Vector3.Dot(point, hullFaceY);
                        candidate.FeatureId = baseFeatureId + endId;

                    }
                    if (latestEntry < earliestExit && latestEntry > 0 && candidateCount < 6)
                    {
                        //Create min contact.
                        var point = hullEdgeOffset * latestEntry + previousVertex - hullFaceOrigin;
                        var newContactIndex = candidateCount++;
                        ref var candidate = ref candidates[newContactIndex];
                        candidate.X = Vector3.Dot(point, hullFaceX);
                        candidate.Y = Vector3.Dot(point, hullFaceY);
                        candidate.FeatureId = baseFeatureId + startId;

                    }

                    previousIndex = index;
                    previousVertex = vertex;
                }
                if (candidateCount < 6)
                {
                    //Try adding the triangle vertex contacts. Project each vertex onto the hull face.
                    //t = dot(triangleVertex - hullFaceVertex, hullFacePlaneNormal) / dot(hullFacePlaneNormal, localNormal) 
                    var closestOnHullX = new Vector4(hullFaceOrigin.X);
                    var closestOnHullY = new Vector4(hullFaceOrigin.Y);
                    var closestOnHullZ = new Vector4(hullFaceOrigin.Z);
                    var hullFaceNormalX = new Vector4(slotFaceNormal.X);
                    var hullFaceNormalY = new Vector4(slotFaceNormal.Y);
                    var hullFaceNormalZ = new Vector4(slotFaceNormal.Z);
                    var closestOnHullToTriangleEdgeStartX = triangleEdgeStartX - closestOnHullX;
                    var closestOnHullToTriangleEdgeStartY = triangleEdgeStartY - closestOnHullY;
                    var closestOnHullToTriangleEdgeStartZ = triangleEdgeStartZ - closestOnHullZ;
                    var vertexProjectionNumerator = (closestOnHullToTriangleEdgeStartX) * hullFaceNormalX + (closestOnHullToTriangleEdgeStartY) * hullFaceNormalY + (closestOnHullToTriangleEdgeStartZ) * hullFaceNormalZ;
                    var vertexProjectionDenominator = new Vector4(Vector3.Dot(slotFaceNormal, slotLocalNormal));
                    var vertexProjectionT = vertexProjectionNumerator / vertexProjectionDenominator;
                    //Normal points from B to A.
                    var projectedVertexX = closestOnHullToTriangleEdgeStartX - vertexProjectionT * slotLocalNormalX;
                    var projectedVertexY = closestOnHullToTriangleEdgeStartY - vertexProjectionT * slotLocalNormalY;
                    var projectedVertexZ = closestOnHullToTriangleEdgeStartZ - vertexProjectionT * slotLocalNormalZ;
                    var hullFaceXX = new Vector4(hullFaceX.X);
                    var hullFaceXY = new Vector4(hullFaceX.Y);
                    var hullFaceXZ = new Vector4(hullFaceX.Z);
                    var hullFaceYX = new Vector4(hullFaceY.X);
                    var hullFaceYY = new Vector4(hullFaceY.Y);
                    var hullFaceYZ = new Vector4(hullFaceY.Z);
                    var projectedTangentX = projectedVertexX * hullFaceXX + projectedVertexY * hullFaceXY + projectedVertexZ * hullFaceXZ;
                    var projectedTangentY = projectedVertexX * hullFaceYX + projectedVertexY * hullFaceYY + projectedVertexZ * hullFaceYZ;
                    //We took the maximum of all trianglevertex-hulledgeplane tests; if a vertex is outside any edge plane, the maximum dot will be positive.
                    if (maximumVertexContainmentDots.X <= 0)
                    {
                        ref var candidate = ref candidates[candidateCount++];
                        candidate.X = projectedTangentX.X;
                        candidate.Y = projectedTangentY.X;
                        candidate.FeatureId = 0;
                    }
                    if (candidateCount == 6)
                        goto SkipVertexCandidates;
                    if (maximumVertexContainmentDots.Y <= 0)
                    {
                        ref var candidate = ref candidates[candidateCount++];
                        candidate.X = projectedTangentX.Y;
                        candidate.Y = projectedTangentY.Y;
                        candidate.FeatureId = 1;
                    }
                    if (candidateCount < 6 && maximumVertexContainmentDots.Z <= 0)
                    {
                        ref var candidate = ref candidates[candidateCount++];
                        candidate.X = projectedTangentX.Z;
                        candidate.Y = projectedTangentY.Z;
                        candidate.FeatureId = 2;
                    }
                SkipVertexCandidates:;
                }
                //We have found all contacts for this hull slot. There may be more contacts than we want (4), so perform a reduction.
                Vector3Wide.ReadSlot(ref localTriangleCenter, slotIndex, out var slotTriangleCenter);
                Vector3Wide.ReadSlot(ref triangleNormal, slotIndex, out var slotTriangleFaceNormal);
                Vector3Wide.ReadSlot(ref offsetB, slotIndex, out var slotOffsetB);
                Matrix3x3Wide.ReadSlot(ref hullOrientation, slotIndex, out var slotHullOrientation);
                ManifoldCandidateHelper.Reduce(candidates, candidateCount, slotTriangleFaceNormal, slotLocalNormal, slotTriangleCenter, hullFaceOrigin, hullFaceX, hullFaceY, epsilonScale[slotIndex], depthThreshold[slotIndex],
                   slotHullOrientation, slotOffsetB, slotIndex, ref manifold);
            }
            //Push contacts to the triangle for the sake of MeshReduction. 
            //This means that future hull collection boundary smoothers that assume contacts on the hull won't work properly with triangles, but that's fine- triangles should be exclusively used for static content anyway.
            //(We did this rather than clip the triangle's edges against hull planes because hulls have variable vertex counts; projecting hull vertices to the triangle would create more reduction overhead.)
            //The reduction does not assign the normal. Fill it in.
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, hullOrientation, out manifold.Normal);
            Vector3Wide.Scale(manifold.Normal, manifold.Depth0, out var offset0);
            Vector3Wide.Scale(manifold.Normal, manifold.Depth1, out var offset1);
            Vector3Wide.Scale(manifold.Normal, manifold.Depth2, out var offset2);
            Vector3Wide.Scale(manifold.Normal, manifold.Depth3, out var offset3);
            Vector3Wide.Subtract(manifold.OffsetA0, offset0, out manifold.OffsetA0);
            Vector3Wide.Subtract(manifold.OffsetA1, offset1, out manifold.OffsetA1);
            Vector3Wide.Subtract(manifold.OffsetA2, offset2, out manifold.OffsetA2);
            Vector3Wide.Subtract(manifold.OffsetA3, offset3, out manifold.OffsetA3);
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
