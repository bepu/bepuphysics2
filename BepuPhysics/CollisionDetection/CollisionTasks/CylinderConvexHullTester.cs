using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CylinderConvexHullTester : IPairTester<CylinderWide, ConvexHullWide, Convex4ContactManifoldWide>
    {
        public int BatchSize => 16;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ProjectOntoCap(Vector3 capCenter, in Matrix3x3 cylinderOrientation, float inverseLocalNormalDotAY, Vector3 localNormal, Vector3 point, out Vector2 projected)
        {
            var pointToCapCenter = capCenter - point;
            var t = Vector3.Dot(pointToCapCenter, cylinderOrientation.Y) * inverseLocalNormalDotAY;
            var projectionOffsetB = localNormal * t;
            var projectedPoint = point - projectionOffsetB;
            var capCenterToProjectedPoint = projectedPoint - capCenter;
            projected = new Vector2(
                Vector3.Dot(capCenterToProjectedPoint, cylinderOrientation.X),
                Vector3.Dot(capCenterToProjectedPoint, cylinderOrientation.Z));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool IntersectLineCircle(in Vector2 linePosition, in Vector2 lineDirection, float radius, out float tMin, out float tMax)
        {
            //||linePosition + lineDirection * t|| = radius
            //dot(linePosition + lineDirection * t, linePosition + lineDirection * t) = radius * radius
            //dot(linePosition, linePosition) - radius * radius + t * 2 * dot(linePosition, lineDirection) + t^2 * dot(lineDirection, lineDirection) = 0
            var a = Vector2.Dot(lineDirection, lineDirection);
            var inverseA = 1f / a;
            var b = Vector2.Dot(linePosition, lineDirection);
            var c = Vector2.Dot(linePosition, linePosition);
            var radiusSquared = radius * radius;
            c -= radiusSquared;
            var d = b * b - a * c;
            if (d < 0)
            {
                tMin = 0;
                tMax = 0;
                return false;
            }
            var tOffset = (float)Math.Sqrt(d) * inverseA;
            var tBase = -b * inverseA;
            if (a < 1e-12f && a > -1e-12f)
            {
                //If the projected line direction is zero, just compress the interval to tBase.
                tMin = tBase;
                tMax = tBase;
            }
            else
            {
                tMin = tBase - tOffset;
                tMax = tBase + tOffset;
            }
            if (tMin < 0)
                tMin = 0;
            if (tMax > 1)
                tMax = 1;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void InsertContact(in Vector3 slotSideEdgeCenter, in Vector3 slotCylinderEdgeAxis, float t,
            in Vector3 hullFaceOrigin, in Vector3 slotHullFaceNormal, float inverseDepthDenominator,
            in Matrix3x3 slotHullOrientation, in Vector3 slotOffsetB, int featureId,
            ref Vector3Wide contactOffsetAWide, ref Vector<float> contactDepthWide, ref Vector<int> contactFeatureIdWide, ref Vector<int> contactExistsWide)
        {
            //Create max contact.
            var localPoint = slotSideEdgeCenter + slotCylinderEdgeAxis * t;
            //depth = dot(faceCenterB - pointOnFaceA, faceNormalB) / dot(faceNormalB, normal)
            var contactDepth = Vector3.Dot(hullFaceOrigin - localPoint, slotHullFaceNormal) * inverseDepthDenominator;
            Matrix3x3.Transform(localPoint, slotHullOrientation, out var contactOffsetA);
            contactOffsetA += slotOffsetB;
            Vector3Wide.WriteFirst(contactOffsetA, ref contactOffsetAWide);
            GatherScatter.GetFirst(ref contactDepthWide) = contactDepth;
            GatherScatter.GetFirst(ref contactFeatureIdWide) = featureId;
            GatherScatter.GetFirst(ref contactExistsWide) = -1;
        }

        public unsafe void Test(ref CylinderWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            Unsafe.SkipInit(out manifold);
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var cylinderOrientation);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var hullOrientation);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(cylinderOrientation, hullOrientation, out var hullLocalCylinderOrientation);

            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, hullOrientation, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);
            Vector3Wide.Length(localOffsetA, out var centerDistance);
            Vector3Wide.Scale(localOffsetA, Vector<float>.One / centerDistance, out var initialNormal);
            var useInitialFallback = Vector.LessThan(centerDistance, new Vector<float>(1e-8f));
            initialNormal.X = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.X);
            initialNormal.Y = Vector.ConditionalSelect(useInitialFallback, Vector<float>.One, initialNormal.Y);
            initialNormal.Z = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.Z);
            var hullSupportFinder = default(ConvexHullSupportFinder);
            var cylinderSupportFinder = default(CylinderSupportFinder);
            var inactiveLanes = BundleIndexing.CreateTrailingMaskForCountInBundle(pairCount);
            b.EstimateEpsilonScale(inactiveLanes, out var hullEpsilonScale);
            var epsilonScale = Vector.Min(Vector.Max(a.HalfLength, a.Radius), hullEpsilonScale);
            var depthThreshold = -speculativeMargin;
            DepthRefiner<ConvexHull, ConvexHullWide, ConvexHullSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.FindMinimumDepth(
                b, a, localOffsetA, hullLocalCylinderOrientation, ref hullSupportFinder, ref cylinderSupportFinder, initialNormal, inactiveLanes, 1e-5f * epsilonScale, depthThreshold,
                out var depth, out var localNormal, out var closestOnHull);

            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.LessThan(depth, depthThreshold));
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

            //Identify the cylinder feature.
            Vector3Wide.Scale(localNormal, depth, out var closestOnCylinderOffset);
            Vector3Wide.Subtract(closestOnHull, closestOnCylinderOffset, out var closestOnCylinder);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(localNormal, hullLocalCylinderOrientation, out var localNormalInA);
            var inverseLocalNormalDotCapNormal = Vector<float>.One / localNormalInA.Y;
            var useCap = Vector.GreaterThan(Vector.Abs(localNormalInA.Y), new Vector<float>(0.70710678118f));
            Unsafe.SkipInit(out Vector3Wide capCenter);
            Unsafe.SkipInit(out Vector2Wide interior0);
            Unsafe.SkipInit(out Vector2Wide interior1);
            Unsafe.SkipInit(out Vector2Wide interior2);
            Unsafe.SkipInit(out Vector2Wide interior3);
            if (Vector.LessThanAny(Vector.AndNot(useCap, inactiveLanes), Vector<int>.Zero))
            {
                var useBottom = Vector.GreaterThan(localNormalInA.Y, Vector<float>.Zero);
                Vector3Wide.Scale(hullLocalCylinderOrientation.Y, Vector.ConditionalSelect(useBottom, -a.HalfLength, a.HalfLength), out capCenter);
                Vector3Wide.Add(capCenter, localOffsetA, out capCenter);

                Vector3Wide.Subtract(closestOnCylinder, localOffsetA, out var hullLocalCylinderToClosestOnCylinder);
                Matrix3x3Wide.TransformByTransposedWithoutOverlap(hullLocalCylinderToClosestOnCylinder, hullLocalCylinderOrientation, out var cylinderLocalCylinderToClosestOnCylinder);
                BoxCylinderTester.GenerateInteriorPoints(a, localNormalInA, cylinderLocalCylinderToClosestOnCylinder, out interior0, out interior1, out interior2, out interior3);
            }

            Unsafe.SkipInit(out Vector3Wide cylinderSideEdgeCenter);
            if (Vector.EqualsAny(Vector.BitwiseOr(useCap, inactiveLanes), Vector<int>.Zero))
            {
                //If the contact is on the cylinder's side, use the closestOnHull-derived position rather than resampling the support function with the local normal to avoid numerical noise.
                Vector3Wide.Subtract(closestOnCylinder, localOffsetA, out var cylinderToClosestOnCylinder);
                Vector3Wide.Dot(cylinderToClosestOnCylinder, hullLocalCylinderOrientation.Y, out var cylinderLocalClosestOnCylinderY);
                Vector3Wide.Scale(hullLocalCylinderOrientation.Y, cylinderLocalClosestOnCylinderY, out var cylinderEdgeCenterToClosestOnCylinder);
                Vector3Wide.Subtract(closestOnCylinder, cylinderEdgeCenterToClosestOnCylinder, out cylinderSideEdgeCenter);
            }
            int maximumCandidateCount = 4;
            for (int slotIndex = 0; slotIndex < pairCount; ++slotIndex)
            {
                //We can create up to 2 contacts per hull edge.
                //Note that this overestimates the number of candidates required (it doesn't narrow the vertices to one face), but that's fine since we're not zeroing anything.
                var slotMaximumCandidateCount = b.Hulls[slotIndex].FaceVertexIndices.Length * 2;
                if (slotMaximumCandidateCount > maximumCandidateCount)
                    maximumCandidateCount = slotMaximumCandidateCount;
            }
            var candidates = stackalloc ManifoldCandidateScalar[maximumCandidateCount];
            Helpers.FillVectorWithLaneIndices(out var slotOffsetIndices);
            var boundingPlaneEpsilon = 1e-3f * epsilonScale;
            for (int slotIndex = 0; slotIndex < pairCount; ++slotIndex)
            {
                if (inactiveLanes[slotIndex] < 0)
                    continue;
                ref var hull = ref b.Hulls[slotIndex];
                ConvexHullTestHelper.PickRepresentativeFace(ref hull, slotIndex, ref localNormal, closestOnHull, slotOffsetIndices, ref boundingPlaneEpsilon, out var slotHullFaceNormal, out var slotLocalNormal, out var bestFaceIndex);
                hull.GetVertexIndicesForFace(bestFaceIndex, out var faceVertexIndices);

                if (useCap[slotIndex] < 0)
                {
                    var candidateCount = 0;
                    //The cap is the representative feature. Clip the hull's edges against the cap's circle, and test the cylinder's heuristically chosen 'vertices' against the hull edges for containment.
                    //Note that we work on the surface of the cap and post-project back onto the hull.
                    Vector3Wide.ReadSlot(ref capCenter, slotIndex, out var slotCapCenter);
                    var slotInverseLocalNormalDotCapNormal = inverseLocalNormalDotCapNormal[slotIndex];

                    ref var interior0Slot = ref GatherScatter.GetOffsetInstance(ref interior0, slotIndex);
                    ref var interior1Slot = ref GatherScatter.GetOffsetInstance(ref interior1, slotIndex);
                    ref var interior2Slot = ref GatherScatter.GetOffsetInstance(ref interior2, slotIndex);
                    ref var interior3Slot = ref GatherScatter.GetOffsetInstance(ref interior3, slotIndex);
                    var interiorPointsX = new Vector4(interior0Slot.X[0], interior1Slot.X[0], interior2Slot.X[0], interior3Slot.X[0]);
                    var interiorPointsY = new Vector4(interior0Slot.Y[0], interior1Slot.Y[0], interior2Slot.Y[0], interior3Slot.Y[0]);
                    var slotRadius = a.Radius[slotIndex];
                    Matrix3x3Wide.ReadSlot(ref hullLocalCylinderOrientation, slotIndex, out var slotCylinderOrientation);

                    var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
                    Vector3Wide.ReadSlot(ref hull.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var hullFaceOrigin);
                    ProjectOntoCap(slotCapCenter, slotCylinderOrientation, slotInverseLocalNormalDotCapNormal, slotLocalNormal, hullFaceOrigin, out var previousVertex);
                    var maximumInteriorContainmentDots = Vector4.Zero;

                    for (int i = 0; i < faceVertexIndices.Length; ++i)
                    {
                        var index = faceVertexIndices[i];
                        Vector3Wide.ReadSlot(ref hull.Points[index.BundleIndex], index.InnerIndex, out var hullVertex);
                        ProjectOntoCap(slotCapCenter, slotCylinderOrientation, slotInverseLocalNormalDotCapNormal, slotLocalNormal, hullVertex, out var vertex);

                        //Test all the cap's interior points against this edge's plane normal (which, since we've projected the vertex, is just a perp dot product).
                        var hullEdgeOffset = vertex - previousVertex;
                        var previousStartX = new Vector4(previousVertex.X);
                        var previousStartY = new Vector4(previousVertex.Y);
                        var hullEdgeOffsetX = new Vector4(hullEdgeOffset.X);
                        var hullEdgeOffsetY = new Vector4(hullEdgeOffset.Y);
                        var interiorPointContainmentDots = (interiorPointsX - previousStartX) * hullEdgeOffsetY - (interiorPointsY - previousStartY) * hullEdgeOffsetX;
                        //If we're generating contacts with the bottom cap, then the visible winding of the hull is flipped and the containment signs will be negated.
                        if (slotInverseLocalNormalDotCapNormal > 0)
                            interiorPointContainmentDots *= -1;
                        maximumInteriorContainmentDots = Vector4.Max(interiorPointContainmentDots, maximumInteriorContainmentDots);

                        //Test the projected hull edge against the cap.
                        if (IntersectLineCircle(previousVertex, hullEdgeOffset, slotRadius, out var tMin, out var tMax))
                        {
                            //We now have a convex hull edge interval. Add contacts for it.
                            //Create max contact if max >= min.
                            //Create min if min < max and min > 0.
                            var startId = (previousIndex.BundleIndex << BundleIndexing.VectorShift) + previousIndex.InnerIndex;
                            var endId = (index.BundleIndex << BundleIndexing.VectorShift) + index.InnerIndex;
                            var baseFeatureId = (startId ^ endId) << 8;
                            if (tMax >= tMin && candidateCount < maximumCandidateCount)
                            {
                                //Create max contact.
                                var newContactIndex = candidateCount++;
                                ref var candidate = ref candidates[newContactIndex];
                                Unsafe.As<float, Vector2>(ref candidate.X) = hullEdgeOffset * tMax + previousVertex;
                                candidate.FeatureId = baseFeatureId + endId;

                            }
                            if (tMin < tMax && tMin > 0 && candidateCount < maximumCandidateCount)
                            {
                                //Create min contact.
                                var newContactIndex = candidateCount++;
                                ref var candidate = ref candidates[newContactIndex];
                                Unsafe.As<float, Vector2>(ref candidate.X) = hullEdgeOffset * tMin + previousVertex;
                                candidate.FeatureId = baseFeatureId + startId;

                            }
                        }

                        previousIndex = index;
                        previousVertex = vertex;
                    }

                    if (candidateCount < maximumCandidateCount)
                    {
                        //Try adding the cylinder 'vertex' contacts.
                        //We took the maximum of all interior-hulledgeplane tests; if a vertex is outside any edge plane, the maximum dot will be positive.
                        if (maximumInteriorContainmentDots.X <= 0)
                        {
                            ref var candidate = ref candidates[candidateCount++];
                            candidate.X = interiorPointsX.X;
                            candidate.Y = interiorPointsY.X;
                            candidate.FeatureId = 0;
                        }
                        if (candidateCount == maximumCandidateCount)
                            goto SkipVertexCandidates;
                        if (maximumInteriorContainmentDots.Y <= 0)
                        {
                            ref var candidate = ref candidates[candidateCount++];
                            candidate.X = interiorPointsX.Y;
                            candidate.Y = interiorPointsY.Y;
                            candidate.FeatureId = 1;
                        }
                        if (candidateCount == maximumCandidateCount)
                            goto SkipVertexCandidates;
                        if (maximumInteriorContainmentDots.Z <= 0)
                        {
                            ref var candidate = ref candidates[candidateCount++];
                            candidate.X = interiorPointsX.Z;
                            candidate.Y = interiorPointsY.Z;
                            candidate.FeatureId = 2;
                        }
                        if (candidateCount < maximumCandidateCount && maximumInteriorContainmentDots.W <= 0)
                        {
                            ref var candidate = ref candidates[candidateCount++];
                            candidate.X = interiorPointsX.W;
                            candidate.Y = interiorPointsY.W;
                            candidate.FeatureId = 3;
                        }
                    SkipVertexCandidates:;
                    }
                    //We have found all contacts for this hull slot. There may be more contacts than we want (4), so perform a reduction.
                    Vector3Wide.ReadSlot(ref offsetB, slotIndex, out var slotOffsetB);
                    Vector3Wide.ReadSlot(ref hullLocalCylinderOrientation.X, slotIndex, out var slotCylinderFaceX);
                    Vector3Wide.ReadSlot(ref hullLocalCylinderOrientation.Z, slotIndex, out var slotCylinderFaceY);
                    Matrix3x3Wide.ReadSlot(ref hullOrientation, slotIndex, out var slotHullOrientation);
                    //Note that we're working on the cylinder's cap, so the parameters get flipped around. Gets pushed back onto the hull in the postpass.
                    ManifoldCandidateHelper.Reduce(candidates, candidateCount, slotHullFaceNormal, -1f / Vector3.Dot(slotLocalNormal, slotHullFaceNormal), hullFaceOrigin, slotCapCenter, slotCylinderFaceX, slotCylinderFaceY, epsilonScale[slotIndex], depthThreshold[slotIndex],
                       slotHullOrientation, slotOffsetB, slotIndex, ref manifold);
                }
                else
                {
                    //The side edge is the representative feature. Clip the cylinder's side edge against the hull edges; similar to capsule-hull. 
                    Vector3Wide.ReadSlot(ref hullLocalCylinderOrientation.Y, slotIndex, out var slotCylinderEdgeAxis);
                    Vector3Wide.ReadSlot(ref cylinderSideEdgeCenter, slotIndex, out var slotSideEdgeCenter);
                    var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
                    Vector3Wide.ReadSlot(ref hull.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var hullFaceOrigin);
                    var previousVertex = hullFaceOrigin;
                    var latestEntryNumerator = float.MaxValue;
                    var latestEntryDenominator = -1f;
                    var earliestExitNumerator = float.MaxValue;
                    var earliestExitDenominator = 1f;
                    for (int i = 0; i < faceVertexIndices.Length; ++i)
                    {
                        var index = faceVertexIndices[i];
                        Vector3Wide.ReadSlot(ref hull.Points[index.BundleIndex], index.InnerIndex, out var vertex);

                        var edgeOffset = vertex - previousVertex;
                        var edgePlaneNormal = Vector3.Cross(edgeOffset, slotLocalNormal);

                        //t = dot(pointOnPlane - capsuleCenter, planeNormal) / dot(planeNormal, rayDirection)
                        //Note that we can defer the division; we don't need to compute the exact t value of *all* planes.
                        var cylinderSideToHullEdgeStart = previousVertex - slotSideEdgeCenter;
                        var numerator = Vector3.Dot(cylinderSideToHullEdgeStart, edgePlaneNormal);
                        var denominator = Vector3.Dot(edgePlaneNormal, slotCylinderEdgeAxis);
                        previousVertex = vertex;

                        //A plane is being 'entered' if the ray direction opposes the face normal.
                        //Entry denominators are always negative, exit denominators are always positive. Don't have to worry about comparison sign flips.
                        var edgePlaneNormalLengthSquared = edgePlaneNormal.LengthSquared();
                        var denominatorSquared = denominator * denominator;

                        const float min = 1e-5f;
                        const float max = 3e-4f;
                        const float inverseSpan = 1f / (max - min);
                        if (denominatorSquared > min * edgePlaneNormalLengthSquared)
                        {
                            if (denominatorSquared < max * edgePlaneNormalLengthSquared)
                            {
                                //As the angle between the axis and edge plane approaches zero, the axis should unrestrict.
                                //angle between capsule axis and edge plane normal = asin(dot(edgePlaneNormal / ||edgePlaneNormal||, capsuleAxis))
                                //sin(angle)^2 * ||edgePlaneNormal||^2 = dot(edgePlaneNormal, capsuleAxis)^2
                                var restrictWeight = (denominatorSquared / edgePlaneNormalLengthSquared - min) * inverseSpan;
                                if (restrictWeight < 0)
                                    restrictWeight = 0;
                                else if (restrictWeight > 1)
                                    restrictWeight = 1;
                                var unrestrictedNumerator = a.HalfLength[slotIndex] * denominator;
                                if (denominator < 0)
                                    unrestrictedNumerator = -unrestrictedNumerator;
                                numerator = restrictWeight * numerator + (1 - restrictWeight) * unrestrictedNumerator;
                            }
                            if (denominator < 0)
                            {
                                if (numerator * latestEntryDenominator > latestEntryNumerator * denominator)
                                {
                                    latestEntryNumerator = numerator;
                                    latestEntryDenominator = denominator;
                                }
                            }
                            else // if (denominator > 0)
                            {
                                if (numerator * earliestExitDenominator < earliestExitNumerator * denominator)
                                {
                                    earliestExitNumerator = numerator;
                                    earliestExitDenominator = denominator;
                                }
                            }
                        }
                    }
                    var slotSideEdgeHalfLength = a.HalfLength[slotIndex];
                    var latestEntry = latestEntryNumerator / latestEntryDenominator;
                    var earliestExit = earliestExitNumerator / earliestExitDenominator;
                    var inverseDepthDenominator = 1f / Vector3.Dot(slotHullFaceNormal, slotLocalNormal);
                    var negatedEdgeLength = -slotSideEdgeHalfLength;
                    if (latestEntry < negatedEdgeLength)
                        latestEntry = negatedEdgeLength;
                    if (latestEntry > slotSideEdgeHalfLength)
                        latestEntry = slotSideEdgeHalfLength;
                    if (earliestExit < negatedEdgeLength)
                        earliestExit = negatedEdgeLength;
                    if (earliestExit > slotSideEdgeHalfLength)
                        earliestExit = slotSideEdgeHalfLength;
                    Matrix3x3Wide.ReadSlot(ref hullOrientation, slotIndex, out var slotHullOrientation);
                    Vector3Wide.ReadSlot(ref offsetB, slotIndex, out var slotOffsetB);
                    ref var slotManifold = ref GatherScatter.GetOffsetInstance(ref manifold, slotIndex);
                    InsertContact(
                        slotSideEdgeCenter, slotCylinderEdgeAxis, earliestExit,
                        hullFaceOrigin, slotHullFaceNormal, inverseDepthDenominator, slotHullOrientation, slotOffsetB, 0,
                        ref slotManifold.OffsetA0, ref slotManifold.Depth0, ref slotManifold.FeatureId0, ref slotManifold.Contact0Exists);
                    if (earliestExit - latestEntry > slotSideEdgeHalfLength * 1e-3f)
                    {
                        InsertContact(
                            slotSideEdgeCenter, slotCylinderEdgeAxis, latestEntry,
                            hullFaceOrigin, slotHullFaceNormal, inverseDepthDenominator, slotHullOrientation, slotOffsetB, 1,
                            ref slotManifold.OffsetA1, ref slotManifold.Depth1, ref slotManifold.FeatureId1, ref slotManifold.Contact1Exists);
                    }
                    else
                    {
                        GatherScatter.GetFirst(ref slotManifold.Contact1Exists) = 0;
                    }
                    GatherScatter.GetFirst(ref slotManifold.Contact2Exists) = 0;
                    GatherScatter.GetFirst(ref slotManifold.Contact3Exists) = 0;
                }
            }
            //Push the manifold onto the hull. This is useful if we ever end up building a 'HullReduction' like we have for MeshReduction, consistent with the other hull-(nottriangle) pairs.
            //The reduction does not assign the normal. Fill it in.
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, hullOrientation, out manifold.Normal);
            Vector3Wide.Scale(manifold.Normal, manifold.Depth0, out var offset0);
            Vector3Wide.Scale(manifold.Normal, manifold.Depth1, out var offset1);
            Vector3Wide.Scale(manifold.Normal, manifold.Depth2, out var offset2);
            Vector3Wide.Scale(manifold.Normal, manifold.Depth3, out var offset3);
            Vector3Wide.Add(manifold.OffsetA0, offset0, out manifold.OffsetA0);
            Vector3Wide.Add(manifold.OffsetA1, offset1, out manifold.OffsetA1);
            Vector3Wide.Add(manifold.OffsetA2, offset2, out manifold.OffsetA2);
            Vector3Wide.Add(manifold.OffsetA3, offset3, out manifold.OffsetA3);
        }

        public void Test(ref CylinderWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref CylinderWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
