using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct BoxConvexHullTester : IPairTester<BoxWide, ConvexHullWide, Convex4ContactManifoldWide>
    {
        public int BatchSize => 16;

        public unsafe void Test(ref BoxWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            Unsafe.SkipInit(out manifold);
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var boxOrientation);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var hullOrientation);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(boxOrientation, hullOrientation, out var hullLocalBoxOrientation);

            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, hullOrientation, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);
            Vector3Wide.Length(localOffsetA, out var centerDistance);
            Vector3Wide.Scale(localOffsetA, Vector<float>.One / centerDistance, out var initialNormal);
            var useInitialFallback = Vector.LessThan(centerDistance, new Vector<float>(1e-8f));
            initialNormal.X = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.X);
            initialNormal.Y = Vector.ConditionalSelect(useInitialFallback, Vector<float>.One, initialNormal.Y);
            initialNormal.Z = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.Z);
            var hullSupportFinder = default(ConvexHullSupportFinder);
            var boxSupportFinder = default(BoxSupportFinder);
            var inactiveLanes = BundleIndexing.CreateTrailingMaskForCountInBundle(pairCount);
            b.EstimateEpsilonScale(inactiveLanes, out var hullEpsilonScale);
            var epsilonScale = Vector.Min(Vector.Max(a.HalfWidth, Vector.Max(a.HalfHeight, a.HalfLength)), hullEpsilonScale);
            var depthThreshold = -speculativeMargin;
            DepthRefiner<ConvexHull, ConvexHullWide, ConvexHullSupportFinder, Box, BoxWide, BoxSupportFinder>.FindMinimumDepth(
                b, a, localOffsetA, hullLocalBoxOrientation, ref hullSupportFinder, ref boxSupportFinder, initialNormal, inactiveLanes, 1e-5f * epsilonScale, depthThreshold,
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


            //Identify the box face.
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(localNormal, hullLocalBoxOrientation, out var localNormalInA);
            Vector3Wide.Abs(localNormalInA, out var absLocalNormalInA);
            var useX = Vector.BitwiseAnd(Vector.GreaterThan(absLocalNormalInA.X, absLocalNormalInA.Y), Vector.GreaterThan(absLocalNormalInA.X, absLocalNormalInA.Z));
            var useY = Vector.AndNot(Vector.GreaterThan(absLocalNormalInA.Y, absLocalNormalInA.Z), useX);
            Vector3Wide.ConditionalSelect(useX, hullLocalBoxOrientation.X, hullLocalBoxOrientation.Z, out var boxFaceNormal);
            Vector3Wide.ConditionalSelect(useY, hullLocalBoxOrientation.Y, boxFaceNormal, out boxFaceNormal);
            Vector3Wide.ConditionalSelect(useX, hullLocalBoxOrientation.Y, hullLocalBoxOrientation.X, out var boxFaceX);
            Vector3Wide.ConditionalSelect(useY, hullLocalBoxOrientation.Z, boxFaceX, out boxFaceX);
            Vector3Wide.ConditionalSelect(useX, hullLocalBoxOrientation.Z, hullLocalBoxOrientation.Y, out var boxFaceY);
            Vector3Wide.ConditionalSelect(useY, hullLocalBoxOrientation.X, boxFaceY, out boxFaceY);
            var negateFace =
                Vector.ConditionalSelect(useX, Vector.GreaterThan(localNormalInA.X, Vector<float>.Zero),
                Vector.ConditionalSelect(useY, Vector.GreaterThan(localNormalInA.Y, Vector<float>.Zero), Vector.GreaterThan(localNormalInA.Z, Vector<float>.Zero)));
            Vector3Wide.ConditionallyNegate(negateFace, ref boxFaceNormal);
            //Winding is important; flip the face bases if necessary.
            Vector3Wide.ConditionallyNegate(Vector.OnesComplement(negateFace), ref boxFaceX);
            var boxFaceHalfWidth = Vector.ConditionalSelect(useX, a.HalfHeight, Vector.ConditionalSelect(useY, a.HalfLength, a.HalfWidth));
            var boxFaceHalfHeight = Vector.ConditionalSelect(useX, a.HalfLength, Vector.ConditionalSelect(useY, a.HalfWidth, a.HalfHeight));
            var boxFaceNormalOffset = Vector.ConditionalSelect(useX, a.HalfWidth, Vector.ConditionalSelect(useY, a.HalfHeight, a.HalfLength));
            Vector3Wide.Scale(boxFaceNormal, boxFaceNormalOffset, out var boxFaceCenterOffset);
            Vector3Wide.Add(boxFaceCenterOffset, localOffsetA, out var boxFaceCenter);
            Vector3Wide.Scale(boxFaceX, boxFaceHalfWidth, out var boxFaceXOffset);
            Vector3Wide.Scale(boxFaceY, boxFaceHalfHeight, out var boxFaceYOffset);
            Vector3Wide.Subtract(boxFaceCenter, boxFaceXOffset, out var v0);
            Vector3Wide.Add(boxFaceCenter, boxFaceXOffset, out var v1);
            Vector3Wide.Subtract(v0, boxFaceYOffset, out var v00);
            Vector3Wide.Add(v0, boxFaceYOffset, out var v01);
            Vector3Wide.Subtract(v1, boxFaceYOffset, out var v10);
            Vector3Wide.Add(v1, boxFaceYOffset, out var v11);

            //To find the contact manifold, we'll clip the box edges against the hull face as usual, but we're dealing with potentially
            //distinct convex hulls. Rather than vectorizing over the different hulls, we vectorize within each hull.
            Helpers.FillVectorWithLaneIndices(out var slotOffsetIndices);
            var boundingPlaneEpsilon = 1e-3f * epsilonScale;
            //There can be no more than 8 contacts (provided there are no numerical errors); 2 per box edge.
            var candidates = stackalloc ManifoldCandidateScalar[8];
            for (int slotIndex = 0; slotIndex < pairCount; ++slotIndex)
            {
                if (inactiveLanes[slotIndex] < 0)
                    continue;
                ref var hull = ref b.Hulls[slotIndex];
                ConvexHullTestHelper.PickRepresentativeFace(ref hull, slotIndex, ref localNormal, closestOnHull, slotOffsetIndices, ref boundingPlaneEpsilon, out var slotFaceNormal, out var slotLocalNormal, out var bestFaceIndex);

                //Test each face edge plane against the box face.
                //Note that we do not use the faceNormal x edgeOffset edge plane, but rather edgeOffset x localNormal.
                //The faces are wound counterclockwise in right handed coordinates.
                //X is 00->10; Y is 10->11; Z is 11->01; W is 01->00.
                ref var v00Slot = ref GatherScatter.GetOffsetInstance(ref v00, slotIndex);
                ref var v10Slot = ref GatherScatter.GetOffsetInstance(ref v10, slotIndex);
                ref var v11Slot = ref GatherScatter.GetOffsetInstance(ref v11, slotIndex);
                ref var v01Slot = ref GatherScatter.GetOffsetInstance(ref v01, slotIndex);
                ref var slotFaceX = ref GatherScatter.GetOffsetInstance(ref boxFaceX, slotIndex);
                ref var slotFaceY = ref GatherScatter.GetOffsetInstance(ref boxFaceY, slotIndex);
                var boxEdgeStartX = new Vector4(v00Slot.X[0], v10Slot.X[0], v11Slot.X[0], v01Slot.X[0]);
                var boxEdgeStartY = new Vector4(v00Slot.Y[0], v10Slot.Y[0], v11Slot.Y[0], v01Slot.Y[0]);
                var boxEdgeStartZ = new Vector4(v00Slot.Z[0], v10Slot.Z[0], v11Slot.Z[0], v01Slot.Z[0]);
                var edgeDirectionX = new Vector4(slotFaceX.X[0], slotFaceY.X[0], -slotFaceX.X[0], -slotFaceY.X[0]);
                var edgeDirectionY = new Vector4(slotFaceX.Y[0], slotFaceY.Y[0], -slotFaceX.Y[0], -slotFaceY.Y[0]);
                var edgeDirectionZ = new Vector4(slotFaceX.Z[0], slotFaceY.Z[0], -slotFaceX.Z[0], -slotFaceY.Z[0]);

                var slotLocalNormalX = new Vector4(slotLocalNormal.X);
                var slotLocalNormalY = new Vector4(slotLocalNormal.Y);
                var slotLocalNormalZ = new Vector4(slotLocalNormal.Z);

                //edgePlaneNormal = edgeDirection x localNormal
                var edgePlaneNormalX = edgeDirectionY * slotLocalNormalZ - edgeDirectionZ * slotLocalNormalY;
                var edgePlaneNormalY = edgeDirectionZ * slotLocalNormalX - edgeDirectionX * slotLocalNormalZ;
                var edgePlaneNormalZ = edgeDirectionX * slotLocalNormalY - edgeDirectionY * slotLocalNormalX;

                hull.GetVertexIndicesForFace(bestFaceIndex, out var faceVertexIndices);
                var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
                Vector3Wide.ReadSlot(ref hull.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var hullFaceOrigin);
                var previousVertex = hullFaceOrigin;
                var candidateCount = 0;
                Helpers.BuildOrthonormalBasis(slotFaceNormal, out var hullFaceX, out var hullFaceY);
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
                    //Containment of a box vertex is tested by checking the sign of the box vertex against the hull's edge plane normal.
                    //Hull edges wound counterclockwise in right handed coordinates; edge plane normal points outward.
                    //vertexOutsideEdgePlane = dot(hullEdgeOffset x slotLocalNormal, boxVertex - hullEdgeStart) > 0
                    var hullEdgePlaneNormal = Vector3.Cross(hullEdgeOffset, slotLocalNormal);
                    var hullEdgePlaneNormalX = new Vector4(hullEdgePlaneNormal.X);
                    var hullEdgePlaneNormalY = new Vector4(hullEdgePlaneNormal.Y);
                    var hullEdgePlaneNormalZ = new Vector4(hullEdgePlaneNormal.Z);
                    var hullEdgeStartToBoxEdgeX = boxEdgeStartX - hullEdgeStartX;
                    var hullEdgeStartToBoxEdgeY = boxEdgeStartY - hullEdgeStartY;
                    var hullEdgeStartToBoxEdgeZ = boxEdgeStartZ - hullEdgeStartZ;
                    var boxVertexContainmentDots = hullEdgePlaneNormalX * hullEdgeStartToBoxEdgeX + hullEdgePlaneNormalY * hullEdgeStartToBoxEdgeY + hullEdgePlaneNormalZ * hullEdgeStartToBoxEdgeZ;
                    maximumVertexContainmentDots = Vector4.Max(maximumVertexContainmentDots, boxVertexContainmentDots);
                    //t = dot(pointOnBoxEdge - hullEdgeStart, edgePlaneNormal) / dot(edgePlaneNormal, hullEdgeOffset)
                    var numerator = hullEdgeStartToBoxEdgeX * edgePlaneNormalX + hullEdgeStartToBoxEdgeY * edgePlaneNormalY + hullEdgeStartToBoxEdgeZ * edgePlaneNormalZ;
                    //Since we're sensitive to the sign of the denominator, the winding of the box edges matters.
                    var denominator = edgePlaneNormalX * hullEdgeOffsetX + edgePlaneNormalY * hullEdgeOffsetY + edgePlaneNormalZ * hullEdgeOffsetZ;
                    var edgeIntersections = numerator / denominator;

                    //A plane is being 'entered' if the ray direction opposes the face normal.
                    //Entry denominators are always negative, exit denominators are always positive. Don't have to worry about comparison sign flips.
                    //TODO: Now that we're off NS2.0, this branchmess could be improved.
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
                    else if (numerator.X < 0)
                    {
                        //The B edge is parallel and outside the edge A, so there can be no intersection.
                        earliestExit = float.MinValue;
                        latestEntry = float.MaxValue;
                    }
                    else
                    {
                        //Parallel, but inside.
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
                    else if (numerator.Y < 0)
                    {
                        earliestExit = float.MinValue;
                        latestEntry = float.MaxValue;
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
                    else if (numerator.Z < 0)
                    {
                        earliestExit = float.MinValue;
                        latestEntry = float.MaxValue;
                    }
                    if (denominator.W < 0)
                    {
                        if (edgeIntersections.W > latestEntry)
                            latestEntry = edgeIntersections.W;
                    }
                    else if (denominator.W > 0)
                    {
                        if (edgeIntersections.W < earliestExit)
                            earliestExit = edgeIntersections.W;
                    }
                    else if (numerator.W < 0)
                    {
                        earliestExit = float.MinValue;
                        latestEntry = float.MaxValue;
                    }

                    //We now have a convex hull edge interval. Add contacts for it.
                    latestEntry = latestEntry < 0 ? 0 : latestEntry;
                    earliestExit = earliestExit > 1 ? 1 : earliestExit;
                    //Create max contact if max >= min.
                    //Create min if min < max and min > 0.
                    var startId = (previousIndex.BundleIndex << BundleIndexing.VectorShift) + previousIndex.InnerIndex;
                    var endId = (index.BundleIndex << BundleIndexing.VectorShift) + index.InnerIndex;
                    var baseFeatureId = (startId ^ endId) << 8;
                    if (earliestExit >= latestEntry && candidateCount < 8)
                    {
                        //Create max contact.
                        var point = hullEdgeOffset * earliestExit + previousVertex - hullFaceOrigin;
                        var newContactIndex = candidateCount++;
                        ref var candidate = ref candidates[newContactIndex];
                        candidate.X = Vector3.Dot(point, hullFaceX);
                        candidate.Y = Vector3.Dot(point, hullFaceY);
                        candidate.FeatureId = baseFeatureId + endId;

                    }
                    if (latestEntry < earliestExit && latestEntry > 0 && candidateCount < 8)
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
                if (candidateCount < 8)
                {
                    //Try adding the box vertex contacts. Project each vertex onto the hull face.
                    //t = dot(boxVertex - hullFaceVertex, hullFacePlaneNormal) / dot(hullFacePlaneNormal, localNormal) 
                    var hullFaceOriginX = new Vector4(hullFaceOrigin.X);
                    var hullFaceOriginY = new Vector4(hullFaceOrigin.Y);
                    var hullFaceOriginZ = new Vector4(hullFaceOrigin.Z);
                    var hullFaceNormalX = new Vector4(slotFaceNormal.X);
                    var hullFaceNormalY = new Vector4(slotFaceNormal.Y);
                    var hullFaceNormalZ = new Vector4(slotFaceNormal.Z);
                    var closestOnHullToBoxEdgeStartX = boxEdgeStartX - hullFaceOriginX;
                    var closestOnHullToBoxEdgeStartY = boxEdgeStartY - hullFaceOriginY;
                    var closestOnHullToBoxEdgeStartZ = boxEdgeStartZ - hullFaceOriginZ;
                    var vertexProjectionNumerator = (closestOnHullToBoxEdgeStartX) * hullFaceNormalX + (closestOnHullToBoxEdgeStartY) * hullFaceNormalY + (closestOnHullToBoxEdgeStartZ) * hullFaceNormalZ;
                    var vertexProjectionDenominator = new Vector4(Vector3.Dot(slotFaceNormal, slotLocalNormal));
                    var vertexProjectionT = vertexProjectionNumerator / vertexProjectionDenominator;
                    //Normal points from B to A.
                    var projectedVertexX = closestOnHullToBoxEdgeStartX - vertexProjectionT * slotLocalNormalX;
                    var projectedVertexY = closestOnHullToBoxEdgeStartY - vertexProjectionT * slotLocalNormalY;
                    var projectedVertexZ = closestOnHullToBoxEdgeStartZ - vertexProjectionT * slotLocalNormalZ;
                    var hullFaceXX = new Vector4(hullFaceX.X);
                    var hullFaceXY = new Vector4(hullFaceX.Y);
                    var hullFaceXZ = new Vector4(hullFaceX.Z);
                    var hullFaceYX = new Vector4(hullFaceY.X);
                    var hullFaceYY = new Vector4(hullFaceY.Y);
                    var hullFaceYZ = new Vector4(hullFaceY.Z);
                    var projectedTangentX = projectedVertexX * hullFaceXX + projectedVertexY * hullFaceXY + projectedVertexZ * hullFaceXZ;
                    var projectedTangentY = projectedVertexX * hullFaceYX + projectedVertexY * hullFaceYY + projectedVertexZ * hullFaceYZ;
                    //We took the maximum of all boxvertex-hulledgeplane tests; if a vertex is outside any edge plane, the maximum dot will be positive.
                    if (maximumVertexContainmentDots.X <= 0)
                    {
                        ref var candidate = ref candidates[candidateCount++];
                        candidate.X = projectedTangentX.X;
                        candidate.Y = projectedTangentY.X;
                        candidate.FeatureId = 0;
                    }
                    if (candidateCount == 8)
                        goto SkipVertexCandidates;
                    if (maximumVertexContainmentDots.Y <= 0)
                    {
                        ref var candidate = ref candidates[candidateCount++];
                        candidate.X = projectedTangentX.Y;
                        candidate.Y = projectedTangentY.Y;
                        candidate.FeatureId = 1;
                    }
                    if (candidateCount == 8)
                        goto SkipVertexCandidates;
                    if (maximumVertexContainmentDots.Z <= 0)
                    {
                        ref var candidate = ref candidates[candidateCount++];
                        candidate.X = projectedTangentX.Z;
                        candidate.Y = projectedTangentY.Z;
                        candidate.FeatureId = 2;
                    }
                    if (candidateCount < 8 && maximumVertexContainmentDots.W <= 0)
                    {
                        ref var candidate = ref candidates[candidateCount++];
                        candidate.X = projectedTangentX.W;
                        candidate.Y = projectedTangentY.W;
                        candidate.FeatureId = 3;
                    }
                SkipVertexCandidates:;
                }
                //We have found all contacts for this hull slot. There may be more contacts than we want (4), so perform a reduction.
                Vector3Wide.ReadSlot(ref boxFaceCenter, slotIndex, out var slotBoxFaceCenter);
                Vector3Wide.ReadSlot(ref boxFaceNormal, slotIndex, out var slotBoxFaceNormal);
                Vector3Wide.ReadSlot(ref offsetB, slotIndex, out var slotOffsetB);
                Matrix3x3Wide.ReadSlot(ref hullOrientation, slotIndex, out var slotHullOrientation);
                ManifoldCandidateHelper.Reduce(candidates, candidateCount, slotBoxFaceNormal, 1f / Vector3.Dot(slotBoxFaceNormal, slotLocalNormal), slotBoxFaceCenter, hullFaceOrigin, hullFaceX, hullFaceY, epsilonScale[slotIndex], depthThreshold[slotIndex],
                   slotHullOrientation, slotOffsetB, slotIndex, ref manifold);
            }
            //The reduction does not assign the normal. Fill it in.
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, hullOrientation, out manifold.Normal);
        }

        public void Test(ref BoxWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref BoxWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
