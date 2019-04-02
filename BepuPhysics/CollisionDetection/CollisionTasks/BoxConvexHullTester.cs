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
        public int BatchSize => 32;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Test(ref BoxWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
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
            ManifoldCandidateHelper.CreateInactiveMask(pairCount, out var inactiveLanes);
            b.EstimateEpsilonScale(inactiveLanes, out var hullEpsilonScale);
            var epsilonScale = Vector.Min(Vector.Max(a.HalfWidth, Vector.Max(a.HalfHeight, a.HalfLength)), hullEpsilonScale);
            var depthThreshold = -speculativeMargin;
            DepthRefiner<ConvexHull, ConvexHullWide, ConvexHullSupportFinder, Box, BoxWide, BoxSupportFinder>.FindMinimumDepth(
                b, a, localOffsetA, hullLocalBoxOrientation, ref hullSupportFinder, ref boxSupportFinder, initialNormal, inactiveLanes, 1e-5f * epsilonScale, depthThreshold,
                out var depth, out var localNormal, out var closestOnHull);

            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.LessThan(depth, -speculativeMargin));
            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //No contacts generated.
                manifold = default;
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
            Vector3Wide.ConditionallyNegate(negateFace, ref boxFaceX);
            Vector3Wide.ConditionallyNegate(negateFace, ref boxFaceY);
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
            //Vector3Wide.Add(v0, boxFaceYOffset, out var v01);
            //Vector3Wide.Subtract(v1, boxFaceYOffset, out var v10);
            Vector3Wide.Add(v1, boxFaceYOffset, out var v11);

            //To find the contact manifold, we'll clip the capsule axis against the face as usual, but we're dealing with potentially
            //distinct convex hulls. Rather than vectorizing over the different hulls, we vectorize within each hull.
            Helpers.FillVectorWithLaneIndices(out var slotOffsetIndices);
            Vector3Wide pointOnFaceBundle;
            var boundingPlaneEpsilon = 1e-4f * epsilonScale;
            //There can be no more than 8 contacts (provided there are no numerical errors); 2 per box face.
            var candidates = stackalloc ManifoldCandidateScalar[8];
            for (int slotIndex = 0; slotIndex < pairCount; ++slotIndex)
            {
                if (inactiveLanes[slotIndex] < 0)
                    continue;
                ref var hull = ref b.Hulls[slotIndex];
                ConvexHullTestHelper.PickRepresentativeFace(ref hull, slotIndex, ref localNormal, closestOnHull, slotOffsetIndices, ref boundingPlaneEpsilon, out var slotFaceNormal, out var slotLocalNormal, out var bestFaceIndex);

                //Test each face edge plane against the box face.
                //Note that we do not use the faceNormal x edgeOffset edge plane, but rather edgeOffset x localNormal.
                //(In other words, testing the *projected* box edge axis on the surface of the convex hull face.)
                //The faces are wound counterclockwise.
                //X is 00->10; Y is 10->11; Z is 11->01; W is 01->00.
                ref var v00Slot = ref GatherScatter.GetOffsetInstance(ref v00, slotIndex);
                ref var v11Slot = ref GatherScatter.GetOffsetInstance(ref v11, slotIndex);
                ref var slotFaceX = ref GatherScatter.GetOffsetInstance(ref boxFaceX, slotIndex);
                ref var slotFaceY = ref GatherScatter.GetOffsetInstance(ref boxFaceY, slotIndex);
                var pointOnBoxEdgeX = new Vector4(v00Slot.X[0], v11Slot.X[0], v11Slot.X[0], v00Slot.X[0]);
                var pointOnBoxEdgeY = new Vector4(v00Slot.Y[0], v11Slot.Y[0], v11Slot.Y[0], v00Slot.Y[0]);
                var pointOnBoxEdgeZ = new Vector4(v00Slot.Z[0], v11Slot.Z[0], v11Slot.Z[0], v00Slot.Z[0]);
                var edgeDirectionX = new Vector4(slotFaceX.X[0], slotFaceY.X[0], -slotFaceX.X[0], -slotFaceY.X[0]);
                var edgeDirectionY = new Vector4(slotFaceX.Y[0], slotFaceY.Y[0], -slotFaceX.Y[0], -slotFaceY.Y[0]);
                var edgeDirectionZ = new Vector4(slotFaceX.Z[0], slotFaceY.Z[0], -slotFaceX.Z[0], -slotFaceY.Z[0]);

                var slotNormalX = new Vector4(slotLocalNormal.X);
                var slotNormalY = new Vector4(slotLocalNormal.Y);
                var slotNormalZ = new Vector4(slotLocalNormal.Z);

                //edgePlaneNormal = edgeDirection x localNormal
                var edgePlaneNormalX = edgeDirectionY * slotNormalZ - edgeDirectionZ * slotNormalY;
                var edgePlaneNormalY = edgeDirectionZ * slotNormalX - edgeDirectionX * slotNormalZ;
                var edgePlaneNormalZ = edgeDirectionX * slotNormalY - edgeDirectionY * slotNormalX;

                hull.GetVertexIndicesForFace(bestFaceIndex, out var faceVertexIndices);
                var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
                Vector3Wide.ReadSlot(ref hull.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var previousVertex);
                Vector3Wide.WriteSlot(previousVertex, slotIndex, ref pointOnFaceBundle);
                var candidateCount = 0;
                Helpers.BuildOrthnormalBasis(slotFaceNormal, out var hullFaceX, out var hullFaceY);
                Vector3Wide.ReadSlot(ref closestOnHull, slotIndex, out var slotClosestOnHull);
                for (int i = 0; i < faceVertexIndices.Length; ++i)
                {
                    var index = faceVertexIndices[i];
                    Vector3Wide.ReadSlot(ref hull.Points[index.BundleIndex], index.InnerIndex, out var vertex);

                    var edgeOffset = vertex - previousVertex;
                    var hullEdgeStartX = new Vector4(previousVertex.X);
                    var hullEdgeStartY = new Vector4(previousVertex.Y);
                    var hullEdgeStartZ = new Vector4(previousVertex.Z);
                    var edgeOffsetX = new Vector4(edgeOffset.X);
                    var edgeOffsetY = new Vector4(edgeOffset.Y);
                    var edgeOffsetZ = new Vector4(edgeOffset.Z);
                    //t = dot(pointOnBoxEdge - hullEdgeStart, edgePlaneNormal) / dot(edgePlaneNormal, hullEdgeOffset)
                    var hullEdgeStartToBoxEdgeX = pointOnBoxEdgeX - hullEdgeStartX;
                    var hullEdgeStartToBoxEdgeY = pointOnBoxEdgeY - hullEdgeStartY;
                    var hullEdgeStartToBoxEdgeZ = pointOnBoxEdgeZ - hullEdgeStartZ;
                    var numerator = hullEdgeStartToBoxEdgeX * edgePlaneNormalX + hullEdgeStartToBoxEdgeY * edgePlaneNormalY + hullEdgeStartToBoxEdgeZ * edgePlaneNormalZ;
                    var denominator = edgePlaneNormalX * edgeOffsetX + edgePlaneNormalY * edgeOffsetY + edgePlaneNormalZ * edgeOffsetZ;
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

                    //We now have a convex hull edge interval. Add contacts for it.
                    latestEntry = latestEntry < 0 ? 0 : latestEntry;
                    earliestExit = earliestExit > 1 ? 1 : earliestExit;
                    //Create max contact if max >= min.
                    //Create min if min < max and min > 0.
                    if (earliestExit >= latestEntry && candidateCount < 8)
                    {
                        //Create max contact.
                        var point = edgeOffset * earliestExit + previousVertex - slotClosestOnHull;
                        var newContactIndex = candidateCount++;
                        ref var candidate = ref candidates[newContactIndex];
                        candidate.X = Vector3.Dot(point, hullFaceX);
                        candidate.Y = Vector3.Dot(point, hullFaceY);
                        candidate.FeatureId = (index.BundleIndex << BundleIndexing.VectorShift) + index.InnerIndex;

                    }
                    if (latestEntry < earliestExit && latestEntry > 0 && candidateCount < 8)
                    {
                        //Create min contact.
                        var point = edgeOffset * latestEntry + previousVertex - slotClosestOnHull;
                        var newContactIndex = candidateCount++;
                        ref var candidate = ref candidates[newContactIndex];
                        candidate.X = Vector3.Dot(point, hullFaceX);
                        candidate.Y = Vector3.Dot(point, hullFaceY);
                        candidate.FeatureId = (previousIndex.BundleIndex << BundleIndexing.VectorShift) + previousIndex.InnerIndex;

                    }

                    previousIndex = index;
                    previousVertex = vertex;
                }
                //We have found all contacts for this hull slot. There may be more contacts than we want (4), so perform a reduction.
                Vector3Wide.ReadSlot(ref boxFaceCenter, slotIndex, out var slotBoxFaceCenter);
                Vector3Wide.ReadSlot(ref boxFaceNormal, slotIndex, out var slotBoxFaceNormal);
                Vector3Wide.ReadSlot(ref offsetB, slotIndex, out var slotOffsetB);
                Matrix3x3Wide.ReadSlot(ref hullOrientation, slotIndex, out var slotHullOrientation);
                ManifoldCandidateHelper.Reduce(candidates, candidateCount, slotBoxFaceNormal, slotLocalNormal, slotBoxFaceCenter, slotClosestOnHull, hullFaceX, hullFaceY, epsilonScale[slotIndex], depthThreshold[slotIndex],
                   slotHullOrientation, slotOffsetB, slotIndex, ref manifold);
            }
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
