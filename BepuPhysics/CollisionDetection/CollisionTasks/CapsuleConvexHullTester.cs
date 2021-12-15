using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CapsuleConvexHullTester : IPairTester<CapsuleWide, ConvexHullWide, Convex2ContactManifoldWide>
    {
        public int BatchSize => 16;

        public void Test(ref CapsuleWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var capsuleOrientation);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var hullOrientation);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(capsuleOrientation, hullOrientation, out var hullLocalCapsuleOrientation);
            ref var localCapsuleAxis = ref hullLocalCapsuleOrientation.Y;

            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, hullOrientation, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);
            Vector3Wide.Length(localOffsetA, out var centerDistance);
            Vector3Wide.Scale(localOffsetA, Vector<float>.One / centerDistance, out var initialNormal);
            var useInitialFallback = Vector.LessThan(centerDistance, new Vector<float>(1e-8f));
            initialNormal.X = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.X);
            initialNormal.Y = Vector.ConditionalSelect(useInitialFallback, Vector<float>.One, initialNormal.Y);
            initialNormal.Z = Vector.ConditionalSelect(useInitialFallback, Vector<float>.Zero, initialNormal.Z);
            var hullSupportFinder = default(ConvexHullSupportFinder);
            var capsuleSupportFinder = default(CapsuleSupportFinder);
            var inactiveLanes = BundleIndexing.CreateTrailingMaskForCountInBundle(pairCount);
            b.EstimateEpsilonScale(inactiveLanes, out var hullEpsilonScale);
            var epsilonScale = Vector.Min(a.Radius, hullEpsilonScale);
            var depthThreshold = -speculativeMargin;
            DepthRefiner<ConvexHull, ConvexHullWide, ConvexHullSupportFinder, Capsule, CapsuleWide, CapsuleSupportFinder>.FindMinimumDepth(
                b, a, localOffsetA, hullLocalCapsuleOrientation, ref hullSupportFinder, ref capsuleSupportFinder, initialNormal, inactiveLanes, 1e-5f * epsilonScale, depthThreshold,
                out var depth, out var localNormal, out var closestOnHull);

            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.LessThan(depth, depthThreshold));
            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //No contacts generated.
                manifold = default;
                return;
            }

            //To find the contact manifold, we'll clip the capsule axis against the face as usual, but we're dealing with potentially
            //distinct convex hulls. Rather than vectorizing over the different hulls, we vectorize within each hull.
            Helpers.FillVectorWithLaneIndices(out var slotOffsetIndices);
            Unsafe.SkipInit(out Vector3Wide faceNormalBundle);
            var boundingPlaneEpsilon = 1e-3f * epsilonScale;
            Unsafe.SkipInit(out Vector<float> latestEntryNumeratorBundle);
            Unsafe.SkipInit(out Vector<float> latestEntryDenominatorBundle);
            Unsafe.SkipInit(out Vector<float> earliestExitNumeratorBundle);
            Unsafe.SkipInit(out Vector<float> earliestExitDenominatorBundle);
            for (int slotIndex = 0; slotIndex < pairCount; ++slotIndex)
            {
                if (inactiveLanes[slotIndex] < 0)
                    continue;
                ref var hull = ref b.Hulls[slotIndex];
                ConvexHullTestHelper.PickRepresentativeFace(ref hull, slotIndex, ref localNormal, closestOnHull, slotOffsetIndices, ref boundingPlaneEpsilon, out var slotFaceNormal, out var slotLocalNormal, out var bestFaceIndex);
                Vector3Wide.WriteSlot(slotFaceNormal, slotIndex, ref faceNormalBundle);

                //Test each face edge plane against the capsule edge.
                //Note that we do not use the faceNormal x edgeOffset edge plane, but rather edgeOffset x localNormal.
                //(In other words, testing the *projected* capsule axis on the surface of the convex hull face.)
                //The faces are wound counterclockwise in right handed coordinates.
                hull.GetVertexIndicesForFace(bestFaceIndex, out var faceVertexIndices);
                var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
                Vector3Wide.ReadSlot(ref hull.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var previousVertex);
                Vector3Wide.ReadSlot(ref localCapsuleAxis, slotIndex, out var slotCapsuleAxis);
                Vector3Wide.ReadSlot(ref localOffsetA, slotIndex, out var slotLocalOffsetA);
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
                    var capsuleToEdge = previousVertex - slotLocalOffsetA;
                    var numerator = Vector3.Dot(capsuleToEdge, edgePlaneNormal);
                    var denominator = Vector3.Dot(edgePlaneNormal, slotCapsuleAxis);
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

                GatherScatter.Get(ref latestEntryNumeratorBundle, slotIndex) = latestEntryNumerator;
                GatherScatter.Get(ref latestEntryDenominatorBundle, slotIndex) = latestEntryDenominator;
                GatherScatter.Get(ref earliestExitNumeratorBundle, slotIndex) = earliestExitNumerator;
                GatherScatter.Get(ref earliestExitDenominatorBundle, slotIndex) = earliestExitDenominator;
            }

            var tEntry = latestEntryNumeratorBundle / latestEntryDenominatorBundle;
            var tExit = earliestExitNumeratorBundle / earliestExitDenominatorBundle;
            var negatedHalfLength = -a.HalfLength;
            tEntry = Vector.Max(negatedHalfLength, Vector.Min(a.HalfLength, tEntry));
            tExit = Vector.Max(negatedHalfLength, Vector.Min(a.HalfLength, tExit));

            Vector3Wide.Scale(localCapsuleAxis, tEntry, out var localOffset0);
            Vector3Wide.Scale(localCapsuleAxis, tExit, out var localOffset1);

            //Compute the depth of each contact based on the projection along the contact normal to the face.
            //depth = dot(contactRelativeToA - pointOnFaceB, faceNormalB) / dot(faceNormalB, normal)
            Vector3Wide.Add(localOffsetB, closestOnHull, out var aToPointOnHullFace);

            Vector3Wide.Dot(faceNormalBundle, localNormal, out var depthDenominator);
            var inverseDepthDenominator = Vector<float>.One / depthDenominator;
            Vector3Wide.Subtract(aToPointOnHullFace, localOffset0, out var contact0ToHullFace);
            Vector3Wide.Subtract(aToPointOnHullFace, localOffset1, out var contact1ToHullFace);
            Vector3Wide.Dot(contact0ToHullFace, faceNormalBundle, out var depthNumerator0);
            Vector3Wide.Dot(contact1ToHullFace, faceNormalBundle, out var depthNumerator1);
            var unexpandedDepth0 = depthNumerator0 * inverseDepthDenominator;
            var unexpandedDepth1 = depthNumerator1 * inverseDepthDenominator;
            manifold.Depth0 = a.Radius + unexpandedDepth0;
            manifold.Depth1 = a.Radius + unexpandedDepth1;
            manifold.FeatureId0 = Vector<int>.Zero;
            manifold.FeatureId1 = Vector<int>.One;
            manifold.Contact0Exists = Vector.AndNot(Vector.GreaterThanOrEqual(manifold.Depth0, depthThreshold), inactiveLanes);
            manifold.Contact1Exists = Vector.AndNot(Vector.BitwiseAnd(Vector.GreaterThan(tExit - tEntry, a.HalfLength * 1e-3f), Vector.GreaterThanOrEqual(manifold.Depth1, depthThreshold)), inactiveLanes);

            Matrix3x3Wide.TransformWithoutOverlap(localOffset0, hullOrientation, out manifold.OffsetA0);
            Matrix3x3Wide.TransformWithoutOverlap(localOffset1, hullOrientation, out manifold.OffsetA1);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, hullOrientation, out manifold.Normal);
            //Push the contacts out to be on the surface of the convex hull.
            //(Capsule vs hull surface doesn't really matter, but we choose hull because we may later build a system similar to MeshReduction that relies on contacts being on the hull surface.)
            Vector3Wide.Scale(manifold.Normal, unexpandedDepth0, out var contactOffset0);
            Vector3Wide.Scale(manifold.Normal, unexpandedDepth1, out var contactOffset1);
            Vector3Wide.Add(manifold.OffsetA0, contactOffset0, out manifold.OffsetA0);
            Vector3Wide.Add(manifold.OffsetA1, contactOffset1, out manifold.OffsetA1);

        }

        public void Test(ref CapsuleWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref CapsuleWide a, ref ConvexHullWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
