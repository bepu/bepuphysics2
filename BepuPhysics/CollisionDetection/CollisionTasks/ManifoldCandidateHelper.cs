using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct ManifoldCandidateScalar
    {
        public float X;
        public float Y;
        public int FeatureId;
    }
    public struct ManifoldCandidate
    {
        public Vector<float> X;
        public Vector<float> Y;
        public Vector<float> Depth;
        public Vector<int> FeatureId;
    }
    public static class ManifoldCandidateHelper
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void AddCandidate(ref ManifoldCandidate candidates, ref Vector<int> count, in ManifoldCandidate candidate, in Vector<int> newContactExists, int pairCount)
        {
            //Incrementally maintaining a list is unfortunately a very poor fit for wide vectorization.
            //Each pair has its own count, so the target memory location for storing a new contact in the list is different.
            //If we had efficient scatters, this would look something like:
            //stride = Unsafe.SizeOf<Candidate>() / Unsafe.SizeOf<float>();
            //laneIndices = new Vector<int> { 0, 1, 2, 3... }
            //targetIndices = count * stride + laneIndices;
            //Scatter(ref candidate.X, ref Unsafe.As<Vector<float>, float>(ref candidates[0].X), count)
            //Scatter(ref candidate.Y, ref Unsafe.As<Vector<float>, float>(ref candidates[0].Y), count)
            //Scatter(ref candidate.FeatureId, ref Unsafe.As<Vector<int>, float>(ref candidates[0].FeatureId), count)
            //But we don't have scatter at the moment. We have two options:
            //1) Maintain the vectorized pipeline and emulate the scatters as well as we can with scalar operations. Some risk for running into undefined behavior or compiler issues.
            //2) Immediately drop to full scalar mode, and output an array of AOS ContactManifolds from this test.
            //Note that we perform a conceptual scatter after the completion of each bundle for vectorized pairs anyway, so this wouldn't be catastrophic- 
            //we did get SOME benefit out of vectorization for all the math above. 
            //My guess is that #2 would have a slight advantage overall, but it requires more work at the API level- we'd need a version of the ExecuteBatch that could handle AOS output.
            //For now, we're going to proceed with #1. Emulate the scatters as best we can with scalar code, and maintain the vectorized API.

            for (int i = 0; i < pairCount; ++i)
            {
                if (newContactExists[i] < 0)
                {
                    var targetIndex = count[i];
                    ref var target = ref GetOffsetInstance(ref Unsafe.Add(ref candidates, targetIndex), i);
                    //TODO: Now that we're free of NS2.0, we could likely intrisify this to reduce some overhead. Still not very vectorization friendly.
                    GetFirst(ref target.X) = candidate.X[i];
                    GetFirst(ref target.Y) = candidate.Y[i];
                    GetFirst(ref target.FeatureId) = candidate.FeatureId[i];
                }
            }
            count = Vector.ConditionalSelect(newContactExists, count + Vector<int>.One, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void AddCandidateWithDepth(ref ManifoldCandidate candidates, ref Vector<int> count, in ManifoldCandidate candidate, in Vector<int> newContactExists, int pairCount)
        {
            //Similar to above, but also takes the candidate's depth. Any user of this codepath is not going to rely on the reduction postpass to calculate depths.
            for (int i = 0; i < pairCount; ++i)
            {
                if (newContactExists[i] < 0)
                {
                    var targetIndex = count[i];
                    ref var target = ref GetOffsetInstance(ref Unsafe.Add(ref candidates, targetIndex), i);
                    //TODO: Now that we're free of NS2.0, we could likely intrisify this to reduce some overhead. Still not very vectorization friendly.
                    GetFirst(ref target.X) = candidate.X[i];
                    GetFirst(ref target.Y) = candidate.Y[i];
                    GetFirst(ref target.Depth) = candidate.Depth[i];
                    GetFirst(ref target.FeatureId) = candidate.FeatureId[i];
                }
            }
            count = Vector.ConditionalSelect(newContactExists, count + Vector<int>.One, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ConditionalSelect(in Vector<int> useA, in ManifoldCandidate a, in ManifoldCandidate b, out ManifoldCandidate result)
        {
            result.X = Vector.ConditionalSelect(useA, a.X, b.X);
            result.Y = Vector.ConditionalSelect(useA, a.Y, b.Y);
            result.Depth = Vector.ConditionalSelect(useA, a.Depth, b.Depth);
            result.FeatureId = Vector.ConditionalSelect(useA, a.FeatureId, b.FeatureId);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void CandidateExists(in ManifoldCandidate candidate, in Vector<float> minimumDepth, in Vector<int> rawContactCount, int i, out Vector<int> exists)
        {
            exists = Vector.BitwiseAnd(Vector.GreaterThan(candidate.Depth, minimumDepth), Vector.LessThan(new Vector<int>(i), rawContactCount));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void SquishMaximumContactCount(Vector<int> rawContactCount, int pairCount, ref int maxCandidateCount, out Vector<int> maskedContactCount)
        {
            //See if we can avoid visiting some of the higher indices.
            //Mask out any contacts generated on the pairs which don't actually exist. They can waste time and cause problems.
            maskedContactCount = rawContactCount;
            ref var maskedBase = ref Unsafe.As<Vector<int>, int>(ref maskedContactCount);
            for (int i = pairCount; i < Vector<int>.Count; ++i)
            {
                Unsafe.Add(ref maskedBase, i) = 0;
            }
            for (int i = maxCandidateCount; i >= 0; --i)
            {
                if (Vector.EqualsAny(maskedContactCount, new Vector<int>(i)))
                {
                    maxCandidateCount = i;
                    break;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ComputeDepthsForReduction(ref int maxCandidateCount, in Vector3Wide faceNormalA, Vector<float> inverseFaceNormalADotNormal,
            in Vector3Wide faceCenterBToFaceCenterA, in Vector3Wide tangentBX, in Vector3Wide tangentBY, Vector<float> minimumDepth, Vector<int> maskedContactCount, ref ManifoldCandidate candidates)
        {
            //It's important to keep the deepest contact if there's any significant depth disparity, so we need to calculate depths before reduction.
            //Conceptually, we cast a ray from the point on face B toward the plane of face A along the contact normal:
            //depth = dot(pointOnFaceB - faceCenterA, faceNormalA) / dot(faceNormalA, normal)
            //dotAxis = faceNormalA / dot(faceNormalA, normal)
            //depth = dot(pointOnFaceB - faceCenterA, dotAxis)
            //depth = dot(faceCenterB + tangentBX * candidate.X + tangentBY * candidate.Y - faceCenterA, dotAxis)
            //depth = dot(faceCenterB - faceCenterA, dotAxis) + dot(tangentBX, dotAxis) * candidate.X + dot(tangentBY, dotAxis) * candidate.Y
            Vector3Wide.Scale(faceNormalA, inverseFaceNormalADotNormal, out var dotAxis);
            Vector3Wide.Dot(faceCenterBToFaceCenterA, dotAxis, out var negativeBaseDot);
            Vector3Wide.Dot(tangentBX, dotAxis, out var xDot);
            Vector3Wide.Dot(tangentBY, dotAxis, out var yDot);

            for (int i = 0; i < maxCandidateCount; ++i)
            {
                ref var candidate = ref Unsafe.Add(ref candidates, i);
                candidate.Depth = candidate.X * xDot + candidate.Y * yDot - negativeBaseDot;
            }

            //See if we can compress the count any due to depth-rejected candidates.
            for (int i = maxCandidateCount - 1; i >= 0; --i)
            {
                ref var candidate = ref Unsafe.Add(ref candidates, i);
                CandidateExists(candidate, minimumDepth, maskedContactCount, i, out var contactExists);
                if (Vector.EqualsAny(contactExists, new Vector<int>(-int.MaxValue)))
                {
                    maxCandidateCount = i + 1;
                    break;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void InternalReduce(ref ManifoldCandidate candidates, int maxCandidateCount,
            Vector<float> epsilonScale, Vector<float> minimumDepth, Vector<int> maskedContactCount,
            out ManifoldCandidate contact0, out ManifoldCandidate contact1, out ManifoldCandidate contact2, out ManifoldCandidate contact3,
            out Vector<int> contact0Exists, out Vector<int> contact1Exists, out Vector<int> contact2Exists, out Vector<int> contact3Exists)
        {
            Unsafe.SkipInit(out contact0);
            Unsafe.SkipInit(out contact1);
            Unsafe.SkipInit(out contact2);
            Unsafe.SkipInit(out contact3);
            if (maxCandidateCount == 0)
            {
                contact0Exists = Vector<int>.Zero;
                contact1Exists = Vector<int>.Zero;
                contact2Exists = Vector<int>.Zero;
                contact3Exists = Vector<int>.Zero;
                return;
            }

            //This early out breaks determinism. Early out and non-early out produce different results, and the choice of early out depends on the entire bundle.
            //Since the bundle is subject to nondeterminism from overlap discovery, this can't be used with determinism on.
            //For now, we'll just keep it disabled and assume the collision batcher produces deterministic results regardless.
            //The actual performance impact is fairly small (0-2% simulation time).
            //if (maxCandidateCount <= 4)
            //{
            //    //There is no need for a reduction; all lanes fit within a 4 contact manifold.
            //    //This can result in some pretty questionably redundant contacts sometimes, but our epsilons are sufficiently small that most such things
            //    //would get let through anyway. 
            //    contact0 = candidates;
            //    contact1 = Unsafe.Add(ref candidates, 1);
            //    contact2 = Unsafe.Add(ref candidates, 2);
            //    contact3 = Unsafe.Add(ref candidates, 3);
            //    CandidateExists(contact0, minimumDepth, maskedContactCount, 0, out contact0Exists);
            //    CandidateExists(contact1, minimumDepth, maskedContactCount, 1, out contact1Exists);
            //    CandidateExists(contact2, minimumDepth, maskedContactCount, 2, out contact2Exists);
            //    CandidateExists(contact3, minimumDepth, maskedContactCount, 3, out contact3Exists);
            //    return;
            //}

            //minor todo: don't really need to waste time initializing to an invalid value.
            var bestScore = new Vector<float>(-float.MaxValue);
            //While depth is the dominant heuristic, extremity is used as a bias to keep initial contact selection a little more consistent in near-equal cases.
            var extremityScale = epsilonScale * 1e-2f;
            for (int i = 0; i < maxCandidateCount; ++i)
            {
                ref var candidate = ref Unsafe.Add(ref candidates, i);
                CandidateExists(candidate, minimumDepth, maskedContactCount, i, out var candidateExists);
                //Note extremity heuristic. We want a few properties:
                //1) Somewhat resilient to collisions in common cases.
                //2) Cheap.
                //3) Incapable of resulting in a speculative contact over an active contact.
                //While conditionally using a scaled abs(candidate.X) works fine for 2 and 3, many use cases result in ties along the x axis alone.
                //X and Y added together is slightly better, but 45 degree angles are not uncommon and can result in the same problem.
                //So we just use a dot product with an arbitrary direction.
                //This is a pretty small detail, but it is cheap enough that there's no reason not to take advantage of it.
                var extremity = Vector.Abs(candidate.X * 0.7946897654f + candidate.Y * 0.60701579614f);
                var candidateScore = candidate.Depth + Vector.ConditionalSelect(Vector.GreaterThanOrEqual(candidate.Depth, Vector<float>.Zero), extremity * extremityScale, Vector<float>.Zero);
                var candidateIsHighestScore = Vector.BitwiseAnd(candidateExists, Vector.GreaterThan(candidateScore, bestScore));
                ConditionalSelect(candidateIsHighestScore, candidate, contact0, out contact0);
                bestScore = Vector.ConditionalSelect(candidateIsHighestScore, candidateScore, bestScore);
            }
            contact0Exists = Vector.GreaterThan(bestScore, new Vector<float>(-float.MaxValue));

            //Find the most distant point from the starting contact.
            var maxDistanceSquared = Vector<float>.Zero;
            for (int i = 0; i < maxCandidateCount; ++i)
            {
                ref var candidate = ref Unsafe.Add(ref candidates, i);
                var offsetX = candidate.X - contact0.X;
                var offsetY = candidate.Y - contact0.Y;
                var distanceSquared = offsetX * offsetX + offsetY * offsetY;
                ////Penalize speculative contacts; they are not as important in general.
                //distanceSquared = Vector.ConditionalSelect(Vector.LessThan(candidate.Depth, Vector<float>.Zero), 0.125f * distanceSquared, distanceSquared);
                CandidateExists(candidate, minimumDepth, maskedContactCount, i, out var candidateExists);
                var candidateIsMostDistant = Vector.BitwiseAnd(Vector.GreaterThan(distanceSquared, maxDistanceSquared), candidateExists);
                maxDistanceSquared = Vector.ConditionalSelect(candidateIsMostDistant, distanceSquared, maxDistanceSquared);
                ConditionalSelect(candidateIsMostDistant, candidate, contact1, out contact1);
            }
            //There's no point in additional contacts if the distance between the first and second candidates is zero. Note that this captures the case where there is 0 or 1 contact.
            contact1Exists = Vector.GreaterThan(maxDistanceSquared, epsilonScale * epsilonScale * new Vector<float>(1e-6f));

            //Now identify two more points. Using the two existing contacts as a starting edge, pick the points which, when considered as a triangle with the edge,
            //have the largest magnitude negative and positive signed areas.
            var edgeOffsetX = contact1.X - contact0.X;
            var edgeOffsetY = contact1.Y - contact0.Y;
            var minSignedArea = Vector<float>.Zero;
            var maxSignedArea = Vector<float>.Zero;
            for (int i = 0; i < maxCandidateCount; ++i)
            {
                //The area of a triangle is proportional to the magnitude of the cross product of two of its edge offsets. 
                //To retain sign, we will conceptually dot against the face's normal. Since we're working on the surface of face B, the arithmetic simplifies heavily to a perp-dot product.
                ref var candidate = ref Unsafe.Add(ref candidates, i);
                var candidateOffsetX = candidate.X - contact0.X;
                var candidateOffsetY = candidate.Y - contact0.Y;
                var signedArea = candidateOffsetX * edgeOffsetY - candidateOffsetY * edgeOffsetX;
                //Penalize speculative contacts; they are not as important in general.
                signedArea = Vector.ConditionalSelect(Vector.LessThan(candidate.Depth, Vector<float>.Zero), 0.25f * signedArea, signedArea);

                CandidateExists(candidate, minimumDepth, maskedContactCount, i, out var candidateExists);
                var isMinArea = Vector.BitwiseAnd(Vector.LessThan(signedArea, minSignedArea), candidateExists);
                minSignedArea = Vector.ConditionalSelect(isMinArea, signedArea, minSignedArea);
                ConditionalSelect(isMinArea, candidate, contact2, out contact2);
                var isMaxArea = Vector.BitwiseAnd(Vector.GreaterThan(signedArea, maxSignedArea), candidateExists);
                maxSignedArea = Vector.ConditionalSelect(isMaxArea, signedArea, maxSignedArea);
                ConditionalSelect(isMaxArea, candidate, contact3, out contact3);
            }

            //Area can be approximated as a box for the purposes of an epsilon test: (edgeLength0 * span)^2 = (edgeLength0 * edgeLength0 * span * span).
            //This comparison is basically checking if 'span' is irrelevantly small, so we can change this to:
            //edgeLength0 * edgeLength0 * (edgeLength0 * tinyNumber) * (edgeLength0 * tinyNumber) = (edgeLength0^2) * (edgelength0^2) * (tinyNumber^2)
            var epsilon = maxDistanceSquared * maxDistanceSquared * new Vector<float>(1e-6f);
            //Note that minSignedArea is guaranteed to be zero or lower by construction, so it's safe to square for magnitude comparison.
            //Note that these epsilons capture the case where there are two or less raw contacts.
            contact2Exists = Vector.GreaterThan(minSignedArea * minSignedArea, epsilon);
            contact3Exists = Vector.GreaterThan(maxSignedArea * maxSignedArea, epsilon);
        }

        public static void Reduce(ref ManifoldCandidate candidates, Vector<int> rawContactCount, int maxCandidateCount,
            in Vector3Wide faceNormalA, Vector<float> inverseFaceNormalDotNormal, in Vector3Wide faceCenterBToFaceCenterA, in Vector3Wide tangentBX, in Vector3Wide tangentBY,
            Vector<float> epsilonScale, Vector<float> minimumDepth, int pairCount,
            out ManifoldCandidate contact0, out ManifoldCandidate contact1, out ManifoldCandidate contact2, out ManifoldCandidate contact3,
            out Vector<int> contact0Exists, out Vector<int> contact1Exists, out Vector<int> contact2Exists, out Vector<int> contact3Exists)
        {
            SquishMaximumContactCount(rawContactCount, pairCount, ref maxCandidateCount, out var maskedContactCount);
            ComputeDepthsForReduction(ref maxCandidateCount, faceNormalA, inverseFaceNormalDotNormal, faceCenterBToFaceCenterA, tangentBX, tangentBY, minimumDepth, maskedContactCount, ref candidates);
            InternalReduce(ref candidates, maxCandidateCount, epsilonScale, minimumDepth, maskedContactCount, out contact0, out contact1, out contact2, out contact3, out contact0Exists, out contact1Exists, out contact2Exists, out contact3Exists);
        }
        public static void ReduceWithoutComputingDepths(ref ManifoldCandidate candidates, Vector<int> rawContactCount, int maxCandidateCount,
            Vector<float> epsilonScale, Vector<float> minimumDepth, int pairCount,
            out ManifoldCandidate contact0, out ManifoldCandidate contact1, out ManifoldCandidate contact2, out ManifoldCandidate contact3,
            out Vector<int> contact0Exists, out Vector<int> contact1Exists, out Vector<int> contact2Exists, out Vector<int> contact3Exists)
        {
            SquishMaximumContactCount(rawContactCount, pairCount, ref maxCandidateCount, out var maskedContactCount);
            InternalReduce(ref candidates, maxCandidateCount, epsilonScale, minimumDepth, maskedContactCount, out contact0, out contact1, out contact2, out contact3, out contact0Exists, out contact1Exists, out contact2Exists, out contact3Exists);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void PlaceCandidateInSlot(in ManifoldCandidateScalar candidate, int contactIndex,
            in Vector3 faceCenterB, in Vector3 faceBX, in Vector3 faceBY, float depth,
            in Matrix3x3 orientationB, in Vector3 offsetB, ref Convex4ContactManifoldWide manifoldSlot)
        {
            var localPosition = candidate.X * faceBX + candidate.Y * faceBY + faceCenterB;
            Matrix3x3.Transform(localPosition, orientationB, out var position);
            position += offsetB;
            Vector3Wide.WriteFirst(position, ref Unsafe.Add(ref manifoldSlot.OffsetA0, contactIndex));
            GetFirst(ref Unsafe.Add(ref manifoldSlot.Depth0, contactIndex)) = depth;
            GetFirst(ref Unsafe.Add(ref manifoldSlot.FeatureId0, contactIndex)) = candidate.FeatureId;
            GetFirst(ref Unsafe.Add(ref manifoldSlot.Contact0Exists, contactIndex)) = -1;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe void RemoveCandidateAt(ManifoldCandidateScalar* candidates, float* depths, int removalIndex, ref int candidateCount)
        {
            var lastIndex = candidateCount - 1;
            if (removalIndex < lastIndex)
            {
                candidates[removalIndex] = candidates[lastIndex];
                depths[removalIndex] = depths[lastIndex];
            }
            --candidateCount;
        }

        public unsafe static void Reduce(ManifoldCandidateScalar* candidates, int candidateCount,
            in Vector3 faceNormalA, float inverseFaceNormalADotLocalNormal, in Vector3 faceCenterA, in Vector3 faceCenterB, in Vector3 tangentBX, in Vector3 tangentBY,
            float epsilonScale, float minimumDepth, in Matrix3x3 rotationToWorld, in Vector3 worldOffsetB, int slotIndex, ref Convex4ContactManifoldWide manifoldWide)
        {
            if (candidateCount == 0)
            {
                //No contacts, no work. Note that we assume the user has set all ContactExists to 0 for all slots.
                return;
            }
            //Note that this does NOT assign the world normal in the manifold.
            ref var manifoldSlot = ref GetOffsetInstance(ref manifoldWide, slotIndex);

            //Calculate the depths of all candidates.
            //It's important to keep the deepest contact if there's any significant depth disparity, so we need to calculate depths before reduction.
            //Conceptually, we cast a ray from the point on face B toward the plane of face A along the contact normal:
            //depth = dot(pointOnFaceB - faceCenterA, faceNormalA) / dot(faceNormalA, normal)
            //dotAxis = faceNormalA / dot(faceNormalA, normal)
            //depth = dot(pointOnFaceB - faceCenterA, dotAxis)
            //depth = dot(faceCenterB + tangentBX * candidate.X + tangentBY * candidate.Y - faceCenterA, dotAxis)
            //depth = dot(faceCenterB - faceCenterA, dotAxis) + dot(tangentBX, dotAxis) * candidate.X + dot(tangentBY, dotAxis) * candidate.Y
            var dotAxis = faceNormalA * inverseFaceNormalADotLocalNormal;
            var faceCenterAToFaceCenterB = faceCenterB - faceCenterA;
            var baseDot = Vector3.Dot(faceCenterAToFaceCenterB, dotAxis);
            var xDot = Vector3.Dot(tangentBX, dotAxis);
            var yDot = Vector3.Dot(tangentBY, dotAxis);
            var candidateDepths = stackalloc float[candidateCount];
            for (int i = candidateCount - 1; i >= 0; --i)
            {
                ref var candidate = ref candidates[i];
                ref var candidateDepth = ref candidateDepths[i];
                candidateDepth = baseDot + candidate.X * xDot + candidate.Y * yDot;
                //Prune out contacts below the depth threshold.
                if (candidateDepth < minimumDepth)
                {
                    RemoveCandidateAt(candidates, candidateDepths, i, ref candidateCount);
                }
            }
            if (candidateCount <= 4)
            {
                //No reduction is necessary; just place the contacts into the manifold.
                for (int i = 0; i < candidateCount; ++i)
                {
                    PlaceCandidateInSlot(candidates[i], i, faceCenterB, tangentBX, tangentBY, candidateDepths[i], rotationToWorld, worldOffsetB, ref manifoldSlot);
                }
                return;
            }

            //minor todo: don't really need to waste time initializing to an invalid value.
            var bestScore0 = float.MinValue;
            var bestIndex0 = 0;
            //While depth is the dominant heuristic, extremity is used as a bias to keep initial contact selection a little more consistent in near-equal cases.
            var extremityScale = epsilonScale * 1e-2f;
            var extremityX = 0.7946897654f * extremityScale;
            var extremityY = 0.60701579614f * extremityScale;
            for (int i = 0; i < candidateCount; ++i)
            {
                ref var candidate = ref candidates[i];
                ref var candidateDepth = ref candidateDepths[i];
                float candidateScore = candidateDepth;
                if (candidateDepth >= 0)
                {
                    //Note extremity heuristic. We want a few properties:
                    //1) Somewhat resilient to collisions in common cases.
                    //2) Cheap.
                    //3) Incapable of resulting in a speculative contact over an active contact.
                    //While conditionally using a scaled abs(candidate.X) works fine for 2 and 3, many use cases result in ties along the x axis alone.
                    //X and Y added together is slightly better, but 45 degree angles are not uncommon and can result in the same problem.
                    //So we just use a dot product with an arbitrary direction.
                    //This is a pretty small detail, but it is cheap enough that there's no reason not to take advantage of it.
                    var extremity = candidate.X * extremityX + candidate.Y * extremityY;
                    if (extremity < 0)
                        extremity = -extremity;
                    candidateScore += extremity;
                }
                if (candidateScore > bestScore0)
                {
                    bestScore0 = candidateScore;
                    bestIndex0 = i;
                }
            }
            var candidate0 = candidates[bestIndex0];
            var depth0 = candidateDepths[bestIndex0];
            PlaceCandidateInSlot(candidate0, 0, faceCenterB, tangentBX, tangentBY, depth0, rotationToWorld, worldOffsetB, ref manifoldSlot);
            RemoveCandidateAt(candidates, candidateDepths, bestIndex0, ref candidateCount);

            //Find the most distant point from the starting contact.
            var maximumDistanceSquared = -1f;
            var bestIndex1 = 0;
            for (int i = 0; i < candidateCount; ++i)
            {
                ref var candidate = ref candidates[i];
                var offsetX = candidate.X - candidate0.X;
                var offsetY = candidate.Y - candidate0.Y;
                var distanceSquared = offsetX * offsetX + offsetY * offsetY;
                ////Penalize speculative contacts; they are not as important in general.
                //distanceSquared = candidateDepths[i] < 0 ? 0.125f * distanceSquared : distanceSquared;
                if (distanceSquared > maximumDistanceSquared)
                {
                    maximumDistanceSquared = distanceSquared;
                    bestIndex1 = i;
                }
            }
            if (maximumDistanceSquared < 1e-6f * epsilonScale * epsilonScale)
            {
                //There's no point in additional contacts if the distance between the first and second candidates is zero. 
                return;
            }
            var candidate1 = candidates[bestIndex1];
            var depth1 = candidateDepths[bestIndex1];
            PlaceCandidateInSlot(candidate1, 1, faceCenterB, tangentBX, tangentBY, depth1, rotationToWorld, worldOffsetB, ref manifoldSlot);
            RemoveCandidateAt(candidates, candidateDepths, bestIndex1, ref candidateCount);


            //Now identify two more points. Using the two existing contacts as a starting edge, pick the points which, when considered as a triangle with the edge,
            //have the largest magnitude negative and positive signed areas.
            var edgeOffsetX = candidate1.X - candidate0.X;
            var edgeOffsetY = candidate1.Y - candidate0.Y;
            var minSignedArea = 0f;
            var maxSignedArea = 0f;
            var bestIndex2 = 0;
            var bestIndex3 = 0;
            for (int i = 0; i < candidateCount; ++i)
            {
                //if (i == bestIndex0 || i == bestIndex1)
                //    continue;
                //The area of a triangle is proportional to the magnitude of the cross product of two of its edge offsets. 
                //To retain sign, we will conceptually dot against the face's normal. Since we're working on the surface of face B, the arithmetic simplifies heavily to a perp-dot product.
                ref var candidate = ref candidates[i];
                var candidateOffsetX = candidate.X - candidate0.X;
                var candidateOffsetY = candidate.Y - candidate0.Y;
                var signedArea = candidateOffsetX * edgeOffsetY - candidateOffsetY * edgeOffsetX;
                //Penalize speculative contacts; they are not as important in general.
                if (candidateDepths[i] < 0)
                    signedArea *= 0.25f;

                if (signedArea < minSignedArea)
                {
                    minSignedArea = signedArea;
                    bestIndex2 = i;
                }
                if (signedArea > maxSignedArea)
                {
                    maxSignedArea = signedArea;
                    bestIndex3 = i;
                }
            }

            //Area can be approximated as a box for the purposes of an epsilon test: (edgeLength0 * span)^2 = (edgeLength0 * edgeLength0 * span * span).
            //This comparison is basically checking if 'span' is irrelevantly small, so we can change this to:
            //edgeLength0 * edgeLength0 * (edgeLength0 * tinyNumber) * (edgeLength0 * tinyNumber) = (edgeLength0^2) * (edgelength0^2) * (tinyNumber^2)
            var areaEpsilon = maximumDistanceSquared * maximumDistanceSquared * 1e-6f;
            if (minSignedArea * minSignedArea > areaEpsilon)
            {
                PlaceCandidateInSlot(candidates[bestIndex2], 2, faceCenterB, tangentBX, tangentBY, candidateDepths[bestIndex2], rotationToWorld, worldOffsetB, ref manifoldSlot);
            }
            if (maxSignedArea * maxSignedArea > areaEpsilon)
            {
                PlaceCandidateInSlot(candidates[bestIndex3], 3, faceCenterB, tangentBX, tangentBY, candidateDepths[bestIndex3], rotationToWorld, worldOffsetB, ref manifoldSlot);
            }
        }
    }
}
