using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
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
        public static void AddCandidate(ref ManifoldCandidate candidates, ref ManifoldCandidate candidate, ref Vector<int> newContactExists, ref Vector<int> count)
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

            for (int i = 0; i < Vector<int>.Count; ++i)
            {
                if (newContactExists[i] < 0)
                {
                    var targetIndex = count[i];
                    ref var target = ref GetOffsetInstance(ref Unsafe.Add(ref candidates, targetIndex), i);
                    //TODO: Check codegen. May be worth doing another offset instance for source data if the compiler inserts bounds checks.
                    GetFirst(ref target.X) = candidate.X[i];
                    GetFirst(ref target.Y) = candidate.Y[i];
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

        public static void Reduce(ref ManifoldCandidate candidates, in Vector<int> rawContactCount, int maxCandidateCount,
            in Vector3Wide faceNormalA, in Vector3Wide normal, in Vector3Wide faceCenterBToFaceCenterA, in Vector3Wide tangentBX, in Vector3Wide tangentBY,
            in Vector<float> epsilonScale,
            out ManifoldCandidate contact0, out ManifoldCandidate contact1, out ManifoldCandidate contact2, out ManifoldCandidate contact3,
            out Vector<int> contact0Exists, out Vector<int> contact1Exists, out Vector<int> contact2Exists, out Vector<int> contact3Exists)
        {
            if (Vector.LessThanOrEqualAll(rawContactCount, new Vector<int>(4)))
            {
                //There is no need for a reduction; all lanes fit within a 4 contact manifold.
                //This can result in some pretty questionably redundant contacts sometimes, but our epsilons are sufficiently small that most such things
                //would get let through anyway. 
                //TODO: Could play with this to see what the net impact on performance is. Probably negligible either way.
                contact0 = candidates;
                contact1 = Unsafe.Add(ref candidates, 1);
                contact2 = Unsafe.Add(ref candidates, 2);
                contact3 = Unsafe.Add(ref candidates, 3);
                contact0Exists = Vector.GreaterThan(rawContactCount, Vector<int>.Zero);
                contact1Exists = Vector.GreaterThan(rawContactCount, Vector<int>.One);
                contact2Exists = Vector.GreaterThan(rawContactCount, new Vector<int>(2));
                contact3Exists = Vector.GreaterThan(rawContactCount, new Vector<int>(3));
                return;
            }
            //See if we can avoid visiting some of the higher indices.
            for (int i = maxCandidateCount; i > 4; --i)
            {
                if (Vector.EqualsAny(rawContactCount, new Vector<int>(i)))
                {
                    maxCandidateCount = i;
                    break;
                }
            }

            //That's too many; four is plenty. We should choose how to get rid of the extra ones.
            //It's important to keep the deepest contact if there's any significant depth disparity, so we need to calculate depths before reduction.
            //Conceptually, we project the points from the surface of face B down onto face A, then measure the separation of those two points along the normal:
            //depth = dot(pointOnFaceB - faceCenterA, faceNormalA) / dot(faceNormalA, normal)
            //dotAxis = faceNormalA / dot(faceNormalA, normal)
            //depth = dot(pointOnFaceB - faceCenterA, dotAxis)
            //depth = dot(faceCenterB + tangentBX * candidate.X + tangentBY * candidate.Y - faceCenterA, dotAxis)
            //depth = dot(faceCenterB - faceCenterA, dotAxis) + dot(tangentBX, dotAxis) * candidate.X + dot(tangentBY, dotAxis) * candidate.Y
            Vector3Wide.Dot(faceNormalA, normal, out var axisScale);
            axisScale = Vector<float>.One / axisScale;
            Vector3Wide.Scale(faceNormalA, axisScale, out var dotAxis);
            Vector3Wide.Dot(faceCenterBToFaceCenterA, dotAxis, out var negativeBaseDot);
            Vector3Wide.Dot(tangentBX, dotAxis, out var xDot);
            Vector3Wide.Dot(tangentBY, dotAxis, out var yDot);
            //minor todo: don't really need to waste time initializing to an invalid value.
            var minDepth = new Vector<float>(float.MaxValue);
            var maxExtreme = new Vector<float>(-float.MaxValue);
            ManifoldCandidate deepest, extreme;
            deepest.Depth = new Vector<float>(-float.MaxValue);

            for (int i = 0; i < maxCandidateCount; ++i)
            {
                ref var candidate = ref Unsafe.Add(ref candidates, i);
                candidate.Depth = candidate.X * xDot + candidate.Y * yDot - negativeBaseDot;
                var index = new Vector<int>(i);
                var indexIsValid = Vector.LessThan(new Vector<int>(i), rawContactCount);
                var candidateIsDeepest = Vector.BitwiseAnd(indexIsValid, Vector.GreaterThan(candidate.Depth, deepest.Depth));
                //Note that we gather the candidate into a local variable rather than storing an index. This avoids the need for a gather (or scalar gather emulation).
                //May be worth doing deeper testing on whether that's worth it, but it'll be a minor difference regardless.
                ConditionalSelect(candidateIsDeepest, candidate, deepest, out deepest);
                //Note X+Y instead of X or Y alone. Shapes are very often stacked in a nonrandom way; choosing +x or +y alone would lead to near-ties in such cases.
                //This is a pretty small detail, but it is cheap enough that there's no reason not to take advantage of it.
                var extremeCandidate = Vector.Max(maxExtreme, candidate.X + candidate.Y);
                var candidateIsMostExtreme = Vector.BitwiseAnd(indexIsValid, Vector.GreaterThan(extremeCandidate, maxExtreme));
                ConditionalSelect(candidateIsMostExtreme, candidate, extreme, out extreme);
                minDepth = Vector.ConditionalSelect(indexIsValid, Vector.Min(candidate.Depth, minDepth), minDepth);
                maxExtreme = Vector.ConditionalSelect(indexIsValid, extremeCandidate, maxExtreme);
            }

            //Choose the starting point for the contact reduction. Two options:
            //If depth disparity is high, use the deepest index.
            //If depth disparity is too small, use the extreme X index to avoid flickering between manifold start locations.
            var useDepthIndex = Vector.GreaterThan(deepest.Depth - minDepth, new Vector<float>(1e-2f) * epsilonScale);
            ConditionalSelect(useDepthIndex, deepest, extreme, out contact0);
            contact0Exists = Vector.GreaterThan(rawContactCount, Vector<int>.Zero);

            //Find the most distant point from the starting contact.
            var maxDistanceSquared = Vector<float>.Zero;
            for (int i = 0; i < maxCandidateCount; ++i)
            {
                ref var candidate = ref Unsafe.Add(ref candidates, i);
                var offsetX = candidate.X - contact0.X;
                var offsetY = candidate.Y - contact0.Y;
                var distanceSquared = offsetX * offsetX + offsetY * offsetY;
                var indexIsValid = Vector.LessThan(new Vector<int>(i), rawContactCount);
                var candidateIsMostDistant = Vector.BitwiseAnd(Vector.GreaterThan(distanceSquared, maxDistanceSquared), indexIsValid);
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

                var indexIsValid = Vector.LessThan(new Vector<int>(i), rawContactCount);
                var isMinArea = Vector.BitwiseAnd(Vector.LessThan(signedArea, minSignedArea), indexIsValid);
                minSignedArea = Vector.ConditionalSelect(isMinArea, signedArea, minSignedArea);
                ConditionalSelect(isMinArea, candidate, contact2, out contact2);
                var isMaxArea = Vector.BitwiseAnd(Vector.GreaterThan(signedArea, maxSignedArea), indexIsValid);
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
    }
}
