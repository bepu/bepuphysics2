using BepuPhysics.Collidables;
using BepuUtilities;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public static class ConvexHullTestHelper
    {
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void PickRepresentativeFace(ref ConvexHull hull, int slotIndex, ref Vector3Wide localNormal, in Vector3Wide closestOnHull, in Vector<int> slotOffsetIndices,
            ref Vector<float> boundingPlaneEpsilon, out Vector3 slotFaceNormal, out Vector3 slotLocalNormal, out int bestFaceIndex)
        {
            //Pick the representative face from the set of faces touching the best sampled support point.
            //The representative face is chosen based on:
            //1) the closest hull point is on the face's plane and 
            //2) of all faces touching the hull point, the face's normal is most aligned with the contact normal.
            Vector3Wide.ReadSlot(ref localNormal, slotIndex, out slotLocalNormal);
            Vector3Wide.Broadcast(slotLocalNormal, out var slotLocalNormalBundle);
            ref var boundingPlaneBundle = ref hull.BoundingPlanes[0];
            var slotBoundingPlaneEpsilon = boundingPlaneEpsilon[slotIndex];
            var slotBoundingPlaneEpsilonBundle = new Vector<float>(slotBoundingPlaneEpsilon);
            var negatedSlotBoundingPlaneEpsilonBundle = -slotBoundingPlaneEpsilonBundle;
            Vector3Wide.Rebroadcast(closestOnHull, slotIndex, out var slotClosestOnHull);
            Vector3Wide.Dot(boundingPlaneBundle.Normal, slotLocalNormalBundle, out var bestFaceDotBundle);
            Vector3Wide.Dot(boundingPlaneBundle.Normal, slotClosestOnHull, out var closestOnHullDot);
            //Note that we primarily search to minimize plane error. Even if it doesn't have great normal alignment, we must terminate with some valid result.
            //(It's numerically possible, though rare, for the bounding plane epsilon to fail to match any face, so we use the minimal error in that case.)
            var bestPlaneErrorBundle = Vector.Abs(closestOnHullDot - boundingPlaneBundle.Offset);
            var bestIndices = slotOffsetIndices;
            for (int i = 1; i < hull.BoundingPlanes.Length; ++i)
            {
                var slotIndices = new Vector<int>(i << BundleIndexing.VectorShift) + slotOffsetIndices;
                //Face normals point outward.
                //(Bundle slots beyond actual face count contain dummy data chosen to avoid being picked.)
                boundingPlaneBundle = ref hull.BoundingPlanes[i];
                Vector3Wide.Dot(boundingPlaneBundle.Normal, slotLocalNormalBundle, out var dot);
                Vector3Wide.Dot(boundingPlaneBundle.Normal, slotClosestOnHull, out closestOnHullDot);
                var candidateError = Vector.Abs(closestOnHullDot - boundingPlaneBundle.Offset);
                var errorImprovement = bestPlaneErrorBundle - candidateError;
                var useCandidate = Vector.BitwiseOr(
                    //If the plane error improvement is significant, use it.
                    Vector.GreaterThanOrEqual(errorImprovement, slotBoundingPlaneEpsilonBundle), 
                    //If the plane error improvement is small, then only use the candidate if it has a better aligned normal.
                    Vector.BitwiseAnd(
                        Vector.GreaterThan(errorImprovement, negatedSlotBoundingPlaneEpsilonBundle), 
                        Vector.GreaterThan(dot, bestFaceDotBundle)));
                bestFaceDotBundle = Vector.ConditionalSelect(useCandidate, dot, bestFaceDotBundle);
                bestPlaneErrorBundle = Vector.ConditionalSelect(useCandidate, candidateError, bestPlaneErrorBundle);
                bestIndices = Vector.ConditionalSelect(useCandidate, slotIndices, bestIndices);
            }
            var bestFaceDot = bestFaceDotBundle[0];
            var bestPlaneError = bestPlaneErrorBundle[0];
            bestFaceIndex = bestIndices[0];
            var negatedSlotBoundingPlaneEpsilon = -slotBoundingPlaneEpsilon;
            for (int i = 1; i < Vector<float>.Count; ++i)
            {
                var dot = bestFaceDotBundle[i];
                var error = bestPlaneErrorBundle[i];
                var improvement = bestPlaneError - error;
                if (improvement >= slotBoundingPlaneEpsilon || (improvement >= negatedSlotBoundingPlaneEpsilon && dot > bestFaceDot))
                {
                    bestFaceDot = dot;
                    bestPlaneError = error;
                    bestFaceIndex = bestIndices[i];
                }
            }
            Debug.Assert(bestFaceIndex >= 0 && bestFaceIndex < hull.FaceToVertexIndicesStart.Length);
            BundleIndexing.GetBundleIndices(bestFaceIndex, out var faceBundleIndex, out var faceInnerIndex);
            Vector3Wide.ReadSlot(ref hull.BoundingPlanes[faceBundleIndex].Normal, faceInnerIndex, out slotFaceNormal);
        }
    }
}
