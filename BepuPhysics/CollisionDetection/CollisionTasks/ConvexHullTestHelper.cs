using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public static class ConvexHullTestHelper
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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
            var slotBoundingPlaneEpsilon = new Vector<float>(boundingPlaneEpsilon[slotIndex]);
            Vector3Wide.Rebroadcast(closestOnHull, slotIndex, out var slotClosestOnHull);
            Vector3Wide.Dot(boundingPlaneBundle.Normal, slotLocalNormalBundle, out var bestFaceDotBundle);
            Vector3Wide.Dot(boundingPlaneBundle.Normal, slotClosestOnHull, out var closestOnHullDot);
            bestFaceDotBundle = Vector.ConditionalSelect(Vector.LessThan(Vector.Abs(closestOnHullDot - boundingPlaneBundle.Offset), slotBoundingPlaneEpsilon), bestFaceDotBundle, new Vector<float>(float.MinValue));
            var bestIndices = slotOffsetIndices;
            for (int i = 1; i < hull.BoundingPlanes.Length; ++i)
            {
                var slotIndices = new Vector<int>(i << BundleIndexing.VectorShift) + slotOffsetIndices;
                //Face normals point outward.
                //(Bundle slots beyond actual face count contain dummy data chosen to avoid being picked.)
                boundingPlaneBundle = ref hull.BoundingPlanes[i];
                Vector3Wide.Dot(boundingPlaneBundle.Normal, slotLocalNormalBundle, out var dot);
                Vector3Wide.Dot(boundingPlaneBundle.Normal, slotClosestOnHull, out closestOnHullDot);
                var useCandidate = Vector.BitwiseAnd(Vector.GreaterThan(dot, bestFaceDotBundle), Vector.LessThan(Vector.Abs(closestOnHullDot - boundingPlaneBundle.Offset), slotBoundingPlaneEpsilon));
                bestFaceDotBundle = Vector.ConditionalSelect(useCandidate, dot, bestFaceDotBundle);
                bestIndices = Vector.ConditionalSelect(useCandidate, slotIndices, bestIndices);
            }
            var bestFaceDot = bestFaceDotBundle[0];
            bestFaceIndex = bestIndices[0];
            for (int i = 1; i < Vector<float>.Count; ++i)
            {
                var dot = bestFaceDotBundle[i];
                if (dot > bestFaceDot)
                {
                    bestFaceDot = dot;
                    bestFaceIndex = bestIndices[i];
                }
            }
            BundleIndexing.GetBundleIndices(bestFaceIndex, out var faceBundleIndex, out var faceInnerIndex);
            Vector3Wide.ReadSlot(ref hull.BoundingPlanes[faceBundleIndex].Normal, faceInnerIndex, out slotFaceNormal);
        }
    }
}
