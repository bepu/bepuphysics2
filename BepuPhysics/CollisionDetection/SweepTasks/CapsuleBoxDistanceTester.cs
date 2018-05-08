using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CapsuleBoxDistanceTester : IPairDistanceTester<CapsuleWide, BoxWide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestAndRefineBoxEdge(
             ref Vector<float> offsetAX, ref Vector<float> offsetAY, ref Vector<float> offsetAZ,
             ref Vector<float> capsuleAxisX, ref Vector<float> capsuleAxisY, ref Vector<float> capsuleAxisZ,
             ref Vector<float> capsuleHalfLength,
             ref Vector<float> boxEdgeCenterX, ref Vector<float> boxEdgeCenterY,
             ref Vector<float> boxHalfWidth, ref Vector<float> boxHalfHeight, ref Vector<float> boxHalfLength,
                out Vector<float> distance, out Vector<float> ta, out Vector<float> nX, out Vector<float> nY, out Vector<float> nZ)
        {
            //TODO: Might be a good idea to check the codegen to make sure it drops the dead paths.
            CapsuleBoxTester.TestBoxEdge(ref offsetAX, ref offsetAY, ref offsetAZ,
                  ref capsuleAxisX, ref capsuleAxisY, ref capsuleAxisZ,
                  ref capsuleHalfLength,
                  ref boxEdgeCenterX, ref boxEdgeCenterY,
                  ref boxHalfWidth, ref boxHalfHeight, ref boxHalfLength,
                  out _, out _, out var closestPointOnA, out nX, out nY, out nZ, out ta, out _);

            //The normal was constructed as the minimal separating direction between the two edges, so we can use it to quickly compute the distance without the need for any clamping. 
            distance = nX * (closestPointOnA.X - boxEdgeCenterX) + nY * (closestPointOnA.Y - boxEdgeCenterY) + nZ * closestPointOnA.Z;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(
            ref Vector<float> distance, ref Vector<float> ta,
            ref Vector<float> localNormalX, ref Vector<float> localNormalY, ref Vector<float> localNormalZ,
            ref Vector<float> distanceCandidate, ref Vector<float> taCandidate,
            ref Vector<float> localNormalCandidateX, ref Vector<float> localNormalCandidateY, ref Vector<float> localNormalCandidateZ)
        {
            var useCandidate = Vector.LessThan(distanceCandidate, distance);
            distance = Vector.Min(distanceCandidate, distance);
            ta = Vector.ConditionalSelect(useCandidate, taCandidate, ta);
            localNormalX = Vector.ConditionalSelect(useCandidate, localNormalCandidateX, localNormalX);
            localNormalY = Vector.ConditionalSelect(useCandidate, localNormalCandidateY, localNormalY);
            localNormalZ = Vector.ConditionalSelect(useCandidate, localNormalCandidateZ, localNormalZ);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Clamp(ref Vector3Wide point, ref BoxWide box, out Vector3Wide offset, out Vector<float> distanceSquared)
        {
            Vector3Wide clamped;
            clamped.X = Vector.Min(box.HalfWidth, Vector.Max(-box.HalfWidth, point.X));
            clamped.Y = Vector.Min(box.HalfHeight, Vector.Max(-box.HalfHeight, point.Y));
            clamped.Z = Vector.Min(box.HalfLength, Vector.Max(-box.HalfLength, point.Z));
            Vector3Wide.Subtract(ref point, ref clamped, out offset);
            Vector3Wide.LengthSquared(ref offset, out distanceSquared);
        }
        public void Test(ref CapsuleWide a, ref BoxWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            CapsuleBoxTester.Prepare(ref b, ref offsetB, ref orientationA, ref orientationB, out var localOffsetA, out var capsuleAxis, out var edgeCenters);

            //Swizzle XYZ -> YZX
            Vector3Wide localNormal;
            TestAndRefineBoxEdge(ref localOffsetA.Y, ref localOffsetA.Z, ref localOffsetA.X,
                ref capsuleAxis.Y, ref capsuleAxis.Z, ref capsuleAxis.X,
                ref a.HalfLength,
                ref edgeCenters.Y, ref edgeCenters.Z,
                ref b.HalfHeight, ref b.HalfLength, ref b.HalfWidth,
                out var edgeDistance, out var ta, out localNormal.Y, out localNormal.Z, out localNormal.X);
            //Swizzle XYZ -> ZXY
            TestAndRefineBoxEdge(ref localOffsetA.Z, ref localOffsetA.X, ref localOffsetA.Y,
                ref capsuleAxis.Z, ref capsuleAxis.X, ref capsuleAxis.Y,
                ref a.HalfLength,
                ref edgeCenters.Z, ref edgeCenters.X,
                ref b.HalfLength, ref b.HalfWidth, ref b.HalfHeight,
                out var eyDistance, out var eyTa, out var eynZ, out var eynX, out var eynY);
            Select(ref edgeDistance, ref ta, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref eyDistance, ref eyTa, ref eynX, ref eynY, ref eynZ);
            //Swizzle XYZ -> XYZ
            TestAndRefineBoxEdge(ref localOffsetA.X, ref localOffsetA.Y, ref localOffsetA.Z,
                ref capsuleAxis.X, ref capsuleAxis.Y, ref capsuleAxis.Z,
                ref a.HalfLength,
                ref edgeCenters.X, ref edgeCenters.Y,
                ref b.HalfWidth, ref b.HalfHeight, ref b.HalfLength,
                out var ezDistance, out var ezTa, out var eznX, out var eznY, out var eznZ);
            Select(ref edgeDistance, ref ta, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref ezDistance, ref ezTa, ref eznX, ref eznY, ref eznZ);

            //The minimum distance is either from an edge test, or from one of the two vertices. Clamp the capsule vertices to the box and test.
            Vector3Wide.Scale(ref capsuleAxis, ref a.HalfLength, out var vertexOffset);
            Vector3Wide.Add(ref localOffsetA, ref vertexOffset, out var positive);
            Clamp(ref positive, ref b, out var positiveOffset, out var positiveDistanceSquared);
            Vector3Wide.Subtract(ref localOffsetA, ref vertexOffset, out var negative);
            Clamp(ref negative, ref b, out var negativeOffset, out var negativeDistanceSquared);

            var usePositiveOffset = Vector.LessThan(positiveDistanceSquared, negativeDistanceSquared);
            var vertexTa = Vector.ConditionalSelect(usePositiveOffset, a.HalfLength, -a.HalfLength);
            var vertexDistanceSquared = Vector.Min(positiveDistanceSquared, negativeDistanceSquared);
            Vector3Wide.ConditionalSelect(ref usePositiveOffset, ref positiveOffset, ref negativeOffset, out var vertexNormal);
            var vertexDistance = Vector.SquareRoot(vertexDistanceSquared);
            var normalScale = Vector<float>.One / vertexDistance;
            Vector3Wide.Scale(ref vertexNormal, ref normalScale, out vertexNormal);

            var useEdge = Vector.LessThan(edgeDistance, vertexDistance);
            distance = Vector.Min(edgeDistance, vertexDistance) - a.Radius;
            Vector3Wide.ConditionalSelect(ref useEdge, ref localNormal, ref vertexNormal, out localNormal);
            QuaternionWide.TransformWithoutOverlap(ref localNormal, ref orientationB, out normal);
            var negativeRadius = -a.Radius;
            QuaternionWide.TransformUnitY(ref orientationA, out var worldCapsuleAxis);
            Vector3Wide.Scale(ref worldCapsuleAxis, ref ta, out closestA);
            Vector3Wide.Scale(ref normal, ref negativeRadius, out var closestOffsetA);
            Vector3Wide.Add(ref closestOffsetA, ref closestA, out closestA);

            intersected = Vector.LessThanOrEqual(distance, Vector<float>.Zero);



        }
    }


}
