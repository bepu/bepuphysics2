using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CapsuleBoxTester : IPairTester<CapsuleWide, BoxWide, Convex2ContactManifoldWide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(
            ref CapsuleWide a, ref BoxWide b,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Convex2ContactManifoldWide manifold)
        {
            QuaternionWide.Conjugate(ref orientationB, out var toLocalB);
            QuaternionWide.TransformWithoutOverlap(ref offsetB, ref toLocalB, out var localOffsetA);
            Vector3Wide.Negate(ref localOffsetA);
            QuaternionWide.ConcatenateWithoutOverlap(ref orientationA, ref toLocalB, out var boxLocalOrientationA);
            QuaternionWide.TransformUnitY(ref boxLocalOrientationA, out var capsuleAxis);

            //For each face of the box, clip the capsule axis against its bounding planes and then clamp the result to the box's extent along that axis.
            //The face which has the largest clipped interval of the capsule line segment will be picked as the representative.
            var positive = Vector<float>.One;
            var negative = -positive;
            var divisionGuard = new Vector<float>(1e-15f);
            //Division by zero is hacked away by clamping the magnitude to some nonzero value and recovering the sign in the numerator.
            //The resulting interval will tend to be enormous and properly signed, so it serves the necessary purpose.
            var inverseCapsuleAxisX = Vector.ConditionalSelect(Vector.LessThan(capsuleAxis.X, Vector<float>.Zero), negative, positive) / Vector.Max(Vector.Abs(capsuleAxis.X), divisionGuard);
            var inverseCapsuleAxisY = Vector.ConditionalSelect(Vector.LessThan(capsuleAxis.Y, Vector<float>.Zero), negative, positive) / Vector.Max(Vector.Abs(capsuleAxis.Y), divisionGuard);
            var inverseCapsuleAxisZ = Vector.ConditionalSelect(Vector.LessThan(capsuleAxis.Z, Vector<float>.Zero), negative, positive) / Vector.Max(Vector.Abs(capsuleAxis.Z), divisionGuard);
            Vector3Wide scaledExtents, scaledOffset;
            scaledExtents.X = b.HalfWidth * Vector.Abs(inverseCapsuleAxisX);
            scaledExtents.Y = b.HalfHeight * Vector.Abs(inverseCapsuleAxisY);
            scaledExtents.Z = b.HalfLength * Vector.Abs(inverseCapsuleAxisZ);
            scaledOffset.X = localOffsetA.X * inverseCapsuleAxisX;
            scaledOffset.Y = localOffsetA.Y * inverseCapsuleAxisY;
            scaledOffset.Z = localOffsetA.Z * inverseCapsuleAxisZ;
            Vector3Wide.Subtract(ref scaledOffset, ref scaledExtents, out var tCandidateMin);
            var negativeHalfLength = -a.HalfLength;
            Vector3Wide.Max(ref negativeHalfLength, ref tCandidateMin, out tCandidateMin);
            Vector3Wide.Add(ref scaledOffset, ref scaledExtents, out var tCandidateMax);
            Vector3Wide.Min(ref a.HalfLength, ref tCandidateMax, out tCandidateMax);

            //We now have clipped intervals for all faces. The faces along axis X, for example, use interval max(tMin.Y, tMin.Z) to min(tMax.Y, tMax.Z).
            //Find the longest interval. Note that it's possible for the interval length to be negative when a line segment does not enter the face's voronoi region.
            //It may be negative for all axes, even; that can happen when the line segment is outside all face voronoi regions. That's fine, though.
            //It's simply a signal that the line segment is in one of the face-adjacent voronoi regions, and clamping the endpoints will provide the closest point on the box to the segment.
            var intervalMaxX = Vector.Min(tCandidateMax.Y, tCandidateMax.Z);
            var intervalMaxY = Vector.Min(tCandidateMax.X, tCandidateMax.Z);
            var intervalMaxZ = Vector.Min(tCandidateMax.X, tCandidateMax.Y);
            var intervalMinX = Vector.Max(tCandidateMin.Y, tCandidateMin.Z);
            var intervalMinY = Vector.Max(tCandidateMin.X, tCandidateMin.Z);
            var intervalMinZ = Vector.Max(tCandidateMin.X, tCandidateMin.Y);
            var intervalLengthX = intervalMaxX - intervalMinX;
            var intervalLengthY = intervalMaxY - intervalMinY;
            var intervalLengthZ = intervalMaxZ - intervalMinZ;
            var maxLength = Vector.Max(intervalLengthX, Vector.Max(intervalLengthY, intervalLengthZ));
            var useX = Vector.Equals(intervalLengthX, maxLength);
            var useY = Vector.Equals(intervalLengthY, maxLength);
            var tMin = Vector.ConditionalSelect(useX, intervalMinX, Vector.ConditionalSelect(useY, intervalMinY, intervalMinZ));
            var tMax = Vector.ConditionalSelect(useX, intervalMaxX, Vector.ConditionalSelect(useY, intervalMaxY, intervalMaxZ));

            Vector3Wide.Scale(ref capsuleAxis, ref tMin, out var capsuleStart);
            Vector3Wide.Scale(ref capsuleAxis, ref tMax, out var capsuleEnd);
            Vector3Wide.Add(ref localOffsetA, ref capsuleStart, out capsuleStart);
            Vector3Wide.Add(ref localOffsetA, ref capsuleEnd, out capsuleEnd);

            //Clamp the start and end to the box. Note that we haven't kept track of which axis we're working on, so we just do clamping on all three axes. Two axes will be wasted,
            //but it's easier to do this than to try to reconstruct which axis is the one we're supposed to work on.
            Vector3Wide boxStart, boxEnd;
            boxStart.X = Vector.Max(Vector.Min(capsuleStart.X, b.HalfWidth), -b.HalfWidth);
            boxStart.Y = Vector.Max(Vector.Min(capsuleStart.Y, b.HalfHeight), -b.HalfHeight);
            boxStart.Z = Vector.Max(Vector.Min(capsuleStart.Z, b.HalfLength), -b.HalfLength);
            boxEnd.X = Vector.Max(Vector.Min(capsuleEnd.X, b.HalfWidth), -b.HalfWidth);
            boxEnd.Y = Vector.Max(Vector.Min(capsuleEnd.Y, b.HalfHeight), -b.HalfHeight);
            boxEnd.Z = Vector.Max(Vector.Min(capsuleEnd.Z, b.HalfLength), -b.HalfLength);

            //Compute the closest point on the line segment to the clamped start and end.
            Vector3Wide.Subtract(ref boxStart, ref localOffsetA, out var aToBoxStart);
            Vector3Wide.Subtract(ref boxEnd, ref localOffsetA, out var aToBoxEnd);
            Vector3Wide.Dot(ref capsuleAxis, ref aToBoxStart, out var dotStart);
            Vector3Wide.Dot(ref capsuleAxis, ref aToBoxEnd, out var dotEnd);
            dotStart = Vector.Max(Vector.Min(dotStart, a.HalfLength), negativeHalfLength);
            dotEnd = Vector.Max(Vector.Min(dotEnd, a.HalfLength), negativeHalfLength);

            //Now compute the distances. The normal will be defined by the shorter of the two offsets.
            Vector3Wide.Scale(ref capsuleAxis, ref dotStart, out var closestFromBoxStart);
            Vector3Wide.Scale(ref capsuleAxis, ref dotEnd, out var closestFromBoxEnd);
            Vector3Wide.Add(ref closestFromBoxStart, ref localOffsetA, out closestFromBoxStart);
            Vector3Wide.Add(ref closestFromBoxEnd, ref localOffsetA, out closestFromBoxEnd);
            //Note that these offsets are pointing from the box to the capsule (B to A), matching contact normal convention.
            Vector3Wide.Subtract(ref closestFromBoxStart, ref boxStart, out var startOffset);
            Vector3Wide.Subtract(ref closestFromBoxEnd, ref boxEnd, out var endOffset);

            Vector3Wide.LengthSquared(ref startOffset, out var startLengthSquared);
            Vector3Wide.LengthSquared(ref endOffset, out var endLengthSquared);

            var minLengthSquared = Vector.Min(startLengthSquared, endLengthSquared);
            var useStart = Vector.Equals(minLengthSquared, startLengthSquared);
            Vector3Wide.ConditionalSelect(ref useStart, ref startOffset, ref endOffset, out manifold.Normal);
            var inverseLength = Vector<float>.One / Vector.SquareRoot(minLengthSquared);
            Vector3Wide.Scale(ref manifold.Normal, ref inverseLength, out manifold.Normal);

            //Note that we defer contact position offseting by the normal * depth until later when we know the normal isn't going to change.
            Vector3Wide.Subtract(ref closestFromBoxStart, ref localOffsetA, out manifold.OffsetA0);
            Vector3Wide.Subtract(ref closestFromBoxEnd, ref localOffsetA, out manifold.OffsetA1);
            manifold.FeatureId0 = Vector<int>.Zero;
            manifold.FeatureId1 = Vector<int>.One;

            //Capsule-box has an extremely important property:
            //Capsule internal line segments almost never intersect the box due to the capsule's radius.
            //We can effectively split this implementation into two tests- exterior and interior.
            //The interior test only needs to be run if one of the subpairs happens to be deeply intersecting, which should be ~never in practice.
            //That's fortunate, because computing an accurate interior normal is painful!
            if (Vector.LessThanAny(minLengthSquared, new Vector<float>(1e-10f)))
            {
                //There are six possible separating axes when the capsule's internal line segment intersects the box:
                //box.X
                //box.Y
                //box.Z
                //box.X x capsule.Y
                //box.Y x capsule.Y
                //box.Z x capsule.Y
                //Choose the axis with the minimum penetration depth.
                //By working in the box's local space, box.X/Y/Z become simply (1,0,0), (0,1,0), and (0,0,1). This cancels out a whole bunch of arithmetic.

                //Note that these depths will not take into account the radius until the very end. We pretend that the capsule is simply a line until then, since the radius changes nothing.
                var faceDepthX = b.HalfWidth - localOffsetA.X - Vector.Abs(capsuleAxis.X * a.HalfLength);
                var faceDepthY = b.HalfHeight - localOffsetA.Y - Vector.Abs(capsuleAxis.Y * a.HalfLength);
                var faceDepthZ = b.HalfLength - localOffsetA.Z - Vector.Abs(capsuleAxis.Z * a.HalfLength);
                var faceDepth = Vector.Min(Vector.Min(faceDepthX, faceDepthY), faceDepthZ);
                var useFaceX = Vector.Equals(faceDepth, faceDepthX);
                var useFaceY = Vector.AndNot(Vector.Equals(faceDepth, faceDepthY), useFaceX);
                var useFaceZ = Vector.OnesComplement(Vector.BitwiseOr(useFaceX, useFaceY));
                //Note that signs are calibrated as a last step so that edge normals do not need their own calibration.
                Vector3Wide faceNormal;
                faceNormal.X = Vector.ConditionalSelect(useFaceX, Vector<float>.One, Vector<float>.Zero);
                faceNormal.Y = Vector.ConditionalSelect(useFaceY, Vector<float>.One, Vector<float>.Zero);
                faceNormal.Z = Vector.ConditionalSelect(useFaceZ, Vector<float>.One, Vector<float>.Zero);

                //For each edge, given the zero components, the cross product boils down to creating a perpendicular 2d vector on the plane of the box axis from the cylinder axis.
                var x2 = capsuleAxis.X * capsuleAxis.X;
                var y2 = capsuleAxis.Y * capsuleAxis.Y;
                var z2 = capsuleAxis.Z * capsuleAxis.Z;

                var infiniteDepth = new Vector<float>(float.MaxValue);
                var bestDepth = infiniteDepth;
                var edgeXLength = Vector.SquareRoot(y2 + z2);
                var edgeYLength = Vector.SquareRoot(x2 + z2);
                var edgeZLength = Vector.SquareRoot(x2 + y2);
                //depth = dot(N, halfExtents - offsetA), where N = (edgeAxis x capsuleAxis) / ||(edgeAxis x capsuleAxis)||, and both N and halfExtents have their signs calibrated.
                //Using abs allows us to handle the sign calibration inline at the cost of splitting the dot product.
                var inverseLengthX = Vector<float>.One / edgeXLength;
                var inverseLengthY = Vector<float>.One / edgeYLength;
                var inverseLengthZ = Vector<float>.One / edgeZLength;
                var edgeXDepth = (Vector.Abs(b.HalfHeight * capsuleAxis.Z - b.HalfLength * capsuleAxis.Y) - Vector.Abs(offsetB.Y * capsuleAxis.Z - offsetB.Z * capsuleAxis.Y)) * inverseLengthX;
                var edgeYDepth = (Vector.Abs(b.HalfWidth * capsuleAxis.Z - b.HalfLength * capsuleAxis.X) - Vector.Abs(offsetB.X * capsuleAxis.Z - offsetB.Z * capsuleAxis.X)) * inverseLengthY;
                var edgeZDepth = (Vector.Abs(b.HalfWidth * capsuleAxis.Y - b.HalfHeight * capsuleAxis.X) - Vector.Abs(offsetB.X * capsuleAxis.Y - offsetB.Y * capsuleAxis.X)) * inverseLengthZ;
                var epsilon = new Vector<float>(1e-9f);
                //If the capsule internal segment and the edge are parallel, we ignore the edge by replacing its depth with ~infinity.
                //Such parallel axes are guaranteed to do no better than one of the face directions since this path is only relevant during segment-box deep collisions.
                edgeXDepth = Vector.ConditionalSelect(Vector.LessThan(edgeXLength, divisionGuard), infiniteDepth, edgeXDepth);
                edgeYDepth = Vector.ConditionalSelect(Vector.LessThan(edgeYLength, divisionGuard), infiniteDepth, edgeYDepth);
                edgeZDepth = Vector.ConditionalSelect(Vector.LessThan(edgeZLength, divisionGuard), infiniteDepth, edgeZDepth);

                var edgeDepth = Vector.Min(Vector.Min(edgeXDepth, edgeYDepth), edgeZDepth);
                var useEdgeX = Vector.Equals(edgeDepth, edgeXDepth);
                var useEdgeY = Vector.Equals(edgeDepth, edgeYDepth);
                Vector3Wide edgeNormal;
                edgeNormal.X = Vector.ConditionalSelect(useEdgeX, Vector<float>.Zero, Vector.ConditionalSelect(useEdgeY, capsuleAxis.Z, capsuleAxis.Y));
                edgeNormal.Y = Vector.ConditionalSelect(useEdgeX, capsuleAxis.Z, Vector.ConditionalSelect(useEdgeY, Vector<float>.Zero, -capsuleAxis.X));
                edgeNormal.Z = Vector.ConditionalSelect(useEdgeX, -capsuleAxis.Y, Vector.ConditionalSelect(useEdgeY, -capsuleAxis.X, Vector<float>.Zero));

                var useEdge = Vector.LessThan(edgeDepth, faceDepth);
                Vector3Wide.ConditionalSelect(ref useEdge, ref edgeNormal, ref faceNormal, out manifold.Normal);

                //Calibrate the normal such that it points from B to A.
                Vector3Wide.Dot(ref manifold.Normal, ref offsetB, out var dot);
                var shouldNegateNormal = Vector.GreaterThan(dot, Vector<float>.Zero);
                manifold.Normal.X = Vector.ConditionalSelect(shouldNegateNormal, -manifold.Normal.X, manifold.Normal.X);
                manifold.Normal.Y = Vector.ConditionalSelect(shouldNegateNormal, -manifold.Normal.Y, manifold.Normal.Y);
                manifold.Normal.Z = Vector.ConditionalSelect(shouldNegateNormal, -manifold.Normal.Z, manifold.Normal.Z);
            }

            //We now have Normal, OffsetA0, and OffsetA1. We may have explicitly calculated depths as a part of that process, but each contact may have its own depth.
            //Imagine a face collision- if the capsule axis isn't fully parallel with the plane's surface, it would be strange to use the same depth for both contacts.
            //Compute the interval of the box on the normal. Note that the normal is already calibrated to point from B to A (box to capsule).
            var boxExtreme = Vector.Abs(manifold.Normal.X * b.HalfWidth) + Vector.Abs(manifold.Normal.Y * b.HalfHeight) + Vector.Abs(manifold.Normal.Z * b.HalfLength);
            Vector3Wide.Dot(ref manifold.Normal, ref closestFromBoxStart, out var dot0);
            Vector3Wide.Dot(ref manifold.Normal, ref closestFromBoxEnd, out var dot1);
            manifold.Depth0 = a.Radius + boxExtreme - dot0;
            manifold.Depth1 = a.Radius + boxExtreme - dot1;

            //Apply the normal offset to the contact positions.           
            var negativeOffsetFromA0 = manifold.Depth0 * 0.5f - a.Radius;
            var negativeOffsetFromA1 = manifold.Depth1 * 0.5f - a.Radius;
            Vector3Wide.Scale(ref manifold.Normal, ref negativeOffsetFromA0, out var normalPush0);
            Vector3Wide.Scale(ref manifold.Normal, ref negativeOffsetFromA1, out var normalPush1);
            Vector3Wide.Add(ref manifold.OffsetA0, ref normalPush0, out manifold.OffsetA0);
            Vector3Wide.Add(ref manifold.OffsetA1, ref normalPush1, out manifold.OffsetA1);

            //Note that it is possible that the two contacts is redundant. Only keep them both if they're separated.
            manifold.Count = Vector.ConditionalSelect(Vector.LessThan(Vector.Abs(dotStart - dotEnd), a.HalfLength * new Vector<float>(1e-7f)), Vector<int>.One, new Vector<int>(2));
        }

        public void Test(ref CapsuleWide a, ref BoxWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref CapsuleWide a, ref BoxWide b, ref Vector3Wide offsetB, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }

    public class CapsuleBoxCollisionTask : CollisionTask
    {
        public CapsuleBoxCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(Capsule).TypeId;
            ShapeTypeIndexB = default(Box).TypeId;
        }


        //Every single collision task type will mirror this general layout.
        public unsafe override void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
        {
            CollisionTaskCommon.ExecuteBatch
                <TContinuations, TFilters,
                Capsule, CapsuleWide, Box, BoxWide, TestPairWide<Capsule, CapsuleWide, Box, BoxWide>,
                Convex2ContactManifoldWide, CapsuleBoxTester>(ref batch, ref batcher, ref continuations, ref filters);
        }
    }
}
