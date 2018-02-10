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
            QuaternionWide.ConcatenateWithoutOverlap(ref orientationA, ref toLocalB, out var localOrientationA);
            QuaternionWide.TransformUnitY(ref localOrientationA, out var capsuleAxis);
            //There are six possible separating axes:
            //box.X
            //box.Y
            //box.Z
            //box.X x capsule.Y
            //box.Y x capsule.Y
            //box.Z x capsule.Y
            //Choose the axis with the minimum penetration depth.
            //By working in the box's local space, box.X/Y/Z become simply (1,0,0), (0,1,0), and (0,0,1). This cancels out a whole bunch of arithmetic.

            //Note that these depths will not take into account the radius until the very end. We pretend that the capsule is simply a line until then, since the radius changes nothing.
            var depthX = b.HalfWidth - Vector.Abs(capsuleAxis.X * a.HalfLength);
            var depthY = b.HalfHeight - Vector.Abs(capsuleAxis.Y * a.HalfLength);
            var depthZ = b.HalfLength - Vector.Abs(capsuleAxis.Z * a.HalfLength);

            //For each edge, given the zero components, the cross product boils down to creating a perpendicular 2d vector on the plane of the box axis from the cylinder axis.
            var x2 = capsuleAxis.X * capsuleAxis.X;
            var y2 = capsuleAxis.Y * capsuleAxis.Y;
            var z2 = capsuleAxis.Z * capsuleAxis.Z;
            //If the edge axis and capsule line segment are parallel, then the normal would be zero length.
            //In this case, fall back to using the projected offset from the edge to the capsule center.
            //Note that the projected offset fallback will fail if the offset is zero.
            //In that case, the edge direction cannot provide a better depth than a face direction anyway, so simply exclude it from consideration).
            //TODO: Better sign implementation would be nice.
            var positive = Vector<float>.One;
            var negative = -positive;
            var offsetBSignX = Vector.ConditionalSelect(Vector.LessThan(offsetB.X, Vector<float>.Zero), negative, positive);
            var offsetBSignY = Vector.ConditionalSelect(Vector.LessThan(offsetB.Y, Vector<float>.Zero), negative, positive);
            var offsetBSignZ = Vector.ConditionalSelect(Vector.LessThan(offsetB.Z, Vector<float>.Zero), negative, positive);
            Vector3Wide halfExtents;
            halfExtents.X = offsetBSignX * b.HalfWidth;
            halfExtents.Y = offsetBSignY * b.HalfHeight;
            halfExtents.Z = offsetBSignZ * b.HalfLength;
            Vector3Wide.Subtract(ref halfExtents.X, ref offsetB, out var edgeFallbackNormalComponents);
            var epsilon = new Vector<float>(1e-5f);
            var infiniteDepth = new Vector<float>(float.MaxValue);
            var bestDepth = infiniteDepth;
            var edgeXNeedsFallback = Vector.LessThan(y2 + z2, epsilon);
            var edgeYNeedsFallback = Vector.LessThan(x2 + z2, epsilon);
            var edgeZNeedsFallback = Vector.LessThan(x2 + y2, epsilon);
            var candidateNormalX = Vector.ConditionalSelect(edgeXNeedsFallback, edgeFallbackNormalComponents.Y, -capsuleAxis.Z);
            var candidateNormalY = Vector.ConditionalSelect(edgeXNeedsFallback, edgeFallbackNormalComponents.Z, capsuleAxis.Y);
            var inverseCandidateNormalLength = Vector<float>.One / Vector.SquareRoot(candidateNormalX * candidateNormalX + candidateNormalY * candidateNormalY);
            candidateNormalX *= inverseCandidateNormalLength;
            candidateNormalY *= inverseCandidateNormalLength;
            //var candidateDepth = dot()
            //Vector3Wide normal;
            //var edgeNormal = Vector.ConditionalSelect(edgeXNeedsFallback, edgeFallbackNormalX, 0);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test2(
            ref CapsuleWide a, ref BoxWide b,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Convex2ContactManifoldWide manifold)
        {
            QuaternionWide.Conjugate(ref orientationB, out var toLocalB);
            QuaternionWide.ConcatenateWithoutOverlap(ref orientationA, ref toLocalB, out var boxLocalOrientationA);
            QuaternionWide.TransformUnitY(ref boxLocalOrientationA, out var capsuleAxis);

            //Capsule-box has an extremely important property:
            //Capsule internal line segments almost never intersect the box due to the capsule's radius.
            //We can effectively split this implementation into two tests- exterior and interior.
            //The interior test only needs to be run if one of the subpairs happens to be deeply intersecting, which should be ~never in practice.
            //That's fortunate, because computing an accurate interior normal is painful!

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
            var faceDepthX = b.HalfWidth - Vector.Abs(capsuleAxis.X * a.HalfLength);
            var faceDepthY = b.HalfHeight - Vector.Abs(capsuleAxis.Y * a.HalfLength);
            var faceDepthZ = b.HalfLength - Vector.Abs(capsuleAxis.Z * a.HalfLength);
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
            edgeXDepth = Vector.ConditionalSelect(Vector.LessThan(edgeXLength, epsilon), infiniteDepth, edgeXDepth);
            edgeYDepth = Vector.ConditionalSelect(Vector.LessThan(edgeYLength, epsilon), infiniteDepth, edgeYDepth);
            edgeZDepth = Vector.ConditionalSelect(Vector.LessThan(edgeZLength, epsilon), infiniteDepth, edgeZDepth);

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
