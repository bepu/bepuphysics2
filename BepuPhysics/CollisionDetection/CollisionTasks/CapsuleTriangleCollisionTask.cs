using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CapsuleTriangleTester : IPairTester<CapsuleWide, TriangleWide, Convex2ContactManifoldWide>
    {
        void TestEdge(in TriangleWide triangle, in Vector3Wide triangleCenter, in Vector3Wide triangleNormal,
            in Vector3Wide edgeStart, in Vector3Wide edgeOffset,
            in Vector3Wide capsuleCenter, in Vector3Wide capsuleAxis, in Vector<float> capsuleHalfLength,
            out Vector3Wide edgeDirection, out Vector<float> ta, out Vector<float> tb, out Vector<float> bMin, out Vector<float> bMax, out Vector<float> depth, out Vector3Wide normal)
        {
            //Borrowing logic from the capsule-capsule test:
            //Compute the closest points between the two line segments. No clamping to begin with.
            //We want to minimize distance = ||(a + da * ta) - (b + db * tb)||.
            //Taking the derivative with respect to ta and doing some algebra (taking into account ||da|| == ||db|| == 1) to solve for ta yields:
            //ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))        
            //(Probably could avoid this square root by revisiting the derivation to permit non-unit length directions, but it's fairly inconsequential.)
            Vector3Wide.Length(edgeOffset, out var edgeLength);
            Vector3Wide.Scale(edgeOffset, Vector<float>.One / edgeLength, out var db);
            Vector3Wide.Subtract(edgeStart, capsuleCenter, out var offsetB);
            Vector3Wide.Dot(capsuleAxis, offsetB, out var daOffsetB);
            Vector3Wide.Dot(db, offsetB, out var dbOffsetB);
            Vector3Wide.Dot(capsuleAxis, db, out var dadb);
            //Note potential division by zero when the axes are parallel. Arbitrarily clamp; near zero values will instead produce extreme values which get clamped to reasonable results.
            ta = (daOffsetB - dbOffsetB * dadb) / Vector.Max(new Vector<float>(1e-15f), Vector<float>.One - dadb * dadb);
            //tb = ta * (da * db) - db * (b - a)
            tb = ta * dadb - dbOffsetB;

            //We cannot simply clamp the ta and tb values to the line segments. Instead, project each line segment onto the other line segment, clamping against the target's interval.
            //That new clamped projected interval is the valid solution space on that line segment. We can clamp the t value by that interval to get the correctly bounded solution.
            //The projected intervals are:
            //B onto A: (0 or edgeLength) * (da * db) + da * offsetB
            //A onto B: +-AHalfLength * (da * db) - db * offsetB
            var aMin = Vector.Max(-capsuleHalfLength, Vector.Min(capsuleHalfLength, daOffsetB));
            var aMax = Vector.Min(capsuleHalfLength, Vector.Max(-capsuleHalfLength, daOffsetB + edgeLength * dadb));
            var aOntoBOffset = capsuleHalfLength * Vector.Abs(dadb);
            bMin = Vector.Max(Vector<float>.Zero, Vector.Min(edgeLength, -aOntoBOffset - dbOffsetB));
            bMax = Vector.Min(edgeLength, Vector.Max(Vector<float>.Zero, aOntoBOffset - dbOffsetB));
            ta = Vector.Min(Vector.Max(ta, aMin), aMax);
            tb = Vector.Min(Vector.Max(tb, bMin), bMax);

            Vector3Wide.Scale(capsuleAxis, ta, out var closestPointOnCapsule);
            Vector3Wide.Add(closestPointOnCapsule, capsuleCenter, out closestPointOnCapsule);
            Vector3Wide.Scale(db, tb, out var closestPointOnEdge);
            Vector3Wide.Add(closestPointOnEdge, edgeStart, out closestPointOnEdge);

            Vector3Wide.Subtract(closestPointOnCapsule, closestPointOnEdge, out normal);
            Vector3Wide.LengthSquared(normal, out var normalLengthSquared);
            //In the event that the normal has zero length due to the capsule internal line segment touching the edge, use the cross product of the edge and axis.
            Vector3Wide.CrossWithoutOverlap(edgeOffset, capsuleAxis, out var fallbackNormal);
            Vector3Wide.LengthSquared(fallbackNormal, out var fallbackNormalLengthSquared);
            var useFallbackNormal = Vector.LessThan(normalLengthSquared, new Vector<float>(1e-15f));
            Vector3Wide.ConditionalSelect(useFallbackNormal, fallbackNormal, normal, out normal);
            normalLengthSquared = Vector.ConditionalSelect(useFallbackNormal, fallbackNormalLengthSquared, normalLengthSquared);
            //Unfortunately, if the edge and axis are parallel, the cross product will ALSO be zero, so we need another fallback. We'll use the edge plane normal.
            //Unless the triangle is degenerate, this can't be zero length.
            Vector3Wide.Cross(triangleNormal, edgeOffset, out var secondFallbackNormal);
            Vector3Wide.LengthSquared(fallbackNormal, out var secondFallbackNormalLengthSquared);
            var useSecondFallbackNormal = Vector.LessThan(normalLengthSquared, new Vector<float>(1e-15f));
            Vector3Wide.ConditionalSelect(useSecondFallbackNormal, secondFallbackNormal, normal, out normal);
            normalLengthSquared = Vector.ConditionalSelect(useSecondFallbackNormal, secondFallbackNormalLengthSquared, normalLengthSquared);
            //Calibrate the normal to point from the triangle to the capsule. Note that this may not be the same direction as the *edge* to the capsule.
            Vector3Wide.Subtract(capsuleCenter, triangleCenter, out var ba);
            Vector3Wide.Dot(normal, ba, out var normalBA);
            var numerator = Vector.ConditionalSelect(Vector.LessThan(normalBA, Vector<float>.Zero), new Vector<float>(-1f), Vector<float>.One);
            Vector3Wide.Scale(normal, numerator / Vector.SquareRoot(normalLengthSquared), out normal);

            //Note that the normal between the closest points is not necessarily perpendicular to both the edge and capsule axis due to clamping, so to compute depth
            //we need to include the extent of both.
            Vector3Wide.Dot(capsuleAxis, normal, out var nAxis);
            Vector3Wide.Dot(normal, capsuleCenter, out var nCapsuleCenter);
            //Normal calibrated to point from triangle to capsule, so the extreme point pushes down.
            var extremeOnCapsule = nCapsuleCenter - Vector.Abs(nAxis) * capsuleHalfLength;
            Vector3Wide.Dot(triangle.A, normal, out var na);
            Vector3Wide.Dot(triangle.B, normal, out var nb);
            Vector3Wide.Dot(triangle.C, normal, out var nc);
            //Normal calibration implies largest triangle value.
            var extremeOnTriangle = Vector.Max(na, Vector.Max(nb, nc));
            depth = extremeOnTriangle - extremeOnCapsule;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ClipAgainstEdgePlane(in Vector3Wide edgeStart, in Vector3Wide edgeOffset, in Vector3Wide faceNormal, in Vector3Wide capsuleCenter, in Vector3Wide capsuleAxis,
            out Vector<float> intersection)
        {
            //t = edgeToCapsule * (edgePlaneNormal / ||edgePlaneNormal||) / (capsuleAxis * (edgePlaneNormal / ||edgePlaneNormal||))
            Vector3Wide.CrossWithoutOverlap(edgeOffset, faceNormal, out var edgePlaneNormal);
            Vector3Wide.Subtract(capsuleCenter, edgeStart, out var edgeToCapsule);
            Vector3Wide.Dot(edgeToCapsule, edgePlaneNormal, out var distance);
            Vector3Wide.Dot(capsuleAxis, edgePlaneNormal, out var velocity);
            //Note that near-zero denominators (parallel axes) result in a properly signed large finite value.
            intersection = Vector.ConditionalSelect(Vector.LessThan(velocity, Vector<float>.Zero), -distance, distance) / Vector.Max(new Vector<float>(1e-15f), Vector.Abs(velocity));

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(
            ref CapsuleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Convex2ContactManifoldWide manifold)
        {
            //Work in the triangle's local space to limit transformation requirements.
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var rB);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, rB, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);
            Vector3Wide.Add(b.A, b.B, out var localTriangleCenter);
            Vector3Wide.Add(b.C, localTriangleCenter, out localTriangleCenter);
            Vector3Wide.Scale(localTriangleCenter, new Vector<float>(1f / 3f), out localTriangleCenter);

            QuaternionWide.TransformUnitY(orientationA, out var worldCapsuleAxis);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(worldCapsuleAxis, rB, out var localCapsuleAxis);

            //There are four sources of separating axis for deep contact, where the capsule axis intersects the triangle:
            //capsuleAxis x AB
            //capsuleAxis x AC
            //capsuleAxis x BC
            //AB X AC
            //However, while those are sufficient for line segment-triangle testing, we are dealing with a capsule whose nonspeculative contacts require points of closest approach.
            //This adds more potential sources:
            //Capsule endpoint vs face (redundant with AB x AC)
            //Capsule endpoint vs edge
            //Capsule endpoint vs vertex
            //Capsule line vs face (redundant with either axis-edge or endpoint vs face)
            //Capsule line vs edge (redundant with axis-edge)
            //Capsule line vs vertex

            //While performing those additional tests explicitly would work for determining a minimal separating axis, we can consider them implicitly.
            //If all axis-edge tests compute the closest points between the edge and capsule axis bounded line segments, the triangle vertices and capsule endpoints are handled.

            Vector3Wide.Subtract(b.B, b.A, out var ab);
            Vector3Wide.Subtract(b.C, b.A, out var ac);

            Vector3Wide.CrossWithoutOverlap(ab, ac, out var abxac);
            Vector3Wide.LengthSquared(abxac, out var faceNormalLengthSquared);
            Vector3Wide.Scale(abxac, Vector<float>.One / Vector.SquareRoot(faceNormalLengthSquared), out var faceNormal);

            Vector3Wide.Subtract(localOffsetA, localTriangleCenter, out var triangleCenterToCapsuleCenter);
            //The depth along the face normal is unaffected by the triangle's extent- the triangle has no extent along its own normal. But the capsule does.
            Vector3Wide.Dot(faceNormal, localCapsuleAxis, out var nDotAxis);
            Vector3Wide.Dot(faceNormal, triangleCenterToCapsuleCenter, out var capsuleOffsetAlongNormal);
            //Note that capsuleOffsetAlongNormal may be negative when the capsule's center is on the non colliding side of the triangle.
            //That's fine- either the edge cases will produce better results, or no contacts will be generated at all.
            var faceDepth = a.HalfLength * Vector.Abs(nDotAxis) - capsuleOffsetAlongNormal;

            TestEdge(b, localTriangleCenter, faceNormal, b.A, ab, localOffsetA, localCapsuleAxis, a.HalfLength,
                out var edgeDirection, out var ta, out var tb, out var bMin, out var bMax, out var edgeDepth, out var edgeNormal);
            TestEdge(b, localTriangleCenter, faceNormal, b.A, ac, localOffsetA, localCapsuleAxis, a.HalfLength,
                out var edgeDirectionCandidate, out var taCandidate, out var tbCandidate, out var bMinCandidate, out var bMaxCandidate, out var edgeDepthCandidate, out var edgeNormalCandidate);
            var useAC = Vector.LessThan(edgeDepthCandidate, edgeDepth);
            Vector3Wide.ConditionalSelect(useAC, edgeDirectionCandidate, edgeDirection, out edgeDirection);
            Vector3Wide.ConditionalSelect(useAC, edgeNormalCandidate, edgeNormal, out edgeNormal);
            ta = Vector.ConditionalSelect(useAC, taCandidate, ta);
            tb = Vector.ConditionalSelect(useAC, tbCandidate, tb);
            bMin = Vector.ConditionalSelect(useAC, bMinCandidate, bMin);
            bMax = Vector.ConditionalSelect(useAC, bMaxCandidate, bMax);
            edgeDepth = Vector.Min(edgeDepthCandidate, edgeDepth);

            Vector3Wide.Subtract(b.C, b.B, out var bc);
            TestEdge(b, localTriangleCenter, faceNormal, b.B, bc, localOffsetA, localCapsuleAxis, a.HalfLength,
                out edgeDirectionCandidate, out taCandidate, out tbCandidate, out bMinCandidate, out bMaxCandidate, out edgeDepthCandidate, out edgeNormalCandidate);
            var useBC = Vector.LessThan(edgeDepthCandidate, edgeDepth);
            Vector3Wide.ConditionalSelect(useBC, b.B, b.A, out var edgeStart);
            Vector3Wide.ConditionalSelect(useBC, edgeDirectionCandidate, edgeDirection, out edgeDirection);
            Vector3Wide.ConditionalSelect(useBC, edgeNormalCandidate, edgeNormal, out edgeNormal);
            ta = Vector.ConditionalSelect(useAC, taCandidate, ta);
            tb = Vector.ConditionalSelect(useBC, tbCandidate, tb);
            bMin = Vector.ConditionalSelect(useBC, bMinCandidate, bMin);
            bMax = Vector.ConditionalSelect(useBC, bMaxCandidate, bMax);
            edgeDepth = Vector.Min(edgeDepthCandidate, edgeDepth);

            var depth = Vector.Min(edgeDepth, faceDepth);
            if (Vector.EqualsAll(Vector.LessThan(depth, -speculativeMargin), new Vector<int>(-1)))
            {
                //There are no contacts in any lane, so we can just skip the rest.
                manifold.Contact0Exists = Vector<int>.Zero;
                manifold.Contact1Exists = Vector<int>.Zero;
                return;
            }
            Vector3Wide localNormal;
            Vector3Wide b0, b1;
            Vector<int> contactCount;
            var useEdge = Vector.LessThan(edgeDepth, faceDepth);
            if (Vector.EqualsAny(useEdge, new Vector<int>(-1)))
            {
                //At least one of the paths uses edges, so go ahead and create all edge contact related information.
                //Borrowing from capsule-capsule again:
                //In the event that the two capsule axes are coplanar, we accept the whole interval as a source of contact.
                //As the axes drift away from coplanarity, the accepted interval rapidly narrows to zero length, centered on ta and tb.
                //We rate the degree of coplanarity based on the angle between the capsule axis and the plane defined by the box edge and contact normal:
                //sin(angle) = dot(da, (db x normal)/||db x normal||)
                //Finally, note that we are dealing with extremely small angles, and for small angles sin(angle) ~= angle,
                //and also that fade behavior is completely arbitrary, so we can directly use squared angle without any concern.
                //angle^2 ~= dot(da, (db x normal))^2 / ||db x normal||^2
                //Note that if ||db x normal|| is the zero, then any da should be accepted as being coplanar because there is no restriction. ConditionalSelect away the discontinuity.
                Vector3Wide.CrossWithoutOverlap(edgeDirection, edgeNormal, out var planeNormal);
                Vector3Wide.LengthSquared(planeNormal, out var planeNormalLengthSquared);
                Vector3Wide.Dot(localCapsuleAxis, planeNormal, out var squaredAngle);
                squaredAngle = Vector.ConditionalSelect(Vector.LessThan(planeNormalLengthSquared, new Vector<float>(1e-10f)), Vector<float>.Zero, squaredAngle / planeNormalLengthSquared);

                //Convert the squared angle to a lerp parameter. For squared angle from 0 to lowerThreshold, we should use the full interval (1). From lowerThreshold to upperThreshold, lerp to 0.
                const float lowerThresholdAngle = 0.001f;
                const float upperThresholdAngle = 0.005f;
                const float lowerThreshold = lowerThresholdAngle * lowerThresholdAngle;
                const float upperThreshold = upperThresholdAngle * upperThresholdAngle;
                var intervalWeight = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (new Vector<float>(upperThreshold) - squaredAngle) * new Vector<float>(1f / (upperThreshold - lowerThreshold))));
                //Note that we're working with tb, the edge parameter, rather than the capsule value. Triangle-related contacts must be on the triangle because of boundary smoothing.
                var weightedTb = tb - tb * intervalWeight;
                bMin = intervalWeight * bMin + weightedTb;
                bMax = intervalWeight * bMax + weightedTb;

                Vector3Wide.Scale(edgeDirection, bMin, out b0);
                Vector3Wide.Add(b0, edgeStart, out b0);
                Vector3Wide.Scale(edgeDirection, bMax, out b1);
                Vector3Wide.Add(b1, edgeStart, out b1);

                Vector3Wide.ConditionalSelect(useEdge, edgeNormal, faceNormal, out localNormal);
                contactCount = Vector.ConditionalSelect(useEdge, Vector.ConditionalSelect(Vector.GreaterThan(bMax, bMin), new Vector<int>(2), Vector<int>.One), Vector<int>.Zero);
            }
            else
            {
                //No edges are used; all contacts must be face contacts.
                localNormal = faceNormal;
                contactCount = Vector<int>.Zero;
            }

            Vector3Wide.Dot(localNormal, faceNormal, out var localNormalDotFaceNormal);
            if(Vector.LessThanOrEqualAll(localNormalDotFaceNormal, Vector<float>.Zero))
            {
                //All contact normals are on the back of the triangle, so we can immediately quit.
                manifold.Contact0Exists = Vector<int>.Zero;
                manifold.Contact1Exists = Vector<int>.Zero;
                return;
            }

            //Any edge contributions are now stored in the b0, b1, localNormal, and depth fields. 
            //1) If face contact won (no edges contributed any contacts), then generate two contacts by clipping the capsule axis against the triangle edge planes.
            //(Clipping is used so that the results can be shared by #2, and in case where the capsule axis is parallel to the face surface so an edge that 'should' have won, didn't.)
            //2) If an edge contact one and only generated one contact, then try to generate one additional face contact by
            //projecting a capsule endpoint onto the triangle if it's on the colliding side and is within the triangle's bounds.
            //The bounds check can share the clipping done for face contacts.
            //3) If an edge contact has generated two contacts, then no additional contacts are required.

            if (Vector.LessThanOrEqualAny(contactCount, Vector<int>.One))
            {
                ClipAgainstEdgePlane(b.A, ab, faceNormal, localOffsetA, localCapsuleAxis, out var abIntersection);
                ClipAgainstEdgePlane(b.A, ac, faceNormal, localOffsetA, localCapsuleAxis, out var acIntersection);
                ClipAgainstEdgePlane(b.B, bc, faceNormal, localOffsetA, localCapsuleAxis, out var bcIntersection);
                var triangleIntervalMin = Vector.Min(abIntersection, Vector.Min(acIntersection, bcIntersection));
                var triangleIntervalMax = Vector.Max(abIntersection, Vector.Max(acIntersection, bcIntersection));

                var overlapIntervalMin = Vector.Max(triangleIntervalMin, -a.HalfLength);
                var overlapIntervalMax = Vector.Min(triangleIntervalMax, a.HalfLength);
                Vector3Wide.Scale(localCapsuleAxis, overlapIntervalMin, out var clippedOnA0);
                Vector3Wide.Add(clippedOnA0, localOffsetA, out clippedOnA0);
                Vector3Wide.Dot(clippedOnA0, faceNormal, out var nA0);
                Vector3Wide.Dot(localTriangleCenter, faceNormal, out var trianglePlaneOffset);
                var distanceAlongNormalA0 = nA0 - trianglePlaneOffset;
                Vector3Wide.Scale(faceNormal, distanceAlongNormalA0, out var toRemoveA0);
                Vector3Wide.Subtract(clippedOnA0, toRemoveA0, out var faceCandidate0);

                Vector3Wide.Scale(localCapsuleAxis, overlapIntervalMax, out var clippedOnA1);
                Vector3Wide.Add(clippedOnA1, localOffsetA, out clippedOnA1);
                Vector3Wide.Dot(clippedOnA1, faceNormal, out var nA1);
                var distanceAlongNormalA1 = nA1 - trianglePlaneOffset;
                Vector3Wide.Scale(faceNormal, distanceAlongNormalA1, out var toRemoveA1);
                Vector3Wide.Subtract(clippedOnA1, toRemoveA1, out var faceCandidate1);

                //Automatically accept the two candidates if there are no contacts.
                var noEdgeContacts = Vector.Equals(contactCount, Vector<int>.Zero);
                var intervalExists = Vector.GreaterThanOrEqual(overlapIntervalMax, overlapIntervalMin);
                Vector3Wide.ConditionalSelect(noEdgeContacts, faceCandidate0, b0, out b0);
                Vector3Wide.ConditionalSelect(noEdgeContacts, faceCandidate1, b1, out b1);
                contactCount = Vector.ConditionalSelect(Vector.BitwiseAnd(noEdgeContacts, intervalExists), new Vector<int>(2), contactCount);

                //If there's one edge contact, only one of the two face contacts should be accepted.
                //Note that it's likely that one of the two clipped interval endpoints will end up very close to the edge contact, so picking that one would be a waste.
                //Choose the endpoint based on which clipped interval endpoint is further from the edge contact on the capsule's axis.
                var useFaceContact1ForSecondContact = Vector.GreaterThan(Vector.Abs(overlapIntervalMax - ta), Vector.Abs(overlapIntervalMin - ta));
                Vector3Wide.ConditionalSelect(useFaceContact1ForSecondContact, faceCandidate1, faceCandidate0, out var secondContactCandidate);
                var secondContactDistanceAlongNormal = Vector.ConditionalSelect(useFaceContact1ForSecondContact, distanceAlongNormalA1, distanceAlongNormalA0);
                //To actually use this as the second contact, these conditions must be met:
                //1) The interval is valid (max > min).
                //2) The number of contacts == 1.
                //3) The candidate is above the triangle.
                var useCandidateForSecondContact = Vector.BitwiseAnd(intervalExists, 
                    Vector.BitwiseAnd(Vector.Equals(contactCount, Vector<int>.One), Vector.GreaterThan(secondContactDistanceAlongNormal, Vector<float>.Zero)));                
                Vector3Wide.ConditionalSelect(useCandidateForSecondContact, secondContactCandidate, b1, out b1);
                contactCount = Vector.ConditionalSelect(useCandidateForSecondContact, new Vector<int>(2), contactCount);
            }

            //Measure the depths for the two contacts by seeing how far they are along the normal from the capsule.
            //We'll do this by creating a plane positioned at the capsule center with a normal pointing against the contact normal.
            //capsuleAxisPlaneNormal = (N x capsuleAxis) x capsuleAxis
            //The depths are derived from testing a ray against that plane, starting from the contact position and going along the contact normal.
            Vector3Wide.CrossWithoutOverlap(faceNormal, localCapsuleAxis, out var nxa);
            Vector3Wide.CrossWithoutOverlap(nxa, localCapsuleAxis, out var capsuleAxisPlaneNormal);

            Vector3Wide.Subtract(localOffsetA, b0, out var offset0);
            Vector3Wide.Dot(capsuleAxisPlaneNormal, offset0, out var planeDistance0);
            Vector3Wide.Subtract(localOffsetA, b1, out var offset1);
            Vector3Wide.Dot(capsuleAxisPlaneNormal, offset1, out var planeDistance1);
            Vector3Wide.Dot(faceNormal, capsuleAxisPlaneNormal, out var velocity);
            var inverseVelocity = Vector<float>.One / velocity;
            var separation0 = planeDistance0 * inverseVelocity;
            var separation1 = planeDistance1 * inverseVelocity;
            manifold.Depth0 = a.Radius - separation0;
            manifold.Depth1 = a.Radius - separation1;

            //Transform contact positions into world space rotation, measured as offsets from the capsule (object A).
            Matrix3x3Wide.Transform(b0, rB, out var contact0RelativeToB);
            Vector3Wide.Add(contact0RelativeToB, offsetB, out manifold.OffsetA0);
            Matrix3x3Wide.Transform(b1, rB, out var contact1RelativeToB);
            Vector3Wide.Add(contact1RelativeToB, offsetB, out manifold.OffsetA1);
            Matrix3x3Wide.Transform(localNormal, rB, out manifold.Normal);

            //If the normal we found points away from the triangle normal, then it it's hitting the wrong side and should be ignored. (Note that we had an early out for this earlier.)
            contactCount = Vector.ConditionalSelect(Vector.GreaterThanOrEqual(localNormalDotFaceNormal, Vector<float>.Zero), contactCount, Vector<int>.Zero);
            var depthThreshold = -speculativeMargin;
            manifold.Contact0Exists = Vector.BitwiseAnd(Vector.GreaterThan(manifold.Depth0, depthThreshold), Vector.GreaterThan(contactCount, Vector<int>.Zero));
            manifold.Contact1Exists = Vector.BitwiseAnd(Vector.GreaterThan(manifold.Depth1, depthThreshold), Vector.GreaterThan(contactCount, Vector<int>.One));
        }


        public void Test(ref CapsuleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref CapsuleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }

    public class CapsuleTriangleCollisionTask : CollisionTask
    {
        public CapsuleTriangleCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(Capsule).TypeId;
            ShapeTypeIndexB = default(Triangle).TypeId;
        }


        //Every single collision task type will mirror this general layout.
        public unsafe override void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            ConvexCollisionTaskCommon.ExecuteBatch
                <TCallbacks,
                Capsule, CapsuleWide, Triangle, TriangleWide, TestPairWide<Capsule, CapsuleWide, Triangle, TriangleWide>,
                Convex2ContactManifoldWide, CapsuleTriangleTester>(ref batch, ref batcher);
        }
    }
}
