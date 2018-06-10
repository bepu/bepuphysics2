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
            ref Vector<float> bestDepth,
            out Vector3Wide b0, out Vector<float> depth0, out Vector3Wide b1, out Vector<float> depth1, out Vector3Wide normal, out Vector<int> contactCount)
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
            var ta = (daOffsetB - dbOffsetB * dadb) / Vector.Max(new Vector<float>(1e-15f), Vector<float>.One - dadb * dadb);
            //tb = ta * (da * db) - db * (b - a)
            var tb = ta * dadb - dbOffsetB;


            //We cannot simply clamp the ta and tb values to the line segments. Instead, project each line segment onto the other line segment, clamping against the target's interval.
            //That new clamped projected interval is the valid solution space on that line segment. We can clamp the t value by that interval to get the correctly bounded solution.
            //The projected intervals are:
            //B onto A: (0 or edgeLength) * (da * db) + da * offsetB
            //A onto B: +-AHalfLength * (da * db) - db * offsetB
            var aMin = Vector.Max(-capsuleHalfLength, Vector.Min(capsuleHalfLength, daOffsetB));
            var aMax = Vector.Min(capsuleHalfLength, Vector.Max(-capsuleHalfLength, daOffsetB + edgeLength * dadb));
            var aOntoBOffset = capsuleHalfLength * Vector.Abs(dadb);
            var bMin = Vector.Max(Vector<float>.Zero, Vector.Min(edgeLength, -aOntoBOffset - dbOffsetB));
            var bMax = Vector.Min(edgeLength, Vector.Max(Vector<float>.Zero, aOntoBOffset - dbOffsetB));
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
            var depth = extremeOnTriangle - extremeOnCapsule;

            //We now have a valid normal and depth, plus the closest points associated with it.
            var shouldUseEdgeResult = Vector.LessThan(depth, bestDepth);
            bestDepth = Vector.Min(depth, bestDepth);
            if (Vector.EqualsAny(shouldUseEdgeResult, new Vector<int>(-1)))
            {
                //TODO: Consider moving all this coplanarity stuff out so that it's only done once. Would require caching more information, but this is sufficiently complicated
                //that it would almost certainly be worth it.

                //Borrowing from capsule-capsule again:
                //In the event that the two capsule axes are coplanar, we accept the whole interval as a source of contact.
                //As the axes drift away from coplanarity, the accepted interval rapidly narrows to zero length, centered on ta and tb.
                //We rate the degree of coplanarity based on the angle between the capsule axis and the plane defined by the box edge and contact normal:
                //sin(angle) = dot(da, (db x normal)/||db x normal||)
                //Finally, note that we are dealing with extremely small angles, and for small angles sin(angle) ~= angle,
                //and also that fade behavior is completely arbitrary, so we can directly use squared angle without any concern.
                //angle^2 ~= dot(da, (db x normal))^2 / ||db x normal||^2
                //Note that if ||db x normal|| is the zero, then any da should be accepted as being coplanar because there is no restriction. ConditionalSelect away the discontinuity.
                Vector3Wide.CrossWithoutOverlap(db, normal, out var planeNormal);
                Vector3Wide.LengthSquared(planeNormal, out var planeNormalLengthSquared);
                Vector3Wide.Dot(capsuleAxis, planeNormal, out var squaredAngle);
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

                Vector3Wide.Scale(db, bMin, out var contactPosition0);
                Vector3Wide.Add(contactPosition0, edgeStart, out contactPosition0);
                Vector3Wide.Scale(db, bMax, out var contactPosition1);
                Vector3Wide.Add(contactPosition1, edgeStart, out contactPosition1);
                //In the coplanar case, the second contact needs to have a reasonable depth. The first contact should have the depth we already computed.
                //However, we don't immediately know which interval endpoint is the nearer one, so we'll do both.
                //Unproject the contact positions back onto the capsule axis.
                //dot(-offsetB + ta0 * da, db) = tb0
                //ta0 = (tb0 + dbOffsetB) / dadb
                //distance0 = dot(b0 - (ta0 * da - offsetB), normal)
                //Note potential division by zero. In that case, just use the main depth.
                var inverseDadb = Vector<float>.One / dadb;
                var ta0 = (bMin + dbOffsetB) * inverseDadb;
                Vector3Wide.Scale(capsuleAxis, ta0, out var unprojected0);
                Vector3Wide.Add(unprojected0, capsuleCenter, out unprojected0);
                Vector3Wide.Subtract(contactPosition0, unprojected0, out var offset0);
                Vector3Wide.Dot(offset0, normal, out var separation0);

                var ta1 = (bMax + dbOffsetB) * inverseDadb;
                Vector3Wide.Scale(capsuleAxis, ta1, out var unprojected1);
                Vector3Wide.Add(unprojected1, capsuleCenter, out unprojected1);
                Vector3Wide.Subtract(contactPosition1, unprojected1, out var offset1);
                Vector3Wide.Dot(offset1, normal, out var separation1);

                var perpendicular = Vector.LessThan(Vector.Abs(dadb), new Vector<float>(1e-7f));
                var depthCandidate0 = Vector.ConditionalSelect(perpendicular, depth, -separation0);
                var depthCandidate1 = Vector.ConditionalSelect(perpendicular, depth, -separation1);

                //Put the closer contact into the first slot.
                var use0AsFirst = Vector.LessThan(depthCandidate0, depthCandidate1);
                depth0 = Vector.ConditionalSelect(use0AsFirst, depthCandidate0, depthCandidate1);
                Vector3Wide.ConditionalSelect(use0AsFirst, contactPosition0, contactPosition1, out b0);
                depth1 = Vector.ConditionalSelect(use0AsFirst, depthCandidate1, depthCandidate0);
                Vector3Wide.ConditionalSelect(use0AsFirst, contactPosition1, contactPosition0, out b1);

                contactCount = Vector.ConditionalSelect(shouldUseEdgeResult,
                    Vector.ConditionalSelect(
                        Vector.GreaterThan(bMax - bMin, Vector<float>.Zero), new Vector<int>(2), Vector<int>.One),
                    Vector<int>.Zero);
            }
            else
            {
                contactCount = Vector<int>.Zero;
            }

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Select(ref Vector<float> depth0, ref Vector3Wide b0, ref Vector<float> depth1, ref Vector3Wide b1, ref Vector3Wide edgeNormal, ref Vector<int> edgeContactCount,
            in Vector<float> depth0Candidate, in Vector3Wide b0Candidate, in Vector<float> depth1Candidate, in Vector3Wide b1Candidate,
            in Vector3Wide edgeNormalCandidate, in Vector<int> edgeContactCountCandidate)
        {
            var useCandidate = Vector.BitwiseAnd(Vector.GreaterThan(edgeContactCountCandidate, Vector<int>.Zero), Vector.LessThan(depth0Candidate, depth0));
            depth0 = Vector.ConditionalSelect(useCandidate, depth0Candidate, depth0);
            Vector3Wide.ConditionalSelect(useCandidate, b0Candidate, b0, out b0);
            depth1 = Vector.ConditionalSelect(useCandidate, depth1Candidate, depth1);
            Vector3Wide.ConditionalSelect(useCandidate, b1Candidate, b1, out b1);
            Vector3Wide.ConditionalSelect(useCandidate, edgeNormalCandidate, edgeNormal, out edgeNormal);
            Vector.ConditionalSelect(useCandidate, edgeContactCountCandidate, edgeContactCount);
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

            var bestDepth = faceDepth;
            TestEdge(b, localTriangleCenter, faceNormal, b.A, ab, localOffsetA, localCapsuleAxis, a.HalfLength, ref bestDepth,
                out var b0, out var depth0, out var b1, out var depth1, out var edgeNormal, out var edgeContactCount);
            TestEdge(b, localTriangleCenter, faceNormal, b.A, ac, localOffsetA, localCapsuleAxis, a.HalfLength, ref bestDepth,
                out var b0Candidate, out var depth0Candidate, out var b1Candidate, out var depth1Candidate, out var edgeNormalCandidate, out var edgeContactCountCandidate);
            Select(ref depth0, ref b0, ref depth1, ref b1, ref edgeNormal, ref edgeContactCount,
                depth0Candidate, b0Candidate, depth1Candidate, b1Candidate, edgeNormalCandidate, edgeContactCountCandidate);
            Vector3Wide.Subtract(b.C, b.B, out var bc);
            TestEdge(b, localTriangleCenter, faceNormal, b.B, bc, localOffsetA, localCapsuleAxis, a.HalfLength, ref bestDepth,
                out b0Candidate, out depth0Candidate, out b1Candidate, out depth1Candidate, out edgeNormalCandidate, out edgeContactCountCandidate);
            Select(ref depth0, ref b0, ref depth1, ref b1, ref edgeNormal, ref edgeContactCount,
                depth0Candidate, b0Candidate, depth1Candidate, b1Candidate, edgeNormalCandidate, edgeContactCountCandidate);

            var allowFaceNormal = Vector.GreaterThanOrEqual(capsuleOffsetAlongNormal, Vector<float>.Zero);
            var noEdgeContacts = Vector.Equals(edgeContactCount, Vector<int>.Zero);
            var noContacts = Vector.AndNot(noEdgeContacts, allowFaceNormal);
            if (Vector.EqualsAll(Vector.BitwiseOr(noContacts, Vector.LessThan(bestDepth, -speculativeMargin)), new Vector<int>(-1)))
            {
                //There are no contacts in any lane, so we can just skip the rest.
                manifold.Contact0Exists = Vector<int>.Zero;
                manifold.Contact1Exists = Vector<int>.Zero;
                return;
            }
            var useFaceNormal = Vector.BitwiseAnd(allowFaceNormal, noEdgeContacts);
            Vector3Wide.ConditionalSelect(useFaceNormal, faceNormal, edgeNormal, out var localNormal);


            //Three possible cases: 0, 1, or 2 edge contacts have been created.
            //In the event that 0 edge contacts have been created, the faceDepth must have been the best.
            //Try to generate two contacts by clipping the capsule axis against the triangle edge planes, then projecting them onto the triangle.
            //(Note that clipping is only required in the case where the capsule axis is perpendicular to the triangle normal because we can't rely an edge test to win numerically.)

            //If there is one edge contact, then try creating a helper contact by projecting capsule endpoints onto the triangle plane.
            //(This can be shared with the clipping above- the clipped interval spans all valid source points.)
            //If the endpoint starts above the triangle and the projection is inside the triangle, use it as the second contact.

            //If there are two edge contacts, just use them.
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
