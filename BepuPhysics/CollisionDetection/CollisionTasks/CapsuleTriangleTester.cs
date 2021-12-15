using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CapsuleTriangleTester : IPairTester<CapsuleWide, TriangleWide, Convex2ContactManifoldWide>
    {
        public int BatchSize => 32;

        public static void TestEdge(in TriangleWide triangle, in Vector3Wide triangleNormal,
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
            Vector3Wide.Scale(edgeOffset, Vector<float>.One / edgeLength, out edgeDirection);
            Vector3Wide.Subtract(edgeStart, capsuleCenter, out var offsetB);
            Vector3Wide.Dot(capsuleAxis, offsetB, out var daOffsetB);
            Vector3Wide.Dot(edgeDirection, offsetB, out var dbOffsetB);
            Vector3Wide.Dot(capsuleAxis, edgeDirection, out var dadb);
            //Note potential division by zero when the axes are parallel. Arbitrarily clamp; near zero values will instead produce extreme values which get clamped to reasonable results.
            ta = (daOffsetB - dbOffsetB * dadb) / Vector.Max(new Vector<float>(1e-15f), Vector<float>.One - dadb * dadb);
            //tb = ta * (da * db) - db * (b - a)
            tb = ta * dadb - dbOffsetB;

            //We cannot simply clamp the ta and tb values to the line segments. Instead, project each line segment onto the other line segment, clamping against the target's interval.
            //That new clamped projected interval is the valid solution space on that line segment. We can clamp the t value by that interval to get the correctly bounded solution.
            //The projected intervals are:
            //B onto A: (0 or edgeLength) * (da * db) + da * offsetB
            //A onto B: +-AHalfLength * (da * db) - db * offsetB
            var ta0 = Vector.Max(-capsuleHalfLength, Vector.Min(capsuleHalfLength, daOffsetB));
            var ta1 = Vector.Min(capsuleHalfLength, Vector.Max(-capsuleHalfLength, daOffsetB + edgeLength * dadb));
            var aMin = Vector.Min(ta0, ta1);
            var aMax = Vector.Max(ta0, ta1);
            var aOntoBOffset = capsuleHalfLength * Vector.Abs(dadb);
            bMin = Vector.Max(Vector<float>.Zero, Vector.Min(edgeLength, -aOntoBOffset - dbOffsetB));
            bMax = Vector.Min(edgeLength, Vector.Max(Vector<float>.Zero, aOntoBOffset - dbOffsetB));
            ta = Vector.Min(Vector.Max(ta, aMin), aMax);
            tb = Vector.Min(Vector.Max(tb, bMin), bMax);

            Vector3Wide.Scale(capsuleAxis, ta, out var closestPointOnCapsule);
            Vector3Wide.Add(closestPointOnCapsule, capsuleCenter, out closestPointOnCapsule);
            Vector3Wide.Scale(edgeDirection, tb, out var closestPointOnEdge);
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
            Vector3Wide.CrossWithoutOverlap(triangleNormal, edgeOffset, out var secondFallbackNormal);
            Vector3Wide.LengthSquared(fallbackNormal, out var secondFallbackNormalLengthSquared);
            var useSecondFallbackNormal = Vector.LessThan(normalLengthSquared, new Vector<float>(1e-15f));
            Vector3Wide.ConditionalSelect(useSecondFallbackNormal, secondFallbackNormal, normal, out normal);
            normalLengthSquared = Vector.ConditionalSelect(useSecondFallbackNormal, secondFallbackNormalLengthSquared, normalLengthSquared);
            //Note that we DO NOT 'calibrate' the normal here! The edge winding should avoid the need for any calibration,
            //and attempting to calibrate this normal can actually result in normals pointing into the triangle face.
            //That can cause backfaces to generate contacts incorrectly.
            Vector3Wide.Scale(normal, Vector<float>.One / Vector.SquareRoot(normalLengthSquared), out normal);

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
        public static void ClipAgainstEdgePlane(in Vector3Wide edgeStart, in Vector3Wide edgeOffset, in Vector3Wide faceNormal, in Vector3Wide capsuleCenter, in Vector3Wide capsuleAxis,
            out Vector<float> entry, out Vector<float> exit)
        {
            //t = -edgeToCapsule * (edgePlaneNormal / ||edgePlaneNormal||) / (capsuleAxis * (edgePlaneNormal / ||edgePlaneNormal||))
            Vector3Wide.CrossWithoutOverlap(faceNormal, edgeOffset, out var edgePlaneNormal);
            Vector3Wide.Subtract(capsuleCenter, edgeStart, out var edgeToCapsule);
            Vector3Wide.Dot(edgeToCapsule, edgePlaneNormal, out var distance);
            Vector3Wide.Dot(capsuleAxis, edgePlaneNormal, out var velocity);
            //Note that near-zero denominators (parallel axes) result in a properly signed large finite value.
            var velocityIsPositive = Vector.GreaterThan(velocity, Vector<float>.Zero);
            var t = Vector.ConditionalSelect(velocityIsPositive, -distance, distance) / Vector.Max(new Vector<float>(1e-15f), Vector.Abs(velocity));
            //The final interval is going to be max(entryAB, entryBC, entryCA) to min(exitAB, exitBC, exitCA). 
            //An intersection is considered an 'entry' if the ray direction opposes the plane normal.
            entry = Vector.ConditionalSelect(velocityIsPositive, new Vector<float>(-float.MaxValue), t);
            exit = Vector.ConditionalSelect(velocityIsPositive, t, new Vector<float>(float.MaxValue));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(
            ref CapsuleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount,
            out Convex2ContactManifoldWide manifold)
        {
            Unsafe.SkipInit(out manifold);
            //Work in the triangle's local space to limit transformation requirements.
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var rB);
            Vector3Wide.Add(b.A, b.B, out var localTriangleCenter);
            Vector3Wide.Add(b.C, localTriangleCenter, out localTriangleCenter);
            Vector3Wide.Scale(localTriangleCenter, new Vector<float>(1f / 3f), out localTriangleCenter);
            //Note that we're not just using the same origin as the triangle, but using the center of the triangle as the origin of the working space.
            //This helps avoid some numerical issues.
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, rB, out var localOffsetB);
            Vector3Wide.Add(localOffsetB, localTriangleCenter, out localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);
            TriangleWide triangle;
            Vector3Wide.Subtract(b.A, localTriangleCenter, out triangle.A);
            Vector3Wide.Subtract(b.B, localTriangleCenter, out triangle.B);
            Vector3Wide.Subtract(b.C, localTriangleCenter, out triangle.C);

            var worldCapsuleAxis = QuaternionWide.TransformUnitY(orientationA);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(worldCapsuleAxis, rB, out var localCapsuleAxis);

            //There are four sources of separating axis for deep contact, where the capsule axis intersects the triangle:
            //capsuleAxis x AB
            //capsuleAxis x AC
            //capsuleAxis x BC
            //AC X AB
            //However, while those are sufficient for line segment-triangle testing, we are dealing with a capsule whose nonspeculative contacts require points of closest approach.
            //This adds more potential sources:
            //Capsule endpoint vs face (redundant with AC x AB)
            //Capsule endpoint vs edge
            //Capsule endpoint vs vertex
            //Capsule line vs face (redundant with either axis-edge or endpoint vs face)
            //Capsule line vs edge (redundant with axis-edge)
            //Capsule line vs vertex

            //While performing those additional tests explicitly would work for determining a minimal separating axis, we can consider them implicitly.
            //If all axis-edge tests compute the closest points between the edge and capsule axis bounded line segments, the triangle vertices and capsule endpoints are handled.

            Vector3Wide.Subtract(b.C, b.A, out var ac);
            Vector3Wide.Subtract(b.B, b.A, out var ab);

            Vector3Wide.CrossWithoutOverlap(ac, ab, out var acxab);
            Vector3Wide.Length(acxab, out var faceNormalLength);
            Vector3Wide.Scale(acxab, Vector<float>.One / faceNormalLength, out var faceNormal);

            //The depth along the face normal is unaffected by the triangle's extent- the triangle has no extent along its own normal. But the capsule does.
            Vector3Wide.Dot(faceNormal, localCapsuleAxis, out var nDotAxis);
            //capsuleOffsetAlongNormal = dot(localOffsetA - triangleCenter, faceNormal), and we've centered our working space on the triangle center.
            Vector3Wide.Dot(faceNormal, localOffsetA, out var capsuleOffsetAlongNormal);
            //Note that capsuleOffsetAlongNormal may be negative when the capsule's center is on the non colliding side of the triangle.
            //That's fine- either the edge cases will produce better results, or no contacts will be generated at all.
            var faceDepth = a.HalfLength * Vector.Abs(nDotAxis) - capsuleOffsetAlongNormal;

            TestEdge(triangle, faceNormal, triangle.A, ab, localOffsetA, localCapsuleAxis, a.HalfLength,
                out var edgeDirection, out var ta, out var tb, out var bMin, out var bMax, out var edgeDepth, out var edgeNormal);
            TestEdge(triangle, faceNormal, triangle.A, ac, localOffsetA, localCapsuleAxis, a.HalfLength,
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
            TestEdge(triangle, faceNormal, triangle.B, bc, localOffsetA, localCapsuleAxis, a.HalfLength,
                out edgeDirectionCandidate, out taCandidate, out tbCandidate, out bMinCandidate, out bMaxCandidate, out edgeDepthCandidate, out edgeNormalCandidate);
            var useBC = Vector.LessThan(edgeDepthCandidate, edgeDepth);
            Vector3Wide.ConditionalSelect(useBC, triangle.B, triangle.A, out var edgeStart);
            Vector3Wide.ConditionalSelect(useBC, edgeDirectionCandidate, edgeDirection, out edgeDirection);
            Vector3Wide.ConditionalSelect(useBC, edgeNormalCandidate, edgeNormal, out edgeNormal);
            ta = Vector.ConditionalSelect(useBC, taCandidate, ta);
            tb = Vector.ConditionalSelect(useBC, tbCandidate, tb);
            bMin = Vector.ConditionalSelect(useBC, bMinCandidate, bMin);
            bMax = Vector.ConditionalSelect(useBC, bMaxCandidate, bMax);
            edgeDepth = Vector.Min(edgeDepthCandidate, edgeDepth);

            var depth = Vector.Min(edgeDepth, faceDepth);
            var useEdge = Vector.LessThan(edgeDepth, faceDepth);
            Vector3Wide.ConditionalSelect(useEdge, edgeNormal, faceNormal, out var localNormal);
            Vector3Wide.Dot(localNormal, faceNormal, out var localNormalDotFaceNormal);
            var collidingWithSolidSide = Vector.GreaterThanOrEqual(localNormalDotFaceNormal, new Vector<float>(TriangleWide.BackfaceNormalDotRejectionThreshold));
            var activeLanes = BundleIndexing.CreateMaskForCountInBundle(pairCount);
            TriangleWide.ComputeNondegenerateTriangleMask(ab, ac, faceNormalLength, out _, out var nondegenerateMask);
            var negativeMargin = -speculativeMargin;
            var allowContacts = Vector.BitwiseAnd(Vector.BitwiseAnd(Vector.GreaterThanOrEqual(depth + a.Radius, negativeMargin), activeLanes), Vector.BitwiseAnd(collidingWithSolidSide, nondegenerateMask));
            if (Vector.EqualsAll(allowContacts, Vector<int>.Zero))
            {
                //All contact normals are on the back of the triangle or the distance is too large for the margin, so we can immediately quit.
                manifold.Contact0Exists = Vector<int>.Zero;
                manifold.Contact1Exists = Vector<int>.Zero;
                return;
            }
            Unsafe.SkipInit(out Vector3Wide b0);
            Unsafe.SkipInit(out Vector3Wide b1);
            Vector<int> contactCount;
            useEdge = Vector.BitwiseAnd(useEdge, allowContacts);
            if (Vector.LessThanAny(useEdge, Vector<int>.Zero))
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
                Vector3Wide.Dot(localCapsuleAxis, planeNormal, out var numeratorUnsquared);
                var squaredAngle = Vector.ConditionalSelect(Vector.LessThan(planeNormalLengthSquared, new Vector<float>(1e-10f)), Vector<float>.Zero, numeratorUnsquared * numeratorUnsquared / planeNormalLengthSquared);

                //Convert the squared angle to a lerp parameter. For squared angle from 0 to lowerThreshold, we should use the full interval (1). From lowerThreshold to upperThreshold, lerp to 0.
                const float lowerThresholdAngle = 0.01f;
                const float upperThresholdAngle = 0.05f;
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
                contactCount = Vector.ConditionalSelect(useEdge, Vector.ConditionalSelect(Vector.GreaterThan(bMax, bMin), new Vector<int>(2), Vector<int>.One), Vector<int>.Zero);
            }
            else
            {
                //No edges are used; all contacts must be face contacts.
                contactCount = Vector<int>.Zero;
            }


            //Any edge contributions are now stored in the b0, b1, localNormal, and depth fields. 
            //1) If face contact won (no edges contributed any contacts), then generate two contacts by clipping the capsule axis against the triangle edge planes.
            //(Clipping is used so that the results can be shared by #2, and in case where the capsule axis is parallel to the face surface so an edge that 'should' have won, didn't.)
            //2) If an edge contact won and only generated one contact, then try to generate one additional face contact by
            //projecting a capsule endpoint onto the triangle if it's on the colliding side and is within the triangle's bounds.
            //The bounds check can share the clipping done for face contacts.
            //3) If an edge contact has generated two contacts, then no additional contacts are required.

            if (Vector.LessThanAny(Vector.BitwiseAnd(Vector.LessThanOrEqual(contactCount, Vector<int>.One), allowContacts), Vector<int>.Zero))
            {
                ClipAgainstEdgePlane(triangle.A, ab, faceNormal, localOffsetA, localCapsuleAxis, out var abEntry, out var abExit);
                ClipAgainstEdgePlane(triangle.B, bc, faceNormal, localOffsetA, localCapsuleAxis, out var bcEntry, out var bcExit);
                //Winding matters. ab, bc, ca.
                Vector3Wide.Negate(ac, out var ca);
                ClipAgainstEdgePlane(triangle.A, ca, faceNormal, localOffsetA, localCapsuleAxis, out var caEntry, out var caExit);
                var triangleIntervalMin = Vector.Max(abEntry, Vector.Max(bcEntry, caEntry));
                var triangleIntervalMax = Vector.Min(abExit, Vector.Min(bcExit, caExit));

                var negativeHalfLength = -a.HalfLength;
                var overlapIntervalMin = Vector.Max(triangleIntervalMin, negativeHalfLength);
                var overlapIntervalMax = Vector.Min(triangleIntervalMax, a.HalfLength);
                //We'll be clamping from both sides for the purposes of generating good face contacts, but that means the one contact case won't have a unilaterally clamped interval to work with.
                //So perform that test up front.
                var intervalIsValidForSecondContact = Vector.GreaterThanOrEqual(overlapIntervalMax, overlapIntervalMin);
                overlapIntervalMin = Vector.Min(overlapIntervalMin, a.HalfLength);
                overlapIntervalMax = Vector.Max(overlapIntervalMax, negativeHalfLength);
                Vector3Wide.Scale(localCapsuleAxis, overlapIntervalMin, out var clippedOnA0);
                Vector3Wide.Add(clippedOnA0, localOffsetA, out clippedOnA0);
                //distanceAlongNormalA0 = dot(a0, faceNormal) - dot(triangleCenter, faceNormal), and triangleCenter is zero since we're working locally to the triangle.
                Vector3Wide.Dot(clippedOnA0, faceNormal, out var distanceAlongNormalA0);
                Vector3Wide.Scale(faceNormal, distanceAlongNormalA0, out var toRemoveA0);
                Vector3Wide.Subtract(clippedOnA0, toRemoveA0, out var faceCandidate0);

                Vector3Wide.Scale(localCapsuleAxis, overlapIntervalMax, out var clippedOnA1);
                Vector3Wide.Add(clippedOnA1, localOffsetA, out clippedOnA1);
                //distanceAlongNormalA1 = dot(a1, faceNormal) - dot(triangleCenter, faceNormal), and triangleCenter is zero since we're working locally to the triangle.
                Vector3Wide.Dot(clippedOnA1, faceNormal, out var distanceAlongNormalA1);
                Vector3Wide.Scale(faceNormal, distanceAlongNormalA1, out var toRemoveA1);
                Vector3Wide.Subtract(clippedOnA1, toRemoveA1, out var faceCandidate1);

                //Automatically accept the two candidates if there are no contacts.
                //Note that we accept it even if the interval is negative length- if there are no edge contacts, it means that the face normal was the best option.
                //If the face normal is the best option, there MUST be face contacts. Any disagreement on the part of clipping is simply a matter of numerical error.
                //As a side effect of being mere numerical error, the distance between the projected points and the triangle should tend to be near epsilon.
                //One exception: if the capsule's center is on the backside of the triangle, no contacts should be created. We don't want contacts to 'pull' objects through-
                //that would create frequent nasty situations in complex meshes.
                var noEdgeContacts = Vector.Equals(contactCount, Vector<int>.Zero);
                var allowFaceContacts = Vector.GreaterThanOrEqual(capsuleOffsetAlongNormal, Vector<float>.Zero);
                var useFaceContacts = Vector.BitwiseAnd(noEdgeContacts, allowFaceContacts);
                Vector3Wide.ConditionalSelect(useFaceContacts, faceCandidate0, b0, out b0);
                Vector3Wide.ConditionalSelect(useFaceContacts, faceCandidate1, b1, out b1);
                contactCount = Vector.ConditionalSelect(useFaceContacts, new Vector<int>(2), contactCount);

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
                var useCandidateForSecondContact = Vector.BitwiseAnd(intervalIsValidForSecondContact,
                    Vector.BitwiseAnd(Vector.Equals(contactCount, Vector<int>.One), Vector.GreaterThan(secondContactDistanceAlongNormal, Vector<float>.Zero)));
                Vector3Wide.ConditionalSelect(useCandidateForSecondContact, secondContactCandidate, b1, out b1);
                contactCount = Vector.ConditionalSelect(useCandidateForSecondContact, new Vector<int>(2), contactCount);
            }

            //While we have computed a global depth, each contact has its own depth.
            //Project the contact on B along the contact normal to the 'face' of A.
            //A is a capsule, but we can treat it as having a faceNormalA = (localNormal x capsuleAxis) x capsuleAxis.
            //The full computation is: 
            //t = dot(contact + localOffsetB, faceNormalA) / dot(faceNormalA, localNormal)
            //depth = dot(t * localNormal, localNormal) = t
            Vector3Wide.CrossWithoutOverlap(localNormal, localCapsuleAxis, out var capsuleTangent);
            Vector3Wide.CrossWithoutOverlap(capsuleTangent, localCapsuleAxis, out var faceNormalA);
            Vector3Wide.Dot(faceNormalA, localNormal, out var faceNormalADotLocalNormal);
            //Don't have to perform any calibration on the faceNormalA; it appears in both the numerator and denominator so the sign and magnitudes cancel.
            var inverseFaceNormalADotLocalNormal = Vector<float>.One / faceNormalADotLocalNormal;
            Vector3Wide.Add(localOffsetB, b0, out var offset0);
            Vector3Wide.Add(localOffsetB, b1, out var offset1);
            Vector3Wide.Dot(offset0, faceNormalA, out var t0);
            Vector3Wide.Dot(offset1, faceNormalA, out var t1);
            t0 *= inverseFaceNormalADotLocalNormal;
            t1 *= inverseFaceNormalADotLocalNormal;
            manifold.Depth0 = a.Radius + t0;
            manifold.Depth1 = a.Radius + t1;

            //If the 'velocity' above is very small, it means that the local normal and capsule axis are very nearly aligned and depths computed using it are likely numerically poor.
            //In this situation, using more than one contact is pretty pointless anyway, so collapse the manifold to only one point and use the previously computed depth.
            var collapse = Vector.LessThan(Vector.Abs(faceNormalADotLocalNormal), new Vector<float>(1e-7f));
            manifold.Depth0 = Vector.ConditionalSelect(collapse, a.Radius + depth, manifold.Depth0);
            //If the normal we found points away from the triangle normal, then it it's hitting the wrong side and should be ignored. (Note that we had an early out for this earlier.)
            contactCount = Vector.ConditionalSelect(collidingWithSolidSide, contactCount, Vector<int>.Zero);
            manifold.Contact0Exists = Vector.BitwiseAnd(allowContacts, Vector.BitwiseAnd(Vector.GreaterThan(contactCount, Vector<int>.Zero), Vector.GreaterThan(manifold.Depth0, negativeMargin)));
            manifold.Contact1Exists = Vector.BitwiseAnd(allowContacts, Vector.BitwiseAnd(Vector.AndNot(Vector.Equals(contactCount, new Vector<int>(2)), collapse), Vector.GreaterThan(manifold.Depth1, negativeMargin)));

            //For feature ids, note that we have a few different potential sources of contacts. While we could go through and force each potential source to output ids,
            //there is a useful single unifying factor: where the contacts occur on the capsule axis. Using this, it doesn't matter if contacts are generated from face or edge cases,
            //so there's a higher chance of properly reusing the previous frame's accumulated impulse.
            //Using the depths, push the contacts out toward the capsule, and then compute their projected location along the capsule axis. In other words:
            //ta0 = ((b0 + N * depth0) - capsuleCenter) * capsuleAxis
            //ta1 = ((b1 + N * depth1) - capsuleCenter) * capsuleAxis
            //If ta0 > ta1, featureId0 = 1 and featureId1 = 0, otherwise featureId0 = 0 and featureId1 = 1.
            //But note that all we really want is a degree of consistency, so pushing all the way back to the capsule axis isn't necessary.
            //Instead, just compute the projection of the contacts along the capsule axis directly. This will tend to agree with the more in-depth formulation.
            Vector3Wide.Subtract(b0, localOffsetA, out var localOffsetA0);
            Vector3Wide.Subtract(b1, localOffsetA, out var localOffsetA1);
            Vector3Wide.Dot(localOffsetA0, localCapsuleAxis, out var ta0);
            Vector3Wide.Dot(localOffsetA1, localCapsuleAxis, out var ta1);
            var flipFeatureIds = Vector.LessThan(ta1, ta0);
            manifold.FeatureId0 = Vector.ConditionalSelect(flipFeatureIds, Vector<int>.One, Vector<int>.Zero);
            manifold.FeatureId1 = Vector.ConditionalSelect(flipFeatureIds, Vector<int>.Zero, Vector<int>.One);

            var faceFlag = Vector.ConditionalSelect(
                Vector.GreaterThanOrEqual(localNormalDotFaceNormal, new Vector<float>(MeshReduction.MinimumDotForFaceCollision)),
                new Vector<int>(MeshReduction.FaceCollisionFlag), Vector<int>.Zero);
            manifold.FeatureId0 += faceFlag;

            //Transform contact positions into world space rotation, measured as offsets from the capsule (object A).
            Matrix3x3Wide.TransformWithoutOverlap(localOffsetA0, rB, out manifold.OffsetA0);
            Matrix3x3Wide.TransformWithoutOverlap(localOffsetA1, rB, out manifold.OffsetA1);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, rB, out manifold.Normal);
        }


        public void Test(ref CapsuleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref CapsuleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
