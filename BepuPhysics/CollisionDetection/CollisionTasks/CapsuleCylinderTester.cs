using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CapsuleCylinderTester : IPairTester<CapsuleWide, CylinderWide, Convex2ContactManifoldWide>
    {
        public int BatchSize => 32;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void Bounce(in Vector3Wide lineOrigin, in Vector3Wide lineDirection, in Vector<float> t, in CylinderWide b, in Vector<float> radiusSquared, out Vector3Wide p, out Vector3Wide clamped)
        {
            //Clamp the point on the capsule line to the bounds of the cylinder, and then project the clamped result back onto the line.
            p.X = lineDirection.X * t + lineOrigin.X;
            p.Y = lineDirection.Y * t + lineOrigin.Y;
            p.Z = lineDirection.Z * t + lineOrigin.Z;
            var horizontalDistanceSquared = p.X * p.X + p.Z * p.Z;
            var needHorizontalClamp = Vector.GreaterThan(horizontalDistanceSquared, radiusSquared);
            var clampScale = b.Radius / Vector.SquareRoot(horizontalDistanceSquared);
            clamped.X = Vector.ConditionalSelect(needHorizontalClamp, clampScale * p.X, p.X);
            clamped.Y = Vector.Max(-b.HalfLength, Vector.Min(b.HalfLength, p.Y));
            clamped.Z = Vector.ConditionalSelect(needHorizontalClamp, clampScale * p.Z, p.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetClosestPointBetweenLineSegmentAndCylinder(in Vector3Wide lineOrigin, in Vector3Wide lineDirection, in Vector<float> halfLength, in CylinderWide b,
            in Vector<int> inactiveLanes, out Vector<float> t, out Vector3Wide offsetFromCylinderToLineSegment)
        {
            var min = -halfLength;
            var max = halfLength;
            t = Vector<float>.Zero;
            var radiusSquared = b.Radius * b.Radius;
            Vector3Wide.Dot(lineDirection, lineOrigin, out var originDot);
            var epsilon = halfLength * 1e-7f;
            var laneDeactivated = inactiveLanes;
            for (int i = 0; i < 12; ++i)
            {
                Bounce(lineOrigin, lineDirection, t, b, radiusSquared, out _, out var clamped);
                Vector3Wide.Dot(clamped, lineDirection, out var conservativeNewT);
                conservativeNewT = Vector.Max(min, Vector.Min(max, conservativeNewT - originDot));
                var change = conservativeNewT - t;
                //Check for deactivated lanes and see if we can exit early.
                var laneShouldDeactivate = Vector.LessThan(Vector.Abs(change), epsilon);
                laneDeactivated = Vector.BitwiseOr(laneDeactivated, laneShouldDeactivate);
                if (Vector.LessThanAll(laneDeactivated, Vector<int>.Zero))
                {
                    //All lanes are done; early out.
                    break;
                }

                //The bounced projection can be thought of as conservative advancement. The sign of the change tells us which way the advancement moved; we can use that to update the bounds.
                var movedUp = Vector.GreaterThan(change, Vector<float>.Zero);
                min = Vector.ConditionalSelect(movedUp, conservativeNewT, min);
                max = Vector.ConditionalSelect(movedUp, max, conservativeNewT);

                //Bisect the remaining interval.
                var newT = 0.5f * (min + max);

                //Deactivated lanes should not be updated; if iteration counts are sensitive to the behavior of a bundle, it creates a dependency on bundle order and kills determinism.
                t = Vector.ConditionalSelect(laneDeactivated, t, newT);

            }
            Bounce(lineOrigin, lineDirection, t, b, radiusSquared, out var pointOnLine, out var clampedToCylinder);
            Vector3Wide.Subtract(pointOnLine, clampedToCylinder, out offsetFromCylinderToLineSegment);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetContactIntervalBetweenSegments(in Vector<float> aHalfLength, in Vector<float> bHalfLength, in Vector3Wide axisA, in Vector3Wide localNormal,
            in Vector<float> inverseHorizontalNormalLengthSquaredB, in Vector3Wide offsetB, out Vector<float> contactTMin, out Vector<float> contactTMax)
        {
            GetClosestPointsBetweenSegments(axisA, offsetB, aHalfLength, bHalfLength, out var ta, out var taMin, out var taMax, out var tb, out var tbMin, out var tbMax);

            //In the event that the two axes are coplanar, we accept the whole interval as a source of contact.
            //As the axes drift away from coplanarity, the accepted interval rapidly narrows to zero length, centered on ta and tb.
            //We rate the degree of coplanarity based on the angle between the capsule axis and the plane defined by the side and contact normal:
            //sin(angle) = dot(da, (db x normal)/||db x normal||)
            //Finally, note that we are dealing with extremely small angles, and for small angles sin(angle) ~= angle,
            //and also that fade behavior is completely arbitrary, so we can directly use squared angle without any concern.
            //angle^2 ~= dot(da, (db x normal))^2 / ||db x normal||^2
            //Note that db x normal is just (normal.Z, -normal.X) since db is (0,1,0).
            var dot = (axisA.X * localNormal.Z - axisA.Z * localNormal.X);
            var squaredAngle = dot * dot * inverseHorizontalNormalLengthSquaredB;

            //Convert the squared angle to a lerp parameter. For squared angle from 0 to lowerThreshold, we should use the full interval (1). From lowerThreshold to upperThreshold, lerp to 0.
            const float lowerThresholdAngle = 0.02f;
            const float upperThresholdAngle = 0.15f;
            const float lowerThreshold = lowerThresholdAngle * lowerThresholdAngle;
            const float upperThreshold = upperThresholdAngle * upperThresholdAngle;
            var intervalWeight = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (new Vector<float>(upperThreshold) - squaredAngle) * new Vector<float>(1f / (upperThreshold - lowerThreshold))));
            //If the line segments intersect, even if they're coplanar, we would ideally stick to using a single point. Would be easy enough,
            //but we don't bother because it's such a weird and extremely temporary corner case. Not really worth handling.
            var weightedTb = tb - tb * intervalWeight;
            contactTMin = intervalWeight * tbMin + weightedTb;
            contactTMax = intervalWeight * tbMax + weightedTb;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetClosestPointsBetweenSegments(in Vector3Wide da, in Vector3Wide localOffsetB, in Vector<float> aHalfLength, in Vector<float> bHalfLength,
            out Vector<float> ta, out Vector<float> taMin, out Vector<float> taMax, out Vector<float> tb, out Vector<float> tbMin, out Vector<float> tbMax)
        {
            //This is similar to the capsule pair execution, but we make use of the fact that we're working in the cylinder's local space where db = (0,1,0).

            //Compute the closest points between the two line segments. No clamping to begin with.
            //We want to minimize distance = ||(a + da * ta) - (b + db * tb)||.
            //Taking the derivative with respect to ta and doing some algebra (taking into account ||da|| == ||db|| == 1) to solve for ta yields:
            //ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))
            Vector3Wide.Dot(da, localOffsetB, out var daOffsetB);
            var dbOffsetB = localOffsetB.Y;
            var dadb = da.Y;
            //Note potential division by zero when the axes are parallel. Arbitrarily clamp; near zero values will instead produce extreme values which get clamped to reasonable results.
            ta = (daOffsetB - dbOffsetB * dadb) / Vector.Max(new Vector<float>(1e-15f), Vector<float>.One - dadb * dadb);
            //tb = ta * (da * db) - db * (b - a)
            tb = ta * dadb - dbOffsetB;

            //We cannot simply clamp the ta and tb values to the capsule line segments. Instead, project each line segment onto the other line segment, clamping against the target's interval.
            //That new clamped projected interval is the valid solution space on that line segment. We can clamp the t value by that interval to get the correctly bounded solution.
            //The projected intervals are:
            //B onto A: +-BHalfLength * (da * db) + da * offsetB
            //A onto B: +-AHalfLength * (da * db) - db * offsetB
            var absdadb = Vector.Abs(dadb);
            var bOntoAOffset = bHalfLength * absdadb;
            var aOntoBOffset = aHalfLength * absdadb;
            taMin = Vector.Max(-aHalfLength, Vector.Min(aHalfLength, daOffsetB - bOntoAOffset));
            taMax = Vector.Min(aHalfLength, Vector.Max(-aHalfLength, daOffsetB + bOntoAOffset));
            tbMin = Vector.Max(-bHalfLength, Vector.Min(bHalfLength, -aOntoBOffset - dbOffsetB));
            tbMax = Vector.Min(bHalfLength, Vector.Max(-bHalfLength, aOntoBOffset - dbOffsetB));
            ta = Vector.Min(Vector.Max(ta, taMin), taMax);
            tb = Vector.Min(Vector.Max(tb, tbMin), tbMax);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(ref CapsuleWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            //Potential normal generators:
            //Capsule endpoint vs cylinder cap plane :
            //Capsule endpoint vs cylinder cap edge  : Only applies during shallow collision; if the capsule line segment intersects the cylinder, the fastest way out for an endpoint is straight up or sideways, not diagonally.
            //Capsule endpoint vs cylinder side      :
            //Capsule edge     vs cylinder cap plane : redundant with endpoint-plane; the capsule segment cannot be closer than either of its endpoints to any plane.
            //Capsule edge     vs cylinder cap edge  : this one is nasty!
            //Capsule edge     vs cylinder side      : provided the other cases are covered, this is simply a closest point between lines test (internal line of capsule versus internal line of cylinder).

            //Capsule edge versus cylinder cap edge is nasty. It involves finding the roots of a fourth degree polynomial; we're going to avoid doing that.

            //Instead, use a numerical solver that seeks the minimum distance between the cylinder and line segment.
            //This captures every shallow-only normal candidate, so there's no need to do capsule endpoint vs cylinder cap edge.
            //This is a bit of a cheat- the solver can't handle the segment-cylinder intersecting case, so we're missing out on some edge normals in deep collision.
            //So, the current normal generators are:
            //Capsule segment  vs cylinder           : numerically solved
            //Capsule endpoint vs cylinder cap plane : only needed if any lane failed the shallow distance test
            //Capsule segment  vs cylinder side      : combined endpoint and edge tests; only needed if any lane failed the shallow distance test
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var worldRB);
            //Work in the cylinder's local space.
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(worldRA, worldRB, out var rA);
            ref var capsuleAxis = ref rA.Y;
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRB, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);

            var inactiveLanes = BundleIndexing.CreateTrailingMaskForCountInBundle(pairCount);
            GetClosestPointBetweenLineSegmentAndCylinder(localOffsetA, capsuleAxis, a.HalfLength, b, inactiveLanes, out var t, out var localNormal);
            Vector3Wide.LengthSquared(localNormal, out var distanceFromCylinderToLineSegmentSquared);
            var internalLineSegmentIntersected = Vector.LessThan(distanceFromCylinderToLineSegmentSquared, new Vector<float>(1e-12f));
            var distanceFromCylinderToLineSegment = Vector.SquareRoot(distanceFromCylinderToLineSegmentSquared);
            //Division by zero is protected by the depth selection- if distance is zero, the depth is set to infinity and this normal won't be selected.
            Vector3Wide.Scale(localNormal, Vector<float>.One / distanceFromCylinderToLineSegment, out localNormal);
            var depth = Vector.ConditionalSelect(internalLineSegmentIntersected, new Vector<float>(float.MaxValue), -distanceFromCylinderToLineSegment);
            var negativeMargin = -speculativeMargin;
            inactiveLanes = Vector.BitwiseOr(Vector.LessThan(depth + a.Radius, negativeMargin), inactiveLanes);
            if (Vector.LessThanAny(Vector.AndNot(internalLineSegmentIntersected, inactiveLanes), Vector<int>.Zero))
            {
                //At least one lane is intersecting deeply, so we need to examine the other possible normals.
                var endpointVsCapDepth = b.HalfLength + Vector.Abs(capsuleAxis.Y * a.HalfLength) - Vector.Abs(localOffsetA.Y);
                var useEndpointCapDepth = Vector.BitwiseAnd(internalLineSegmentIntersected, Vector.LessThan(endpointVsCapDepth, depth));
                depth = Vector.ConditionalSelect(useEndpointCapDepth, endpointVsCapDepth, depth);
                localNormal.X = Vector.ConditionalSelect(useEndpointCapDepth, Vector<float>.Zero, localNormal.X);
                //Normal calibrated to point from B to A.
                localNormal.Y = Vector.ConditionalSelect(useEndpointCapDepth, Vector.ConditionalSelect(Vector.GreaterThan(localOffsetA.Y, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f)), localNormal.Y);
                localNormal.Z = Vector.ConditionalSelect(useEndpointCapDepth, Vector<float>.Zero, localNormal.Z);

                GetClosestPointsBetweenSegments(capsuleAxis, localOffsetB, a.HalfLength, b.HalfLength, out var ta, out _, out _, out var tb, out _, out _);

                //offset = da * ta - (db * tb + offsetB)
                Vector3Wide.Scale(capsuleAxis, ta, out var closestA);
                Vector3Wide.Subtract(closestA, localOffsetB, out var offset);
                offset.Y -= tb;

                Vector3Wide.Length(offset, out var distance);
                var inverseDistance = Vector<float>.One / distance;
                Vector3Wide.Scale(offset, inverseDistance, out var internalEdgeNormal);
                var useFallback = Vector.LessThan(distance, new Vector<float>(1e-7f));
                internalEdgeNormal.X = Vector.ConditionalSelect(useFallback, Vector<float>.One, internalEdgeNormal.X);
                internalEdgeNormal.Y = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, internalEdgeNormal.Y);
                internalEdgeNormal.Z = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, internalEdgeNormal.Z);

                //Compute the depth along the internal edge normal.
                Vector3Wide.Dot(localOffsetA, internalEdgeNormal, out var centerSeparationAlongNormal);
                var cylinderContribution = Vector.Abs(b.HalfLength * internalEdgeNormal.Y) + b.Radius * Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One - internalEdgeNormal.Y * internalEdgeNormal.Y));
                Vector3Wide.Dot(capsuleAxis, internalEdgeNormal, out var capsuleAxisDotNormal);
                var capsuleContribution = Vector.Abs(capsuleAxisDotNormal) * a.HalfLength;
                var internalEdgeDepth = cylinderContribution + capsuleContribution - centerSeparationAlongNormal;

                var useInternalEdgeDepth = Vector.BitwiseAnd(internalLineSegmentIntersected, Vector.LessThan(internalEdgeDepth, depth));
                depth = Vector.ConditionalSelect(useInternalEdgeDepth, internalEdgeDepth, depth);
                Vector3Wide.ConditionalSelect(useInternalEdgeDepth, internalEdgeNormal, localNormal, out localNormal);
            }
            //All of the above excluded any consideration of the capsule's radius. Include it now.
            depth += a.Radius;
            inactiveLanes = Vector.BitwiseOr(Vector.LessThan(depth, negativeMargin), inactiveLanes);
            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //All lanes have a depth which cannot create any contacts due to the speculative margin. We can early out.
                //This is determinism-safe; even if execution continued, there would be no contacts created and the manifold would be equivalent for all lanes.
                manifold = default;
                return;
            }
            //We now have depth and a collision normal. Use it to identify representative features on each shape.
            //Note that we exclude edges as representatives; only features with area are used so that contact generation can produce better speculative contacts.
            //Potential features pairs are:
            //Capsule segment vs cylinder side
            //Capsule segment vs cylinder cap
            //Segment-side case is handled in the same way as capsule-capsule- create an interval by projecting the segment onto the cylinder segment and then narrow the interval in response to noncoplanarity.
            //Segment-cap is easy too; project the segment down onto the cap plane. Clip it against the cap circle (solve a quadratic).

            var useCapContacts = Vector.AndNot(Vector.GreaterThan(Vector.Abs(localNormal.Y), new Vector<float>(0.70710678118f)), inactiveLanes);

            //First, assume non-cap contacts.
            //Phrase the problem as a segment-segment test.
            //For any lane that needs side contacts, we know the projected normal will be nonzero length based on the condition above.
            var inverseHorizontalNormalLengthSquared = Vector<float>.One / (localNormal.X * localNormal.X + localNormal.Z * localNormal.Z);
            var scale = b.Radius * Vector.SquareRoot(inverseHorizontalNormalLengthSquared);
            var cylinderSegmentOffsetX = localNormal.X * scale;
            var cylinderSegmentOffsetZ = localNormal.Z * scale;
            Vector3Wide aToSideSegmentCenter;
            aToSideSegmentCenter.X = localOffsetB.X + cylinderSegmentOffsetX;
            aToSideSegmentCenter.Y = localOffsetB.Y;
            aToSideSegmentCenter.Z = localOffsetB.Z + cylinderSegmentOffsetZ;
            GetContactIntervalBetweenSegments(a.HalfLength, b.HalfLength, capsuleAxis, localNormal, inverseHorizontalNormalLengthSquared, aToSideSegmentCenter, out var contactTMin, out var contactTMax);

            Vector3Wide contact0, contact1;
            contact0.X = cylinderSegmentOffsetX;
            contact0.Y = contactTMin;
            contact0.Z = cylinderSegmentOffsetZ;
            contact1.X = cylinderSegmentOffsetX;
            contact1.Y = contactTMax;
            contact1.Z = cylinderSegmentOffsetZ;

            var contactCount = Vector.ConditionalSelect(Vector.LessThan(Vector.Abs(contactTMax - contactTMin), b.HalfLength * new Vector<float>(1e-5f)), Vector<int>.One, new Vector<int>(2));

            if (Vector.LessThanAny(useCapContacts, Vector<int>.Zero))
            {
                //At least one lane requires a cap contact.
                //An important note: for highest quality, all clipping takes place on the *normal plane*. So segment-cap doesn't merely set the y component to zero (projecting along B's Y axis).
                //Instead, the endpoints are casted along the normal to the cap plane.            
                //t = dot(capsuleOrigin +- capsuleDirection * a.HalfLength - (0, normal.Y > 0 ? b.HalfLength : a.HalfLength, 0), cylinderY) / dot(normal, cylinderY)
                //t = (capsuleOrigin.Y +- capsuleDirection.Y * a.HalfLength - (normal.Y > 0 ? b.HalfLength : a.HalfLength)) / normal.Y
                //Note that the cap will only be chosen as a representative if normal.Y dominates the horizontal direction, so there is no need to test for division by zero.
                var capHeight = Vector.ConditionalSelect(Vector.GreaterThan(localNormal.Y, Vector<float>.Zero), b.HalfLength, -b.HalfLength);
                var inverseNormalY = Vector<float>.One / localNormal.Y;
                Vector3Wide.Scale(capsuleAxis, a.HalfLength, out var endpointOffset);
                Vector3Wide positive, negative;
                positive.X = localOffsetA.X + endpointOffset.X;
                positive.Y = localOffsetA.Y + endpointOffset.Y - capHeight;
                positive.Z = localOffsetA.Z + endpointOffset.Z;
                negative.X = localOffsetA.X - endpointOffset.X;
                negative.Y = localOffsetA.Y - endpointOffset.Y - capHeight;
                negative.Z = localOffsetA.Z - endpointOffset.Z;
                var tNegative = negative.Y * inverseNormalY;
                var tPositive = positive.Y * inverseNormalY;
                Vector2Wide projectedPositive, projectedNegative;
                projectedNegative.X = negative.X - localNormal.X * tNegative;
                projectedNegative.Y = negative.Z - localNormal.Z * tNegative;
                projectedPositive.X = positive.X - localNormal.X * tPositive;
                projectedPositive.Y = positive.Z - localNormal.Z * tPositive;

                //Intersect the line segment (projectedNegative, projectedPositive) with the circle with radius b.Radius positioned at (0,0).
                Vector2Wide.Subtract(projectedPositive, projectedNegative, out var projectedOffset);
                //||a + ab * t|| = radius
                //dot(a + ab * t, a + ab * t) = radius * radius
                //dot(a,a) - radius * radius + t * 2 * dot(a, ab) + t^2 * dot(ab, ab) = 0
                Vector2Wide.Dot(projectedNegative, projectedNegative, out var coefficientC);
                coefficientC -= b.Radius * b.Radius;
                Vector2Wide.Dot(projectedNegative, projectedOffset, out var coefficientB);
                Vector2Wide.Dot(projectedOffset, projectedOffset, out var coefficientA);
                var inverseA = Vector<float>.One / coefficientA;
                var tOffset = Vector.SquareRoot(Vector.Max(Vector<float>.Zero, coefficientB * coefficientB - coefficientA * coefficientC)) * inverseA;
                var tBase = -coefficientB * inverseA;
                var tMin = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, tBase - tOffset));
                var tMax = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, tBase + tOffset));
                //If the projected length is zero, just treat both points as being in the same location (at tNegative).
                var useFallback = Vector.LessThan(Vector.Abs(coefficientA), new Vector<float>(1e-12f));
                tMin = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, tMin);
                tMax = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, tMax);
                Vector3Wide capContact0, capContact1;
                capContact0.X = tMin * projectedOffset.X + projectedNegative.X;
                capContact0.Y = capHeight;
                capContact0.Z = tMin * projectedOffset.Y + projectedNegative.Y;
                capContact1.X = tMax * projectedOffset.X + projectedNegative.X;
                capContact1.Y = capHeight;
                capContact1.Z = tMax * projectedOffset.Y + projectedNegative.Y;
                //Fixed epsilon- the t value scales an offset that is generally proportional to object sizes.
                var capContactCount = Vector.ConditionalSelect(Vector.GreaterThan(tMax - tMin, new Vector<float>(1e-5f)), new Vector<int>(2), Vector<int>.One);
                contactCount = Vector.ConditionalSelect(useCapContacts, capContactCount, contactCount);
                Vector3Wide.ConditionalSelect(useCapContacts, capContact0, contact0, out contact0);
                Vector3Wide.ConditionalSelect(useCapContacts, capContact1, contact1, out contact1);
            }
            //While we have computed a global depth, each contact has its own depth.
            //Project the contact on B along the contact normal to the 'face' of A.
            //A is a capsule, but we can treat it as having a faceNormalA = (localNormal x capsuleAxis) x capsuleAxis.
            //The full computation is: 
            //t = dot(contact + localOffsetB, faceNormalA) / dot(faceNormalA, localNormal)
            //depth = dot(t * localNormal, localNormal) = t
            Vector3Wide.CrossWithoutOverlap(localNormal, capsuleAxis, out var capsuleTangent);
            Vector3Wide.CrossWithoutOverlap(capsuleTangent, capsuleAxis, out var faceNormalA);
            Vector3Wide.Dot(faceNormalA, localNormal, out var faceNormalADotLocalNormal);
            //Don't have to perform any calibration on the faceNormalA; it appears in both the numerator and denominator so the sign and magnitudes cancel.
            var inverseFaceNormalADotLocalNormal = Vector<float>.One / faceNormalADotLocalNormal;
            Vector3Wide.Add(localOffsetB, contact0, out var offset0);
            Vector3Wide.Add(localOffsetB, contact1, out var offset1);
            Vector3Wide.Dot(offset0, faceNormalA, out var t0);
            Vector3Wide.Dot(offset1, faceNormalA, out var t1);
            t0 *= inverseFaceNormalADotLocalNormal;
            t1 *= inverseFaceNormalADotLocalNormal;
            manifold.Depth0 = a.Radius + t0;
            manifold.Depth1 = a.Radius + t1;

            //If the capsule axis is parallel with the normal, then the contacts collapse to one point and we can use the initially computed depth.
            //In this case, both contact positions should be extremely close together anyway.
            var collapse = Vector.LessThan(Vector.Abs(faceNormalADotLocalNormal), new Vector<float>(1e-7f));
            manifold.Depth0 = Vector.ConditionalSelect(collapse, depth, manifold.Depth0);
            manifold.Contact0Exists = Vector.AndNot(Vector.GreaterThanOrEqual(manifold.Depth0, negativeMargin), inactiveLanes);
            manifold.Contact1Exists = Vector.AndNot(Vector.BitwiseAnd(Vector.AndNot(Vector.Equals(contactCount, new Vector<int>(2)), collapse), Vector.GreaterThanOrEqual(manifold.Depth1, negativeMargin)), inactiveLanes);

            //Push the contacts into world space.
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, worldRB, out manifold.Normal);
            Matrix3x3Wide.TransformWithoutOverlap(contact0, worldRB, out manifold.OffsetA0);
            Matrix3x3Wide.TransformWithoutOverlap(contact1, worldRB, out manifold.OffsetA1);
            Vector3Wide.Add(manifold.OffsetA0, offsetB, out manifold.OffsetA0);
            Vector3Wide.Add(manifold.OffsetA1, offsetB, out manifold.OffsetA1);

            manifold.FeatureId0 = Vector<int>.Zero;
            manifold.FeatureId1 = Vector<int>.One;
        }


        public void Test(ref CapsuleWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref CapsuleWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
