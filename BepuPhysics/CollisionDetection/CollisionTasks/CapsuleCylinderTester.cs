using BepuPhysics.Collidables;
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
            out Vector<float> t, out Vector3Wide offsetFromCylinderToLineSegment)
        {
            var min = -halfLength;
            var max = halfLength;
            t = Vector<float>.Zero;
            var radiusSquared = b.Radius * b.Radius;
            var negativeCylinderHalfLength = -b.HalfLength;
            Vector3Wide.Dot(lineDirection, lineOrigin, out var originDot);
            var epsilon = halfLength * 1e-7f;
            var laneDeactivated = Vector<int>.Zero;
            for (int i = 0; i < 12; ++i)
            {
                Bounce(lineOrigin, lineDirection, t, b, radiusSquared, out _, out var clamped);
                Vector3Wide.Dot(clamped, lineDirection, out var conservativeNewT);
                conservativeNewT = Vector.Max(min, Vector.Min(max, conservativeNewT - originDot));
                var change = conservativeNewT - t;

                //The bounced projection can be thought of as conservative advancement. The sign of the change tells us which way the advancement moved; we can use that to update the bounds.
                var movedUp = Vector.GreaterThan(change, Vector<float>.Zero);
                min = Vector.ConditionalSelect(movedUp, conservativeNewT, min);
                max = Vector.ConditionalSelect(movedUp, max, conservativeNewT);

                //Bisect the remaining interval.
                var newT = 0.5f * (min + max);

                //Check for deactivated lanes and see if we can exit early.
                var laneShouldDeactivate = Vector.LessThan(Vector.Abs(change), epsilon);
                laneDeactivated = Vector.BitwiseOr(laneDeactivated, laneShouldDeactivate);
                //Deactivated lanes should not be updated; if iteration counts are sensitive to the behavior of a bundle, it creates a dependency on bundle order and kills determinism.
                t = Vector.ConditionalSelect(laneDeactivated, t, newT);
                if (Vector.LessThanAll(laneDeactivated, Vector<int>.Zero))
                {
                    //All lanes are done; early out.
                    break;
                }
            }
            Bounce(lineOrigin, lineDirection, t, b, radiusSquared, out var pointOnLine, out var clampedToCylinder);
            Vector3Wide.Subtract(pointOnLine, clampedToCylinder, out offsetFromCylinderToLineSegment);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetClosestPointsBetweenSegments(in Vector3Wide da, in Vector3Wide localOffsetB, in Vector<float> aHalfLength, in Vector<float> bHalfLength, out Vector3Wide normal)
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
            var ta = (daOffsetB - dbOffsetB * dadb) / Vector.Max(new Vector<float>(1e-15f), Vector<float>.One - dadb * dadb);
            //tb = ta * (da * db) - db * (b - a)
            var tb = ta * dadb - dbOffsetB;

            //We cannot simply clamp the ta and tb values to the capsule line segments. Instead, project each line segment onto the other line segment, clamping against the target's interval.
            //That new clamped projected interval is the valid solution space on that line segment. We can clamp the t value by that interval to get the correctly bounded solution.
            //The projected intervals are:
            //B onto A: +-BHalfLength * (da * db) + da * offsetB
            //A onto B: +-AHalfLength * (da * db) - db * offsetB
            var absdadb = Vector.Abs(dadb);
            var bOntoAOffset = bHalfLength * absdadb;
            var aOntoBOffset = aHalfLength * absdadb;
            var aMin = Vector.Max(-aHalfLength, Vector.Min(aHalfLength, daOffsetB - bOntoAOffset));
            var aMax = Vector.Min(aHalfLength, Vector.Max(-aHalfLength, daOffsetB + bOntoAOffset));
            var bMin = Vector.Max(-bHalfLength, Vector.Min(bHalfLength, -aOntoBOffset - dbOffsetB));
            var bMax = Vector.Min(bHalfLength, Vector.Max(-bHalfLength, aOntoBOffset - dbOffsetB));
            ta = Vector.Min(Vector.Max(ta, aMin), aMax);
            tb = Vector.Min(Vector.Max(tb, bMin), bMax);

            //offset = da * ta - (db * tb + offsetB)
            Vector3Wide.Scale(da, ta, out var closestA);
            Vector3Wide.Subtract(closestA, localOffsetB, out normal);
            normal.Y -= tb;

            Vector3Wide.Length(normal, out var distance);
            var inverseDistance = Vector<float>.One / distance;
            Vector3Wide.Scale(normal, inverseDistance, out normal);
            var useFallback = Vector.LessThan(distance, new Vector<float>(1e-7f));
            normal.X = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, normal.X);
            normal.Y = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, normal.Y);
            normal.Z = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, normal.Z);

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
            //TODO: May want to include a cylinder center->capsule line normal to sorta-kinda cover that case. Not going to be a common issue, though.
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

            GetClosestPointBetweenLineSegmentAndCylinder(localOffsetA, capsuleAxis, a.HalfLength, b, out var t, out var localNormal);
            Vector3Wide.LengthSquared(localNormal, out var distanceFromCylinderToLineSegmentSquared);
            var internalLineSegmentIntersected = Vector.LessThan(distanceFromCylinderToLineSegmentSquared, new Vector<float>(1e-12f));
            var distanceFromCylinderToLineSegment = Vector.SquareRoot(distanceFromCylinderToLineSegmentSquared);
            //Division by zero is protected by the depth selection- if distance is zero, the depth is set to infinity and this normal won't be selected.
            Vector3Wide.Scale(localNormal, Vector<float>.One / distanceFromCylinderToLineSegment, out localNormal);
            var depth = Vector.ConditionalSelect(internalLineSegmentIntersected, new Vector<float>(float.MaxValue), -distanceFromCylinderToLineSegment);

            if (Vector.EqualsAny(internalLineSegmentIntersected, new Vector<int>(-1)))
            {
                //At least one lane is intersecting deeply, so we need to examine the other possible normals.
                var endpointVsCapDepth = b.HalfLength - Vector.Abs(localOffsetA.Y) - Vector.Abs(capsuleAxis.Y * a.HalfLength);
                var useEndpointCapDepth = Vector.LessThan(endpointVsCapDepth, depth);
                depth = Vector.Min(endpointVsCapDepth, depth);
                localNormal.X = Vector.ConditionalSelect(useEndpointCapDepth, Vector<float>.Zero, localNormal.X);
                //Normal calibrated to point from B to A.
                localNormal.Y = Vector.ConditionalSelect(useEndpointCapDepth, Vector.ConditionalSelect(Vector.GreaterThan(localOffsetA.Y, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f)), localNormal.Y);
                localNormal.Z = Vector.ConditionalSelect(useEndpointCapDepth, Vector<float>.Zero, localNormal.Z);

                //TODO: Consider using segment-segment to offer a little bit of cover for the deep intersection edge case. The clamps don't take much more than the segment-line variant.
                GetClosestPointsBetweenSegments(capsuleAxis, localOffsetB, a.HalfLength, b.HalfLength, out var internalEdgeNormal);

                //Compute the depth along the internal edge normal.
                var horizontalLengthSquared = internalEdgeNormal.X * internalEdgeNormal.X + internalEdgeNormal.Z * internalEdgeNormal.Z;
                var scale = Vector.ConditionalSelect(Vector.LessThan(horizontalLengthSquared, new Vector<float>(1e-12f)), Vector<float>.Zero, b.Radius / Vector.SquareRoot(horizontalLengthSquared));
                Vector3Wide extremeOnCylinder;
                extremeOnCylinder.X = scale * internalEdgeNormal.X;
                extremeOnCylinder.Y = Vector.ConditionalSelect(Vector.GreaterThan(internalEdgeNormal.Y, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1));
                extremeOnCylinder.Z = scale * internalEdgeNormal.Z;
                Vector3Wide.Dot(internalEdgeNormal, capsuleAxis, out var axisDot);
                var capsuleExtremeT = Vector.ConditionalSelect(Vector.GreaterThan(axisDot, Vector<float>.Zero), -a.HalfLength, a.HalfLength);
                Vector3Wide.Scale(capsuleAxis, capsuleExtremeT, out var extremeOnCapsule);
                Vector3Wide.Add(localOffsetA, extremeOnCapsule, out extremeOnCapsule);
                Vector3Wide.Subtract(extremeOnCylinder, extremeOnCapsule, out var extremeOffset);
                Vector3Wide.Dot(extremeOffset, internalEdgeNormal, out var internalEdgeDepth);

                var useInternalEdgeDepth = Vector.LessThan(internalEdgeDepth, depth);
                depth = Vector.Min(internalEdgeDepth, depth);
                Vector3Wide.ConditionalSelect(useInternalEdgeDepth, internalEdgeNormal, localNormal, out localNormal);
            }
            //All of the above excluded any consideration of the capsule's radius. Include it now.
            depth += a.Radius;
            if (Vector.LessThanAll(depth, -speculativeMargin))
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

            Vector3Wide contact0 = default, contact1 = default;
            Vector<int> contactCount = default;
            var useCapContacts = Vector.GreaterThan(Vector.Abs(localNormal.Y), new Vector<float>(0.70710678118f));
            if (Vector.EqualsAny(useCapContacts, Vector<int>.Zero))
            {
                //At least one lane requires a non-cap contact.
            }
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
                var centerOffsetAlongY = localOffsetA.Y - capHeight;
                var endpointOffsetAlongY = capsuleAxis.Y * a.HalfLength;
                var tNegative = negative.Y * inverseNormalY;
                var tPositive = positive.Y * inverseNormalY;
                Vector2Wide projectedPositive, projectedNegative;
                projectedNegative.X = localNormal.X * tNegative + negative.X;
                projectedNegative.Y = localNormal.Z * tNegative + negative.Z;
                projectedPositive.X = localNormal.X * tPositive + positive.X;
                projectedPositive.Y = localNormal.Z * tPositive + positive.Z;

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
                Vector3Wide capContact0, capContact1;
                capContact0.X = tMin * projectedOffset.X + projectedNegative.X;
                capContact0.Y = capHeight;
                capContact0.Z = tMin * projectedOffset.Y + projectedNegative.Y;
                capContact1.X = tMax * projectedOffset.X + projectedPositive.X;
                capContact1.Y = capHeight;
                capContact1.Z = tMax * projectedOffset.Y + projectedPositive.Y;
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
            //t = dot(localOffsetA - contact, faceNormalA) / dot(faceNormalA, localNormal)
            //depth = dot(contact + t * localNormal, localNormal) = dot(contact, localNormal) + t
            Vector3Wide.CrossWithoutOverlap(localNormal, capsuleAxis, out var capsuleTangent);
            Vector3Wide.CrossWithoutOverlap(capsuleTangent, capsuleAxis, out var faceNormalA);
            Vector3Wide.Dot(faceNormalA, localNormal, out var faceNormalADotLocalNormal);
            //Don't have to perform any calibration on the faceNormalA; it appears in both the numerator and denominator so the sign and magnitudes cancel.
            var inverseFaceNormalADotLocalNormal = Vector<float>.One / faceNormalADotLocalNormal;
            Vector3Wide.Scale(faceNormalA, faceNormalADotLocalNormal, out var scaledFaceNormalA);
            Vector3Wide.Subtract(localOffsetA, contact0, out var offset0);
            Vector3Wide.Subtract(localOffsetA, contact1, out var offset1);
            Vector3Wide.Dot(offset0, faceNormalA, out var t0);
            Vector3Wide.Dot(offset1, faceNormalA, out var t1);
            t0 *= inverseFaceNormalADotLocalNormal;
            t1 *= inverseFaceNormalADotLocalNormal;
            Vector3Wide.Dot(contact0, localNormal, out var dot0);
            Vector3Wide.Dot(contact1, localNormal, out var dot1);
            manifold.Depth0 = dot0 + t0 + a.Radius;
            manifold.Depth1 = dot1 + t1 + a.Radius;

            //If the capsule axis is parallel with the normal, then the contacts collapse to one point and we can use the initially computed depth.
            //In this case, both contact positions should be extremely close together anyway.
            var collapse = Vector.LessThan(Vector.Abs(faceNormalADotLocalNormal), new Vector<float>(1e-14f));
            manifold.Depth0 = Vector.ConditionalSelect(collapse, depth, manifold.Depth0);
            var negativeMargin = -speculativeMargin;
            manifold.Contact0Exists = Vector.GreaterThan(manifold.Depth0, negativeMargin);
            manifold.Contact1Exists = Vector.BitwiseAnd(Vector.AndNot(Vector.Equals(contactCount, new Vector<int>(2)), collapse), Vector.GreaterThan(manifold.Depth1, negativeMargin));

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
