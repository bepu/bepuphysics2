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
            var depth = Vector.ConditionalSelect(internalLineSegmentIntersected, new Vector<float>(float.MaxValue), distanceFromCylinderToLineSegment);

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
            depth -= a.Radius;
            if(Vector.LessThanAll(depth, -speculativeMargin))
            {
                //All lanes have a depth which cannot create any contacts due to the speculative margin. We can early out.
                //This is determinism-safe; even if execution continued, there would be no contacts created and the manifold would be equivalent for all lanes.
                manifold = default;
                return;
            }
            manifold = default;
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
