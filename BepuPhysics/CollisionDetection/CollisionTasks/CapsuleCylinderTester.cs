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
            out Vector<float> t, out Vector<float> min, out Vector<float> max, out Vector3Wide offsetFromCylinderToLineSegment)
        {
            min = -halfLength;
            max = halfLength;
            //TODO: could use a better initial guess.
            t = Vector<float>.Zero;
            var radiusSquared = b.Radius * b.Radius;
            var negativeCylinderHalfLength = -b.HalfLength;
            Vector3Wide.Dot(lineDirection, lineOrigin, out var originDot);
            for (int i = 0; i < 10; ++i)
            {
                Bounce(lineOrigin, lineDirection, t, b, radiusSquared, out _, out var clamped);
                Vector3Wide.Dot(clamped, lineDirection, out var conservativeNewT);
                conservativeNewT -= originDot;
                var change = conservativeNewT - t;

                //The bounced projection can be thought of as conservative advancement. The sign of the change tells us which way the advancement moved; we can use that to update the bounds.
                var movedUp = Vector.GreaterThan(change, Vector<float>.Zero);
                min = Vector.ConditionalSelect(movedUp, conservativeNewT, min);
                max = Vector.ConditionalSelect(movedUp, max, conservativeNewT);

                //Rather than being fully conservative, we move the next test location forward aggressively by scaling the change.
                t += change * 2;
                //If the new target exited the interval, then back off and bisect the remaining space between the conservative T and violated bound instead.
                t = Vector.ConditionalSelect(Vector.GreaterThan(t, max), 0.5f * (conservativeNewT + max), t);
                t = Vector.ConditionalSelect(Vector.LessThan(t, min), 0.5f * (conservativeNewT + min), t);
            }
            Bounce(lineOrigin, lineDirection, t, b, radiusSquared, out var pointOnLine, out var clampedToCylinder);
            Vector3Wide.Subtract(pointOnLine, clampedToCylinder, out offsetFromCylinderToLineSegment);
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
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRB, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);
            //Note that the localNormal contains a not-yet normalized offset. We defer normalization until all potential normals have been examined.
            GetClosestPointBetweenLineSegmentAndCylinder(localOffsetA, rA.Y, a.HalfLength, b, out var t, out _, out _, out var localNormal);
            Vector3Wide.LengthSquared(localNormal, out var distanceFromCylinderToLineSegmentSquared);
            var internalLineSegmentIntersected = Vector.LessThan(distanceFromCylinderToLineSegmentSquared, new Vector<float>(1e-12f));
            var distanceFromCylinderToLineSegment = Vector.SquareRoot(distanceFromCylinderToLineSegmentSquared);
            var depth = Vector.ConditionalSelect(internalLineSegmentIntersected, new Vector<float>(float.MaxValue), distanceFromCylinderToLineSegment - a.Radius);
            
            if (Vector.EqualsAny(internalLineSegmentIntersected, new Vector<int>(-1)))
            {
                //At least one lane is intersecting deeply, so we need to examine the other possible normals.

                //TODO: Consider using segment-segment to offer a little bit of cover for the deep intersection edge case. The clamps don't take much more than the segment-line variant.
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
