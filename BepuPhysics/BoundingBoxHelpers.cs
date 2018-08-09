using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics
{
    public static class BoundingBoxHelpers
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetAngularBoundsExpansion(in Vector<float> angularSpeed, in Vector<float> vectorDt, in Vector<float> maximumRadius,
            in Vector<float> maximumAngularExpansion, out Vector<float> angularExpansion)
        {
            /*
            Angular requires a bit more care. Since the goal is to create a tight bound, simply using a v = w * r approximation isn't ideal. A slightly tighter can be found:
            1) The maximum displacement along ANY axis during an intermediate time is equal to the distance from a starting position at MaximumRadius 
            to the position of that point at the intermediate time.
            2) The expansion cannot exceed the maximum radius, so angular deltas greater than pi/3 do not need to be considered. 
            (An expansion equal to the maximum radius would result in an equilateral triangle, which has an angle of 60 degrees in each corner.) 
            Larger values can simply be clamped.
            3) The largest displacement along any axis, at any time, is the distance from the starting position to the position at dt. Note that this only holds because of the clamp: 
            if the angle was allowed to wrap around, it the distance would start to go down again.
            4) position(time) = {radius * sin(angular speed * time), radius * cos(angular speed * time)}
            5) largest expansion required = ||position(dt) - position(0)|| = sqrt(2 * radius^2 * (1 - cos(dt * w)))
            6) Don't have any true SIMD sin function, but we can approximate it using a taylor series, like: cos(x) = 1 - x^2 / 2! + x^4 / 4! - x^6 / 6!
            7) Note that the cosine approximation should stop at a degree where it is smaller than the true value of cosine for the interval 0 to pi/3: this guarantees that the distance,
            which is larger when the cosine is smaller, is conservative and fully bounds the angular motion.

            Why do this extra work?
            1) The bounding box calculation phase, as a part of the pose integration phase, tends to be severely memory bound. 
            Spending a little of ALU time to get a smaller bounding box isn't a big concern, even though it includes a couple of sqrts.
            An extra few dozen ALU cycles is unlikely to meaningfully change the execution time.
            2) Shrinking the bounding box reduces the number of collision pairs. Collision pairs are expensive- many times more expensive than the cost of shrinking the bounding box.
            */
            var a = Vector.Min(angularSpeed * vectorDt, new Vector<float>(MathHelper.Pi / 3f));
            var a2 = a * a;
            var a4 = a2 * a2;
            var a6 = a4 * a2;
            var cosAngleMinusOne = a2 * new Vector<float>(-1f / 2f) + a4 * new Vector<float>(1f / 24f) - a6 * new Vector<float>(1f / 720f);
            //Note that it's impossible for angular motion to cause an increase in bounding box size beyond (maximumRadius-minimumRadius) on any given axis.
            //That value, or a conservative approximation, is stored as the maximum angular expansion.
            angularExpansion = Vector.Min(maximumAngularExpansion,
                Vector.SquareRoot(new Vector<float>(-2f) * maximumRadius * maximumRadius * cosAngleMinusOne));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetBoundsExpansion(in Vector3Wide linearVelocity, in Vector3Wide angularVelocity, float dt,
            in Vector<float> maximumRadius, in Vector<float> maximumAngularExpansion, out Vector3Wide minExpansion, out Vector3Wide maxExpansion)
        {
            /*
            If an object sitting on a plane had a raw (unexpanded) AABB that is just barely above the plane, no contacts would be generated. 
            If the velocity of the object would shove it down into the plane in the next frame, then it would generate contacts in the next frame- and, 
            potentially, this cycle would repeat and cause jitter.

            To solve this, there are a couple of options:
            1) Introduce an 'allowed penetration' so that objects can overlap a little bit. This tends to confuse people a little bit when they notice it, 
            and in some circumstances objects can be seen settling into the allowed penetration slowly. It looks a bit odd.
            2) Make contact constraints fight to maintain zero penetration depth, but expand the bounding box with velocity and allow contacts to be generated speculatively- 
            contacts with negative penetration depth.

            #2 is a form of continuous collision detection, but it's handy for general contact stability too. 
            In this version of the engine, all objects generate speculative contacts by default, though only within a per-collidable-tuned 'speculative margin'. 
            It's kind of like BEPUphysics v1's AllowedPenetration, except inverted. Speculative contacts that fall within the speculative margin- 
            that is, those with negative depth of a magnitude less than the margin- are kept.

            So, a user could choose to have a very large speculative margin, and the speculative contact generation would provide a form of continuous collision detection. 
            The main purpose, though, is just contact stability. With this in isolation, there's no strong reason to expand the bounding box more than the speculative margin. 
            This is the 'discrete' mode.

            However, consider what would happen if an object A with high velocity and this 'discrete' mode was headed towards an object B in a 'continuous' mode.
            Object B only expands its bounding box by its own velocity, and object A doesn't expand beyond its speculative margin. The collision between A and B could easily be missed. 
            To account for this, there is an intermediate mode- 'passive'- where the bounding box is allowed to expand beyond the margin, 
            but no further continuous collision detection is performed.

            The fully continuous modes fully expand the bounding boxes. Notably, the inner sphere continuous collision detection mode could get by with less, 
            but it would be pretty confusing to have the same kind of missed collision possibility if the other object in the pair was a substepping object.
            Two different inner sphere modes could be offered, but I'm unsure about the usefulness versus the complexity.

            (Note that there ARE situations where a bounding box which contains the full unconstrained motion will fail to capture constrained motion. 
            Consider object A flying at high speed to impact the stationary object B, which sits next to another stationary object C. 
            Object B's bounding box doesn't overlap with object C's bounding box- they're both stationary, so there's no velocity expansion. But during one frame, 
            object A slams into B, and object B's velocity during that frame now forces it to tunnel all the way through C unimpeded, because no contacts were generated between B and C. 
            There are ways to address this- all of which are a bit expensive- but CCD as implemented is not a hard guarantee. 
            It's a 'best effort' that compromises with performance. Later on, if it's really necessary, we could consider harder guarantees with higher costs, but... 
            given that no one seemed to have much of an issue with v1's rather limited CCD, it'll probably be fine.)

            So, how is the velocity expansion calculated?
            There's two parts, linear and angular.

            Linear is pretty simple- expand the bounding box in the direction of linear displacement (linearVelocity * dt).
            */

            Vector<float> vectorDt = new Vector<float>(dt);
            Vector3Wide.Scale(linearVelocity, vectorDt, out var linearDisplacement);

            var zero = Vector<float>.Zero;
            Vector3Wide.Min(zero, linearDisplacement, out minExpansion);
            Vector3Wide.Max(zero, linearDisplacement, out maxExpansion);
            Vector3Wide.Length(angularVelocity, out var angularSpeed);
            GetAngularBoundsExpansion(angularSpeed, vectorDt, maximumRadius, maximumAngularExpansion, out var angularExpansion);
            Vector3Wide.Subtract(minExpansion, angularExpansion, out minExpansion);
            Vector3Wide.Add(maxExpansion, angularExpansion, out maxExpansion);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ExpandBoundingBoxes(ref Vector3Wide min, ref Vector3Wide max, ref BodyVelocities velocities, float dt,
            ref Vector<float> maximumRadius, ref Vector<float> maximumAngularExpansion, ref Vector<float> maximumExpansion)
        {
            GetBoundsExpansion(velocities.Linear, velocities.Angular, dt, maximumRadius, maximumAngularExpansion, out var minDisplacement, out var maxDisplacement);
            //The maximum expansion passed into this function is the speculative margin for discrete mode collidables, and ~infinity for passive or continuous ones.
            Vector3Wide.Max(-maximumExpansion, minDisplacement, out minDisplacement);
            Vector3Wide.Min(maximumExpansion, maxDisplacement, out maxDisplacement);

            Vector3Wide.Add(min, minDisplacement, out min);
            Vector3Wide.Add(max, maxDisplacement, out max);
        }


        //This is simply a internally vectorized version of the above.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetAngularBoundsExpansion(in Vector3 angularVelocity, float dt,
            float maximumRadius, float maximumAngularExpansion, out Vector3 expansion)
        {
            var angularVelocityMagnitude = angularVelocity.Length();
            var a = MathHelper.Min(angularVelocityMagnitude * dt, MathHelper.Pi / 3f);
            var a2 = a * a;
            var a4 = a2 * a2;
            var a6 = a4 * a2;
            var cosAngleMinusOne = a2 * (-1f / 2f) + a4 * (1f / 24f) - a6 * (1f / 720f);
            //Note that it's impossible for angular motion to cause an increase in bounding box size beyond (maximumRadius-minimumRadius) on any given axis.
            //That value, or a conservative approximation, is stored as the maximum angular expansion.
            expansion = new Vector3(MathHelper.Min(maximumAngularExpansion,
                (float)Math.Sqrt(-2f * maximumRadius * maximumRadius * cosAngleMinusOne)));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetBoundsExpansion(in Vector3 linearVelocity, in Vector3 angularVelocity, float dt,
            float maximumRadius, float maximumAngularExpansion, float maximumAllowedExpansion, out Vector3 minExpansion, out Vector3 maxExpansion)
        {
            var linearDisplacement = linearVelocity * dt;
            Vector3 zero = default;
            minExpansion = Vector3.Min(zero, linearDisplacement);
            maxExpansion = Vector3.Max(zero, linearDisplacement);
            GetAngularBoundsExpansion(angularVelocity, dt, maximumRadius, maximumAngularExpansion, out var angularExpansion);

            var maximumAllowedExpansionBroadcasted = new Vector3(maximumAllowedExpansion);
            minExpansion = Vector3.Max(-maximumAllowedExpansionBroadcasted, minExpansion - angularExpansion);
            maxExpansion = Vector3.Min(maximumAllowedExpansionBroadcasted, maxExpansion + angularExpansion);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ExpandBoundingBox(ref Vector3 min, ref Vector3 max, in Vector3 linearVelocity, in Vector3 angularVelocity, float dt,
        float maximumRadius, float maximumAngularExpansion, float maximumAllowedExpansion)
        {
            GetBoundsExpansion(linearVelocity, angularVelocity, dt, maximumRadius, maximumAngularExpansion, maximumAllowedExpansion, out var minExpansion, out var maxExpansion);
            min += minExpansion;
            max += maxExpansion;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void ExpandBoundingBox(in Vector3Wide expansion, ref Vector3Wide min, ref Vector3Wide max)
        {
            Vector3Wide.Min(Vector<float>.Zero, expansion, out var minExpansion);
            Vector3Wide.Max(Vector<float>.Zero, expansion, out var maxExpansion);
            Vector3Wide.Add(min, minExpansion, out min);
            Vector3Wide.Add(max, maxExpansion, out max);
        }

        /// <summary>
        /// Expands the bounding box surrounding a shape A in the local space of some other collidable B.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ExpandLocalBoundingBoxes(ref Vector3Wide min, ref Vector3Wide max,
            in Vector<float> radiusA, in Vector3Wide localPositionA, in Vector3Wide localRelativeLinearVelocityA, in Vector3Wide angularVelocityA, in Vector3Wide angularVelocityB, float dt,
            in Vector<float> maximumRadius, in Vector<float> maximumAngularExpansion, in Vector<float> maximumAllowedExpansion)
        {
            GetBoundsExpansion(localRelativeLinearVelocityA, angularVelocityA, dt,
                maximumRadius + radiusA, maximumAngularExpansion + radiusA, out var minExpansion, out var maxExpansion);
            Vector3Wide.LengthSquared(angularVelocityB, out var angularSpeedBSquared);
            if (Vector.GreaterThanAny(angularSpeedBSquared, Vector<float>.Zero))
            {
                //Worst case radius assumes the linear motion is separating the objects as directly as possible.
                Vector3Wide.Length(localPositionA, out var radiusB);
                Vector3Wide.Length(localRelativeLinearVelocityA, out var linearSpeed);
                var worstCaseRadius = linearSpeed * dt + radiusB;
                GetAngularBoundsExpansion(Vector.SquareRoot(angularSpeedBSquared), maximumRadius + worstCaseRadius, maximumAngularExpansion + worstCaseRadius, new Vector<float>(dt), out var angularExpansionB);
                Vector3Wide.Subtract(minExpansion, angularExpansionB, out minExpansion);
                Vector3Wide.Add(maxExpansion, angularExpansionB, out maxExpansion);
            }

            //Clamp the expansion to the pair imposed limit. Discrete pairs don't need to look beyond their speculative margin.
            Vector3Wide.Min(maximumAllowedExpansion, maxExpansion, out maxExpansion);
            Vector3Wide.Max(-maximumAllowedExpansion, minExpansion, out minExpansion);

            Vector3Wide.Add(minExpansion, min, out min);
            Vector3Wide.Add(maxExpansion, max, out max);
            Vector3Wide.Add(min, localPositionA, out min);
            Vector3Wide.Add(max, localPositionA, out max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void ExpandBoundingBox(in Vector3 expansion, ref Vector3 min, ref Vector3 max)
        {
            var minExpansion = Vector3.Min(default, expansion);
            var maxExpansion = Vector3.Max(default, expansion);
            min += minExpansion;
            max += maxExpansion;
        }

        /// <summary>
        /// Computes the bounding box of shape A in the local space of some other collidable B with a sweep direction representing the net linear motion.
        /// </summary>
        public static unsafe void GetLocalBoundingBoxForSweep(TypedIndex shapeIndex, Shapes shapes, in RigidPose localPoseA, in Quaternion orientationA, in BodyVelocity velocityA,
            in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float dt, out Vector3 sweep, out Vector3 min, out Vector3 max)
        {
            //TODO: For any significant amount of B angular velocity, the resulting bounding boxes can be enormous in local space.
            //You should strongly consider heuristically choosing a world space path. For tree-based compounds, this would require a dedicated slow world space traversal.
            //For a list compound, the world space test is always the right choice. IBoundsQueryableCompound could expose heuristically useful information.
            Quaternion.Conjugate(orientationB, out var inverseOrientationB);
            Quaternion.TransformWithoutOverlap(velocityA.Linear - velocityB.Linear * dt, inverseOrientationB, out sweep);
            Quaternion.TransformWithoutOverlap(offsetB, inverseOrientationB, out var localOffsetB);
            Quaternion.ConcatenateWithoutOverlap(orientationA, inverseOrientationB, out var localOrientationA);
            Compound.GetRotatedChildPose(localPoseA, localOrientationA, out var pose);

            shapes[shapeIndex.Type].ComputeBounds(shapeIndex.Index, pose.Orientation, out var maximumRadiusA, out var maximumAngularExpansionA, out min, out max);
            BoundingBoxHelpers.GetAngularBoundsExpansion(velocityA.Angular, dt, maximumRadiusA, maximumAngularExpansionA, out var angularExpansionA);
            //The furthest the convex can be from the compound is no further than the sweep pushing it directly away from the compound.
            var worstCaseRadiusB = sweep.Length() + localOffsetB.Length();
            BoundingBoxHelpers.GetAngularBoundsExpansion(velocityB.Angular, dt, worstCaseRadiusB + maximumRadiusA, worstCaseRadiusB + maximumAngularExpansionA, out var angularExpansionB);
            var combinedAngularExpansion = angularExpansionA + angularExpansionB;

            min = min - localOffsetB - combinedAngularExpansion;
            max = max - localOffsetB + combinedAngularExpansion;
        }
        /// <summary>
        /// Computes the bounding box of shape A in the local space of some other collidable B with a sweep direction representing the net linear motion.
        /// </summary>
        public static unsafe void GetLocalBoundingBoxForSweep<TConvex>(ref TConvex shape, in RigidPose localPoseA, in Quaternion orientationA, in BodyVelocity velocityA,
            in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float dt, out Vector3 sweep, out Vector3 min, out Vector3 max) where TConvex : struct, IConvexShape
        {
            //TODO: For any significant amount of B angular velocity, the resulting bounding boxes can be enormous in local space.
            //You should strongly consider heuristically choosing a world space path. For tree-based compounds, this would require a dedicated slow world space traversal.
            //For a list compound, the world space test is always the right choice. IBoundsQueryableCompound could expose heuristically useful information.
            Quaternion.Conjugate(orientationB, out var inverseOrientationB);
            Quaternion.TransformWithoutOverlap(velocityA.Linear - velocityB.Linear * dt, inverseOrientationB, out sweep);
            Quaternion.TransformWithoutOverlap(offsetB, inverseOrientationB, out var localOffsetB);
            Quaternion.ConcatenateWithoutOverlap(orientationA, inverseOrientationB, out var localOrientationA);

            shape.ComputeAngularExpansionData(out var maximumRadiusA, out var maximumAngularExpansionA);
            BoundingBoxHelpers.GetAngularBoundsExpansion(velocityA.Angular, dt, maximumRadiusA, maximumAngularExpansionA, out var angularExpansionA);
            //The furthest the convex can be from the compound is no further than the sweep pushing it directly away from the compound.
            var worstCaseRadiusB = sweep.Length() + localOffsetB.Length();
            BoundingBoxHelpers.GetAngularBoundsExpansion(velocityB.Angular, dt, worstCaseRadiusB + maximumRadiusA, worstCaseRadiusB + maximumAngularExpansionA, out var angularExpansionB);
            var combinedAngularExpansion = angularExpansionA + angularExpansionB;

            shape.ComputeBounds(localOrientationA, out min, out max);
            min = min - localOffsetB - combinedAngularExpansion;
            max = max - localOffsetB + combinedAngularExpansion;
        }
    }
}
