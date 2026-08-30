using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints.Contact
{
    /// <summary>
    /// Handles the tangent friction implementation for one body contact constraints.
    /// </summary>
    public static class TangentFrictionOneBody
    {
        public struct Jacobians
        {
            public Matrix2x3Wide LinearA;
            public Matrix2x3Wide AngularA;
        }

        //Since this is an unshared specialized implementation, the jacobian calculation is kept in here rather than in the batch.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobians(in Vector3Wide tangentX, in Vector3Wide tangentY, in Vector3Wide offsetA, out Jacobians jacobians)
        {
            //TODO: there would be a minor benefit in eliminating this copy manually, since it's very likely that the compiler won't. And it's probably also introducing more locals init.
            jacobians.LinearA.X = tangentX;
            jacobians.LinearA.Y = tangentY;
            Vector3Wide.CrossWithoutOverlap(offsetA, tangentX, out jacobians.AngularA.X);
            Vector3Wide.CrossWithoutOverlap(offsetA, tangentY, out jacobians.AngularA.Y);
        }

        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in Jacobians jacobians, in BodyInertiaWide inertiaA,
            in Vector2Wide correctiveImpulse, ref BodyVelocityWide wsvA)
        {
            Matrix2x3Wide.Transform(correctiveImpulse, jacobians.LinearA, out var linearImpulseA);
            Matrix2x3Wide.Transform(correctiveImpulse, jacobians.AngularA, out var angularImpulseA);
            BodyVelocityWide correctiveVelocityA;
            Vector3Wide.Scale(linearImpulseA, inertiaA.InverseMass, out correctiveVelocityA.Linear);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseA, inertiaA.InverseInertiaTensor, out correctiveVelocityA.Angular);
            Vector3Wide.Add(wsvA.Linear, correctiveVelocityA.Linear, out wsvA.Linear);
            Vector3Wide.Add(wsvA.Angular, correctiveVelocityA.Angular, out wsvA.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(in BodyVelocityWide wsvA, in Symmetric2x2Wide effectiveMass, in Jacobians jacobians,
            in Vector<float> maximumImpulse, ref Vector2Wide accumulatedImpulse, out Vector2Wide correctiveCSI)
        {
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvA.Linear, jacobians.LinearA, out var csvaLinear);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvA.Angular, jacobians.AngularA, out var csvaAngular);
            Vector2Wide.Add(csvaLinear, csvaAngular, out var csv);
            //Required corrective velocity is the negation of the current constraint space velocity.
            Symmetric2x2Wide.TransformWithoutOverlap(csv, effectiveMass, out var negativeCSI);

            var previousAccumulated = accumulatedImpulse;
            Vector2Wide.Subtract(accumulatedImpulse, negativeCSI, out accumulatedImpulse);
            //The maximum force of friction depends upon the normal impulse. The maximum is supplied per iteration.
            Vector2Wide.Length(accumulatedImpulse, out var accumulatedMagnitude);
            //Note division by zero guard.
            var scale = Vector.Min(Vector<float>.One, maximumImpulse / Vector.Max(new Vector<float>(1e-16f), accumulatedMagnitude));
            Vector2Wide.Scale(accumulatedImpulse, scale, out accumulatedImpulse);

            Vector2Wide.Subtract(accumulatedImpulse, previousAccumulated, out correctiveCSI);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(in Vector3Wide tangentX, in Vector3Wide tangentY, in Vector3Wide offsetToManifoldCenterA, in BodyInertiaWide inertiaA, in Vector2Wide accumulatedImpulse, ref BodyVelocityWide wsvA)
        {
            ComputeJacobians(tangentX, tangentY, offsetToManifoldCenterA, out var jacobians);
            //TODO: If the previous frame and current frame are associated with different time steps, the previous frame's solution won't be a good solution anymore.
            //To compensate for this, the accumulated impulse should be scaled if dt changes.
            ApplyImpulse(jacobians, inertiaA, accumulatedImpulse, ref wsvA);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(in Vector3Wide tangentX, in Vector3Wide tangentY, in Vector3Wide offsetToManifoldCenterA, in BodyInertiaWide inertiaA,
            in Vector<float> maximumImpulse, ref Vector2Wide accumulatedImpulse, ref BodyVelocityWide wsvA)
        {
            ComputeJacobians(tangentX, tangentY, offsetToManifoldCenterA, out var jacobians);
            //Compute effective mass matrix contributions.
            Symmetric2x2Wide.SandwichScale(jacobians.LinearA, inertiaA.InverseMass, out var linearContributionA);

            Symmetric3x3Wide.MatrixSandwich(jacobians.AngularA, inertiaA.InverseInertiaTensor, out var angularContributionA);

            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            Symmetric2x2Wide.Add(linearContributionA, angularContributionA, out var inverseEffectiveMass);
            Symmetric2x2Wide.InvertWithoutOverlap(inverseEffectiveMass, out var effectiveMass);

            ComputeCorrectiveImpulse(wsvA, effectiveMass, jacobians, maximumImpulse, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(jacobians, inertiaA, correctiveCSI, ref wsvA);
        }

    }
}
