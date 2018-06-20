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
        public struct Projection
        {
            //Jacobians are generated on the fly from the tangents and offsets.
            //The tangents are reconstructed from the surface basis.
            //This saves 11 floats per constraint relative to the seminaive baseline of two shared linear jacobians and four angular jacobians. 
            public Vector3Wide OffsetA;
            public Symmetric2x2Wide EffectiveMass;
        }

        //Since this is an unshared specialized implementation, the jacobian calculation is kept in here rather than in the batch.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobians(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref Vector3Wide offsetA, out Jacobians jacobians)
        {
            //TODO: there would be a minor benefit in eliminating this copy manually, since it's very likely that the compiler won't. And it's probably also introducing more locals init.
            jacobians.LinearA.X = tangentX;
            jacobians.LinearA.Y = tangentY;
            Vector3Wide.CrossWithoutOverlap(offsetA, tangentX, out jacobians.AngularA.X);
            Vector3Wide.CrossWithoutOverlap(offsetA, tangentY, out jacobians.AngularA.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref Vector3Wide offsetA, ref BodyInertias inertiaA,
            out Projection projection)
        {
            ComputeJacobians(ref tangentX, ref tangentY, ref offsetA, out var jacobians);
            //Compute effective mass matrix contributions.
            Symmetric2x2Wide.SandwichScale(jacobians.LinearA, inertiaA.InverseMass, out var linearContributionA);
            Symmetric3x3Wide.MatrixSandwich(jacobians.AngularA, inertiaA.InverseInertiaTensor, out var angularContributionA);

            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            Symmetric2x2Wide.Add(linearContributionA, angularContributionA, out var inverseEffectiveMass);
            Symmetric2x2Wide.InvertWithoutOverlap(inverseEffectiveMass, out projection.EffectiveMass);
            projection.OffsetA = offsetA;

            //Note that friction constraints have no bias velocity. They target zero velocity.
        }

        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref Jacobians jacobians, ref BodyInertias inertiaA,
            ref Vector2Wide correctiveImpulse, ref BodyVelocities wsvA)
        {
            Matrix2x3Wide.Transform(correctiveImpulse, jacobians.LinearA, out var linearImpulseA);
            Matrix2x3Wide.Transform(correctiveImpulse, jacobians.AngularA, out var angularImpulseA);
            BodyVelocities correctiveVelocityA;
            Vector3Wide.Scale(linearImpulseA, inertiaA.InverseMass, out correctiveVelocityA.Linear);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseA, inertiaA.InverseInertiaTensor, out correctiveVelocityA.Angular);
            Vector3Wide.Add(wsvA.Linear, correctiveVelocityA.Linear, out wsvA.Linear);
            Vector3Wide.Add(wsvA.Angular, correctiveVelocityA.Angular, out wsvA.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref Projection projection, ref BodyInertias inertiaA,
            ref Vector2Wide accumulatedImpulse, ref BodyVelocities wsvA)
        {
            ComputeJacobians(ref tangentX, ref tangentY, ref projection.OffsetA, out var jacobians);
            //TODO: If the previous frame and current frame are associated with different time steps, the previous frame's solution won't be a good solution anymore.
            //To compensate for this, the accumulated impulse should be scaled if dt changes.
            ApplyImpulse(ref jacobians, ref inertiaA, ref accumulatedImpulse, ref wsvA);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA, ref Projection data, ref Jacobians jacobians,
            ref Vector<float> maximumImpulse, ref Vector2Wide accumulatedImpulse, out Vector2Wide correctiveCSI)
        {
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvA.Linear, jacobians.LinearA, out var csvaLinear);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvA.Angular, jacobians.AngularA, out var csvaAngular);
            Vector2Wide.Add(csvaLinear, csvaAngular, out var csv);
            //Required corrective velocity is the negation of the current constraint space velocity.
            Symmetric2x2Wide.TransformWithoutOverlap(csv, data.EffectiveMass, out var negativeCSI);

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
        public static void Solve(ref Vector3Wide tangentX, ref Vector3Wide tangentY,
            ref Projection projection, ref BodyInertias inertiaA, ref Vector<float> maximumImpulse, ref Vector2Wide accumulatedImpulse, ref BodyVelocities wsvA)
        {
            ComputeJacobians(ref tangentX, ref tangentY, ref projection.OffsetA, out var jacobians);
            ComputeCorrectiveImpulse(ref wsvA, ref projection, ref jacobians, ref maximumImpulse, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(ref jacobians, ref inertiaA, ref correctiveCSI, ref wsvA);

        }

    }
}
