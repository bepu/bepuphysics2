using System.Diagnostics;
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
            public Triangular2x2Wide EffectiveMass;
        }

        //Since this is an unshared specialized implementation, the jacobian calculation is kept in here rather than in the batch.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobians(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref Vector3Wide offsetA, out Jacobians jacobians)
        {
            //TODO: there would be a minor benefit in eliminating this copy manually, since it's very likely that the compiler won't. And it's probably also introducing more locals init.
            jacobians.LinearA.X = tangentX;
            jacobians.LinearA.Y = tangentY;
            Vector3Wide.CrossWithoutOverlap(ref offsetA, ref tangentX, out jacobians.AngularA.X);
            Vector3Wide.CrossWithoutOverlap(ref offsetA, ref tangentY, out jacobians.AngularA.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref Vector3Wide offsetA, ref BodyInertias inertiaA,
            out Projection projection)
        {
            ComputeJacobians(ref tangentX, ref tangentY, ref offsetA, out var jacobians);
            //Compute effective mass matrix contributions.
            Triangular2x2Wide.SandwichScale(ref jacobians.LinearA, ref inertiaA.InverseMass, out var linearContributionA);
            Triangular3x3Wide.MatrixSandwich(ref jacobians.AngularA, ref inertiaA.InverseInertiaTensor, out var angularContributionA);

            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            Triangular2x2Wide.Add(ref linearContributionA, ref angularContributionA, out var inverseEffectiveMass);
            Triangular2x2Wide.InvertWithoutOverlap(ref inverseEffectiveMass, out projection.EffectiveMass);
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
            Matrix2x3Wide.Transform(ref correctiveImpulse, ref jacobians.LinearA, out var linearImpulseA);
            Matrix2x3Wide.Transform(ref correctiveImpulse, ref jacobians.AngularA, out var angularImpulseA);
            BodyVelocities correctiveVelocityA;
            Vector3Wide.Scale(ref linearImpulseA, ref inertiaA.InverseMass, out correctiveVelocityA.LinearVelocity);
            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(ref angularImpulseA, ref inertiaA.InverseInertiaTensor, out correctiveVelocityA.AngularVelocity);
            Vector3Wide.Add(ref wsvA.LinearVelocity, ref correctiveVelocityA.LinearVelocity, out wsvA.LinearVelocity);
            Vector3Wide.Add(ref wsvA.AngularVelocity, ref correctiveVelocityA.AngularVelocity, out wsvA.AngularVelocity);
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
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvA.LinearVelocity, ref jacobians.LinearA, out var csvaLinear);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvA.AngularVelocity, ref jacobians.AngularA, out var csvaAngular);
            Vector2Wide.Add(ref csvaLinear, ref csvaAngular, out var csv);
            //Required corrective velocity is the negation of the current constraint space velocity.
            Triangular2x2Wide.TransformBySymmetricWithoutOverlap(ref csv, ref data.EffectiveMass, out var negativeCSI);

            var previousAccumulated = accumulatedImpulse;
            Vector2Wide.Subtract(ref accumulatedImpulse, ref negativeCSI, out accumulatedImpulse);
            //The maximum force of friction depends upon the normal impulse. The maximum is supplied per iteration.
            Vector2Wide.Length(ref accumulatedImpulse, out var accumulatedMagnitude);
            //Note division by zero guard.
            var scale = Vector.Min(Vector<float>.One, maximumImpulse / Vector.Max(new Vector<float>(1e-16f), accumulatedMagnitude));
            Vector2Wide.Scale(ref accumulatedImpulse, ref scale, out accumulatedImpulse);

            Vector2Wide.Subtract(ref accumulatedImpulse, ref previousAccumulated, out correctiveCSI);

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
