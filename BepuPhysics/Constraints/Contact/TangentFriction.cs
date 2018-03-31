using BepuUtilities;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints.Contact
{
    /// <summary>
    /// Handles the tangent friction implementation.
    /// </summary>
    public static class TangentFriction
    {
        public struct Projection
        {
            //Jacobians are generated on the fly from the tangents and offsets.
            //The tangents are reconstructed from the surface basis.
            //This saves 11 floats per constraint relative to the seminaive baseline of two shared linear jacobians and four angular jacobians. 
            public Vector3Wide OffsetA;
            public Vector3Wide OffsetB;
            public Triangular2x2Wide EffectiveMass;
        }

        public struct Jacobians
        {
            public Matrix2x3Wide LinearA;
            public Matrix2x3Wide AngularA;
            public Matrix2x3Wide AngularB;
        }
        //Since this is an unshared specialized implementation, the jacobian calculation is kept in here rather than in the batch.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobians(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref Vector3Wide offsetA, ref Vector3Wide offsetB,
            out Jacobians jacobians)
        {
            //Two velocity constraints: 
            //dot(velocity(p, A), tangentX) = dot(velocity(p, B), tangentX)
            //dot(velocity(p, A), tangentY) = dot(velocity(p, B), tangentY)
            //where velocity(p, A) is the velocity of a point p attached to object A.
            //velocity(p, A) = linearVelocityA + angularVelocityA x (p - positionA) = linearVelocityA + angularVelocityA x offsetA
            //so:
            //dot(velocity(p, A), tangentX) = dot(linearVelocityA, tangentX) + dot(angularVelocityA x offsetA, tangentX)
            //dot(velocity(p, A), tangentX) = dot(linearVelocityA, tangentX) + dot(offsetA x tangentX, angularVelocityA)
            //Restating the two constraints:
            //dot(linearVelocityA, tangentX) + dot(offsetA x tangentX, angularVelocityA) = dot(linearVelocityB, tangentX) + dot(offsetB x tangentX, angularVelocityB)
            //dot(linearVelocityA, tangentY) + dot(offsetA x tangentY, angularVelocityA) = dot(linearVelocityB, tangentY) + dot(offsetB x tangentY, angularVelocityB)
            //dot(linearVelocityA, tangentX) + dot(offsetA x tangentX, angularVelocityA) - dot(linearVelocityB, tangentX) - dot(offsetB x tangentX, angularVelocityB) = 0
            //dot(linearVelocityA, tangentY) + dot(offsetA x tangentY, angularVelocityA) - dot(linearVelocityB, tangentY) - dot(offsetB x tangentY, angularVelocityB) = 0

            //Since there are two constraints (2DOFs), there are two rows in the jacobian, which based on the above is:
            //jLinearA = [ tangentX ]
            //           [ tangentY ]
            //jAngularA = [ offsetA x tangentX ]
            //            [ offsetA x tangentY ]
            //jLinearB = [ -tangentX ]
            //           [ -tangentY ]
            //jAngularB = [ -offsetB x tangentX ] = [ tangentX x offsetB ]
            //            [ -offsetB x tangentY ]   [ tangentY x offsetB ]
            //TODO: there would be a minor benefit in eliminating this copy manually, since it's very likely that the compiler won't. And it's probably also introducing more locals init.
            jacobians.LinearA.X = tangentX;
            jacobians.LinearA.Y = tangentY;
            Vector3Wide.CrossWithoutOverlap(ref offsetA, ref tangentX, out jacobians.AngularA.X);
            Vector3Wide.CrossWithoutOverlap(ref offsetA, ref tangentY, out jacobians.AngularA.Y);
            Vector3Wide.CrossWithoutOverlap(ref tangentX, ref offsetB, out jacobians.AngularB.X);
            Vector3Wide.CrossWithoutOverlap(ref tangentY, ref offsetB, out jacobians.AngularB.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref Vector3Wide offsetA, ref Vector3Wide offsetB,
            ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            out Projection projection)
        {
            ComputeJacobians(ref tangentX, ref tangentY, ref offsetA, ref offsetB, out var jacobians);
            //Compute effective mass matrix contributions.
            Triangular2x2Wide.SandwichScale(ref jacobians.LinearA, ref inertiaA.InverseMass, out var linearContributionA);
            Triangular2x2Wide.SandwichScale(ref jacobians.LinearA, ref inertiaB.InverseMass, out var linearContributionB);

            Triangular3x3Wide.MatrixSandwich(ref jacobians.AngularA, ref inertiaA.InverseInertiaTensor, out var angularContributionA);
            Triangular3x3Wide.MatrixSandwich(ref jacobians.AngularB, ref inertiaB.InverseInertiaTensor, out var angularContributionB);

            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            Triangular2x2Wide.Add(ref linearContributionA, ref linearContributionB, out var linear);
            Triangular2x2Wide.Add(ref angularContributionA, ref angularContributionB, out var angular);
            Triangular2x2Wide.Add(ref linear, ref angular, out var inverseEffectiveMass);
            Triangular2x2Wide.InvertSymmetricWithoutOverlap(ref inverseEffectiveMass, out projection.EffectiveMass);
            projection.OffsetA = offsetA;
            projection.OffsetB = offsetB;

            //Note that friction constraints have no bias velocity. They target zero velocity.
        }

        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref Jacobians jacobians, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref Vector2Wide correctiveImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            Matrix2x3Wide.Transform(ref correctiveImpulse, ref jacobians.LinearA, out var linearImpulseA);
            Matrix2x3Wide.Transform(ref correctiveImpulse, ref jacobians.AngularA, out var angularImpulseA);
            Matrix2x3Wide.Transform(ref correctiveImpulse, ref jacobians.AngularB, out var angularImpulseB);
            BodyVelocities correctiveVelocityA, correctiveVelocityB;
            Vector3Wide.Scale(ref linearImpulseA, ref inertiaA.InverseMass, out correctiveVelocityA.Linear);
            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(ref angularImpulseA, ref inertiaA.InverseInertiaTensor, out correctiveVelocityA.Angular);
            Vector3Wide.Scale(ref linearImpulseA, ref inertiaB.InverseMass, out correctiveVelocityB.Linear);
            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(ref angularImpulseB, ref inertiaB.InverseInertiaTensor, out correctiveVelocityB.Angular);
            Vector3Wide.Add(ref wsvA.Linear, ref correctiveVelocityA.Linear, out wsvA.Linear);
            Vector3Wide.Add(ref wsvA.Angular, ref correctiveVelocityA.Angular, out wsvA.Angular);
            Vector3Wide.Subtract(ref wsvB.Linear, ref correctiveVelocityB.Linear, out wsvB.Linear); //note subtract- we based it on the LinearA jacobian.
            Vector3Wide.Add(ref wsvB.Angular, ref correctiveVelocityB.Angular, out wsvB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref TangentFriction.Projection projection, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref Vector2Wide accumulatedImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeJacobians(ref tangentX, ref tangentY, ref projection.OffsetA, ref projection.OffsetB, out var jacobians);
            //TODO: If the previous frame and current frame are associated with different time steps, the previous frame's solution won't be a good solution anymore.
            //To compensate for this, the accumulated impulse should be scaled if dt changes.
            ApplyImpulse(ref jacobians, ref inertiaA, ref inertiaB, ref accumulatedImpulse, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref TangentFriction.Projection data, ref Jacobians jacobians,
            ref Vector<float> maximumImpulse, ref Vector2Wide accumulatedImpulse, out Vector2Wide correctiveCSI)
        {
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvA.Linear, ref jacobians.LinearA, out var csvaLinear);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvA.Angular, ref jacobians.AngularA, out var csvaAngular);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvB.Linear, ref jacobians.LinearA, out var csvbLinear);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvB.Angular, ref jacobians.AngularB, out var csvbAngular);
            //Note that the velocity in constraint space is (csvaLinear - csvbLinear + csvaAngular + csvbAngular).
            //The subtraction there is due to sharing the linear jacobian between both bodies.
            //In the following, we need to compute the constraint space *violating* velocity- which is the negation of the above velocity in constraint space.
            //So, (csvbLinear - csvaLinear - (csvaAngular + csvbAngular)).
            Vector2Wide.Subtract(ref csvbLinear, ref csvaLinear, out var csvLinear);
            Vector2Wide.Add(ref csvaAngular, ref csvbAngular, out var csvAngular);
            Vector2Wide.Subtract(ref csvLinear, ref csvAngular, out var csv);

            Triangular2x2Wide.TransformBySymmetricWithoutOverlap(ref csv, ref data.EffectiveMass, out var csi);

            var previousAccumulated = accumulatedImpulse;
            Vector2Wide.Add(ref accumulatedImpulse, ref csi, out accumulatedImpulse);
            //The maximum force of friction depends upon the normal impulse. The maximum is supplied per iteration.
            Vector2Wide.Length(ref accumulatedImpulse, out var accumulatedMagnitude);
            //Note division by zero guard.
            var scale = Vector.Min(Vector<float>.One, maximumImpulse / Vector.Max(new Vector<float>(1e-16f), accumulatedMagnitude));
            Vector2Wide.Scale(ref accumulatedImpulse, ref scale, out accumulatedImpulse);

            Vector2Wide.Subtract(ref accumulatedImpulse, ref previousAccumulated, out correctiveCSI);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref TangentFriction.Projection projection, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector<float> maximumImpulse, ref Vector2Wide accumulatedImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeJacobians(ref tangentX, ref tangentY, ref projection.OffsetA, ref projection.OffsetB, out var jacobians);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection, ref jacobians, ref maximumImpulse, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(ref jacobians, ref inertiaA, ref inertiaB, ref correctiveCSI, ref wsvA, ref wsvB);

        }

    }
}
